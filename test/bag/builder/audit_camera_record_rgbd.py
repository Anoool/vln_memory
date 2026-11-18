"""比较 camera_record 中指定观测的 RGB 和深度边缘位置。"""
import json
from pathlib import Path
import sys

import cv2
import numpy as np

SAMPLES = (635, 2701, 3374, 3360, 1564, 1578, 1607, 1633)
# 在固定水平切线上测量 RGB 边缘的 x 坐标。深度边缘的 x 坐标取同一窗口中的中位数，
# 仅统计跳变两侧深度均有效的位置。
SLICES = (
    (2701, "chair leg left", 445, 1038, 1140, 1085.5),
    (2701, "chair leg left", 530, 1005, 1100, 1060.5),
    (3374, "cabinet left trim", 350, 805, 884, 834.5),
    (3360, "foreground post left", 250, 553, 723, 581.5),
    (1578, "chair base inner edge", 465, 770, 855, 801.5),
    (1607, "desk leg inner edge", 345, 923, 1000, 969.5),
    (1633, "desk leg left", 355, 1010, 1068, 1029.5),
)


def rotation(pose):
    """将录制的光学坐标系到地图坐标系的四元数转换为旋转矩阵。"""
    x, y, z, w = pose["rotation_xyzw"]
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def edge_points(depth):
    """对跳变两侧深度均有效的位置，返回靠近相机一侧的像素。"""
    mask = np.zeros(depth.shape, bool)
    for a, b, out_a, out_b in (
        (depth[:, :-1], depth[:, 1:], mask[:, :-1], mask[:, 1:]),
        (depth[:-1], depth[1:], mask[:-1], mask[1:]),
    ):
        jump = (a > 0) & (b > 0) & (abs(a-b) > np.maximum(0.06, 0.06*np.minimum(a,b)))
        out_a |= jump & (a < b)
        out_b |= jump & (b < a)
    y, x = np.nonzero(mask)
    return np.column_stack([x,y]), depth[y,x]


def project(uv, depths, row, motion, rectified=False):
    """将保存的深度像素投影到原始 RGB 图像上，可选用深度和 RGB 各自的时间戳。"""
    depth, rgb = row["depth_model"], row["rgb_model"]
    kd = np.array(depth["k"]).reshape(3,3)
    kr = np.array(rgb["k"]).reshape(3,3)
    dd = np.zeros(5) if rectified else np.array(depth["d"])
    rays = cv2.undistortPoints(uv.astype(float), kd, dd).reshape(-1,2)
    xyz = np.column_stack([rays,np.ones(len(rays))])*depths[:,None]
    if motion:
        a,b = row["depth_camera_pose_map"],row["camera_pose_map"]
        xyz = (xyz @ rotation(a).T + np.array(a["translation_m"]) - np.array(b["translation_m"])) @ rotation(b)
    valid = xyz[:,2] > 0
    uv, _ = cv2.projectPoints(xyz[valid],np.zeros(3),np.zeros(3),kr,np.array(rgb["d"]))
    return uv.reshape(-1,2)


def overlay(image, points):
    """将投影后的深度跳变位置绘制到图像副本中，供检查。"""
    result = image.copy()
    h,w = result.shape[:2]
    x,y = np.rint(points).astype(int).T
    keep = (x>=0)&(y>=0)&(x<w)&(y<h)
    mask = np.zeros((h,w),np.uint8)
    mask[y[keep],x[keep]] = 1
    result[mask!=0] = (0,0,255)
    return result


def inspect(dataset, output):
    """保存三张对照网格图和精确的像素坐标。"""
    source = json.loads((dataset/"evidence/source.json").read_text())
    if [f["sha256"] for f in source["source_files"]] != [
        "5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d"
    ]:
        raise ValueError("Expected the audited camera_record bag")
    output.mkdir(parents=True, exist_ok=True)
    rows = {}
    samples = []
    measurements = []
    for line in (dataset/"observations.jsonl").read_text().splitlines():
        row = json.loads(line)
        number = int(Path(row["rgb_media"]).stem.rsplit("_",1)[1])
        rows[number] = row
    for number in SAMPLES:
        row = rows[number]
        rgb = cv2.imread(str(dataset/row["rgb_media"]))
        raw = cv2.imread(str(dataset/row["depth_media"]),cv2.IMREAD_UNCHANGED)
        assert raw.shape == rgb.shape[:2] and raw.dtype == np.uint16
        uv,z = edge_points(raw.astype(np.float64)*0.001)
        points = {"raw":uv, "tf":project(uv,z,row,True),
                  "rectified":project(uv,z,row,False,True), "rectified_tf":project(uv,z,row,True,True)}
        samples.append({"rgb_message_id": row["rgb_message_id"],
                        "depth_message_id": row["depth_message_id"],
                        "rgb_stamp": row["rgb_stamp"], "depth_stamp": row["depth_stamp"],
                        "delta_ns": row["delta_ns"], "two_valid_depth_edge_pixels": len(uv)})
        for frame, feature, y, lo, hi, rgb_x in SLICES:
            if frame != number:
                continue
            sample = {"rgb_number": number, "feature": feature, "y": y, "rgb_x": rgb_x}
            for name, xy in points.items():
                window = (abs(xy[:,1]-y) <= 5) & (xy[:,0] >= lo) & (xy[:,0] <= hi)
                if window.any():
                    sample[f"{name}_abs_error_px"] = round(abs(float(np.median(xy[window,0]))-rgb_x), 2)
            measurements.append(sample)
        for name, xy in points.items():
            cv2.imwrite(str(output/f"{number}-{name}.png"),overlay(rgb,xy))
    report = {"source_files": source["source_files"], "samples": samples,
              "measurements": measurements, "alignment_status": "unknown",
              "capture_sync_status": "unknown"}
    (output/"result.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    inspect(Path(sys.argv[1]), Path(sys.argv[2]))
