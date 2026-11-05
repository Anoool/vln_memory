"""Independently compare one SQLite recording with a CameraModel preparation dataset."""

import argparse
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def rows(path):
    """Read strict JSONL objects without accepting NaN/Infinity extensions."""
    def reject(value):
        """Reject non-JSON floating point constants."""
        raise ValueError(value)
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            yield json.loads(line, parse_constant=reject)


def source_messages(db, selected_topics):
    """Index original message bytes by topic, record time and hash; do not alter SQLite."""
    connection = sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True)
    try:
        assert connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
        result = Counter()
        for topic, timestamp, data in connection.execute(
            "SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id"
        ):
            if topic in selected_topics:
                result[(topic, timestamp, hashlib.sha256(data).hexdigest())] += 1
        return result
    finally:
        connection.close()


def message_bytes(root, item, connection):
    """Read retained CDR or find exact source bytes in the read-only recording."""
    if item["raw_path"] is not None:
        data = (root / item["raw_path"]).read_bytes()
        assert len(data) == item["serialized_bytes"]
        assert hashlib.sha256(data).hexdigest() == item["payload_sha256"]
        return data
    cursor = connection.execute(
        "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
        "WHERE t.name=? AND m.timestamp=? ORDER BY m.id",
        (item["topic"], int(item["bag_record_time_ns"])))
    for (data,) in cursor:
        if len(data) == item["serialized_bytes"] and hashlib.sha256(data).hexdigest() == item["payload_sha256"]:
            return data
    raise AssertionError("Source message not found: " + item["message_id"])


def check_messages(root, source, connection):
    """Verify every exported raw CDR against original bytes and consume duplicate multiplicity."""
    messages = {}
    for item in rows(root / "messages.jsonl"):
        raw = message_bytes(root, item, connection)
        digest = hashlib.sha256(raw).hexdigest()
        assert digest == item["payload_sha256"] and len(raw) == item["serialized_bytes"]
        key = (item["topic"], int(item["bag_record_time_ns"]), digest)
        assert source[key] > 0
        source[key] -= 1
        assert item["message_id"] not in messages
        messages[item["message_id"]] = item
    assert sum(source.values()) == 0, "Source has messages not represented by this full dataset"
    return messages


def check_calibrations(root, messages, connection):
    """Compare raw coefficients/matrices/header and derive D5 independently of C++ normalization."""
    calibrations = {}
    for item in rows(root / "camera_info.jsonl"):
        identifier = item["calibration_id"]
        source = messages[identifier]
        message = deserialize_message(message_bytes(root, source, connection), CameraInfo)
        assert item["d"] == list(message.d)
        for name in ("k", "r", "p"):
            assert item[name] == list(getattr(message, name))
        assert item["stamp"] == {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}
        assert item["frame_id"] == message.header.frame_id
        assert (item["width"], item["height"]) == (message.width, message.height)
        assert (item["binning_x"], item["binning_y"]) == (message.binning_x, message.binning_y)
        for field in ("x_offset", "y_offset", "width", "height", "do_rectify"):
            assert item["roi"][field] == getattr(message.roi, field)
        assert item["distortion_model"] == message.distortion_model == "plumb_bob"
        assert len(message.d) == 8 and list(message.d[5:]) == [0.0, 0.0, 0.0]
        assert item["effective_d"] == list(message.d[:5])
        assert item["validation_code"] == "invalid_distortion"
        assert item["normalized_code"] == "none" and item["normalization_applied"]
        assert item["compatibility"] == "plumb_bob_zero_tail_d8_v1"
        calibrations[identifier] = item
    return calibrations


def decode_pixels(message, kind):
    """Decode original JPEG/16UC1 compressedDepth with Python and apply explicit millimetres."""
    if isinstance(message, CompressedImage):
        payload = bytes(message.data)
        if kind == "rgb":
            return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        offset = payload.find(b"\x89PNG\r\n\x1a\n")
        assert offset >= 0
        raw = cv2.imdecode(np.frombuffer(payload[offset:], np.uint8), cv2.IMREAD_UNCHANGED)
    else:
        assert kind == "depth" and message.encoding == "16UC1"
        endian = ">u2" if message.is_bigendian else "<u2"
        raw = np.ndarray((message.height, message.width), dtype=endian,
                         buffer=bytes(message.data), strides=(message.step, 2))
    assert raw is not None and raw.dtype == np.uint16
    depth = (raw.astype(np.float64) * 0.001).astype("<f4")
    depth[raw == 0] = np.nan
    return depth


def check_model(frame, calibration):
    """Compute actual grid matrices with independent matrix multiplication A*K and A*P."""
    model = frame["camera_model"]
    assert frame["effective_model_computed"] and model["code"] == "ready"
    assert model["image_domain"] == "raw"
    assert (model["width"], model["height"], model["frame_id"]) == (
        frame["width"], frame["height"], frame["frame_id"])
    assert model["d"] == calibration["effective_d"] and model["normalization_applied"]
    bx, by = calibration["binning_x"] or 1, calibration["binning_y"] or 1
    roi = calibration["roi"]
    adjustment = np.array([[1 / bx, 0, -roi["x_offset"] / bx],
                           [0, 1 / by, -roi["y_offset"] / by], [0, 0, 1]])
    np.testing.assert_allclose(np.reshape(model["k"], (3, 3)),
                               adjustment @ np.reshape(calibration["k"], (3, 3)), rtol=0, atol=1e-12)
    np.testing.assert_allclose(np.reshape(model["p"], (3, 4)),
                               adjustment @ np.reshape(calibration["p"], (3, 4)), rtol=0, atol=1e-12)
    assert model["r"] == calibration["r"]


def check_frames(root, messages, calibrations, types, connection):
    """Check all source headers, actual models, JPEG pixels and metric depth arrays."""
    frames = {}
    for frame in rows(root / "frames.jsonl"):
        identifier = frame["source"]["message_id"]
        source = messages[identifier]
        message_type = Image
        if types[source["topic"]] == "sensor_msgs/msg/CompressedImage":
            message_type = CompressedImage
        message = deserialize_message(message_bytes(root, source, connection), message_type)
        assert frame["stamp"] == {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}
        assert frame["frame_id"] == message.header.frame_id
        assert frame["decode_code"] == "none"
        assert frame["map_camera_tf"]["stamp"] == frame["stamp"]
        if "map_body_tf" in frame:
            assert frame["map_body_tf"]["stamp"] == frame["stamp"]
        calibration = calibrations[frame["calibration"]["calibration_id"]]
        assert frame["calibration"]["code"] == "matched"
        check_model(frame, calibration)
        original = decode_pixels(message, frame["kind"])
        if frame["kind"] == "rgb":
            stored = cv2.imread(str(root / frame["media_path"]), cv2.IMREAD_COLOR)
        else:
            storage = frame["details"].get("storage")
            if storage and storage["format"] == "png":
                raw = cv2.imread(str(root / frame["media_path"]), cv2.IMREAD_UNCHANGED)
                assert raw.dtype == np.uint16
                stored = (raw.astype(np.float64) * storage["scale_to_m"]).astype("<f4")
                stored[raw == 0] = np.nan
            else:
                stored = np.load(root / frame["media_path"], allow_pickle=False)
            bounds = storage or {}
            minimum = bounds.get("min_depth_m", 0)
            maximum = bounds.get("max_depth_m")
            original[original < minimum] = np.nan
            if maximum is not None:
                original[original > maximum] = np.nan
        assert stored.shape[:2] == (frame["height"], frame["width"])
        np.testing.assert_equal(stored, original)
        frames[identifier] = frame
    return frames


def check_pairs(root, frames, summary):
    """Recompute signed nanosecond deltas, one-to-one usage and both-model status."""
    def ns(stamp):
        """Convert raw integer seconds/nanoseconds without floating point."""
        return stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
    used_rgb, used_depth = set(), set()
    for pair in rows(root / "rgbd_pairs.jsonl"):
        rgb, depth = pair["rgb_message_id"], pair["depth_message_id"]
        assert rgb not in used_rgb and depth not in used_depth
        used_rgb.add(rgb); used_depth.add(depth)
        assert pair["rgb_stamp"] == frames[rgb]["stamp"] and pair["depth_stamp"] == frames[depth]["stamp"]
        delta = ns(pair["depth_stamp"]) - ns(pair["rgb_stamp"])
        assert int(pair["delta_ns"]) == delta and abs(delta) <= int(pair["sync_tolerance_ns"])
        assert pair["exact"] == (delta == 0)
        assert pair["effective_model_computed"]
        assert not any("calibration_unavailable" in reason or "camera_model_unavailable" in reason
                       for reason in pair["reasons"])
    assert len(used_rgb) == len(used_depth) == summary["pairs"]


def main():
    """Audit the supplied full recording and write a compact reproducible result."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True, type=Path)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()
    root = args.dataset
    manifest = json.loads((root / "bag_manifest.json").read_text())
    assert manifest["builder_version"] in (4, 5, 6, 7)
    types = {topic["name"]: topic["type"] for topic in manifest["topics"]}
    assert manifest["effective_options"]["depth"]["input_unit"] == "mm"
    selected = {topic["name"] for topic in manifest["topics"] if topic["selected"]}
    with closing(sqlite3.connect(args.db.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
        messages = check_messages(root, source_messages(args.db, selected), connection)
        print("Source message bytes verified:", len(messages), flush=True)
        calibrations = check_calibrations(root, messages, connection)
        print("Raw/derived calibration verified:", len(calibrations), flush=True)
        frames = check_frames(root, messages, calibrations, types, connection)
    summary = json.loads((root / "reports/summary.json").read_text())
    check_pairs(root, frames, summary)
    assert summary["camera_models"] == len(frames)
    issues = Counter(item["code"] for item in rows(root / "reports/issues.jsonl"))
    assert "invalid_distortion" not in issues
    result = {"messages": len(messages), "calibrations": len(calibrations), "frames": len(frames),
              "camera_models": summary["camera_models"], "pairs": summary["pairs"],
              "prepared_pairs": summary["prepared_pairs"], "issues": dict(issues),
              "cdr_header_calibration_model_pixels_pair_mismatches": 0}
    args.report.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
