"""Independently read C++ artifacts with Python JSON, SHA-256 and NumPy."""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def read_json(path):
    """Read strict JSON; reject nonstandard NaN/Infinity tokens."""
    def reject_constant(value):
        """Fail on a non-JSON numerical literal instead of accepting Python's extension."""
        raise ValueError(f"Nonstandard JSON constant: {value}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def read_lines(path):
    """Read each nonempty JSONL object and retain source order for reference checks."""
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        assert isinstance(row, dict)
        assert row["schema_version"] == "bag_preparation/1"
        rows.append(row)
    return rows


def check_dataset(root):
    """Verify image references, NPY metric values, raw bytes and TF endpoint references."""
    frames = read_lines(root / "frames.jsonl")
    messages = read_lines(root / "messages.jsonl")
    message_ids = set()
    for message in messages:
        message_id = message["message_id"]
        assert message_id not in message_ids
        message_ids.add(message_id)
        raw = (root / message["raw_path"]).read_bytes()
        assert len(raw) == message["serialized_bytes"]
        assert hashlib.sha256(raw).hexdigest() == message["payload_sha256"]
    depth_arrays = []
    for frame in frames:
        assert frame["source"]["message_id"] in message_ids
        assert not frame["pose_computed"]
        if not frame["metadata_available"]:
            assert frame["stamp"] is None
            assert frame["decode_code"] == "deserialize_failed"
        if frame["media_path"] is None:
            assert frame["decode_code"] != "none"
            continue
        assert (root / frame["media_path"]).is_file()
        if frame["kind"] == "depth":
            values = np.load(root / frame["media_path"], allow_pickle=False)
            assert values.dtype == np.dtype("<f4")
            assert values.shape == (1, 2)
            depth_arrays.append(values)
        for name in ("map_camera_tf", "map_body_tf"):
            if name not in frame:
                continue
            for edge in frame[name]["edges"]:
                assert edge["before"]["message_id"] in message_ids
                if edge["after"] is not None:
                    assert edge["after"]["message_id"] in message_ids
    assert len(depth_arrays) == 3
    np.testing.assert_equal(depth_arrays[0], np.array([[1.0, np.nan]], dtype="<f4"))
    np.testing.assert_equal(depth_arrays[1], np.array([[2.0, 3.0]], dtype="<f4"))
    assert np.isnan(depth_arrays[2]).all()
    pair_ids = set()
    used_depth = set()
    for pair in read_lines(root / "rgbd_pairs.jsonl"):
        assert pair["pair_id"] not in pair_ids
        pair_ids.add(pair["pair_id"])
        assert pair["rgb_message_id"] in message_ids
        assert pair["depth_message_id"] in message_ids
        assert pair["depth_message_id"] not in used_depth
        used_depth.add(pair["depth_message_id"])
        assert pair["exact"] is True
        assert pair["delta_ns"] == "0"
        assert pair["rgb_stamp"] == pair["depth_stamp"]
        assert pair["policy"] == "exact_header_one_to_one_v1"
        assert pair["capture_sync_status"] == "unknown"

    manifest = read_json(root / "bag_manifest.json")
    for relative in manifest["files"]:
        assert (root / relative).is_file()
    assert manifest["artifact_kind"] == "bag_preparation"
    assert not manifest["pose_computed"]
    assert manifest["effective_options"]["sync_tolerance_ns"] == "0"
    summary = read_json(root / "reports/summary.json")
    assert summary["exact_pairs"] == summary["pairs"]
    assert summary["approximate_pairs"] == 0
    assert summary["valid_time_rgb"] == summary["pairs"] + summary["unmatched_rgb"]
    assert summary["valid_time_depth"] == summary["pairs"] + summary["unused_depth"]


def check_nearest_dataset(root):
    """Recompute signed deltas and uniqueness from original CDR and exported frames."""
    import struct
    messages = {}
    for message in read_lines(root / "messages.jsonl"):
        messages[message["message_id"]] = message
    frames = {}
    for frame in read_lines(root / "frames.jsonl"):
        message_id = frame["source"]["message_id"]
        frames[message_id] = frame
        if frame["metadata_available"]:
            raw = (root / messages[message_id]["raw_path"]).read_bytes()
            # ROS CDR encapsulation precedes Header stamp; support recorded byte order.
            endian = ">"
            if raw[1] & 1:
                endian = "<"
            sec, nanosec = struct.unpack_from(endian + "iI", raw, 4)
            assert frame["stamp"] == {"sec": sec, "nanosec": nanosec}
            if "map_camera_tf" in frame:
                assert frame["map_camera_tf"]["stamp"] == frame["stamp"]
    check_pair_times(root, frames)


def check_pair_times(root, frames):
    """Check unique pairs, signed deltas, actual equality and report conservation."""
    used_rgb = set()
    used_depth = set()
    summary = read_json(root / "reports/summary.json")
    tolerance_ns = int(summary["sync_tolerance_ns"])
    count = 0
    exact_count = 0
    max_abs_delta = 0
    for pair in read_lines(root / "rgbd_pairs.jsonl"):
        rgb_id = pair["rgb_message_id"]
        depth_id = pair["depth_message_id"]
        assert rgb_id not in used_rgb and depth_id not in used_depth
        used_rgb.add(rgb_id)
        used_depth.add(depth_id)
        assert pair["rgb_stamp"] == frames[rgb_id]["stamp"]
        assert pair["depth_stamp"] == frames[depth_id]["stamp"]
        rgb_ns = pair["rgb_stamp"]["sec"] * 1000000000 + pair["rgb_stamp"]["nanosec"]
        depth_ns = pair["depth_stamp"]["sec"] * 1000000000 + pair["depth_stamp"]["nanosec"]
        delta_ns = depth_ns - rgb_ns
        assert int(pair["delta_ns"]) == delta_ns
        assert abs(delta_ns) <= tolerance_ns
        assert pair["exact"] == (delta_ns == 0)
        assert pair["sync_tolerance_ns"] == str(tolerance_ns)
        assert pair["policy"] == summary["timestamp_match_policy"]
        assert pair["capture_sync_status"] == "unknown"
        max_abs_delta = max(max_abs_delta, abs(delta_ns))
        count += 1
        if delta_ns == 0:
            exact_count += 1
    assert summary["pairs"] == count
    assert summary["exact_pairs"] == exact_count
    assert summary["approximate_pairs"] == count - exact_count
    assert int(summary["max_abs_delta_ns"]) == max_abs_delta
    assert summary["valid_time_rgb"] == count + summary["unmatched_rgb"]
    assert summary["valid_time_depth"] == count + summary["unused_depth"]


def main():
    """Find the latest C++ fixture, verify it, then compare two deterministic exports."""
    parent = Path(sys.argv[1])
    root = Path((parent / "latest.txt").read_text().strip())
    check_dataset(root / "normal_out")
    check_dataset(root / "repeat_out")
    for prefix in ("near_raw", "near_compressed"):
        for tolerance_ns in (0, 1, 2):
            check_nearest_dataset(root / f"{prefix}_{tolerance_ns}_out")
    check_nearest_dataset(root / "nearest_exact_out")
    check_nearest_dataset(root / "nearest_empty_out")
    first_files = sorted((root / "normal_out").rglob("*"))
    for path in first_files:
        if path.is_file():
            other = root / "repeat_out" / path.relative_to(root / "normal_out")
            assert other.read_bytes() == path.read_bytes(), f"Nondeterministic output: {path}"
    print(f"Independent NumPy/JSON/raw evidence verification passed: {root}")


if __name__ == "__main__":
    main()
