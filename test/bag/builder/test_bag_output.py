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
    manifest = read_json(root / "bag_manifest.json")
    for relative in manifest["files"]:
        assert (root / relative).is_file()
    assert manifest["artifact_kind"] == "bag_preparation"
    assert not manifest["pose_computed"]


def main():
    """Find the latest C++ fixture, verify it, then compare two deterministic exports."""
    parent = Path(sys.argv[1])
    root = Path((parent / "latest.txt").read_text().strip())
    check_dataset(root / "normal_out")
    check_dataset(root / "repeat_out")
    first_files = sorted((root / "normal_out").rglob("*"))
    for path in first_files:
        if path.is_file():
            other = root / "repeat_out" / path.relative_to(root / "normal_out")
            assert other.read_bytes() == path.read_bytes(), f"Nondeterministic output: {path}"
    print(f"Independent NumPy/JSON/raw evidence verification passed: {root}")


if __name__ == "__main__":
    main()
