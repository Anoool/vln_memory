"""Exercise installed-style observation dataset CLI against BagBuilder fixtures."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def read_json(path):
  """Load one artifact with its declared JSON object format."""
  return json.loads(path.read_text(encoding="utf-8"))


def rows(path):
  """Load rows in their published order."""
  with path.open(encoding="utf-8") as stream:
    return [json.loads(line) for line in stream]


def call(binary, source, output, purpose="rgb", map_frame="map", cwd=None):
  """Run the public CLI and keep return code and structured outcome."""
  completed = subprocess.run([
    str(binary.resolve()), "--input", str(source.resolve()), "--output", str(output), "--purpose", purpose,
    "--batch-size", "1", "--map-id", "fixture", "--geometry-revision", "v1",
    "--map-frame", map_frame, "--map-basis", "Fixture recorded TF"],
    cwd=cwd, capture_output=True, text=True, check=False)
  assert completed.stdout, completed.stderr
  return completed.returncode, json.loads(completed.stdout)


def check_published(source, output, expected):
  """Verify batch order, identity, time preservation, source binding and file layout."""
  manifest = read_json(output / "observation_manifest.json")
  assert manifest["schema_version"] == "observation_dataset/1"
  fingerprint = hashlib.sha256((source / "bag_manifest.json").read_bytes()).hexdigest()
  assert manifest["source_manifest_sha256"] == fingerprint
  assert Path(manifest["preparation_root"]) == source.resolve()
  records = rows(output / manifest["files"]["observations"])
  batches = rows(output / manifest["files"]["batches"])
  selection = read_json(output / manifest["files"]["selection"])
  assert len(records) == manifest["observations"] == expected
  assert len(batches) == manifest["batches"] == expected
  assert len(rows(output / manifest["files"]["issues"])) == manifest["issues"]
  assert [b["batch_index"] for b in batches] == list(range(expected))
  assert [b["observation_ids"] for b in batches] == [[o["id"]] for o in records]
  assert len({o["id"] for o in records}) == expected
  assert selection["batches"] == [[o["pair_id"]] for o in records]
  pairs = {p["pair_id"]: p for p in rows(source / "rgbd_pairs.jsonl")}
  for record in records:
    saved = pairs[record["pair_id"]]
    assert record["source_manifest_sha256"] == fingerprint
    assert record["stamp"] == record["rgb_stamp"] == saved["rgb_stamp"]
    assert record["depth_stamp"] == saved["depth_stamp"]
    assert record["depth_camera_pose_map"]["stamp"] == record["depth_stamp"]
    assert record["camera_pose_map"]["stamp"] == record["rgb_stamp"]
    assert record["depth_message_id"] == saved["depth_message_id"]
    assert record["source_pending_reasons"] == saved["reasons"]


def main():
  """Check happy path, bad depth, empty selection, atomic failure, and overwrite refusal."""
  parser = argparse.ArgumentParser()
  parser.add_argument("--binary", type=Path, required=True)
  parser.add_argument("--fixture-pointer", type=Path, required=True)
  parser.add_argument("--output-parent", type=Path, required=True)
  args = parser.parse_args()
  fixture = Path((args.fixture_pointer / "latest.txt").read_text().strip())
  source = fixture / "normal_png_out"
  expected = len(rows(source / "rgbd_pairs.jsonl"))
  assert expected >= 2
  args.output_parent.mkdir(parents=True, exist_ok=True)
  root = Path(tempfile.mkdtemp(prefix="dataset-", dir=args.output_parent))
  good = root / "good"
  code, result = call(args.binary, source, good, "prepared_rgbd")
  assert code == 0 and result["observations"] == result["verified_depth_payloads"] == expected, result
  check_published(source, good, expected)
  code, result = call(args.binary, source, good)
  assert code == 1 and result["code"] == "output_exists", result
  check_published(source, good, expected)
  code, result = call(args.binary, source, Path("relative_output"), cwd=root)
  assert code == 0 and Path(result["output_directory"]) == root / "relative_output", result
  check_published(source, root / "relative_output", expected)
  link = root / "dangling_output"
  link.symlink_to(root / "uncreated_target", target_is_directory=True)
  code, result = call(args.binary, source, link)
  assert code == 1 and result["code"] == "output_exists" and link.is_symlink(), result
  code, result = call(args.binary, source, source / "overlap")
  assert code == 1 and not (source / "overlap").exists(), result

  damaged = root / "damaged_source"
  shutil.copytree(source, damaged)
  first = rows(damaged / "rgbd_pairs.jsonl")[0]
  frame = next(f for f in rows(damaged / "frames.jsonl") if f["source"]["message_id"] == first["depth_message_id"])
  media = damaged / frame["media_path"]
  # Keep the complete header so only the full depth read detects the truncation.
  media.write_bytes(media.read_bytes()[:26])
  damaged_output = root / "damaged"
  code, result = call(args.binary, damaged, damaged_output)
  assert code == 0 and result["observations"] == result["verified_depth_payloads"] == expected - 1, result
  assert any(i["pair_id"] == first["pair_id"] and i["code"] == "invalid_media"
             for i in rows(damaged_output / "reports/issues.jsonl"))
  check_published(damaged, damaged_output, expected - 1)

  damaged_rgb = root / "damaged_rgb_source"
  shutil.copytree(source, damaged_rgb)
  frame = next(f for f in rows(damaged_rgb / "frames.jsonl") if f["source"]["message_id"] == first["rgb_message_id"])
  media = damaged_rgb / frame["media_path"]
  media.write_bytes(media.read_bytes()[:26])
  rgb_output = root / "damaged_rgb"
  code, result = call(args.binary, damaged_rgb, rgb_output)
  assert code == 0 and result["observations"] == expected - 1, result
  assert any(i["pair_id"] == first["pair_id"] and i["code"] == "invalid_media"
             for i in rows(rgb_output / "reports/issues.jsonl"))
  check_published(damaged_rgb, rgb_output, expected - 1)

  broken = root / "broken_source"
  shutil.copytree(source, broken)
  frames = rows(broken / "frames.jsonl")
  frame = next(f for f in frames if f["source"]["message_id"] == first["rgb_message_id"])
  frame["map_camera_tf"]["pose"]["translation_m"][0] += 9.0
  (broken / "frames.jsonl").write_text("".join(json.dumps(f) + "\n" for f in frames))
  failed = root / "failed"
  code, result = call(args.binary, broken, failed)
  assert code == 1 and result["code"] == "invalid_pose" and not failed.exists(), result
  assert (Path(result["work_directory"]) / "reports/issues.jsonl").is_file()

  empty = root / "empty_source"
  shutil.copytree(source, empty)
  (empty / "rgbd_pairs.jsonl").write_text("")
  empty_output = root / "empty"
  code, result = call(args.binary, empty, empty_output)
  assert code == 0 and result["candidates"] == result["observations"] == 0, result
  check_published(empty, empty_output, 0)
  code, result = call(args.binary, empty, root / "wrong-map", map_frame="wrong_map")
  assert code == 1 and result["code"] == "invalid_context", result


if __name__ == "__main__":
  main()
