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


def call(binary, source, output, purpose="rgb", map_frame="map", cwd=None, mode="reference"):
  """Run the public CLI and keep return code and structured outcome."""
  completed = subprocess.run([
    str(binary.resolve()), "--input", str(source.resolve()), "--output", str(output), "--purpose", purpose,
    "--batch-size", "1", "--map-id", "fixture", "--geometry-revision", "v1",
    "--map-frame", map_frame, "--map-basis", "Fixture recorded TF", "--preparation-mode", mode],
    cwd=cwd, capture_output=True, text=True, check=False)
  assert completed.stdout, completed.stderr
  return completed.returncode, json.loads(completed.stdout)


def check_published(source, output, expected):
  """Verify batch order, identity, time preservation, source binding and file layout."""
  manifest = read_json(output / "observation_manifest.json")
  assert manifest["schema_version"] == "observation_dataset/2"
  assert manifest["file_sha256"] == {
    name: hashlib.sha256((output / relative).read_bytes()).hexdigest()
    for name, relative in manifest["files"].items()}
  fingerprint = hashlib.sha256((source / "bag_manifest.json").read_bytes()).hexdigest()
  assert manifest["source_manifest_sha256"] == fingerprint
  assert (output / manifest["preparation_root"]).resolve() == source.resolve()
  records = rows(output / manifest["files"]["observations"])
  batches = rows(output / manifest["files"]["batches"])
  selection = read_json(output / manifest["files"]["selection"])
  assert len(records) == manifest["observations"] == expected
  assert len(batches) == manifest["batches"] == expected
  assert len(rows(output / manifest["files"]["issues"])) == manifest["issues"]
  assert [b["batch_index"] for b in batches] == list(range(expected))
  assert [b["observation_ids"] for b in batches] == [[o["id"]] for o in records]
  assert len({o["id"] for o in records}) == expected
  assert selection["preparation_root"] == manifest["preparation_root"]
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


def check_reader(binary, root, source, good, expected, empty_output):
  """Cover indexed retrieval, old schema, relocation, damaged data, and changes after open."""
  def check(mode, output, *args):
    """Run the normal C++ consumer and preserve failure output."""
    subprocess.run([str(binary.resolve()), mode, str(output), *map(str, args)], check=True)

  def legacy_copy(name):
    """Construct the old manifest over unchanged v1 observation rows."""
    output = root / name
    shutil.copytree(good, output)
    manifest_file = output / "observation_manifest.json"
    data = read_json(manifest_file)
    del data["file_sha256"]
    data["schema_version"] = "observation_dataset/1"
    manifest_file.write_text(json.dumps(data))
    return output

  check("--verify", good, expected, "v2")
  check("--verify", empty_output, 0, "v2")
  check("--verify", legacy_copy("legacy"), expected, "v1")
  check("--wrong-source", good, source.parent / "normal_out")
  for mode in ("missing_pair", "changed_pair_stamp"):
    modified_source = root / mode
    shutil.copytree(source, modified_source)
    pair_file = modified_source / "rgbd_pairs.jsonl"
    pairs = rows(pair_file)
    first_pair_id = rows(good / "observations.jsonl")[0]["pair_id"]
    first_pair = next(pair for pair in pairs if pair["pair_id"] == first_pair_id)
    if mode == "missing_pair":
      pairs.remove(first_pair)
      message = "Missing referenced preparation pair"
    else:
      stamp = first_pair["depth_stamp"]
      stamp["nanosec"] = (stamp["nanosec"] + 1) % 1000000000
      message = "Preparation pair identity or timing differs"
    pair_file.write_text("".join(json.dumps(pair) + "\n" for pair in pairs))
    assert (modified_source / "bag_manifest.json").read_bytes() == (source / "bag_manifest.json").read_bytes()
    check("--wrong-source", good, modified_source, message)

  relocated = root / "relocated"
  shutil.copytree(good, relocated)
  migrated_source = root / "migrated_preparation"
  shutil.copytree(source, migrated_source)
  manifest_file = relocated / "observation_manifest.json"
  data = read_json(manifest_file)
  data["preparation_root"] = str(root / "old_missing_preparation")
  manifest_file.write_text(json.dumps(data))
  check("--verify", relocated, expected, "v2", migrated_source)

  lost = legacy_copy("lost_file")
  (lost / "reports/issues.jsonl").unlink()
  check("--reject", lost, "manifest")
  escaped = legacy_copy("escaped_file")
  (escaped / "reports/issues.jsonl").unlink()
  (escaped / "reports/issues.jsonl").symlink_to(good / "reports/issues.jsonl")
  check("--reject", escaped, "manifest")
  truncated = legacy_copy("truncated_observations")
  with (truncated / "observations.jsonl").open() as stream:
    first = next(stream)
  (truncated / "observations.jsonl").write_text(first)
  check("--reject", truncated, "record")

  misordered = legacy_copy("misordered_batch")
  batch_file = misordered / "batches.jsonl"
  batches = rows(batch_file)
  batches[0]["observation_ids"], batches[1]["observation_ids"] = (
    batches[1]["observation_ids"], batches[0]["observation_ids"])
  batch_file.write_text("".join(json.dumps(batch) + "\n" for batch in batches))
  check("--reject", misordered, "batch")

  malformed = legacy_copy("invalid_record")
  records_file = malformed / "observations.jsonl"
  records = rows(records_file)
  records[0]["length_unit"] = "cm"
  records_file.write_text("".join(json.dumps(row) + "\n" for row in records))
  check("--reject-on-read", malformed)

  damaged = root / "changed_digest"
  shutil.copytree(good, damaged)
  selection = damaged / "build_input_manifest.json"
  selection.write_bytes(selection.read_bytes() + b" ")
  check("--reject", damaged, "digest")

  changed = root / "changed_after_open"
  shutil.copytree(good, changed)
  check("--changed", changed)
  changed_manifest = root / "changed_manifest_after_open"
  shutil.copytree(good, changed_manifest)
  check("--changed-manifest", changed_manifest)


def check_bundle(binary, reader, store, root, source, expected):
  """Move a portable release after removing its old source path and exercise normal consumers."""
  private_source = root / "bundle_input"
  shutil.copytree(source, private_source)
  output = root / "bundle_output"
  code, result = call(binary, private_source, output, "prepared_rgbd", mode="bundle")
  assert code == 0 and result["observations"] == expected, result
  manifest = read_json(output / "observation_manifest.json")
  assert manifest["preparation_mode"] == "bundle" and manifest["preparation_root"] == "preparation"
  preparation = output / "preparation"
  p_manifest = read_json(preparation / "bag_manifest.json")
  assert p_manifest["effective_options"]["save_raw_cdr"] is False
  assert not any(preparation.rglob("*.cdr"))
  private_source.rename(root / "former_bundle_input")
  relocated = root / "bundle_relocated"
  output.rename(relocated)
  check_published(relocated / "preparation", relocated, expected)
  subprocess.run([str(reader.resolve()), "--verify", str(relocated), str(expected), "v2"], check=True)
  subprocess.run([str(reader.resolve()), "--project", str(relocated), "0", "0"], check=True)
  subprocess.run([str(store.resolve()), "--published", str(relocated), "0", "0", str(root),
                  "--fixture-confirmation"], check=True)


def main():
  """Check happy path, bad depth, empty selection, atomic failure, and overwrite refusal."""
  parser = argparse.ArgumentParser()
  parser.add_argument("--binary", type=Path, required=True)
  parser.add_argument("--reader", type=Path, required=True)
  parser.add_argument("--store", type=Path, required=True)
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
  check_reader(args.reader, root, source, good, expected, empty_output)
  check_bundle(args.binary, args.reader, args.store, root, source, expected)
  failed_bundle = root / "failed_bundle"
  code, result = call(args.binary, damaged, failed_bundle, mode="bundle")
  assert code == 1 and result["code"] == "invalid_media" and not failed_bundle.exists(), result
  code, result = call(args.binary, empty, root / "wrong-map", map_frame="wrong_map")
  assert code == 1 and result["code"] == "invalid_context", result


if __name__ == "__main__":
  main()
