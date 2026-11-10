"""Exercise installed-style observation dataset CLI against BagBuilder fixtures."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
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


def check_semantic(binary, dataset, database):
  """Apply structured results to a portable dataset and verify saved relation semantics."""
  first = rows(dataset / "observations.jsonl")[0]
  raw = {key: first[key] for key in ("rgb_stamp", "depth_stamp", "source_manifest_sha256")}

  def run(requests, code=0):
    completed = subprocess.run([str(binary.resolve()), "--dataset", str(dataset), "--store", str(database)],
                               input="".join(json.dumps(row) + "\n" for row in requests),
                               capture_output=True, text=True, timeout=20)
    assert completed.returncode == code, (completed.stdout, completed.stderr)
    return [json.loads(line) for line in completed.stdout.splitlines()]

  lookup, obj, region, other = run([
    {"op": "lookup", **raw},
    {"op": "create", "entity_key": "box-1", "kind": "object", "label": "box", **raw},
    {"op": "create", "entity_key": "room-1", "kind": "region", "label": "room",
     "observation_id": first["id"], "confirmation_ref": "fixture/inside-room"},
    {"op": "create", "entity_key": "room-2", "kind": "region", "label": "room",
     "observation_id": first["id"], "confirmation_ref": "fixture/inside-room"},
  ])
  assert lookup["observation_id"] == first["id"]
  assert obj["status"] == "pending_location"
  assert region["status"] == other["status"] == "located"
  inside = {"op": "inside", "subject_id": obj["entity_id"], "target_id": region["entity_id"],
            "observation_id": first["id"], "evidence_ref": "fixture/object-inside"}
  near = {"op": "near", "subject_id": region["entity_id"], "target_id": other["entity_id"], "threshold_m": 0.1}
  added, retried, measured, reverse, loaded = run([inside, inside, near,
    {**near, "subject_id": near["target_id"], "target_id": near["subject_id"]},
    {"op": "get", "entity_id": obj["entity_id"]}])
  assert added["ok"] and retried["code"] == "already_linked"
  assert measured == reverse and measured["related"] and measured["distance_m"] == 0.0
  assert loaded["region_id"] == region["entity_id"]
  wrong_time = {**raw, "depth_stamp": {**raw["depth_stamp"],
      "nanosec": (raw["depth_stamp"]["nanosec"] + 1) % 1000000000}}
  unmatched, conflict, duplicate, pending = run([
    {"op": "lookup", **wrong_time}, {**inside, "target_id": other["entity_id"]},
    {"op": "create", "entity_key": "box-1", "kind": "object", "label": "box", **raw},
    {**near, "subject_id": obj["entity_id"]},
  ], 1)
  assert unmatched["code"] == "observation_not_found"
  assert conflict["code"] == "evidence_conflict"
  assert duplicate["code"] == "already_exists" and duplicate["entity_id"] == obj["entity_id"]
  assert pending["code"] == "location_unavailable"
  with sqlite3.connect(database) as store:
    assert dict(store.execute("SELECT predicate,count(*) FROM entity_relations GROUP BY predicate")) == {
        "inside": 1, "near": 1}


def check_bundle(binary, reader, store, semantic, root, source, expected):
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
  check_semantic(semantic, relocated, root / "semantic.sqlite3")


def check_direct(reader, semantic, root, fixture, legacy_output):
  """Move both direct depth formats, read their media and reject changed bytes."""
  legacy = rows(legacy_output / "observations.jsonl")
  for storage in ("metric_npy", "source_png16"):
    direct = root / ("moved_" + storage)
    shutil.copytree(fixture / ("direct_" + storage), direct)
    assert {path.name for path in direct.iterdir()} == {"observations.jsonl", "evidence"}
    records = rows(direct / "observations.jsonl")
    keys = ("id", "pair_id", "rgb_stamp", "depth_stamp", "camera_pose_map", "depth_camera_pose_map",
            "body_pose_map", "rgb_model", "depth_model", "pending_reasons")
    assert [{key: row[key] for key in keys} for row in records] == [
        {key: row[key] for key in keys} for row in legacy]
    subprocess.run([str(reader.resolve()), "--direct", str(direct), "0", "0"], check=True)
    if storage == "source_png16":
      check_semantic(semantic, direct, root / "direct.sqlite3")
    for field in ("rgb_media", "depth_media"):
      media = direct / records[0][field]
      original = media.read_bytes()
      media.write_bytes(original[:-1] + bytes([original[-1] ^ 1]))
    subprocess.run([str(reader.resolve()), "--direct-corrupt", str(direct)], check=True)


def check_project_bag_builder(binary, root, fixture):
  """Run the single-YAML entry point with one good pair and one corrupt depth image."""
  output = root / "project_memory"
  config = root / "project.yaml"
  # A JSON document is valid YAML and needs no test-only parser dependency.
  config.write_text(json.dumps({
      "dataset": {"bag_path": str(fixture / "bad_depth_compressed_bag"), "memory_dir": str(output),
                  "map_id": "fixture", "geometry_revision": "v1", "map_basis": "Fixture recorded TF"},
      "topics": {"rgb": "/rgb", "depth": "/depth", "rgb_camera_info": "/rgb_info",
                 "depth_camera_info": "/depth_info", "tf": "/tf", "tf_static": "/tf_static"},
      "frames": {"map": "map", "body": "base", "rgb_optical": "camera", "depth_optical": "camera"},
      "preprocess": {"sync_tolerance_ms": 0, "depth_scale_to_m": 0.001},
  }), encoding="utf-8")
  result = subprocess.run([str(binary.resolve()), "--config", str(config)],
                          check=True, text=True, capture_output=True)
  report = json.loads(result.stdout)
  records = rows(output / "observations.jsonl")
  assert report["pairs"] == 2 and report["observations"] == len(records) == 1, report
  summary = read_json(output / "evidence/summary.json")
  assert summary["observations"] == len(records) and not summary["keyframes_selected"], summary
  assert summary["pending_observations"] == report["pending_observations"], summary
  record, = records
  assert record["rgb_stamp"]["sec"] == record["depth_stamp"]["sec"] == 11
  for field in ("rgb_media", "depth_media"):
    assert (output / record[field]).is_file(), record[field]
  issues = rows(output / "evidence/issues.jsonl")
  assert any(item["code"] == "missing_rgbd_media" for item in issues)
  pending = rows(output / "evidence/pending.jsonl")
  assert {"invalid_frame", "unmatched_rgb", "unused_depth"} <= {item["kind"] for item in pending}


def check_memory_engine(binary, root, fixture):
  """Exercise the YAML tool entry point and verify the optional point artifact."""
  dataset = root / "engine_dataset"
  shutil.copytree(fixture / "direct_source_png16", dataset)
  config = root / "engine.yaml"
  config.write_text(json.dumps({"dataset": {"memory_dir": str(dataset)}}), encoding="utf-8")
  observation = rows(dataset / "observations.jsonl")[0]
  observation_id = observation["id"]
  point = {"type": "point", "pixel_xy": [0, 0]}
  requests = [
      {"name": "get_observation", "arguments": json.dumps({"observation_id": observation_id})},
      {"name": "project_depth", "arguments": {"observation_id": observation_id, "selector": point,
                                               "include_points": True}},
      {"name": "project_depth", "arguments": {"observation_id": observation_id,
                                              "selector": {"type": "point", "pixel_xy": [1, 0],
                                                           "image": "depth"}}},
  ]
  inputs = "".join(json.dumps(row) + "\n" for row in requests)
  completed = subprocess.run([str(binary.resolve()), "--config", str(config)],
                             input=inputs, text=True, capture_output=True, check=False)
  assert completed.returncode == 1, (completed.stdout, completed.stderr)
  info, projected, invalid = [json.loads(line) for line in completed.stdout.splitlines()]
  assert info["status"] == "ok" and info["rgb_media"] == observation["rgb_media"]
  assert info["depth_media"] == observation["depth_media"]
  assert info["rgb_stamp"] == observation["rgb_stamp"] and info["depth_stamp"] == observation["depth_stamp"]
  assert projected["status"] == "ok" and projected["depth_pixel_xy"] == [0, 0]
  assert projected["optical_z_m"] == 1 and projected["sample_count"] == 1
  assert rows(dataset / projected["points_path"])[0]["position_map"] == projected["position_map"]
  assert invalid["status"] == "invalid_depth" and "position_map" not in invalid


def check_entity_tools(binary, root, fixture):
  """Build and reopen one memory, including stale updates and atomic rollback."""
  dataset = root / "entity_tool_dataset"
  shutil.copytree(fixture / "direct_source_png16", dataset)
  config = root / "entity_tools.yaml"
  config.write_text(json.dumps({"dataset": {"memory_dir": str(dataset)}}), encoding="utf-8")
  first, second = [row["id"] for row in rows(dataset / "observations.jsonl")]

  def run(name, args, status="ok"):
    """Call the installed-style JSONL interface in a fresh process."""
    request = json.dumps({"name": name, "arguments": args}) + "\n"
    completed = subprocess.run([str(binary.resolve()), "--config", str(config)], input=request,
                               text=True, capture_output=True, timeout=20, check=False)
    result = json.loads(completed.stdout)
    assert result["status"] == status, result
    assert completed.returncode == (0 if status == "ok" else 1), completed.stderr
    return result

  def region(label):
    """Create an area from an explicit interior confirmation."""
    return run("create_entity", {"kind": "region", "semantic_label": label,
               "add_observations": [{"observation_id": first, "confirmation_ref": "fixture/interior"}]})["entity"]

  def view(observation, column):
    """Select a valid pixel on the depth grid."""
    return {"observation_id": observation, "selector": {"type": "point", "image": "depth", "pixel_xy": [column, 0]}}

  def relation(kind, ids):
    """Give a relation actual observation evidence."""
    return {"type": kind, "entity_ids": ids, "evidence_observation_ids": [first]}

  def spatial(kind, ids):
    """Check one relation in the engine's view of the database."""
    return run("spatial_query", {"operation": kind, "entity_ids": ids})

  room, hall = region("room"), region("hall")
  inside = relation("inside", ["$self", room["id"]])
  printer = run("create_entity", {"kind": "object", "semantic_label": "printer",
                "attributes": {"material": "plastic"}, "add_observations": [view(first, 0)],
                "relations": [inside]})["entity"]
  assert printer["region_id"] == room["id"] and printer["geometry_quality"] == "visible_surface_estimate"

  cabinet = run("create_entity", {"kind": "object", "semantic_label": "cabinet",
                 "add_observations": [view(second, 1)]})["entity"]
  door = run("create_entity", {"kind": "object", "semantic_label": "door",
              "add_observations": [view(second, 0)],
              "relations": [relation("connects_regions", ["$self", room["id"], hall["id"]])]})["entity"]
  assert cabinet["id"] != door["id"] and cabinet["position_map"] != door["position_map"]
  assert spatial("connects_regions", [door["id"], hall["id"], room["id"]])["reused"]

  edit = {"entity_id": printer["id"], "expected_revision": printer["revision"],
          "set_fields": {"description": "office device", "attributes": {"color": "white"}},
          "add_observations": [view(second, 0)],
          "relations": [relation("on", ["$self", cabinet["id"]]),
                        relation("hanging_on", ["$self", door["id"]]),
                        relation("inside", ["$self", cabinet["id"]])]}
  updated = run("update_entity", edit)["entity"]
  assert updated["id"] == printer["id"] and updated["position_map"] == printer["position_map"]
  assert updated["attributes"] == {"material": "plastic", "color": "white"}
  assert [ref["observation_id"] for ref in updated["observation_refs"]] == [first, second]
  assert updated["observation_refs"][1]["selector"] == view(second, 0)["selector"]
  assert updated["region_id"] == room["id"]
  run("update_entity", edit, "revision_conflict")
  repeat = run("update_entity", {**edit, "expected_revision": updated["revision"]})["entity"]
  assert repeat["revision"] == updated["revision"] and repeat["observation_refs"] == updated["observation_refs"]

  run("update_entity", {"entity_id": printer["id"], "expected_revision": updated["revision"],
      "set_fields": {"description": "must roll back"},
      "relations": [relation("on", ["$self", "missing"])]}, "entity_not_found")
  recovered = run("search_entities", {"query": "office device", "region_id": room["id"]})["entities"]
  assert len(recovered) == 1 and recovered[0]["revision"] == updated["revision"]
  for kind, target in (("inside", room), ("inside", cabinet), ("on", cabinet), ("hanging_on", door)):
    result = spatial(kind, [printer["id"], target["id"]])
    assert result["related"] and result["reused"] and result["relation"]["evidence_observation_ids"] == [first]
  assert spatial("on", [cabinet["id"], printer["id"]])["related"] is None
  near = spatial("near", [room["id"], hall["id"]])
  assert near["distance_m"] == 0 and near["parameters"]["dimension"] == "xy" and not near["reused"]
  reused = spatial("near", [hall["id"], room["id"]])
  assert reused["reused"] and reused["distance_m"] == near["distance_m"]
  assert reused["parameters"] == near["parameters"]
  pending = run("create_entity", {"kind": "object", "semantic_label": "unlocated",
                "add_observations": [view(first, 1)]}, "pending_location")
  assert not run("search_entities", {"query": "unlocated"})["entities"]
  with sqlite3.connect(dataset / "entities.sqlite3") as store:
    payload, = store.execute("SELECT request_json FROM pending_entities WHERE pending_id=?",
                             (pending["pending_id"],)).fetchone()
  assert json.loads(payload)["location_pending_reason"] == "invalid_depth"


def main():
  """Check happy path, bad depth, empty selection, atomic failure, and overwrite refusal."""
  parser = argparse.ArgumentParser()
  parser.add_argument("--binary", type=Path, required=True)
  parser.add_argument("--reader", type=Path, required=True)
  parser.add_argument("--bag-builder", type=Path, required=True)
  parser.add_argument("--engine", type=Path, required=True)
  parser.add_argument("--store", type=Path, required=True)
  parser.add_argument("--semantic", type=Path, required=True)
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
  check_bundle(args.binary, args.reader, args.store, args.semantic, root, source, expected)
  check_direct(args.reader, args.semantic, root, fixture, good)
  check_project_bag_builder(args.bag_builder, root, fixture)
  check_memory_engine(args.engine, root, fixture)
  check_entity_tools(args.engine, root, fixture)
  failed_bundle = root / "failed_bundle"
  code, result = call(args.binary, damaged, failed_bundle, mode="bundle")
  assert code == 1 and result["code"] == "invalid_media" and not failed_bundle.exists(), result
  code, result = call(args.binary, empty, root / "wrong-map", map_frame="wrong_map")
  assert code == 1 and result["code"] == "invalid_context", result


if __name__ == "__main__":
  main()
