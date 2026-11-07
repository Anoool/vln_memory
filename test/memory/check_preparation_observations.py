"""Test-only preparation adapter: raw metadata -> observation/1 -> C++ reader comparison.

This does not implement the production observation_builder. Map context below is an
explicit acceptance declaration, not proof of physical calibration/registration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile


def rows(path):
  """Read metadata only; preserve each JSONL source order."""
  return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def fingerprint(path):
  """Hash source metadata to identify it and detect test-time modification."""
  return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(record):
  """Match C++ length-prefix identity using UTF-8 byte lengths."""
  value = "obs1:"
  for name in ("bag_id", "pair_id", "rgb_message_id", "depth_message_id", "clock_id", "segment_id"):
    part = record[name]
    value += str(len(part.encode("utf-8"))) + ":" + part
  return value


def model(frame):
  """Copy effective parameters only when the source model is ready."""
  value = frame.get("camera_model")
  if not value or value["code"] != "ready":
    return None
  return {key: value[key] for key in (
    "width", "height", "frame_id", "image_domain", "distortion_model",
    "d", "k", "r", "p", "normalization_applied")}


def image(frame):
  """Keep image metadata and source topic; stamp is encoded at the root."""
  return {"topic": frame["source"]["topic"], "frame_id": frame["frame_id"],
      "width": frame["width"], "height": frame["height"]}


def calibration(record, frame, selection, sources):
  """Preserve original D8/ROI/binning/stamp and the actual matching policy."""
  if record is None:
    return None
  sources.append(record["source"])
  if frame.get("calibration", {}).get("code") != "matched":
    return None
  original = {key: record[key] for key in (
    "stamp", "frame_id", "width", "height", "distortion_model",
    "d", "k", "r", "p", "binning_x", "binning_y", "roi")}
  original["topic"] = record["source"]["topic"]
  chosen = dict(selection)
  chosen["clock_relation"] = "same_clock"
  return {"calibration_id": record["calibration_id"],
      "source_message_id": record["source"]["message_id"],
      "original": original, "selection": chosen}


def pose(frame, name, transforms, sources):
  """Resolve every saved TF endpoint to its original transform metadata."""
  value = frame.get(name)
  if not value or value["pose_code"] != "ready":
    return None
  edges = []
  for source_edge in value["edges"]:
    edge = {"inverted": source_edge["inverted"]}
    for endpoint in ("before", "after"):
      ref = source_edge[endpoint]
      if ref is None:
        edge[endpoint] = None
        continue
      raw = transforms[(ref["message_id"], int(ref["transform_index"]))]
      sources.append(raw["source"])
      edge[endpoint] = {"kind": "static" if raw["static"] else "dynamic",
               "stamp": raw["stamp"], "parent_frame": raw["parent_frame"],
               "child_frame": raw["child_frame"], "translation_m": raw["translation_m"],
               "rotation_xyzw": raw["rotation_xyzw"], "source_topic": raw["source"]["topic"],
               "source_message_id": raw["source"]["message_id"],
               "source_transform_index": str(raw["transform_index"])}
    edges.append(edge)
  return {"stamp": value["stamp"], "frames": value["frames"], "edges": edges,
      "translation_m": value["pose"]["translation_m"], "rotation_xyzw": value["pose"]["rotation_xyzw"]}


def storage(frame, options):
  """Distinguish metric NPY, source PNG16 and the historical v5 fallback."""
  if frame["media_path"] is None:
    return None
  details = frame["details"]
  saved = details.get("storage", {
    "format": "npy", "dtype": "float32", "unit": "m", "scale_to_m": 1.0,
    "invalid_rule": "nan", "min_depth_m": options["min_depth_m"],
    "max_depth_m": options["max_depth_m"]})
  result = dict(saved)
  for name in ("source_encoding", "source_unit", "source_scale_to_m"):
    result[name] = details[name]
  result["valid_pixels"] = str(details["valid_pixels"])
  result["invalid_pixels"] = str(details["invalid_pixels"])
  return result


def evidence(source):
  """Keep original message locators, exact integers and optional raw CDR."""
  result = {key: source[key] for key in (
    "message_id", "topic", "payload_sha256", "raw_path")}
  for name in ("file_index", "topic_ordinal", "message_order", "bag_record_time_ns", "serialized_bytes"):
    result[name] = str(source[name])
  return result


def pending(record):
  """Independently enumerate required missing-field and quality reasons."""
  result = []
  for reason in record["source_pending_reasons"] + ["physical_geometry_unverified"]:
    if reason not in result:
      result.append(reason)
  for field, reason in (
    ("rgb_media", "missing_rgb_media"), ("depth_media", "missing_depth_media"),
    ("rgb_calibration", "missing_rgb_calibration"), ("depth_calibration", "missing_depth_calibration"),
    ("rgb_model", "missing_rgb_model"), ("depth_model", "missing_depth_model"),
    ("camera_pose_map", "missing_rgb_map_pose"), ("depth_camera_pose_map", "missing_depth_map_pose"),
    ("body_pose_map", "missing_body_map_pose")):
    if record[field] is None and reason not in result:
      result.append(reason)
  extra = []
  if record["depth_storage"] is not None and int(record["depth_storage"]["valid_pixels"]) == 0:
    extra.append("no_valid_depth")
  if record["depth_alignment"] == "unknown":
    extra.append("depth_rgb_alignment_unverified")
  if record["depth_alignment"] == "not_aligned":
    extra.append("depth_rgb_not_aligned")
  if record["capture_sync_status"] == "unknown":
    extra.append("capture_sync_unverified")
  if record["capture_sync_status"] == "not_synchronized":
    extra.append("capture_not_synchronized")
  for reason in extra:
    if reason not in result:
      result.append(reason)
  return result


def adapt(root, manifest_sha):
  """Construct test records without modifying source metadata or media."""
  manifest = json.loads((root / "bag_manifest.json").read_text())
  options = manifest["effective_options"]
  frames = {row["source"]["message_id"]: row for row in rows(root / "frames.jsonl")}
  calibrations = {row["calibration_id"]: row for row in rows(root / "camera_info.jsonl")}
  transforms = {(row["source"]["message_id"], row["transform_index"]): row for row in rows(root / "tf.jsonl")}
  messages = {row["message_id"]: row for row in rows(root / "messages.jsonl")}
  records = []
  for pair in rows(root / "rgbd_pairs.jsonl"):
    rgb = frames[pair["rgb_message_id"]]
    depth = frames[pair["depth_message_id"]]
    sources = [rgb["source"], depth["source"]]
    record = {
      "schema_version": "observation/1", "length_unit": "m", "rotation_convention": "xyzw",
      "depth_convention": "optical_z", "rgb_format": "png_rgb8", "revision": "1",
      "bag_id": manifest["bag_id"], "pair_id": pair["pair_id"],
      "rgb_message_id": pair["rgb_message_id"], "depth_message_id": pair["depth_message_id"],
      "source_manifest_sha256": manifest_sha, "clock_id": options["clock_id"],
      "segment_id": options["segment_id"], "clock_basis": options["clock_basis"],
      "segment_basis": options["segment_basis"], "map_id": "acceptance_" + manifest["bag_id"],
      "geometry_revision": "source_tf_acceptance_v1", "map_frame": options["map_frame"],
      "body_frame": options["body_frame"],
      "map_basis": "Acceptance-only source frame reuse; physical registration not asserted",
      "stamp": pair["rgb_stamp"], "rgb_stamp": pair["rgb_stamp"], "depth_stamp": pair["depth_stamp"],
      "rgb_image": image(rgb), "depth_image": image(depth), "delta_ns": pair["delta_ns"],
      "sync_tolerance_ns": pair["sync_tolerance_ns"],
      "time_match": "exact" if int(pair["delta_ns"]) == 0 else "approximate",
      "pairing_policy": pair["policy"], "source_pair_status": pair["status"],
      "source_pending_reasons": pair["reasons"], "capture_sync_status": pair["capture_sync_status"],
      "capture_sync_basis": pair.get("capture_sync_basis", ""), "alignment_basis": depth.get("details", {}).get("alignment_basis", ""),
      "rgb_media": rgb["media_path"], "depth_media": depth["media_path"],
      "depth_storage": storage(depth, options["depth"]),
      "rgb_calibration": calibration(calibrations.get(pair["rgb_calibration_id"]), rgb, options["rgb_camera"], sources),
      "depth_calibration": calibration(calibrations.get(pair["depth_calibration_id"]), depth, options["depth_camera"], sources),
      "rgb_model": model(rgb), "depth_model": model(depth),
      "tf_max_gap_ns": options["tf_max_gap_ns"],
      "tf_jump_limits": {"translation_m": options["tf_max_translation_step_m"],
               "rotation_rad": options["tf_max_rotation_step_rad"]},
      "camera_pose_map": pose(rgb, "map_camera_tf", transforms, sources),
      "depth_camera_pose_map": pose(depth, "map_camera_tf", transforms, sources),
      "body_pose_map": pose(rgb, "map_body_tf", transforms, sources)}
    alignment = depth.get("details", {}).get("alignment_code", "unknown")
    record["depth_alignment"] = "unknown"
    if alignment == "aligned":
      record["depth_alignment"] = "aligned_to_rgb"
    elif alignment == "not_aligned":
      record["depth_alignment"] = "not_aligned"
    unique = {}
    for source in sources:
      actual = messages[source["message_id"]]
      for name, value in source.items():
        assert actual[name] == value, "Source summary mismatch"
      unique[source["message_id"]] = evidence(source)
    record["evidence"] = sorted(unique.values(), key=lambda item: (int(item["message_order"]), item["message_id"]))
    record["pending_reasons"] = pending(record)
    record["id"] = identity(record)
    records.append(record)
  return records


def main():
  """Run generated records through C++ and compare every decoded JSON value."""
  parser = argparse.ArgumentParser()
  parser.add_argument("--binary", required=True, type=Path)
  parser.add_argument("--input", action="append", default=[], type=Path)
  parser.add_argument("--fixture-pointer", type=Path)
  parser.add_argument("--output-parent", required=True, type=Path)
  args = parser.parse_args()
  if args.fixture_pointer:
    latest = (args.fixture_pointer / "latest.txt").read_text().strip()
    args.input.append(Path(latest) / "normal_no_cdr_out")
  if not args.input:
    parser.error("At least one source dataset is required")
  destination = args.output_parent.resolve()
  for source in args.input:
    source = source.resolve()
    if destination == source or source in destination.parents:
      raise RuntimeError("Output parent must be outside each input dataset")
  args.output_parent.mkdir(parents=True, exist_ok=True)
  output = Path(tempfile.mkdtemp(prefix="run-", dir=args.output_parent))
  report = {"output": str(output), "datasets": []}
  for index, source in enumerate(args.input):
    source = source.resolve()
    if source == output or source in output.parents or output in source.parents:
      raise RuntimeError("Input/output overlap")
    names = ("bag_manifest.json", "rgbd_pairs.jsonl", "frames.jsonl", "camera_info.jsonl", "tf.jsonl", "messages.jsonl")
    before = {name: fingerprint(source / name) for name in names}
    records = adapt(source, before["bag_manifest.json"])
    encoded = output / (str(index) + "-observations.jsonl")
    roundtrip = output / (str(index) + "-roundtrip.jsonl")
    with encoded.open("w", encoding="utf-8") as stream:
      for record in records:
        stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
    completed = subprocess.run([str(args.binary.resolve()), str(source), str(encoded), str(roundtrip),
                  before["bag_manifest.json"]], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output / (str(index) + ".log")).write_text(completed.stdout)
    if completed.returncode:
      raise RuntimeError(completed.stdout)
    assert rows(roundtrip) == records, "Formal JSON round-trip changed source content"
    assert before == {name: fingerprint(source / name) for name in names}, "Source changed"
    report["datasets"].append({"source": str(source), "pairs": len(records),
                 "pending": sum(bool(row["pending_reasons"]) for row in records),
                 "source_manifest_sha256": before["bag_manifest.json"]})
  (output / "report.json").write_text(json.dumps(report, indent=2))
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
