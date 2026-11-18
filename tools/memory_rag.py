"""Build and query a local vector index for a published spatial memory version."""

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import unicodedata

import numpy as np
import yaml


MODEL_ID = "intfloat/multilingual-e5-small"
MODEL_REVISION = "ccc66d3bcd826f577e26b9a4072cc5fe3a7ad6a3"
MODEL_FILE = "model_qint8_avx512_vnni.onnx"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/memory_rag.yaml"
DIMENSION = 384
BATCH_SIZE = 16
RELATION_TYPES = ("inside", "near", "on", "hanging_on", "connects_regions")


class E5Encoder:
    """Run the pinned E5-small ONNX model locally and return unit vectors."""

    def __init__(self, model_dir):
        """Open the local model and tokenizer once for this command."""
        import onnxruntime as ort
        from tokenizers import Tokenizer

        model_path = model_dir / MODEL_FILE
        tokenizer_path = model_dir / "tokenizer.json"
        if not model_path.is_file() or not tokenizer_path.is_file():
            raise ValueError(f"Missing E5-small files in {model_dir}; see the RAG setup instructions")
        self.tokenizer = Tokenizer.from_file(str(tokenizer_path))
        self.tokenizer.enable_truncation(max_length=512)
        self.tokenizer.enable_padding(pad_id=self.tokenizer.token_to_id("<pad>"), pad_token="<pad>")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        self.session = ort.InferenceSession(str(model_path), options, providers=["CPUExecutionProvider"])

    def encode(self, texts, prefix):
        """Encode prefixed passages or queries in bounded batches, preserving input order."""
        vectors = np.empty((len(texts), DIMENSION), dtype=np.float32)
        for start in range(0, len(texts), BATCH_SIZE):
            end = start + BATCH_SIZE
            batch = self.tokenizer.encode_batch([prefix + text for text in texts[start:end]])
            attention = np.array([item.attention_mask for item in batch], dtype=np.int64)
            inputs = {
                "input_ids": np.array([item.ids for item in batch], dtype=np.int64),
                "attention_mask": attention,
                "token_type_ids": np.array([item.type_ids for item in batch], dtype=np.int64),
            }
            hidden, = self.session.run(["last_hidden_state"], inputs)
            mask = attention[..., None].astype(np.float32)
            pooled = (hidden * mask).sum(axis=1) / mask.sum(axis=1)
            norms = np.linalg.norm(pooled, axis=1, keepdims=True)
            if not np.isfinite(norms).all() or np.any(norms == 0):
                raise ValueError("E5-small produced an invalid vector")
            vectors[start:end] = pooled / norms
        return vectors


def read_rows(release, manifest, name):
    """Read a published JSONL table after checking its digest and declared count."""
    path = release / f"{name}.jsonl"
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != manifest["file_sha256"][path.name]:
        raise ValueError(f"Published file digest does not match the manifest: {path.name}")
    rows = [json.loads(line) for line in raw.splitlines()]
    if len(rows) != manifest["counts"][name]:
        raise ValueError(f"Published file count does not match the manifest: {path.name}")
    return rows


def load_release(release):
    """Load the sealed entity and annotation tables and bind them to one release."""
    raw = (release / "manifest.json").read_bytes()
    manifest = json.loads(raw)
    if manifest["artifact_kind"] != "spatial_memory" or manifest["schema_version"] != "spatial_memory/1":
        raise ValueError("Expected a spatial_memory/1 published directory")
    entities = {}
    for row in read_rows(release, manifest, "entities"):
        entity_id = row["entity_id"]
        if not entity_id or entity_id in entities:
            raise ValueError("Published entities must have unique non-empty IDs")
        entities[entity_id] = row

    annotations = {}
    for row in read_rows(release, manifest, "annotations"):
        entity_id = row["entity_id"]
        if entity_id not in entities or entity_id in annotations:
            raise ValueError("Published annotations must reference distinct active entities")
        annotations[entity_id] = row

    metadata = {
        "schema_version": "memory_rag/1",
        "memory_version": manifest["memory_version"],
        "release_manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "model_id": MODEL_ID,
        "model_revision": MODEL_REVISION,
        "model_file": MODEL_FILE,
        "dimension": DIMENSION,
        "entities": len(entities),
    }
    return manifest, metadata, entities, annotations


def entity_names(entity, annotation):
    """Return the current display name, aliases and original semantic label."""
    names = [annotation.get("display_name"), entity["semantic_label"], *annotation.get("aliases", [])]
    return list(dict.fromkeys(name.strip() for name in names if name and name.strip()))


def passage(entity, annotation):
    """Assemble one short E5 passage for an entity, including selected attributes."""
    parts = entity_names(entity, annotation)
    if entity.get("description"):
        parts.append(entity["description"])
    attributes = entity.get("attributes", {})
    for key in ("color", "material", "state", "type"):
        if attributes.get(key):
            parts.append(f"{key}: {attributes[key]}")
    return ". ".join(parts)


def build_index(index_dir, model_dir, metadata, entities, annotations):
    """Encode one vector per entity and publish a complete index in a new directory."""
    if index_dir.exists():
        raise ValueError(f"Index already exists; choose a new directory: {index_dir}")
    texts = [passage(row, annotations.get(entity_id, {})) for entity_id, row in entities.items()]
    vectors = np.empty((0, DIMENSION), dtype=np.float32)
    if texts:
        vectors = E5Encoder(model_dir).encode(texts, "passage: ")

    index_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".rag-", dir=index_dir.parent) as temporary:
        staging = Path(temporary) / "index"
        staging.mkdir()
        np.save(staging / "vectors.npy", vectors, allow_pickle=False)
        (staging / "entity_ids.json").write_text(json.dumps(list(entities)) + "\n", encoding="utf-8")
        manifest = json.dumps(metadata, ensure_ascii=False, indent=2) + "\n"
        (staging / "index.json").write_text(manifest, encoding="utf-8")
        staging.rename(index_dir)
    return dict(metadata, status="ok")


def load_index(index_dir, metadata, entities):
    """Reject stale or misaligned indexes before using their vectors."""
    manifest_path = index_dir / "index.json"
    if not manifest_path.is_file():
        raise ValueError(f"Index is missing or incomplete; run build-index: {index_dir}")
    saved = json.loads(manifest_path.read_text(encoding="utf-8"))
    ids = json.loads((index_dir / "entity_ids.json").read_text(encoding="utf-8"))
    if saved != metadata or ids != list(entities):
        raise ValueError("Index does not match this release or model; rebuild the index")
    vectors = np.load(index_dir / "vectors.npy", allow_pickle=False)
    if (not isinstance(vectors, np.ndarray) or vectors.shape != (len(ids), DIMENSION)
            or vectors.dtype != np.float32 or not np.isfinite(vectors).all()):
        raise ValueError("Invalid vector matrix; rebuild the index")
    return ids, vectors


def normalize_name(text):
    """Normalize case, Unicode width and repeated whitespace for name matching."""
    return " ".join(unicodedata.normalize("NFKC", text).casefold().split())


def rank_entities(ids, vectors, metadata, entities, annotations, text, top_k, encode_query):
    """Return a unique name match or ranked vector candidates for the query."""
    name = normalize_name(text)
    if not name or top_k < 1:
        raise ValueError("Query text must be non-empty and top-k must be positive")
    result = {"memory_version": metadata["memory_version"], "status": "empty", "matches": []}
    exact = []
    for entity_id, row in entities.items():
        names = entity_names(row, annotations.get(entity_id, {}))
        if any(normalize_name(candidate) == name for candidate in names):
            exact.append(entity_id)
    if len(exact) == 1:
        entity_id = exact[0]
        result["status"] = "exact"
        result["matches"] = [{"entity_id": entity_id, "semantic_label": entities[entity_id]["semantic_label"]}]
    elif ids:
        vector = encode_query(text)
        scores = vectors @ vector
        result["status"] = "candidates"
        for i in np.argsort(-scores, kind="stable")[:top_k]:
            entity_id = ids[i]
            result["matches"].append({"entity_id": entity_id, "semantic_label": entities[entity_id]["semantic_label"],
                                      "cosine_similarity": float(scores[i])})
    return result


def checked_observations(release, manifest):
    """Index sealed observations from the release's map and geometry revision."""
    observations = {}
    for row in read_rows(release, manifest, "observations"):
        obs_id = row["id"]
        if obs_id in observations:
            raise ValueError(f"Duplicate published observation: {obs_id}")
        if (row["map_id"], row["map_frame"], row["geometry_revision"]) != (
                manifest["map_id"], manifest["frame_id"], manifest["geometry_revision"]):
            raise ValueError(f"Observation geometry does not match the release: {obs_id}")
        observations[obs_id] = row
    return observations


def location_fields(release, entity, observations):
    """Resolve the entity's selected observation and return its original evidence references."""
    fields = dict.fromkeys(("observation_id", "observation_revision", "selector",
                            "rgb_media", "depth_media", "depth_alignment", "capture_sync_status"))
    location = entity["location_observation"]
    if location is None:
        return fields
    obs_id = location["observation_id"]
    revision = location["observation_revision"]
    observation = observations.get(obs_id)
    if observation is None or int(observation["revision"]) != revision:
        raise ValueError(f"Missing location observation at revision {revision}: {obs_id}")
    for ref in entity["observation_refs"]:
        if ref["observation_id"] == obs_id and ref["observation_revision"] == revision:
            fields.update(observation_id=obs_id, observation_revision=revision, selector=ref.get("selector"))
            break
    else:
        raise ValueError(f"Missing entity evidence for location observation: {obs_id}")
    for role in ("rgb_media", "depth_media"):
        media = observation[role]
        path = Path(media)
        resolved = (release / path).resolve()
        if path.is_absolute() or not resolved.is_relative_to(release / "evidence") or not resolved.is_file():
            raise ValueError(f"Invalid {role} in published observation: {obs_id}")
        fields[role] = media
    for key in ("depth_alignment", "capture_sync_status"):
        fields[key] = observation.get(key)
    return fields


def relations_by_entity(release, manifest, entities, observations, relation_type):
    """Group supported relations by endpoint while keeping their published direction and basis."""
    grouped = {}
    for row in read_rows(release, manifest, "relations"):
        if (row["type"] != relation_type or row["recognition_state"] != "supported"
                or row["geometry_revision"] != manifest["geometry_revision"]):
            continue
        endpoints = row["entity_ids"]
        for entity_id in endpoints:
            if entity_id not in entities:
                raise ValueError(f"Missing published relation endpoint: {entity_id}")
        evidence_ids = row["evidence_observation_ids"]
        for obs_id in evidence_ids:
            if obs_id not in observations:
                raise ValueError(f"Missing published relation observation: {obs_id}")
        if not evidence_ids and not row.get("evidence_ref") and "distance_m" not in row:
            continue
        for entity_id in endpoints:
            other_entities = [{"entity_id": other, "semantic_label": entities[other]["semantic_label"]}
                              for other in endpoints if other != entity_id]
            grouped.setdefault(entity_id, []).append(dict(row, related_entities=other_entities))
    return grouped


def add_evidence(result, release, manifest, entities, annotations, relation_type):
    """Enrich each search hit with coordinates and evidence from the same published version."""
    for key in ("map_id", "frame_id", "geometry_revision"):
        result[key] = manifest[key]
    if relation_type is not None:
        result["relation_type"] = relation_type
    if not result["matches"]:
        return
    observations = checked_observations(release, manifest)
    relations = {}
    if relation_type is not None:
        relations = relations_by_entity(release, manifest, entities, observations, relation_type)
    for match in result["matches"]:
        entity_id = match["entity_id"]
        entity = entities[entity_id]
        if entity["geometry_revision"] != manifest["geometry_revision"]:
            raise ValueError(f"Entity geometry does not match the release: {entity_id}")
        match["display_name"] = annotations.get(entity_id, {}).get("display_name")
        match["location_status"] = entity["status"]
        match["position_map"] = entity.get("position_map")
        match.update(location_fields(release, entity, observations))
        if relation_type is not None:
            linked = relations.get(entity_id, [])
            match["relations"] = linked
            match["relation_status"] = "insufficient_evidence"
            if linked:
                match["relation_status"] = "evidence_found"


def load_config(config_path):
    """Read YAML settings and resolve relative paths from the configuration directory."""
    path = config_path.resolve(strict=True)
    settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(settings, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    path_keys = ("release", "index_dir", "model_dir")
    unknown = settings.keys() - {*path_keys, "top_k", "service_name"}
    if unknown:
        raise ValueError(f"Unknown RAG settings: {', '.join(map(str, unknown))}")

    for key in path_keys:
        value = settings.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"RAG setting '{key}' must be a non-empty path")
        settings[key] = (path.parent / Path(value).expanduser()).resolve()
    top_k = settings.get("top_k")
    if type(top_k) is not int or top_k < 1:
        raise ValueError("RAG setting 'top_k' must be a positive integer")
    service_name = settings.setdefault("service_name", "/vln_memory/search_memory")
    if not isinstance(service_name, str) or not service_name.strip():
        raise ValueError("RAG setting 'service_name' must be a non-empty ROS service name")
    return settings


class MemorySearch:
    """Keep one validated release and its index loaded for repeated local queries."""

    def __init__(self, settings):
        """Read index and release data once; load the local encoder on the first vector query."""
        self.release = settings["release"]
        self.model_dir = settings["model_dir"]
        self.top_k = settings["top_k"]
        self.manifest, self.metadata, self.entities, self.annotations = load_release(self.release)
        self.ids, self.vectors = load_index(settings["index_dir"], self.metadata, self.entities)
        self.encoder = None

    def encode_query(self, text):
        """Reuse the same encoder for every vector query in this process."""
        if self.encoder is None:
            self.encoder = E5Encoder(self.model_dir)
        return self.encoder.encode([text], "query: ")[0]

    def query(self, text, relation=None):
        """Return ranked hits and published evidence without changing the release."""
        if relation is not None and relation not in RELATION_TYPES:
            raise ValueError(f"Unsupported memory relation: {relation}")
        result = rank_entities(self.ids, self.vectors, self.metadata, self.entities, self.annotations,
                               text, self.top_k, self.encode_query)
        add_evidence(result, self.release, self.manifest, self.entities, self.annotations, relation)
        return result


def main():
    """Parse CLI arguments and emit a single JSON result or a diagnostic on stderr."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="RAG YAML settings")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("build-index")
    search = commands.add_parser("query")
    search.add_argument("--text", required=True)
    search.add_argument("--relation", choices=RELATION_TYPES, help="Return supported published relations of this type")
    args = parser.parse_args()

    try:
        settings = load_config(args.config)
        release = settings["release"]
        index_dir = settings["index_dir"]
        if index_dir.is_relative_to(release):
            raise ValueError("Index must be outside the published directory")
        if args.command == "build-index":
            _, metadata, entities, annotations = load_release(release)
            result = build_index(index_dir, settings["model_dir"], metadata, entities, annotations)
        else:
            result = MemorySearch(settings).query(args.text, args.relation)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f"memory_rag: {error}\n")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
