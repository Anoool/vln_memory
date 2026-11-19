"""为已发布的空间记忆版本构建和查询本地向量索引。"""

import argparse
import hashlib
import json
import re
from pathlib import Path
import tempfile
import unicodedata

import numpy as np
import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config/memory_rag.yaml"
ENCODER_FIELDS = (
    "model_id", "model_revision", "dimension", "max_length", "onnx_output",
    "pooling", "pad_token", "passage_prefix", "query_prefix",
)
RELATION_TYPES = ("inside", "near", "on", "hanging_on", "connects_regions")
CONTEXT_ENGLISH_RELATION = re.compile(
    r" (?:next to|close to|in front of|with|near|beside|by|against|at|in|on|under|"
    r"behind|above|below|inside|hung|hanging|leaning) "
)
CONTEXT_LOCATION = (
    r"(?:旁边|边上|附近|里面|外面|上面|下面|前面|后面|上方|下方|门口|入口|出口|旁|里|内|外|上|下|前|后)"
)
CONTEXT_LEADING_LANDMARK = re.compile(
    rf"{CONTEXT_LOCATION}\s*(?:的|有)|(?:靠近|挨着|紧邻)[^的，。！？；,.!?;]+的"
)
CONTEXT_TRAILING_LANDMARK = re.compile(
    rf"(?<![现正存])在[^在，。！？；,.!?;]*{CONTEXT_LOCATION}(?=\s*(?:[吗么呢吧？?，,。.!！；;]|$))"
    r"|(?:靠近|挨着|紧邻)(?=\s*[^\s，。！？；,.!?;])"
)


class OnnxEncoder:
    """依据 YAML 加载本地 ONNX 编码器和分词器，返回单位向量。"""

    def __init__(self, settings, model_files):
        """用已完成身份校验的模型文件初始化 CPU 会话，读取输入契约。"""
        import onnxruntime as ort
        from tokenizers import Tokenizer

        self.settings = settings
        tokenizer_path = settings["tokenizer_path"]
        model_path = settings["model_path"]
        check_encoder_files(settings, model_files)
        try:
            self.tokenizer = Tokenizer.from_file(str(tokenizer_path))
            options = ort.SessionOptions()
            options.intra_op_num_threads = settings["intra_op_threads"]
            self.session = ort.InferenceSession(str(model_path), options, providers=["CPUExecutionProvider"])
        except Exception as error:
            raise ValueError(f"Cannot load ONNX model {model_path} or tokenizer {tokenizer_path}: {error}") from error
        check_encoder_files(settings, model_files)

        pad_token = settings["pad_token"]
        pad_id = self.tokenizer.token_to_id(pad_token)
        if pad_id is None:
            raise ValueError(f"Pad token {pad_token!r} is not in {tokenizer_path}")
        self.tokenizer.enable_truncation(max_length=settings["max_length"])
        self.tokenizer.enable_padding(pad_id=pad_id, pad_token=pad_token)

        fields = {"input_ids": "ids", "attention_mask": "attention_mask", "token_type_ids": "type_ids"}
        dtypes = {"tensor(int32)": np.int32, "tensor(int64)": np.int64}
        self.inputs = {}
        for item in self.session.get_inputs():
            if item.name not in fields or item.type not in dtypes:
                raise ValueError(f"Unsupported ONNX input {item.name!r}: {item.type}")
            self.inputs[item.name] = (fields[item.name], dtypes[item.type])
        if "input_ids" not in self.inputs:
            raise ValueError("ONNX encoder must declare an input_ids input")
        if settings["onnx_output"] not in {item.name for item in self.session.get_outputs()}:
            raise ValueError(f"Configured ONNX output is missing: {settings['onnx_output']}")

    def encode(self, texts, prefix):
        """按配置限制批大小和序列长度，按输入顺序返回 float32 向量。"""
        settings = self.settings
        batch_size = settings["batch_size"]
        dimension = settings["dimension"]
        vectors = np.empty((len(texts), dimension), dtype=np.float32)
        for start in range(0, len(texts), batch_size):
            end = start + batch_size
            batch = self.tokenizer.encode_batch([prefix + text for text in texts[start:end]])
            inputs = {}
            for name, (field, dtype) in self.inputs.items():
                inputs[name] = np.array([getattr(item, field) for item in batch], dtype=dtype)
            try:
                hidden, = self.session.run([settings["onnx_output"]], inputs)
            except Exception as error:
                raise ValueError(f"ONNX inference failed for {settings['model_path']}: {error}") from error
            pooled = self.pool(hidden, batch)
            if pooled.shape != (len(batch), dimension):
                raise ValueError(f"ONNX embedding shape {pooled.shape} does not match dimension {dimension}")
            norms = np.linalg.norm(pooled, axis=1, keepdims=True)
            if not np.isfinite(norms).all() or np.any(norms == 0):
                raise ValueError("ONNX encoder produced an invalid vector")
            vectors[start:end] = pooled / norms
        return vectors

    def pool(self, output, batch):
        """根据 YAML 汇总词元向量，或使用模型已导出的句向量。"""
        if not isinstance(output, np.ndarray) or output.dtype.kind != "f":
            raise ValueError("ONNX encoder output must be a floating-point tensor")
        output = output.astype(np.float32, copy=False)
        pooling = self.settings["pooling"]
        if pooling == "none" and output.ndim == 2:
            return output
        if pooling != "none" and output.ndim == 3 and output.shape[1] == len(batch[0].ids):
            if pooling == "cls":
                return output[:, 0, :]
            attention = np.array([item.attention_mask for item in batch], dtype=np.float32)
            weighted = np.einsum("btd,bt->bd", output, attention, dtype=np.float32)
            return weighted / attention.sum(axis=1, keepdims=True)
        raise ValueError(f"ONNX output shape {output.shape} is incompatible with pooling {pooling!r}")


def file_sha256(path):
    """分块读取模型文件，避免为校验值将整个权重文件载入内存。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(release, manifest, name):
    """逐行读取已发布的 JSONL，并在遍历结束时核对摘要与记录数。"""
    path = release / f"{name}.jsonl"
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        for line in stream:
            digest.update(line)
            count += 1
            yield json.loads(line)
    if digest.hexdigest() != manifest["file_sha256"][path.name]:
        raise ValueError(f"Published file digest does not match the manifest: {path.name}")
    if count != manifest["counts"][name]:
        raise ValueError(f"Published file count does not match the manifest: {path.name}")


def encoder_files(settings):
    """取得两份模型文件的身份和修改状态，供延迟加载前后比较。"""
    files = []
    for key in ("model_path", "tokenizer_path"):
        info = settings[key].stat()
        files.append((info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns))
    return files


def check_encoder_files(settings, previous_files):
    """拒绝运行期间替换的模型文件，防止旧索引搭配新权重。"""
    if encoder_files(settings) != previous_files:
        raise ValueError("ONNX model or tokenizer changed while loading; restart or rebuild the index")


def load_release(release, settings):
    """加载已封存的实体与标注，生成发布版本和编码配置对应的索引签名。"""
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

    model_files = encoder_files(settings)
    metadata = {
        "schema_version": "memory_rag/2",
        "memory_version": manifest["memory_version"],
        "release_manifest_sha256": hashlib.sha256(raw).hexdigest(),
        "entities": len(entities),
        **{key: settings[key] for key in ENCODER_FIELDS},
        "model_sha256": file_sha256(settings["model_path"]),
        "tokenizer_sha256": file_sha256(settings["tokenizer_path"]),
    }
    check_encoder_files(settings, model_files)
    return manifest, metadata, entities, annotations, model_files


def entity_names(entity, annotation):
    """返回当前显示名称、别名和原始语义标签。"""
    names = [annotation.get("display_name"), entity["semantic_label"], *annotation.get("aliases", [])]
    return list(dict.fromkeys(name.strip() for name in names if name and name.strip()))


def passage(entity, annotation):
    """将实体及其选定属性组织成一段可供 E5 编码的短文本。"""
    parts = entity_names(entity, annotation)
    if entity.get("description"):
        parts.append(entity["description"])
    attributes = entity.get("attributes", {})
    for key in ("color", "material", "state", "type"):
        if attributes.get(key):
            parts.append(f"{key}: {attributes[key]}")
    return ". ".join(parts)


def build_index(index_dir, settings, metadata, entities, annotations, model_files):
    """为每个实体编码一个向量，并在新目录中发布完整索引。"""
    if index_dir.exists():
        raise ValueError(f"Index already exists; choose a new directory: {index_dir}")
    texts = [passage(row, annotations.get(entity_id, {})) for entity_id, row in entities.items()]
    vectors = np.empty((0, settings["dimension"]), dtype=np.float32)
    if texts:
        vectors = OnnxEncoder(settings, model_files).encode(texts, settings["passage_prefix"])

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
    """在使用向量之前，拒绝过期索引或与当前发布版本不一致的索引。"""
    manifest_path = index_dir / "index.json"
    if not manifest_path.is_file():
        raise ValueError(f"Index is missing or incomplete; run build-index: {index_dir}")
    saved = json.loads(manifest_path.read_text(encoding="utf-8"))
    ids = json.loads((index_dir / "entity_ids.json").read_text(encoding="utf-8"))
    if saved != metadata or ids != list(entities):
        raise ValueError("Index does not match this release or model; rebuild the index")
    vectors = np.load(index_dir / "vectors.npy", mmap_mode="r", allow_pickle=False)
    if not isinstance(vectors, np.ndarray) or vectors.shape != (len(ids), metadata["dimension"]) or vectors.dtype != np.float32:
        raise ValueError("Invalid vector matrix; rebuild the index")
    for start in range(0, len(ids), 1024):
        if not np.isfinite(vectors[start:start + 1024]).all():
            raise ValueError("Invalid vector matrix; rebuild the index")
    return ids, vectors


def normalize_name(text):
    """统一字母大小写和 Unicode 字符宽度，合并连续空白，用于名称匹配。"""
    return " ".join(unicodedata.normalize("NFKC", text).casefold().split())


def context_has_term(text, term):
    """中文术语按子串匹配，英文术语须位于首尾或字母数字边界。"""
    if not term.isascii():
        return term in text
    return re.search(r"(?<![a-z0-9])" + re.escape(term) + r"(?![a-z0-9])", text) is not None


def context_topics(text, aliases):
    """返回查询或实体名称提到的同义词组编号。"""
    return {i for i, group in enumerate(aliases) if any(context_has_term(text, alias) for alias in group)}


def context_target(text):
    """从规范化的查询或名称中提取常见空间描述的目标，用于筛选实体。"""
    # ponytail: 只识别常用空间句式，复合指代需要另行理解句意。
    target = CONTEXT_LEADING_LANDMARK.split(text)[-1].strip()
    trailing = CONTEXT_TRAILING_LANDMARK.search(target)
    if trailing and trailing.start() > 0:
        target = target[:trailing.start()].rstrip()
    return CONTEXT_ENGLISH_RELATION.split(target, maxsplit=1)[0]


def context_entity_topics(names, aliases):
    """只用名称主体确定实体类别，避免把名称中提到的周边物体归为该实体。"""
    return context_topics(" ".join(context_target(name) for name in names), aliases)


def rank_entities(
        ids, vectors, metadata, entities, annotations, text, top_k, encode_query, *,
        vector_only=False, relevant_ids=None):
    """默认优先精确名称，自动检索直接编码并返回排序后的候选项。"""
    name = normalize_name(text)
    if not name or top_k < 1:
        raise ValueError("Query text must be non-empty and top-k must be positive")
    result = {"memory_version": metadata["memory_version"], "status": "empty", "matches": []}
    if ids and relevant_ids is not None and not relevant_ids:
        result["status"] = "no_relevant_match"
        return result
    exact = []
    if not vector_only:
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
        for i in np.argsort(-scores, kind="stable"):
            entity_id = ids[i]
            if relevant_ids is not None and entity_id not in relevant_ids:
                continue
            result["matches"].append({"entity_id": entity_id, "semantic_label": entities[entity_id]["semantic_label"],
                                      "cosine_similarity": float(scores[i])})
            if len(result["matches"]) == top_k:
                break
    return result


def checked_observations(release, manifest):
    """只保留同版观测的修订、媒体路径和同步状态，供查询复用。"""
    fields = ("revision", "rgb_media", "depth_media", "depth_alignment", "capture_sync_status")
    observations = {}
    for row in read_rows(release, manifest, "observations"):
        obs_id = row["id"]
        if obs_id in observations:
            raise ValueError(f"Duplicate published observation: {obs_id}")
        if (row["map_id"], row["map_frame"], row["geometry_revision"]) != (
                manifest["map_id"], manifest["frame_id"], manifest["geometry_revision"]):
            raise ValueError(f"Observation geometry does not match the release: {obs_id}")
        observations[obs_id] = {key: row.get(key) for key in fields}
    return observations


def location_fields(release, entity, observations):
    """定位实体选用的观测，并返回原始证据引用。"""
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
        media = observation.get(role)
        if not isinstance(media, str) or not media:
            raise ValueError(f"Missing {role} in published observation: {obs_id}")
        path = Path(media)
        resolved = (release / path).resolve()
        if path.is_absolute() or not resolved.is_relative_to(release / "evidence") or not resolved.is_file():
            raise ValueError(f"Invalid {role} in published observation: {obs_id}")
        fields[role] = media
    for key in ("depth_alignment", "capture_sync_status"):
        fields[key] = observation.get(key)
    return fields


def relations_by_entity(release, manifest, entities, observations, relation_type):
    """按端点整理有证据支持的关系，保留发布时的方向和依据。"""
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


def add_evidence(result, release, manifest, entities, annotations, observations, relation_type):
    """为每个检索结果补充同一发布版本中的坐标与证据。"""
    for key in ("map_id", "frame_id", "geometry_revision"):
        result[key] = manifest[key]
    if relation_type is not None:
        result["relation_type"] = relation_type
    if not result["matches"]:
        return
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
    """读取 YAML 配置，以配置文件所在目录为基准解析相对路径。"""
    path = config_path.resolve(strict=True)
    settings = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(settings, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    runtime_keys = ("batch_size", "intra_op_threads")
    if "environment" in settings or "environments" in settings:
        environment = settings.get("environment")
        environments = settings.pop("environments", None)
        if not isinstance(environment, str) or not isinstance(environments, dict) or environment not in environments:
            raise ValueError("RAG setting 'environment' must select a profile from 'environments'")
        runtime = environments[environment]
        if not isinstance(runtime, dict) or runtime.keys() != set(runtime_keys):
            raise ValueError("An environment profile must set only 'batch_size' and 'intra_op_threads'")
        if settings.keys() & runtime_keys:
            raise ValueError("Set batch size and CPU threads within the environment profile")
        settings.update(runtime)

    path_keys = ("release", "index_dir", "model_path", "tokenizer_path")
    integer_keys = ("top_k", "dimension", "max_length", *runtime_keys)
    allowed_keys = {*path_keys, *integer_keys, *ENCODER_FIELDS, "environment", "service_name",
                    "context_service_name", "context_aliases"}
    unknown = settings.keys() - allowed_keys
    if "model_dir" in settings:
        raise ValueError("Legacy model_dir is no longer supported; use model_path and tokenizer_path from config/memory_rag.yaml")
    if unknown:
        raise ValueError(f"Unknown RAG settings: {', '.join(map(str, unknown))}")

    for key in path_keys:
        value = settings.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"RAG setting '{key}' must be a non-empty path")
        settings[key] = (path.parent / Path(value).expanduser()).resolve()
    if settings["index_dir"].is_relative_to(settings["release"]):
        raise ValueError("Index must be outside the published directory")
    for key in integer_keys:
        value = settings.get(key)
        if type(value) is not int or value < 1:
            raise ValueError(f"RAG setting '{key}' must be a positive integer")
    for key in ENCODER_FIELDS:
        if key in integer_keys:
            continue
        value = settings.get(key)
        if not isinstance(value, str):
            raise ValueError(f"RAG setting '{key}' must be a string")
        if key not in ("passage_prefix", "query_prefix") and not value.strip():
            raise ValueError(f"RAG setting '{key}' must be a non-empty string")
    if settings["pooling"] not in ("mean", "cls", "none"):
        raise ValueError("RAG setting 'pooling' must be mean, cls or none")
    settings.setdefault("service_name", "/vln_memory/search_memory")
    settings.setdefault("context_service_name", "/vln_memory/retrieve_context")
    aliases = settings.setdefault("context_aliases", [])
    if not isinstance(aliases, list):
        raise ValueError("context_aliases must be a list of non-empty synonym groups")
    for group in aliases:
        if not isinstance(group, list) or not group or any(
                not isinstance(alias, str) or not normalize_name(alias) for alias in group):
            raise ValueError("Each context_aliases group must contain non-empty strings")
    for key in ("service_name", "context_service_name"):
        value = settings[key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"RAG setting '{key}' must be a non-empty ROS service name")
    if settings["service_name"] == settings["context_service_name"]:
        raise ValueError("Search and context retrieval services must have distinct names")
    return settings


class MemorySearch:
    """保留一份已经过验证的发布版本及其索引，供本地重复查询。"""

    def __init__(self, settings):
        """只读取一次索引和发布数据，并在首次进行向量检索时加载本地编码器。"""
        self.release = settings["release"]
        self.settings = settings
        self.manifest, self.metadata, self.entities, self.annotations, self.model_files = load_release(
            self.release, settings)
        self.ids, self.vectors = load_index(settings["index_dir"], self.metadata, self.entities)
        self.encoder = None
        self.observations = None
        self.context_aliases = []
        self.context_names = {}
        self.context_entity_topics = {}
        for group in settings.get("context_aliases", []):
            self.context_aliases.append([normalize_name(alias) for alias in group])
        if self.context_aliases:
            for entity_id, row in self.entities.items():
                names = [normalize_name(name) for name in entity_names(row, self.annotations.get(entity_id, {}))]
                self.context_names[entity_id] = names
                self.context_entity_topics[entity_id] = context_entity_topics(names, self.context_aliases)

    def encode_query(self, text):
        """在本进程的所有向量检索中复用同一个编码器。"""
        if self.encoder is None:
            self.encoder = OnnxEncoder(self.settings, self.model_files)
        return self.encoder.encode([text], self.settings["query_prefix"])[0]

    def relevant_context_ids(self, text):
        """用查询目标的名称或同义类筛选自动候选，缺少匹配时返回空集。"""
        query = normalize_name(text)
        target = context_target(query)
        topics = context_topics(target, self.context_aliases)
        relevant_ids = set()
        for entity_id, names in self.context_names.items():
            if (query in names or topics & self.context_entity_topics[entity_id]
                    or any(context_has_term(target, name) for name in names)):
                relevant_ids.add(entity_id)
        return relevant_ids

    def query(self, text, relation=None, *, vector_only=False, context=False):
        """返回排序后的匹配及证据，自动上下文可拒绝低相关输入。"""
        if relation is not None and relation not in RELATION_TYPES:
            raise ValueError(f"Unsupported memory relation: {relation}")
        relevant_ids = None
        if context and self.context_aliases:
            relevant_ids = self.relevant_context_ids(text)
        result = rank_entities(self.ids, self.vectors, self.metadata, self.entities, self.annotations,
                               text, self.settings["top_k"], self.encode_query, vector_only=vector_only or context,
                               relevant_ids=relevant_ids)
        if result["matches"] and self.observations is None:
            self.observations = checked_observations(self.release, self.manifest)
        add_evidence(result, self.release, self.manifest, self.entities, self.annotations, self.observations, relation)
        return result


def main():
    """解析 CLI 参数，输出一份 JSON 结果，或将诊断信息写入标准错误。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="RAG YAML settings")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("build-index")
    search = commands.add_parser("query")
    search.add_argument("--text", required=True)
    mode = search.add_mutually_exclusive_group()
    mode.add_argument("--vector-only", action="store_true", help="Rank all entities by vector similarity")
    mode.add_argument("--context", action="store_true", help="Apply the automatic context relevance check")
    search.add_argument("--relation", choices=RELATION_TYPES, help="Return supported published relations of this type")
    args = parser.parse_args()

    try:
        settings = load_config(args.config)
        release = settings["release"]
        index_dir = settings["index_dir"]
        if args.command == "build-index":
            _, metadata, entities, annotations, model_files = load_release(release, settings)
            result = build_index(index_dir, settings, metadata, entities, annotations, model_files)
        else:
            result = MemorySearch(settings).query(args.text, args.relation, vector_only=args.vector_only,
                                                  context=args.context)
    except (OSError, ValueError, KeyError, ImportError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f"memory_rag: {error}\n")
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
