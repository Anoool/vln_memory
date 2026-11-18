"""验证 YAML 选型与向量索引的绑定规则，使用独立的临时目录。"""

import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from memory_rag import DEFAULT_CONFIG, MemorySearch, build_index, check_encoder_files, load_config, load_release, rank_entities


def check(root):
    """用空记忆发布验证索引兼容性，测试无需下载真实权重。"""
    release = root / "release"
    release.mkdir()
    empty_digest = hashlib.sha256(b"").hexdigest()
    manifest = {
        "artifact_kind": "spatial_memory",
        "schema_version": "spatial_memory/1",
        "memory_version": "test",
        "counts": {"entities": 0, "annotations": 0},
        "file_sha256": {"entities.jsonl": empty_digest, "annotations.jsonl": empty_digest},
    }
    for table in manifest["counts"]:
        (release / f"{table}.jsonl").write_bytes(b"")
    (release / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    config = root / "memory.yaml"
    source = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    source.update(release="release", index_dir="index", model_path="test.onnx",
                  tokenizer_path="tokenizer.json", dimension=7, pooling="cls",
                  max_length=16, passage_prefix="", query_prefix="")
    source.update(environment="desktop", environments={
        "desktop": {"batch_size": 4, "intra_op_threads": 2},
        "jetson": {"batch_size": 1, "intra_op_threads": 1},
    })
    config.write_text(yaml.safe_dump(source), encoding="utf-8")
    settings = load_config(config)
    assert settings["model_path"] == root / "test.onnx"
    settings["model_path"].write_bytes(b"model")
    settings["tokenizer_path"].write_bytes(b"tokenizer")
    _, metadata, entities, annotations, model_files = load_release(release, settings)
    build_index(settings["index_dir"], settings, metadata, entities, annotations, model_files)
    source["environment"] = "jetson"
    config.write_text(yaml.safe_dump(source), encoding="utf-8")
    jetson = load_config(config)
    assert settings["environment"] == "desktop"
    assert jetson["batch_size"] != settings["batch_size"]
    assert jetson["intra_op_threads"] != settings["intra_op_threads"]
    assert jetson["index_dir"] == settings["index_dir"]
    search = MemorySearch(jetson)
    assert search.metadata == metadata
    source["environment"] = "unknown"
    config.write_text(yaml.safe_dump(source), encoding="utf-8")
    try:
        load_config(config)
    except ValueError as error:
        assert "select a profile" in str(error), error
    else:
        raise AssertionError("An unknown environment was accepted")
    assert search.vectors.shape == (0, 7)
    assert isinstance(search.vectors, np.memmap)
    assert search.encoder is None and search.observations is None

    assert_incompatible(dict(settings, max_length=32))
    assert_incompatible(dict(settings, pooling="mean"))
    assert_incompatible(dict(settings, query_prefix="query: "))
    settings["tokenizer_path"].write_bytes(b"changed tokenizer")
    assert_incompatible(settings)
    settings["tokenizer_path"].write_bytes(b"tokenizer")
    settings["model_path"].write_bytes(b"other weights")
    assert_incompatible(settings)
    try:
        check_encoder_files(settings, search.model_files)
    except ValueError as error:
        assert "changed while loading" in str(error), error
    else:
        raise AssertionError("The encoder accepted model files changed after verification")


def assert_incompatible(settings):
    """不同模型或编码语义必须在向量查询前被拒绝。"""
    try:
        MemorySearch(settings)
    except ValueError as error:
        assert "rebuild the index" in str(error), error
    else:
        raise AssertionError("Incompatible settings accepted an existing index")


def check_retrieval_modes():
    """同一精确名称经主动入口直接命中，经自动入口仍调用编码器。"""
    entities = {"bag": {"semantic_label": "背包"}, "chair": {"semantic_label": "椅子"}}
    calls = []

    def encode(text):
        """记录实际交给编码器的文本，并返回第一条记录的单位向量。"""
        calls.append(text)
        return np.array([1.0, 0.0], dtype=np.float32)

    inputs = (list(entities), np.eye(2, dtype=np.float32), {"memory_version": "test"},
              entities, {}, "背包", 1, encode)
    assert rank_entities(*inputs)["status"] == "exact"
    assert calls == []
    result = rank_entities(*inputs, vector_only=True)
    assert result["status"] == "candidates"
    assert result["matches"][0]["entity_id"] == "bag"
    assert result["matches"][0]["cosine_similarity"] == 1.0
    assert calls == ["背包"]


if __name__ == "__main__":
    with TemporaryDirectory() as directory:
        check(Path(directory))
    check_retrieval_modes()
