"""将 Viewer 的人工标注提交给与当前发布同源的 MemoryEngine。"""

from contextlib import closing
from dataclasses import dataclass
import json
from pathlib import Path
import sqlite3
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class AnnotationStore:
  """本机标注入口和它绑定的工作数据库路径。"""

  engine: Path
  config: Path
  database: Path

  @classmethod
  def from_config(cls, directory, settings):
    """按 Viewer YAML 解析 Engine 路径；Engine 工作目录为项目根。"""
    engine = (directory / settings["engine"]).resolve(strict=True)
    config = (directory / settings["engine_config"]).resolve(strict=True)
    work = yaml.safe_load(config.read_text(encoding="utf-8"))["dataset"]["memory_dir"]
    database = (ROOT / work / "entities.sqlite3").resolve(strict=True)
    return cls(engine, config, database)

  def annotations(self, manifest):
    """只读校验工作库与发布来源，并返回当前活动实体的标注及实体修订。"""
    with closing(sqlite3.connect(self.database.as_uri() + "?mode=ro")) as db:
      db.execute("BEGIN")
      binding = db.execute(
        "SELECT dataset_manifest_sha256, source_manifest_sha256, map_id, map_frame, geometry_revision "
        "FROM store_meta WHERE singleton=1"
      ).fetchone()
      fields = ("dataset_manifest_sha256", "source_manifest_sha256", "map_id", "frame_id", "geometry_revision")
      if binding != tuple(manifest[key] for key in fields):
        raise ValueError("工作库与当前发布包不是同一套观测和地图。")
      version = db.execute("PRAGMA user_version").fetchone()[0]
      query = (
        "SELECT e.entity_id, e.revision, a.annotation_revision, a.display_name, a.aliases_json, a.note "
        "FROM entities e LEFT JOIN entity_annotations a USING (entity_id)"
      )
      if version < 7:
        raise ValueError("工作库尚未启用人工标注。")
      if version >= 8:
        query += " WHERE e.retired=0"
      rows = {}
      for entity_id, entity_revision, annotation_revision, name, aliases, note in db.execute(query):
        rows[entity_id] = (entity_revision, {
          "annotation_revision": annotation_revision or 0, "display_name": name,
          "aliases": json.loads(aliases or "[]"), "note": note,
        })
      return rows

  def save(self, entity_id, annotation_revision, display_name, aliases, note):
    """调用唯一允许写入的人工标注工具；不传图像、坐标或关系。"""
    distinct_aliases = dict.fromkeys(line.strip() for line in aliases.splitlines() if line.strip())
    request = {"name": "memory_annotate", "arguments": {
      "entity_id": entity_id, "expected_annotation_revision": annotation_revision,
      "display_name": display_name.strip() or None, "aliases": list(distinct_aliases), "note": note.strip() or None,
    }}
    result = subprocess.run(
      [str(self.engine), "--config", str(self.config)], cwd=ROOT,
      input=json.dumps(request, ensure_ascii=False) + "\n", encoding="utf-8", capture_output=True, timeout=12,
    )
    if not result.stdout.strip():
      raise ValueError(result.stderr.strip() or "MemoryEngine 未返回标注结果。")
    payload = json.loads(result.stdout)
    if payload["status"] == "revision_conflict":
      raise ValueError("这条标注已被修改，请刷新后再保存。")
    if result.returncode != 0 or payload["status"] != "ok":
      raise ValueError(payload.get("message") or payload["status"])
    return payload["annotation"]
