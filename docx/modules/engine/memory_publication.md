# 记忆版本发布

`memory_engine --config FILE --publish OUTPUT_DIR` 将 `dataset.memory_dir` 中的正式观测和 SQLite 工作库发布为可独立读取的目录。目标目录名即 `memory_version`，使用字母、数字、下划线、连字符或点。目标路径和工作目录不能互相包含。命令读取已提交的 SQLite 快照。需要已有的 `entities.sqlite3`。若旧库低于 v6，先用正常工具模式打开升级。

~~~bash
ros2 run vln_memory memory_engine --config config/vln_memory.yaml --publish /path/to/releases/site_s4-v1
~~~

| 路径 | 内容 |
| --- | --- |
| `manifest.json` | `spatial_memory/1`、版本、地图、坐标系、几何版本、来源、行数和文件 SHA-256。 |
| `entities.jsonl`、`relations.jsonl` | 带稳定 ID、修订、观测引用的实体和关系。实体中包含选定定位证据及选区。 |
| `observations.jsonl`、`evidence/` | 原始正式观测与来源清单，以及正式观测引用的所有双图。保留原媒体索引用于来源校验，其中也列有未被正式观测引用的消息。`ObservationDatasetReader` 可以直接打开发布目录。 |
| `annotations.jsonl` | SQLite 快照中的当前[人工标注](memory_annotate.md)，用 `entity_id` 关联实体。没有记录时为空。 |
| `indexes/` | `media_sha256.jsonl` 只列实际打包的媒体。`entities_by_observation.jsonl` 按观测 ID 回查关联实体。 |
| `reports/` | 待定位实体原请求、采集报告，以及可追溯到原图的 `corrections.jsonl`。 |

文件先写入同级 `.partial-*` 目录。观测、实体引用和图像摘要核对通过后，整个目录一次改名发布。已有版本不会被覆盖。发布失败时目标目录不会出现，可以查看保留的 partial 目录。每次发布使用新的版本目录名。

用发布目录作为 `dataset.memory_dir` 时，可调用 `get_observation` 或不导出点文件的 `project_depth`。返回值携带清单里的 `memory_version`。实体和人工标注可直接读取发布文件。需要构建、纠错、编辑标注或导出投影点时，在工作目录上运行对应命令。
