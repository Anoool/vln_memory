# 人工标注

通过 `memory_engine --config FILE` 的手动 JSONL 入口提交 `memory_annotate`。实体 ID 取自 `search_entities`。人工标注单独写入 SQLite，原语义标签、实体修订、位置和观测引用继续按原记录读取。

~~~jsonl
{"name":"memory_annotate","arguments":{"entity_id":"ent_2338c347cf7ffd3caa65ecd6a1de1564","expected_annotation_revision":0,"display_name":"白色印花帆布袋","aliases":["白色手提袋"],"note":"现场简称帆布袋"}}
{"name":"search_entities","arguments":{"query":"白色手提袋"}}
~~~

首次标注传 `expected_annotation_revision: 0`。成功返回 `annotation`，其中 `annotation_revision` 是下一次修改要传的修订号。修订不符时返回 `revision_conflict`，先重新搜索实体并查看标注，再提交修改。

至少提供一个标注字段。省略的字段保留原值，名称和备注传 `null` 清空，别名传 `[]` 清空。已标注的实体清空所有字段后仍保留当前修订号。搭配 `call_id` 重放同一请求会返回上次保存的结果。

`search_entities` 按原标签、描述、人工名称和别名匹配，返回的实体包含 `annotation`（有标注记录时）。`--publish` 从同一 SQLite 快照导出 `annotations.jsonl`，可用 `entity_id` 关联 `entities.jsonl`。查看[发布目录格式](memory_publication.md)。

现有 v6 工作库仍可查询和发布。首次成功写入标注时，工作库在同一事务中升级为 v7。自动视觉模型批次仅可调用 `create_entity` 和 `update_entity`。
