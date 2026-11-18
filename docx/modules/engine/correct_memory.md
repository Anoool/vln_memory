# 实体纠错

`correct_memory` 走 `memory_engine --config FILE` 的手动 JSONL 入口。先用 `search_entities` 取得活动实体的 ID 和修订。请求需要 `operation`、`entity_id`、`expected_revision` 和非空 `reason`。成功返回 `correction_id`、仍活动的结果实体及撤回的实体 ID。`call_id` 可安全重放同一请求。

| 操作 | 额外参数 | 保存结果 |
| --- | --- | --- |
| `update` | `set_fields`、`add_observations`、`relations` 沿用 `update_entity`，也可用 `location_observation_id` 选择已关联的定位证据。 | 保留实体 ID。更换代表点时，原有已确认关系进入复核。 |
| `retract` | 共用参数即可。 | 实体退出当前查询和发布，原实体、证据及标注仍存工作库，原关系进入纠错历史。 |
| `merge` | `entity_id` 为保留目标，另传 `source_entity_id`、`source_expected_revision`。 | 同类型源实体的证据追加至目标。目标保留自己的标注和已有定位，源退出当前集合。相同观测的不同选区会返回冲突，整个操作回滚。 |
| `split` | `parts` 至少两组，每组含 `semantic_label` 和 `observation_ids`。 | 每条原观测证据必须恰好分给一组。每组分配新 ID 和各自的定位证据，原实体退出当前集合。各组的人工名称和关系可分别确认。 |

例如，已定位的实体可用另一条已有位置证据重新选点。下面的 ID 和修订从查询结果替换。

~~~jsonl
{"name":"correct_memory","arguments":{"operation":"update","entity_id":"ENTITY_ID","expected_revision":2,"reason":"另一帧的可见表面更清楚","location_observation_id":"OBSERVATION_ID"}}
~~~

修改、撤回和历史在同一 SQLite 事务提交。修订不符时返回 `revision_conflict`，重新查询后再提交。源实体的历史证据仍按原观测 ID 回查。合并或拆分涉及的原关系留在历史里，由人工复核后写入适用的新关系。

首次成功纠错时，旧工作库升级到 v8。`--publish` 在 `reports/corrections.jsonl` 导出原因、实体前后记录、变更前关系及撤回 ID，并在 `manifest.json` 记录条数和摘要。发布会保留历史引用的原双图。
