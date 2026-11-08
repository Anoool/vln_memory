# 从结构化识别结果构建语义记忆

`memory_semantic_build` 读取已发布的 v2 观测目录和标准输入的 JSONL。每行是一条操作，标准输出返回一行结果。不同操作分别提交，任一失败时进程最终返回非零。数据集在同一进程内索引一次，实体库在首次实体操作时打开或创建。

```bash
ros2 run vln_memory memory_semantic_build \
  --dataset /path/to/observations --store /path/to/entities.sqlite3 \
  < requests.jsonl > results.jsonl
```

已知正式观测 ID 时，可以直接写入一条纯 RGB 识别结果。将下面的 `obs_ID` 替换为该数据集中的 ID。

```jsonl
{"op":"create","observation_id":"obs_ID","entity_key":"session-01/object-03","kind":"object","label":"椅子"}
```

| op | 必需字段 | 行为 |
| --- | --- | --- |
| lookup | 观测引用 | 读回已发布的观测 ID、修订和待处理原因。 |
| create | 观测引用、`entity_key`、`kind`、`label` | 创建 `object` 或 `region`。宿主在数据集内为实体分配稳定键，重放同键返回 `already_exists` 和已有 ID。 |
| append | 观测引用、`entity_id`、`expected_revision` | 在原 ID 下追加一条证据。 |
| resolve_location | 观测引用、`entity_id`、`expected_revision` | 选择已关联的有效几何证据补定位。 |
| inside | 观测引用、`subject_id`、`target_id`、`evidence_ref` | 保存显式确认的物品到区域归属。证据观测必须关联到该物品，重复归属可重试，冲突归属返回错误。 |
| near | `subject_id`、`target_id`、`threshold_m` | 根据双方已选地图点计算三维欧氏距离。正向结果保存为对称关系，并记录阈值、距离和 `geometry_revision`。 |
| get | `entity_id` | 读回实体 revision、状态、观测引用和 `region_id`。 |

观测引用可直接使用 `observation_id`。回放场景也可以传 `rgb_stamp`、`depth_stamp` 和宿主从清单确认的 `source_manifest_sha256`。时间采用原始 `{sec,nanosec}`。读取器匹配已发布的唯一配对。两路时间中的任意一路改变时，调用方会收到找不到配对或匹配歧义的结果。

物品可选 `depth_bbox`，格式为深度原图的半开坐标 `[x0,y0,x1,y1]`。区域可选 `confirmation_ref`，引用上游的内部确认。纯 RGB 识别可以创建带观测来源的待定位实体。深度投影使用正式观测中已有的有效模型和历史位姿，配准与采集同步状态仍在观测记录中。

## 与 vln_agent 接入

`capture_image` 的 `images[0].metadata` 提供原始 `rgb_stamp`、`depth_stamp`。回放宿主将录包绑定到同源的已发布观测目录及 preparation 摘要，再向本命令发送 `lookup`。取得 `observation_id` 后，把模型输出整理为上述 JSONL 操作。实体 ID 和修订由返回结果回传给宿主，用于后续追加和关系操作。

本地 Agent 的 `EnvironmentMemory::saveResponse` 已接收 `environment_observation`。宿主可复用这份结构化输出，把 `objects[].label` 转成物品的 `create` 或 `append`。模型的 `objects[].id` 只在单次响应内有效，由宿主确认跨帧身份并维护 `entity_key → entity_id/revision`。图像参考系下的 `left_of` 等关系保留在响应记录中。地图工作库的 `inside` 需要显式归属证据，`near` 根据已选定位点计算。两个 SQLite 文件各自保持现有用途，适配器通过 JSONL 交接。

在线队列和离线构建默认使用 30 ms 门限。按帧周期使用时，可将队列的 `sync_tolerance_sec` 设为 `0.066666667`，离线将 `bag.sync_tolerance_ns` 设为 `66666667`。在线队列与离线选帧可产生不同的配对，跨项目回查继续使用确切的已发布配对。机器人实时运行的录包身份由宿主分配。

ROS 构造夹具覆盖回放时间匹配、结构化实体写入、归属冲突、无向距离关系和重复创建防重。SQLite schema v4 在打开旧库时事务增加关系表，物品的 `region_id` 直接来自 `inside` 关系。
