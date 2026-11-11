# MemoryEngine

## 模块职责与边界

从 BagBuilder 的双图目录回读观测、用深度计算地图坐标，并在观测目录维护 `entities.sqlite3`。可逐行调用工具，也可运行 Responses 视觉模型批次。模型提交语义判断和选区，坐标由本地计算。手动 JSONL 提供六项工具，纠错、人工标注和完整版本发布见[实施规划](../../project/system_plan.md)。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["正式观测 + 双图证据"] --> B["ObservationDatasetReader"]
  B --> C["MemoryTools：观测、投影、实体与关系"]
  D["JSONL 请求"] --> C
  E["Responses 模型"] <-->|"函数请求 / 工具结果"| C
  C --> F["SQLite：业务记录、回执、恢复点"]
~~~

定位使用原图坐标。彩色选区用两图各自的模型和历史位姿做 Depth→RGB 重投影，落入同一彩色像素时取 RGB 相机中最近的深度点。分簇和定位依据保留在原深度像素上。物品用实测可见表面点，空间用经确认在内部的底盘位置。

批次按发布顺序取 1–4 条观测，附原始 RGB 和最多两张相关历史彩色图。每轮先保存模型输出，再执行工具。业务写入与 `call_id` 回执在同一事务提交，批次完成才推进观测位置。已处理的观测在恢复时跳过。

自动构建只向模型提供 `create_entity` 和 `update_entity`，提交选区时由本地执行定位，实体 ID 齐全后可用这两个工具写入关系。

构建提示词要求模型看整图，把同一物体的可见部件归到一个实体，并结合整图判断实体关系，记录支持关系的观测 ID。物品的 `selector` 使用原始 RGB 上同一块可见、连续表面内的小框，避开轮廓、空隙和遮挡边缘。标签、描述和属性记录整个物体。确认跨帧身份后才追加观测，新建实体或改标签时显式提供 `recognition_state`。没有可辨认的实体时允许完成空批。

## 数据结构

| 数据 | 关键字段 |
| --- | --- |
| JSONL 请求 | `name`、`arguments`，可选 `call_id`。每条请求对应一行结果。 |
| 像素选择器 `selector` | `image` 默认为 `rgb`，也可用 `depth`。`point` 用 `pixel_xy`，`bbox` 用右、下端不包含的 `xyxy_px`，`polygon` 用 `vertices_xy_px`。 |
| 工具结果 | `status` 和业务字段。正常业务响应携带 `frame_id`、`map_id`、`geometry_revision`，完整版本发布前 `memory_version` 为 `null`。 |
| SQLite | 工作库按观测清单摘要绑定。实体、观测证据、关系、待定位请求、回执和进度的字段见[记忆数据与存储](../memory/memory.md)。 |

## 公共 API

`memory_engine --config FILE` 接收 JSONL，追加 `--build` 时运行模型批次。

| 工具 | 请求 | 返回与作用 |
| --- | --- | --- |
| `get_observation` | `observation_id` | 两张图的相对路径、尺寸、时间、深度存储语义和相机/底盘位姿。 |
| `project_depth` | `observation_id`、`selector`；可选 `include_points` | 地图代表点、深度像素、点数、范围或歧义候选。`include_points: true` 可返回可信簇的 `points_path`。 |
| `search_entities` | 可选 `query`、`region_id` | 匹配的实体及其 ID、修订、坐标、观测引用。 |
| `create_entity` | `kind`、`semantic_label`、`add_observations`；可附描述、属性、关系 | 创建物品或空间。物品证据附 `selector`，空间证据附 `confirmation_ref`。定位不足可返回 `pending_location`。 |
| `update_entity` | `entity_id`、`expected_revision`；可附 `set_fields`、`add_observations`、`relations` | 在同一 ID 上更新字段、证据或关系，返回最新修订。重复证据不会创建第二份。 |
| `spatial_query` | `operation`、`entity_ids`；`near` 可用 `parameters.dimension`（`xy/xyz`）和 `parameters.threshold_m` | 查询 `inside/on/hanging_on/connects_regions` 等已存证据，或计算 `near`。数值距离默认取 YAML 中 `geometry.near_dimension` / `near_threshold_m`。 |

关系写入的条目含 `type`、`entity_ids`、`evidence_observation_ids`，`$self` 表示当前写入实体。`inside` 从被包含实体提交。缺可靠关系证据时返回 `related: null`。几何版本、维度和阈值都匹配时才复用数值 `near`。

| C++ 入口 | 调用方式 |
| --- | --- |
| [observation_dataset_reader.hpp](../../../include/vln_memory/engine/observation_dataset_reader.hpp) | `openObservationDataset(root)` 后按 ID/时间或批次读观测。直出 `observation_dataset/3` 用 `loadRgb` / `loadDepth` 校验并回读媒体。历史 v1/v2 用 `openSourcePreparation` 绑定来源。链接 `vln_memory::vln_memory_observation_dataset`。 |
| [observation_depth_region.hpp](../../../include/vln_memory/engine/observation_depth_region.hpp) | 在正式观测和来源媒体上计算选区的地图位置。 |
| [entity_builder.hpp](../../../include/vln_memory/engine/entity_builder.hpp) · [observed_entity_store.hpp](../../../include/vln_memory/engine/observed_entity_store.hpp) | `createObjectEntity` / `createRegionEntity` 构造记录。`createObservedEntityStore` 绑定数据集，`insertObservedEntity`、`appendObservedEvidence`、`resolveEntityLocation` 校验引用后写入。 |
| [build_input.hpp](../../../include/vln_memory/engine/build_input.hpp) · [observation_builder.hpp](../../../include/vln_memory/engine/observation_builder.hpp) · [observation_dataset.hpp](../../../include/vln_memory/engine/observation_dataset.hpp) | 旧 preparation 的筛选、分批、观测组装和数据集发布。对应命令 `memory_build_input` 与 `memory_observation_dataset` 可用 `--help` 查看参数。 |

## 快速开始与典型示例

`dataset.memory_dir` 指向 BagBuilder 的输出目录。替换 `obs_ID` 为 `observations.jsonl` 中的实际 ID。

~~~bash
ros2 run vln_memory memory_engine --config config/vln_memory.yaml <<'JSONL'
{"name":"get_observation","arguments":{"observation_id":"obs_ID"}}
{"name":"project_depth","arguments":{"observation_id":"obs_ID","selector":{"type":"point","pixel_xy":[446,361]}}}
{"name":"create_entity","arguments":{"kind":"object","semantic_label":"打印机","add_observations":[{"observation_id":"obs_ID","selector":{"type":"bbox","xyxy_px":[540,300,740,420]}}]}}
JSONL
~~~

跑模型时，在 [vln_memory.yaml](../../../config/vln_memory.yaml) 设置 `model.base_url`、`model.model`，在同目录本地 `api_key.yaml` 填 `api_key: "你的密钥"`。`model.api_key_file` 相对于主配置；无需鉴权时写空字符串，旧配置仍可用 `api_key_env`。密钥文件由 Git 忽略，也不会随包安装。`--build` 会将彩色观测和选中的历史彩色视图发送到配置的模型服务。

~~~bash
ros2 run vln_memory memory_engine --config config/vln_memory.yaml --build
~~~

合成双图验收使用的是 `base_url: https://api.deepseek.com` 和 `model: deepseek-flash`。也可配置其他兼容服务的 API 基地址或完整 `/responses` 地址。

终端给出 `status`、`total_observations`、`resumed_from`、`processed_observations`。实体和详细回执查 `entities.sqlite3`，例如执行 `SELECT label, revision, status FROM entities;`。

兼容入口 `memory_semantic_build --dataset DIR --store DB < requests.jsonl` 接收带 `op` 的结构化请求（`lookup/create/append/resolve_location/inside/near/get`）。Agent 可按两路原始时间与 `source_manifest_sha256` 找回唯一已发布配对，再用稳定 `entity_key` 创建或用 `entity_id/expected_revision` 追加。单次识别响应的局部 ID 由宿主维护映射。下面是写入一条纯 RGB 识别结果的请求，缺几何时实体带观测来源进入待定位状态。完整协议见 [CLI 实现](../../../src/apps/memory_semantic_build_main.cpp)。

~~~jsonl
{"op":"create","observation_id":"obs_ID","entity_key":"session-01/object-03","kind":"object","label":"椅子"}
~~~

## 错误处理

| 情况 | 处理 |
| --- | --- |
| `observation_not_found` / `observation_read_failed` | 核对观测 ID、清单和媒体，重开正确的数据目录。C++ 读取器可进一步区分 `integrity_mismatch` 等原因。 |
| `invalid_region` / `invalid_depth` / `missing_map_pose` / `insufficient_support` / `ambiguous_region` | 检查选区、深度、位姿与候选点。坐标不可用时查看返回状态和证据。 |
| `pending_location` / `revision_conflict` / `call_id_conflict` | 保留待定位请求。更新冲突先读最新修订；同一调用 ID 只重放同一请求。 |
| 模型 HTTP 失败、超时、轮数不足 | `build.resume: true` 时用原模型及地址重跑同一配置，必要时提高 `build.max_tool_rounds`，已提交的回执可重放。 |
