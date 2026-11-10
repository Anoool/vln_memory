# 记忆数据与存储

## 模块职责与边界

定义正式观测、实体和观测证据的契约，以及 SQLite 工作库的事务读写。Observation 自己持有记录字段，通过相对路径引用媒体。写库层维护身份、版本和引用，由 [MemoryEngine](../engine/memory_engine.md) 提供工具语义、投影和模型调度。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["Observation：双图、时间和来源"] --> B["EntityObservation：选区及定位证据"]
  B --> C["Entity：稳定 ID 和修订"]
  C --> D["关系及空间归属"]
  B --> E["SQLite 工作库"]
  C --> E
  D --> E
  F["工具回执与批次恢复点"] --> E
~~~

观测 ID 由录包、配对、两路原始消息、时钟和段构成，是不透明键。实体 ID 在证据追加和补定位后保留。实体选中的定位证据和新增观测分开记录，位姿变化或证据冲突可触发复核。

## 数据结构

| `observation/1` 字段组 | 含义 |
| --- | --- |
| `id`、`revision`、`bag_id`、`pair_id`、`clock_id`、`segment_id` | 观测身份与修订，来源另用 `source_manifest_sha256` 绑定。 |
| `map_id`、`map_frame`、`body_frame`、`geometry_revision`、`map_basis` | 地图上下文、几何版本与依据。 |
| `stamp`、`rgb_stamp`、`depth_stamp`、`delta_ns` | 原始整数 `{sec,nanosec}`；`stamp` 取 RGB 时刻，`delta_ns = Depth − RGB`。 |
| `rgb_image` / `depth_image`、`rgb_media` / `depth_media` | 各自话题、frame、网格、时间和媒体相对路径。对应的相机模型与原始标定独立保存。 |
| `camera_pose_map`、`depth_camera_pose_map`、`body_pose_map` | 分别为 RGB、Depth、RGB 参考时刻的位姿和所用 TF 端点。 |
| `depth_storage`、`capture_sync_status`、`depth_alignment`、`pending_reasons` | 深度格式、单位、比例及质量，以及同步、配准与待处理原因。消息证据保留原始来源和摘要。 |

| SQLite 表（schema v6） | 内容 |
| --- | --- |
| `store_meta` | 观测集及来源清单摘要、地图、frame 和几何版本。 |
| `entities` | 稳定 `entity_id`、`kind`（`object/region`）、标签、正整数修订、定位状态、业务字段和所选定位证据。 |
| `entity_observations` | 观测 ID、修订、来源摘要、关联修订及 `evidence_json`。物品证据保存请求选区、深度像素和地图表面点；空间证据保存内部确认与底盘锚点。 |
| `entity_relations` | 类型、端点、几何版本、关系依据及状态。数值 `near` 另保存距离、维度和阈值。 |
| `pending_entities`、`tool_calls`、`build_progress` | 待定位请求、按 `call_id` 保存的请求与结果，以及当前恢复点。 |

`located` 说明存在选中的定位证据，`recognition_state` 独立描述语义或关系的确认程度。确认的区域 `inside` 关系决定 `region_id`。正式记忆版本尚未发布。

## 公共 API

| 公开头与目标 | 入口 |
| --- | --- |
| [observation.hpp](../../../include/vln_memory/memory/observation.hpp) · `vln_memory::vln_memory_observation` | `makeObservationId`、`collectObservationPendingReasons`、`validateObservation`、`serializeObservation`、`deserializeObservation`。 |
| [entity.hpp](../../../include/vln_memory/memory/entity.hpp) · `vln_memory::vln_memory_entity` | `Entity`、`EntityObservation`、实体与证据校验，`selectedEntityPoint` 回读所选代表点。 |
| [entity_store.hpp](../../../include/vln_memory/memory/entity_store.hpp) · `vln_memory::vln_memory_entity_store` | `createEntityStore` / `openEntityStore`，`loadEntity`、事务插入、追加、补定位与关系查询。用正式观测证据写库时优先走 Engine 的 `observed_entity_store.hpp`。 |

## 快速开始与典型示例

~~~cpp
#include <vln_memory/memory/observation.hpp>
#include <vln_memory/memory/entity_store.hpp>

auto observation = vln_memory::deserializeObservation(line);
auto opened = vln_memory::openEntityStore(db_path);
if (observation.ok() && opened.ok()) {
  auto entity = opened.store->loadEntity(entity_id);
  // 用 observation.observation 和 entity.entity 读取记录，先检查各自的状态。
}
~~~

需要查入库证据时，可用 SQLite 将 `entities.entity_id` 和 `entity_observations.entity_id` 连接。定位坐标在选中的证据中，工具响应会把它整理为 `position_map`。

## 错误处理

记录解析或校验失败时，检查返回的 `code/message`。按操作检查媒体、标定和位姿，合法记录可带 pending 原因。工作库遇到来源或版本不匹配时应重开对应数据集；遇到 `revision_conflict` 时回读最新修订再更新。事务失败时按返回状态处理，避免只重试事务中的某一步。
