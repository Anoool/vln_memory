# MemoryViewer（规划）

## 模块职责与边界

供使用者检查已发布的空间记忆，联动地图、观测原图和证据选区，并提交人工命名和备注。Viewer 不负责录包提取、模型推理或直接写工作库。当前仓库没有 Viewer 可执行程序。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["同一已发布记忆版本"] --> B["地图：实体、关系、轨迹"]
  A --> C["按 ID / 关系 / 时间定位观测"]
  B --> C
  C --> D["RGB、Depth 与证据选区"]
  D --> E["人工标注"]
  E --> F["MemoryEngine 标注接口（规划）"]
~~~

检查入口共享同一个记忆版本。点选地图实体或关系时沿证据引用回到原始观测。标注写入交给 Engine，提交后回读新版本确认。

## 数据结构

计划读取 `manifest.json`、实体/观测/关系/标注 JSONL、`evidence/` 和索引。地图图层使用 `map_id`、`frame_id`、`geometry_revision`；图像及选区使用 `observation_id` 和观测修订；标注挂到稳定实体 ID。完整发布待 Engine 实现。

## 公共 API

计划提供 `memory_viewer --config FILE`。标注计划通过 Engine 的 `memory_annotate` 处理名称、别名和备注。两者均尚未实现。

## 快速开始与典型示例

当前可用 [MemoryEngine](../engine/memory_engine.md#快速开始与典型示例) 的 `search_entities` 查实体和观测引用，再用 `get_observation` 查图像与位姿。Viewer 的典型验收是点选一个地图实体，打开其两张原图和选区，提交名称后重开并回读标注。

## 错误处理

计划在版本、地图或引用不一致时拒绝混读。证据文件缺失时提示具体引用。标注发生修订冲突时重读实体并让用户重新确认。
