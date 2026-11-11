# 离线构建实施规划

按《VLN_SPATIAL_SEMANTIC_MEMORY_DESIGN.md》第 2 部分交付离线构建。第 1 部分的数据、证据和发布边界见[设计条款对照](sections_1_2_scope.md)。

| 阶段 | 状态与验收 |
| --- | --- |
| A · [BagBuilder](../modules/bag/bag_builder.md) | 已实现统一 YAML 和双图直出。核对每条正式观测的两张图、原始时间、历史位姿、标定及来源，问题帧和未配对帧查报告。 |
| B1 · [MemoryEngine](../modules/engine/memory_engine.md) 观测与定位 | 已实现 `get_observation`、`project_depth`。用已知像素和位姿核对实测点，检查无效深度、缺位姿和多簇歧义。 |
| B2 · 实体与关系 | 已实现 `search_entities`、`create_entity`、`update_entity`、`spatial_query`。验收永久 ID、观测证据、修订冲突、归属、距离定义和事务。 |
| B3 · 模型批次 | 已实现 Responses 工具循环、跨帧冲突拒写和同 ID 补定位。用固定录包四帧从空库验收定位实体、带观测依据的关系和跨批次追加。另以实际帧验证旧证据保留和位置冲突回滚。 |
| B4 · 纠错、标注、发布 | 待实现 `correct_memory` 的 update/merge/split/retract 和 `memory_annotate`。保留证据、修改理由和人工名称。完整文件由单写者写完后再原子发布。 |
| C · [MemoryViewer](../modules/viewer/memory_viewer.md) | 待实现。验收从同一真实录包构建、检查地图、按实体/关系/时间回查原图和选区、提交标注再重开回读。 |

BagBuilder 和 MemoryEngine 共用 [vln_memory.yaml](../../config/vln_memory.yaml)，以 `dataset.memory_dir` 交接。项目录包固定包含彩色和深度图。地图、时钟段、深度比例和几何版本应明确，采样时间相近不能作为物理配准或曝光同步的证据。第 3 部分的完整在线查询和导航接入另行处理。
