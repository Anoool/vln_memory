# memory/entity

公开头为 [entity.hpp](../../../include/vln_memory/memory/entity.hpp)，安装目标为 `vln_memory::vln_memory_entity`。记录拥有字段内容，生命周期独立于 Reader 和 Observation。

| 字段 | 语义 |
| --- | --- |
| entity_id | `ent_` 加 32 个小写十六进制字符。创建时分配 128 位随机 ID，后续修订沿用该值。同标签、同观测内的两个物品可以使用不同 ID。 |
| kind | `object` 或 `region`。物品保留深度选区，空间保留内部确认及底盘锚点。 |
| semantic_label | 物品语义标签，原样保留。 |
| status / location_pending_reason | `located` 有该类实体对应的代表点；`pending_location` 记录未定位原因，代表点为空。 |
| revision | 实体自身修订号，初始为 1。 |
| map_id / map_frame / geometry_revision | 创建时取自正式观测，规定代表点使用的地图和几何版本。 |
| observations | 正式观测 ID、观测修订号和 preparation manifest 指纹，选区归属对应的观测。 |

物品选区证据保存深度原图 bbox 或 polygon、计算选项、可空投影状态、像素数和首簇点数。实测代表点保留像素坐标、optical-z 米值和 map 米制坐标；它表示可见表面的位置。缺几何时保留已验证选区，投影状态和代表点均为空。

空间实体通过对应观测引用保存 `region_interior`。其中 `confirmation_ref` 引用上游的内部确认记录，`body_anchor` 保存观测参考时刻、底盘 frame 和已核对的 `body_pose_map` 平移（米）。已确认但缺位姿时，确认引用仍保留，锚点为空。底盘朝向属于观测，空间边界和朝向需各自的证据。两种实体的创建入口见 [entity_builder](../engine/entity_builder.md)。
