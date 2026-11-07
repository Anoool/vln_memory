# 正式观测组装

公开头为 [observation_builder.hpp](../../../include/vln_memory/engine/observation_builder.hpp)，安装目标为 `vln_memory::vln_memory_observation_builder`。模块在 ROS 和无 ROS 构建中都可使用。

`buildObservation(pair, context)` 消费 Reader 已回读的 `PreparedPair`。输入 `ObservationBuildContext` 提供地图 ID、几何版本、地图 frame、依据、观测修订及可选待处理原因。地图 frame 必须与 preparation 相同。组装器从已保存配对复制两路原始时间、标定选择与模型、独立历史位姿、媒体路径及消息摘要，生成稳定观测 ID 并调用 [正式记录校验](../memory/observation.md)。

已持有 Reader 与配对 ID 时，可调用 `buildObservation(reader, pair_id, context)`，该重载先完整回读配对。数据集发布对候选执行完整回读，组装入选记录时改用元数据回读，并传给接受 `PreparedPair` 的重载。详见 [观测数据集发布](observation_dataset.md)。

成功返回独立拥有的 `Observation`，允许带待处理原因。失败返回 Reader 或正式观测校验错误。返回记录通过相对路径引用来源，像素不随组装结果保存。
