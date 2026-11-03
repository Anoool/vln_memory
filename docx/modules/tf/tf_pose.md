# TF 数值位姿

lookupPose(target, source, stamp, options, jump_limits) 调用 TfIndex::checkTime。成功时返回 T_target_source，按 p_target = R * p_source + t 使用，平移单位米，旋转为 xyzw 单位四元数。结果拥有数值和原始样本证据，生命周期独立于索引。查询失败时 pose 为空。

transformTfPoint 将一个米制点从源帧变换到目标帧，复用同一旋转运算。四元数模长与 1 的差须不超过 1e-9，非有限输入或变换结果返回空值。inverseTransformTfPoint 先扣除目标帧平移，再用同一四元数的逆旋转求源帧坐标，使用相同的数值约束。深度投影入口负责核对时间和坐标帧。

逐边在原父坐标系计算 T_parent_child。静态与精确样本直接使用，动态端点间使用线性平移与最短弧四元数插值。先插值，再对反向边求逆，最后按 source 到 target 的顺序左乘。旋转归一化只发生在派生计算中。TfTimeCheck 仍提供查询时间、坐标链、端点和来源；TfPoseCode 另行报告求解状态及失败边。

TfJumpLimits 的可选正上限分别为米与弧度，作用于插值端点差。端点选择遵循 TfTimeOptions 的同钟/同段和最大间隔，拒绝外推。跳变上限适用于两个端点之间的插值，不检查整个轨迹。

inferInterpolationGapNs 对所需链上的动态边去重，每条至少三个样本的边取相邻间隔下中位数的两倍，再取最大值作为该段共用门限。缺少足够样本时返回 0。该估计每段调用一次，逐帧只查询有序样本。多速率链共用同一门限，使用方可按实际采样策略指定显式上限。

核心回归使用构造样本核查坐标方向、反向、插值、跳变、时间缺口和来源。测试入口见 [TF 核心测试](tests/test_tf.md)。

2026-11-03 验证：无 ROS CTest 9/9，非录包 ROS CTest 14/14。ROS 测试使用构造的 TransformStamped 与 tf2 对照。安装后的 TF 消费者使用不同的 RGB/Depth 时间得到不同位姿，BagBuilder 消费者检查配置失败契约。按本轮要求未运行录包或 SQLite 录包夹具。
