# Observation 测试

test/memory/test_observation.cpp 是 Release 构建也执行的检查程序，目标 test_observation，CTest 名称 observation_core。安装后复用为 observation_consumer，仅链接 vln_memory::vln_memory_observation 与测试私有 JSON 库。

覆盖完整/缺项记录、D8 原值与有效模型、PNG16/NPY、静态与插值 TF、独立 RGB/Depth 时刻、原始消息引用、未核实声明及严格 JSON 往返。边界包括负/零/极端 int32 时间、int64/uint64 精度、错模型/位姿/端点/限制、路径越界、缺 pending、无效枚举、重复键、未知 schema/字段、浮点整数、矩阵长度、深度/字节上限及尾随数据。

test_observation_preparation 与 check_preparation_observations.py 为测试适配链路。Python 从原始 JSONL 保存完整原始标定、端点和来源，写正式记录；C++ 对正式记录校验/往返并逐对对照独立 PreparationReader。Python 再逐字段比较完整 JSON，并确认源元数据摘要未改变。测试声明的 map 上下文不构成物理配准验收。

ROS CTest observation_preparation 复用 bag_builder_dataset 夹具。核心构建同样提供该比较程序，可手动传真实 preparation。命令与实际结果见 [验收](../../../reviews/observation_verification.md)。
