# 正式 Observation 模块实施方案与实现前自查

日期：2026-11-05。用户要求先列方案、自查，再搭建 memory/observation。本文件在实现前写入；后续验收独立记录。

## 1. 本轮交付

新增 include/vln_memory/memory/observation.hpp、src/memory/observation.cpp 与 observation_json.cpp。实现拥有型正式记录、稳定身份、内部一致性校验、pending 汇总和 observation/1 JSON 严格编解码。新增 test/memory 回归、核心/ROS 构建与安装消费者，以及同名模块文档。模块依赖现有相机/深度/TF 基础和私有 JSON 库，无 ROS、builder 或模型服务链接依赖。

Observation 包含来源 bag/pair/两消息/时钟段及依据、source_manifest_sha256、显式 map_id/geometry_revision/map_frame/map_basis、记录 revision、RGB 参考时刻与各自图像网格、配对门限/差值/策略、媒体相对路径及深度存储解释、完整原始标定与选择参数/有效模型、各自时刻的相机/底盘位姿及原始 TF 端点、源消息摘要及可选 CDR 路径、注册/同步声明与依据和 pending。

geometry_revision 为非空不透明字符串，revision 为正 uint64 十进制字符串，schema 为 observation/1；几何采用米、xyzw 四元数和 optical-z。原始两路时间仍保留整数 sec/nanosec。

## 2. 输入输出与边界

- makeObservationId：根据 bag/pair/RGB 消息/Depth 消息/clock/segment 的长度前缀元组生成确定 ID。无需新增哈希依赖，不受地图或几何 revision 变化影响；ID 为不透明键，不作为文件路径。
- collectObservationPendingReasons：保留来源原因并补充缺媒体、模型、位姿、无有效深度和未核实状态等规范原因，不修改图像时间、配对或几何。
- validateObservation：返回稳定错误码及说明；检查结构、数值、引用与重算一致性，不修改输入。
- serializeObservation / deserializeObservation：仅处理一条正式记录；失败无部分对象/JSON，整数精度不经过浮点，固定 schema 严格拒绝未知字段与重复键。
- JSON 可逐条写入 JSONL；批次组装、文件发布、媒体解码及源 bag/CDR 实物审计由后续消费/发布模块完成。本轮不创建空 engine/observation_builder，不修改现有 preparation 格式或选择行为。

媒体路径相对于 source_manifest_sha256 标识的数据集根，由调用方显式解析。结构校验拒绝绝对路径、跨平台盘符、反斜线和 . / .. 路径组件；它不以文件存在代替内容/物理证据。

## 3. 实现前自查与决策

| 检查项 | 发现及采用的处理 |
| --- | --- |
| 配对身份 | pair_id 可能仅基于 RGB；ID 加入实际 Depth/RGB 消息和时钟段，防止不同配对配置混淆。几何更新保留观测身份。 |
| 时间 | stamp 固定为 RGB；Depth 位姿用 Depth 时刻，body 用 RGB。复用 checkSensorTimes 核对有符号差值和门限。 |
| 媒体语义 | PNG16 与米制 NPY 不可都声明文件单位为米；分别保存 format/dtype/unit/scale/invalid_rule、源单位及有效范围/计数，运行时仍米制 optical-z。 |
| 模型 | 同网格不等于注册；用原始 CameraInfo 与匹配参数重新 buildCameraModel，对比保存的有效模型。原 D8 不改写。 |
| TF | 仅有数值位姿不足；保留完整端点/源 ID，以 TfIndex 和显式间隔/跳变限制重新求解，检查链/查询时刻/数值。 |
| 缺项与质量 | null 是合法 pending；要求缺项对应原因，解析成功不提升来源状态，不把 unknown 同步或注册变成 verified。 |
| 证据 | 消息摘要保留文件索引/顺序/记录时间/长度/SHA-256/可选 CDR；所有模型和 TF 引用须存在且话题一致。摘要格式正确不证明源文件真实。 |
| 外部校验 | 本模块处理内部一致性；后续组装器继续借助 reader/独立源审计核验媒体与消息，不把形式校验写成真实物理验收。 |
| JSON 边界 | 固定字段、严格数值类型、十进制字符串、版本检查、重复键和深度/大小上限；写入前验证，拒绝非有限数值。 |
| 依赖与开销 | 复用现有几何求解，不复制像素，不引入 ORM/服务或通用工厂。单记录校验使用消息 ID 索引，避免每个端点扫描所有证据。 |

结论：按上述收敛方案实施。现有 input 清单与原始 preparation 保持有效；本轮完成正式记录模块，批量组装仍是后续任务。

## 4. 验收计划

1. 完整、合法 pending、零/负/近似时间、uint64/int64 边界的 JSON 往返。
2. 错时刻/网格/标定、TF 端点/数值篡改、断引用、错误深度单位/比例、路径越界、非有限值、未知 schema/字段/重复键拒绝。
3. 同输入 ID 稳定、不同 Depth/时钟段 ID 不同、几何 revision 更新 ID 不变。
4. 真实 v5/v6/v7 preparation 的适配测试逐对保留时间、模型、TF、媒体和 pending；记录测试用 map 上下文为显式测试声明，物理依据仍 pending。
5. 核心/ROS 回归与安装后的独立消费者；保留新验收产物及完整日志，不修改真实输入。

## 5. 实现后复核（2026-11-06）

核心 12/12、ROS 22/22 及两种安装消费者均通过。真实 v6/v7 三种表示和独立 v5 兼容副本完成往返。正向夹具发现并修复旧 build_input 误用 prepared 状态的错误，实际 producer 值为 prepared_for_geometry；已补正向入选断言。另收紧静态标定枚举、恒等位姿分支及测试输出路径边界。完整证据见 [验收](../reviews/observation_verification.md)。
