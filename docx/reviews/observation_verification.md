# 正式 Observation 模块验收与自查结论

日期：2026-11-06。实现分支：codex/observation-module。按用户要求先完成 [实施方案与自查](../project/observation_implementation_plan.md)，随后搭建记录/校验/JSON 模块。

## 实际交付

新增正式 Observation、ObservationCalibration、ObservationDepthStorage、ObservationEvidence 顶层类型。提供稳定来源身份、pending 汇总、内部一致性校验和 observation/1 严格 JSON 编解码。复用现有 CameraModel/TF 求解，记录不持有图像像素；库无 builder/ROS 链接依赖。

本轮未实现生产 engine/observation_builder、文件级完整版本发布或物理配准。测试适配脚本仅生成验收用记录，地图上下文明确为测试声明。

## 实现后边界复核及修正

| 发现 | 修正及验证 |
| --- | --- |
| pair_id 只基于 RGB 不能区分 Depth 改配 | 稳定 ID 包含实际 RGB/Depth 消息和时钟段；几何/记录 revision 变化不改身份，元组边界有回归 |
| 存储与兼容策略的既有词汇必须精确保留 | 使用 PNG 的 zero_or_out_of_range_to_nan 与 D8 的 plumb_bob_zero_tail_d8_v1；真实 D8/PNG16 往返一致 |
| 静态标定的共享检查不使用部分时间选项 | 正式记录额外拒绝未知 clock_relation 枚举与负容差，避免 JSON 静默归一 |
| 同 frame 恒等位姿无 TF 边 | 明确检查恒等平移/旋转，覆盖合法恒等及微小非恒等旋转拒绝 |
| 正向夹具暴露上一轮 build_input 状态判断错误 | 实际 BagBuilder 状态为 prepared_for_geometry；修正 observation 与 build_input 消费处，增加已就绪配对必须入选的正向断言 |
| 真正的文件/物理质量不能由结构合法推断 | 保留 source hash、摘要和原始声明/pending；外部 reader 比较媒体/源记录，物理质量继续标为未验收 |

## 软件与安装验证

- 核心 CTest 12/12，日志 build/observation-core/ctest.log。
- ROS CTest 22/22，日志 build/observation-ros/ctest.log；包括 observation_core、preparation 适配、原有录包/输入/reader/NumPy 和适配器测试。
- 核心和 ROS 安装后的 observation_consumer 均通过，头文件/链接仅通过安装导出目标获取。
- 两种安装后的 observation_preparation_consumer 分别通过下表全部数据对照。
- ldd 无缺失库，正式观测库没有 vln_memory_bag_builder 或 rclcpp 依赖；详见验收目录的 core-ldd.log、ros-ldd.log。
- 就绪配对正向夹具 2 对全部选中；真实 pending 配对保持 pending。

边界覆盖：完整/缺项往返；D8、ROI/binning、静态标定及模型篡改；静/动态 TF、独立查询时间、端点/数值/限制篡改；PNG16/NPY 单位与范围；消息断引用；缺 pending；负/零/极端时间及 int64/uint64 精度；NaN/Inf、路径越界、未知枚举/字段/schema、重复键、浮点整数、矩阵长度、解析深度/字节上限和尾随数据。

## 固定真实短包与历史兼容

完整记录与日志位于 build/observation-acceptance-20261106-54mmphn2/acceptance.json；各模式输出包含 observations.jsonl、roundtrip.jsonl、report.json 和逐数据集比较日志。

| 来源 | 核心安装消费者 | ROS 安装消费者 | pending |
| --- | --- | --- | --- |
| 原始 v6 米制 NPY | 10 对通过 | 10 对通过 | 10 |
| 原始 v6 源 PNG16 | 10 对通过 | 10 对通过 | 10 |
| 原始 v7 精简源 PNG16 | 10 对通过 | 10 对通过 | 10 |
| 基于真实 v6 NPY 构造的 v5 兼容副本 | 10 对通过 | 10 对通过 | 10 |

这是同一固定短包的不同存储/兼容表示，不是 80 对独立真实样本。v5 为独立兼容副本：调整 builder_version、移除新存储字段，保留原媒体/来源；不将其描述为历史原生 v5 录包。

Python 从原始 JSONL 独立保留完整标定、TF 端点、源摘要和声明，C++ 严格解析并与现有 reader 逐对比较，随后比较完整 JSON 往返。所有源元数据 SHA-256 在运行前后保持不变。真实输入的注册待核实和 unknown 曝光同步没有被提升；另保留 physical_geometry_unverified 测试原因。物理尺度/配准/同步和部署精度仍需外部实测。

## 复跑

```bash
ctest --test-dir build/observation-core --output-on-failure
ctest --test-dir build/observation-ros --output-on-failure
python3 test/memory/check_preparation_observations.py \
  --binary build/observation-consumer-core/observation_preparation_consumer \
  --input build/depth-png-ab-20261104/metric_npy \
  --input build/depth-png-ab-20261104/source_png16 \
  --input build/cdr-real-check-mpttQq/export_source_png16 \
  --output-parent build/observation-real-rerun
```

该脚本每次写入新的独立目录，创建任何输出前先拒绝位于输入数据集内的输出父目录。合法空文件也允许往返，是否有可用观测由下游检查。
