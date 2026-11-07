# PreparationReader 组装元数据：验收与实现后自查

日期：2026-11-06。分支：codex/preparation-assembly-metadata。按已讨论方案先实现，再检查完整调用链及针对性回归。修改前快照位于 build/reader-metadata-baseline-9cs5h7m9，用于区分本轮修改与此前已暂存工作。

## 实际交付

- 扩展 PreparedPair，新增 PreparedCalibration、PreparedMessageSummary、PreparedDepthStorage；原有字段与两个入口签名保留。
- 返回原始标定、配置/实际选择与时间匹配、规范化状态、门限、消息闭包、深度存储解释、body/TF 限制、manifest SHA-256、注册检查及规范相对路径。
- 新旧入口共用 readPair 和私有 preparation_metadata；没有 reader→memory/ROS 依赖，也没有生产组装器中的第二套源 JSONL 解析。
- 默认仍完整恢复深度；metadata_only 不包含在本轮。公开 C++ 数据布局扩展，下游必须重新编译。

## 自查发现、修正与针对性验证

| 检查项 | 修正 | 证据 |
| --- | --- | --- |
| 单对错误隔离 | 把门限/策略/实际时差的语义检查放回 loadPair；open 只建立全局格式/键和目录索引 | 第一对矛盾时仍可打开并读取另一对，失败 pair 为空 |
| 索引开销 | 数据集配置/manifest 哈希缓存一次，目录项不复制整份两路相机配置；同一消息引用只做一次文件检查 | 源序与两入口完整返回一致，读取前后变更检测保持 |
| 原始数据与 ready 模型 | 拒绝定长矩阵多余元素、错误规范化状态与篡改参数；从原始标定按保存配置重建 | 参数篡改、数组长度、normalized_code 矛盾均拒绝；D8 原值及模型分别保留 |
| 非有限源标定 | original 为空并保留不可恢复原因，仍检查必要头字段，不用 null 填默认矩阵 | 合法坏候选可读，丢失 stamp 的坏格式明确失败 |
| 选择参数/时序 | 无候选仍返回绑定配置及失败状态；同步/静态实际结果分别保存并核对 | 同步 delta=0 正向通过，改成 1ns 的匹配结果被拒绝 |
| 路径边界 | 增加嵌入 NUL 拒绝，规范相对路径与原绝对路径对应 | NUL 路径失败，媒体路径及消息 raw_path 与实际文件对应 |
| 层间 pending 兼容 | 正式 Observation 允许已有 Depth 契约中的 not_aligned+空 basis；正面 aligned 声明仍要求依据/网格 | 新增负注册状态往返与缺正面依据拒绝测试 |

返回值的字符串、原始模型/参数、摘要及路径均独立拥有；销毁 reader 后继续访问新增数据的测试通过。审查了边界、溢出、类型、生命周期、引用闭包、错误范围和扫描/复制开销；本轮范围内未发现尚未修复的阻塞项。

## 软件与 SDK 验证

- 无 ROS 核心 CTest 12/12；完整 reader 夹具检查也在核心二进制运行通过。
- ROS CTest 22/22，包括录包生成、旧 reader/压缩/NumPy/适配、构建输入及正式观测测试。
- 两种安装模式的 preparation_reader_consumer、observation_consumer 和 observation_preparation_consumer 均编译/运行通过。
- ldd 确认 reader 私有依赖 Crypto，没有 vln_memory_observation 或 rclcpp 依赖，无缺失库。
- 编译日志未出现新增警告；格式检查通过。

日志：build/reader-metadata-core、build/reader-metadata-ros 下的 ctest.log、reader-fixture.log、consumer-fixture.log 及 consumer-build.log。完整安装/数据对照见 build/reader-metadata-acceptance-20261106-7hsl8q1l/acceptance.json。

## 真实短包、历史兼容与类型组装

| 输入 | 核心安装消费者 | ROS 安装消费者 |
| --- | --- | --- |
| 原始 v6 米制 NPY | 10 对通过 | 10 对通过 |
| 原始 v6 源 PNG16 | 10 对通过 | 10 对通过 |
| 原始 v7 精简源 PNG16 | 10 对通过 | 10 对通过 |
| 从真实 v6 NPY 构造的 v5 兼容副本 | 10 对通过 | 10 对通过 |
| 通过转换接口新发布的 v5→v7 无逐帧 storage 副本 | 10 对通过 | 10 对通过 |

以上为同一固定真实短包的存储/兼容表示，不是 100 对独立采集样本。v5 是兼容副本，不称为原生历史 v5。最终转换副本明确使用成功发布路径，排除回归中故意留下的损坏 partial 目录。

test_observation_preparation.cpp 的 assembleForCheck 只读取 PreparedPair 与外部地图业务上下文；不读取来源 JSONL。每个正式记录的规范 JSON 与独立 Python 对照逐字段一致，包含原始 D8、选择参数、精确大整数、消息闭包、TF/媒体/存储、门限及来源指纹。源 metadata 的运行前后 SHA-256 保持一致。原注册/同步 pending 没有被提升。

两种安装 reader 均复跑既有真实短包投影数值参考，结果保持原验收阈值。这里验证软件提取、恢复、内部数据一致性；没有重新完成物理尺度、注册或曝光同步实测。

## 复跑与边界

```bash
ctest --test-dir build/reader-metadata-core --output-on-failure
ctest --test-dir build/reader-metadata-ros --output-on-failure
build/reader-metadata-consumer-core/preparation_reader_consumer build/reader-metadata-ros/test_artifacts
python3 test/memory/check_preparation_observations.py \
  --binary build/reader-metadata-consumer-core/observation_preparation_consumer \
  --input build/depth-png-ab-20261104/metric_npy \
  --input build/depth-png-ab-20261104/source_png16 \
  --input build/cdr-real-check-mpttQq/export_source_png16 \
  --output-parent build/reader-metadata-rerun
```

manifest 指纹标识该文件原始字节，不是整个 preparation 或源消息内容的完整性证明；文件变化检测继续遵循既有路径/大小/修改时间保证。源 bag 内容对照和物理质量仍需独立审计。生产 ObservationBuilder、完整数据集发布继续为后续工作。
