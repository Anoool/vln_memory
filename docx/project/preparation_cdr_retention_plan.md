# Preparation 可省略 CDR 的存储方案

日期为 2026-11-05。状态为待审阅设计，本文中的配置和新接口均为拟议内容。

## 目标

按本次需求，提取后的 preparation 默认保存可消费的图像、深度和结构化记录，省略每条消息的 CDR 副本。来源 ID、原始时间、标定、TF 和配对关系继续可回查。

目前 [scanMessages](../../src/bag/builder/bag_reader_ros.cpp) 无条件写入 evidence/raw，并由 [sourceJson](../../src/bag/builder/jsonl_writer.cpp) 为消息及嵌套 source 生成 raw_path。[发布校验](../../src/bag/builder/bag_builder.cpp)与[配对读取](../../src/bag/builder/preparation_reader.cpp)均要求对应文件存在。需要同时调整这四处契约。

## 1. 导出策略

新增一个明确的布尔配置，建议默认 false。

```yaml
bag:
  save_raw_cdr: false
```

BagBuildOptions 提供同名布尔字段，YAML 严格识别类型，manifest.effective_options 记录实际值。true 对应保留 CDR 的导出模式，供需要原始消息审计的调用方显式选择。

在扫描阶段，原始序列化消息继续参与反序列化、长度预算和 SHA-256 计算。两遍读取的来源一致性检查继续使用消息 ID、原始时间、长度和摘要。

save_raw_cdr 为 false 时直接省略 CDR 写入，输出预算仅累计实际生成的文件。运行中仍会读取源 bag 的消息缓冲，缓冲按现有逐条处理方式释放。

## 2. 保留的文件和字段

| 产物 | 内容 |
| --- | --- |
| bag_manifest.json | 数据集身份、源 bag 指纹、话题类型、有效配置、时间/地图上下文和发布状态 |
| messages.jsonl | 每条消息的 message_id、topic、分卷和序号、记录时间、原始长度及摘要 |
| frames.jsonl | 两路图像原始时间、网格、模型、位姿、媒体路径及质量状态 |
| rgbd_pairs.jsonl | 已保存的 RGB-D 关联、时差、策略和 pending 原因 |
| camera_info.jsonl | 提取的 K/D/R/P、ROI/binning、标定时间及来源 |
| tf.jsonl | 提取的静态/动态变换、时间、原始条目身份及状态 |
| evidence/rgb 和 evidence/depth | 标准 RGB PNG 和深度 PNG/NPY |
| reports | 问题、pending 和数量统计 |

messages.jsonl 记录消息身份与来源信息。像素由媒体文件保存，标定和 TF 数值由对应 JSONL 保存，这些文件共同组成后续读取所需的 preparation。

原始 serialized_bytes 和 payload_sha256 在省略 CDR 时仍表示源消息的长度及指纹。它们可以在有源 bag 时用于对照，也用于构建期间的两遍一致性检查。

## 3. raw_path 与格式版本

新产物建议使用 builder_version=7，并继续采用 bag_preparation/1 的公共记录 schema。v7 明确要求 manifest.effective_options.save_raw_cdr，读取器按 builder_version 和该字段解释保存策略。旧读取器通过已有 builder_version 检查拒绝尚不支持的 v7。

省略 CDR 的消息记录示例如下。

```json
{
  "schema_version": "bag_preparation/1",
  "message_id": "m_example_f0_t0_0",
  "topic": "/camera/color/image_raw",
  "file_index": 0,
  "topic_ordinal": "0",
  "message_order": "0",
  "bag_record_time_ns": "1000000000",
  "serialized_bytes": 123456,
  "payload_sha256": "<源消息的 SHA-256>",
  "raw_path": null
}
```

true 模式中的 raw_path 继续为实际保存文件的相对路径。false 模式统一输出 null，明确表达省略了源字节副本。

frames、camera_info 和 tf 中的嵌套 source 也由统一序列化函数生成相同的 raw_path 状态。这样嵌套来源与 messages 行的字段一致性校验仍然有效。

manifest.files 按其现有文件清单含义核对实际产物，保存策略记录在 effective_options。相关容量统计按实际写入量更新。

## 4. 读取器调整

loadPreparedPair 与持有索引的 PreparationReader 共用以下规则。

| 数据集情况 | CDR 校验规则 |
| --- | --- |
| 历史 v5/v6 | 沿用原始契约，raw_path 必须指向存在且长度一致的文件 |
| v7，save_raw_cdr=true | 要求每条相关来源的 CDR 路径有效，长度一致 |
| v7，save_raw_cdr=false | 要求 raw_path=null，核对来源身份、消息引用及其他提取产物 |

合法省略和文件遗失分别处理。v5/v6 或声明保存 CDR 的 v7 数据若缺失文件，继续返回明确的引用错误。v7 保存策略缺项或类型错误属于元数据错误。

PreparedPair 中 rgb_raw_evidence 和 depth_raw_evidence 建议改为 optional<filesystem::path>。有实际 CDR 时返回路径，省略时为空。这是公开字段类型调整，需要同步 C++ 消费者及对应测试。

来源校验仍检查配对、帧、标定和 TF 引用是否属于同一数据集，嵌套 source 是否与 messages 对应。图像/深度路径、格式、模型、位姿、质量计数的校验继续沿用各自契约。

配对的 prepared_for_geometry 和 pending 原因由传感器与几何条件决定。合法省略 CDR 本身属于存储选择。

## 5. 已有数据集转换

已有数据可通过一次存储转换生成新目录，复用已经提取的 JSONL 和媒体。

1. 检查输入 manifest、JSONL 的引用关系及需要保留的媒体。
2. 在新建的 partial 目录复制 JSONL、媒体和报告。
3. 将 messages 与嵌套 source 中的 raw_path 统一转为 null。
4. 目标采用 v7 存储契约并声明 save_raw_cdr=false，同时记录转换操作、来源数据集及原 builder_version。
5. 核对实际文件清单和受影响的容量统计。
6. 使用新版 PreparationReader 校验目标数据的每个配对及引用。
7. 全部通过后原子发布新目录，返回转换统计与结果路径。

转换保留 bag_id、message_id、pair_id、原始时间、模型、位姿和 pending 原因。目标生成结果与原数据的几何读取结果应一致。处理失败时保留 partial 目录供排查。

源目录清理放在目标发布并核对之后，由明确的清理操作执行。转换阶段只读取源目录。

## 6. 验证与审计

实施时需要覆盖以下检查。

- 默认和显式 false 导出均产生完整 JSONL、媒体及报告，输出中没有 CDR 文件。
- true 模式能通过现有原始消息长度与 SHA-256 对照。
- 同一录包两种导出模式的消息/配对 ID、时间、标定、位姿、pending 和深度值一致。
- 单对及索引读取器均能消费省略 CDR 的 v7。
- v5/v6 与 true 模式中的缺失 CDR 仍导致引用错误。
- raw_path 状态与 manifest 策略矛盾时明确失败。
- 原始字节长度上限继续在反序列化前生效，输出容量按实际文件计算。
- 已有数据转换失败时目标不会被发布，重试使用新目标目录。
- C++ 安装消费者适配可选 raw evidence 路径，两种构建模式均验证。

test_bag_output.py 与 audit_camera_dataset.py 目前依赖 CDR 回读，需要分别提供提取产物一致性检查和源消息审计路径。省略 CDR 的数据集可以在提供源 bag 时独立重新读取并校验消息字节、标定和像素。审计报告应明确实际执行的检查范围。

省略 CDR 后，preparation 可以独立提供已提取的媒体与结构化信息。原始 ROS 消息恢复、重放、坏消息字节分析及原始非有限浮点位模式对照需要源 bag。JSON 中已有的 null 和错误状态继续表达提取结果。

## 7. 实施顺序

| 顺序 | 工作 |
| --- | --- |
| 1 | 增加保存配置、v7 版本契约和统一 nullable raw_path 序列化 |
| 2 | 调整扫描写入、发布前引用校验及实际输出容量统计 |
| 3 | 同步两个读取入口与 PreparedPair 可选证据字段 |
| 4 | 增加 true/false、旧格式、矛盾状态和结果一致性回归 |
| 5 | 提供已有数据转换入口，完成目标校验和原子发布 |
| 6 | 更新配置样例、接口、审计说明和 README |

本方案的默认策略依据本次需求设为 false。审阅通过后先落地新导出与读取，再完成已有数据转换。
