# Preparation 配对读取

[公开头](../../../include/vln_memory/bag/builder/preparation_reader.hpp) 提供 `loadPreparedPair(root, pair_id)`，以及面向多对访问的 `openPreparation(root)` 和 `PreparationReader`。调用方链接 `vln_memory::vln_memory_preparation_reader`，ROS 与独立 C++ 构建均可使用。

输入为已发布的 BagBuilder v5/v6/v7 `bag_preparation/1`。两路图像保留各自原始采样时间、保存的配对关系和 pending 原因。结果独立持有元数据、可选模型、相机及底盘位姿、媒体路径和可选米制 `DepthFrame`。

多对调用先打开目录，再遍历轻量目录或按 ID 读取。

```cpp
auto opened = vln_memory::openPreparation(preparation_root);
if (!opened.ok()) {
  // 记录 opened.code 和 opened.message。
  return;
}

const auto & reader = *opened.reader;
for (const auto & info : reader.pairs()) {
  auto result = reader.loadPair(info.pair_id);
  if (!result.ok()) {
    // 记录该 pair_id 的读取错误，由消费端决定是否继续。
    continue;
  }
  // 使用 result.pair 中的元数据、位姿、路径及可选深度。
}
```

`openPreparation` 校验 manifest、地图/时钟上下文，扫描配对、帧、标定、TF、消息五类 JSONL，保存 ID、二进制偏移和行长度。全局 JSON 格式、schema、必要目录字段、重复键、超过 64 层或单行及 manifest 超过 16 MiB 时返回失败及空 reader。合法空目录返回成功及空 `pairs()`。

`pairs()` 返回源 `rgbd_pairs.jsonl` 顺序的 `PreparationPairInfo` 目录，字段为配对 ID、两帧消息 ID、原始时间、时差、状态和 pending 原因。目录是打开时的摘要，其引用有效期与 reader 一致。帧关联及媒体完整性在 `loadPair` 时核验。

`loadPair(pair_id)` 按索引读取当前配对引用的记录，并复用单对入口的标定、位姿链、证据和媒体校验。完整入口解码 RGB 以核验载荷，并恢复深度像素。结果独立持有存储，reader 销毁后仍可使用。只需组装元数据时可调用 loadPairMetadata。

RGB 媒体校验相对路径和 PNG 尺寸、格式与载荷，返回绝对路径供消费端按需解码。NPY 支持 BagBuilder 写出的 v1.0 二维 C 顺序小端 float32，v5 的深度范围取自 manifest。v6 源 PNG 以单通道 uint16 读取，复用深度解码的比例、范围和无效值处理。CDR 路径与长度按版本和保存策略校验，完整读取另核对落盘 CDR 的 SHA-256。v5/v6 与 v7 的 save_raw_cdr=true 要求文件存在，v7 false 要求 raw_path=null。rgb_raw_evidence 和 depth_raw_evidence 为可选路径。源消息字节对照由提供源 bag 的审计工具负责。

| 情况 | 结果 |
| --- | --- |
| 未知配对 ID | pair_not_found，pair 为空 |
| 断引用、错时刻位姿或路径越界 | invalid_reference，pair 为空 |
| 损坏的媒体或深度样本 | invalid_media，pair 为空 |
| 合法缺项或 pending 配对 | 读取成功，保留空字段和原始原因 |
| 打开后元数据路径、大小或修改时间变化 | source_changed，pair 为空，需重新打开 |

每次读取前后核对 manifest 和五类 JSONL 的文件信息。该检查适用于读取期间保持稳定的已发布数据集，相同大小且恢复修改时间的文件替换超出检查的保证范围。单对引用或媒体错误独立返回，其他配对仍可访问。

`loadPreparedPair(root, pair_id)` 保留原先按选中 ID 扫描 JSONL 的契约。新 reader 在打开时校验全局索引，因此无关记录的重复键或 schema 错误会使 open 失败。两种入口共享配对组装和校验逻辑，回归分别验证其错误范围。

设元数据字节数为 B、索引记录数为 N、一对引用的记录数及字节数为 Q/J、深度像素数为 D。在有界 ID 长度下，打开约为 O(B + N log N)，索引占用 O(N) 级空间。逐对读取约为 O(Q log N + J + D)，像素内存随调用方持有的读取结果增长。

[回归程序](../../../test/bag/builder/test_preparation_reader.cpp)复用现有录包夹具，覆盖全目录新旧入口对照、v5/v6/v7、PNG/NPY、源顺序及零/负/相同时间戳、空目录、重复键、合法缺项、断引用、坏媒体、路径/大小/修改时间变化和结果生命周期。CTest 的 preparation_reader 用例复用 bag_builder_ros 产物。

2026-11-05 的索引版本通过 ROS CTest 18/18、核心 CTest 10/10 和两种安装模式的读取器消费者。固定真实短包的两种媒体各有 10 对，全部通过新旧入口对照，并保留注册依据未核实的 pending 原因。真实短包的单点投影继续使用既有独立数值参考。复跑命令如下。

```bash
build/preparation-index-core/test_preparation_reader \
  build/depth-png-ab-20261104/metric_npy \
  build/depth-png-ab-20261104/source_png16 \
  build/depth-geometry-acceptance-20261104/run-1.json
```

2026-11-05 的 v7 CDR 存储策略及精简转换结果见 [验收](../../reviews/cdr_storage_verification.md)。下游需重新编译以适配可选 raw evidence 字段。

## 2026-11-06 组装所需元数据

PreparedPair 现统一返回以下拥有型信息，两个读取入口一致：

| 字段/类型 | 返回内容 |
| --- | --- |
| rgb_calibration / depth_calibration：PreparedCalibration | 原始 CameraInfo、标定/源消息 ID、CameraInfoMatchOptions、选择代码/消息、匹配错误码、静态复用与可选 timing、原始验证代码/消息、规范化代码/策略及标记 |
| sync_tolerance_ns | 当前配对实际门限；load 时核对 manifest、delta、原始时间、exact 和策略的一致性 |
| message_summaries | 图像、选中标定和实际引用 TF 记录的消息闭包，按 ID 去重，再按原 message_order / message_id 排序 |
| depth_storage：PreparedDepthStorage | format/dtype/unit/scale/invalid_rule、源 encoding/unit/scale、范围及质量计数；不含像素 |
| body_frame、tf_max_gap_ns、tf_jump_limits | 实际保存的底盘坐标系与 TF 限制，不由构建器猜测 |
| builder_version、source_manifest_sha256 | 来源版本与 manifest 原始字节 SHA-256；新 reader 打开时计算并缓存，不对每个 pair 重新读取/哈希 |
| rgb_media_relative / depth_media_relative | 经实际路径检查后的根目录内规范相对路径；旧的绝对媒体路径继续保留 |
| depth_alignment、depth_alignment_check、alignment_basis | 保留实际注册检查及依据；没有检查时保持 unknown/空值，不用全局配置猜成功 |
| capture_sync_basis | 仅保留来源实际记录的依据；历史 unknown 没有依据时返回空字符串 |

PreparedMessageSummary 包含 message_id/topic、file_index、topic_ordinal、message_order、bag_record_time_ns、serialized_bytes、payload_sha256 和可选 raw_path。raw_path 是检查后根目录内的规范相对路径。v7 省略 CDR 时摘要保留，raw_path=null；元数据读取核对摘要格式、引用及已有文件长度，完整读取可核对保存的 CDR 摘要。录包原件的独立对照由源 bag 审计负责。

没有标定候选时，selection 配置和失败状态仍返回，original 为空。原始 D8 与有效模型分别拥有，不互相覆盖；source_nonfinite 对应的 JSON null 不能忠实恢复 NaN/Inf，original 保持空并提供 original_unavailable_reason。原值可表达但语义坏的候选仍可保留为 pending。保存的 matched/ready 状态与原始参数、配置或有效模型矛盾时读取失败。

depth_storage 可以在媒体为空但来源明确保存 storage 说明时存在，它不证明文件存在。正式观测组装应在有 depth_media 时才映射文件存储引用。无 storage 的历史媒体采用 legacy_metric_npy 契约，实际 NPY 解码仍严格核对；v5 转换成 v7 后仍可使用该历史契约。32FC1 保持 NPY，不按全局 source_png16 配置猜测每帧文件格式。

门限/策略等单对语义矛盾只影响当前 loadPair；全局索引可打开并读取其他配对。所有返回值不借用 JSON DOM 或 reader 的内部指针，reader 销毁后结果可继续使用。公开结构增加了字段，二进制消费者必须重新编译。

核心构建新增对已有 OpenSSL Crypto 的私有链接依赖，用于 manifest SHA-256，需 libssl-dev；公开头没有 JSON/OpenSSL 类型。新解析实现与边界见 [preparation_metadata](preparation_metadata.md)，验证见 [本轮验收](../../reviews/preparation_reader_assembly_metadata_verification.md)。

`PreparationReader::loadPairMetadata(pair_id)` 保留完整的组装元数据、媒体路径、文件头与证据校验，`PreparedPair::depth` 留空。PNG16 模式检查头部格式，NPY 模式同时检查应有文件长度。需要实际深度值与像素计数校验时使用完整入口 `loadPair`。两种入口返回媒体和可选 CDR 的规范相对路径、大小及修改时间快照，可用 `verifyUnchanged` 连同索引来源复核。带媒体索引的产物还携带期望的媒体摘要。观测数据集用完整读取筛选，再用元数据入口组装并在发布前复核快照。

### 媒体完整性

新导出的 v7 preparation 通过 `bag_manifest.json` 中的 `media_integrity.index_sha256` 绑定 `media_sha256.jsonl`。每行保存原消息 ID、媒体根相对路径和文件 SHA-256。完整读取对 PNG/NPY 核对内容摘要，元数据读取核对绑定并把期望摘要交给发布前复查。读旧版或无摘要的 v7 保持兼容；可用 `preparation_compact` 发布带摘要的新副本。旧副本的摘要基线是转换时读取的媒体字节。新副本的 manifest 指纹会变化，正式观测应由新 preparation 重新发布。
