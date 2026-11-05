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

`openPreparation` 校验 manifest、地图/时钟上下文，扫描配对、帧、标定、TF、消息五类 JSONL，保存 ID、二进制偏移和行长度。全局 JSON 格式、schema、必要目录字段或重复键错误返回失败及空 reader。合法空目录返回成功及空 `pairs()`。

`pairs()` 返回源 `rgbd_pairs.jsonl` 顺序的 `PreparationPairInfo` 目录，字段为配对 ID、两帧消息 ID、原始时间、时差、状态和 pending 原因。目录是打开时的摘要，其引用有效期与 reader 一致。帧关联及媒体完整性在 `loadPair` 时核验。

`loadPair(pair_id)` 按索引读取当前配对引用的记录，并复用单对入口的标定、位姿链、证据和媒体校验。当前实现采用完整深度读取契约。结果独立持有存储，reader 销毁后仍可使用。metadata_only 属于后续设计步骤，见 [分步方案](../../project/preparation_multi_pair_reading_plan.md)。

RGB 媒体校验相对路径和 PNG 头部尺寸、格式，返回绝对路径供消费端解码。NPY 支持 BagBuilder 写出的 v1.0 二维 C 顺序小端 float32，v5 的深度范围取自 manifest。v6 源 PNG 以单通道 uint16 读取，复用深度解码的比例、范围和无效值处理。CDR 路径与长度按版本和保存策略校验。v5/v6 与 v7 的 save_raw_cdr=true 要求文件存在，v7 false 要求 raw_path=null。rgb_raw_evidence 和 depth_raw_evidence 为可选路径。源消息字节对照由提供源 bag 的审计工具负责。

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
