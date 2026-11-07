# Preparation 组装元数据解析

实现日期：2026-11-06。src/bag/builder/preparation_metadata.hpp/.cpp 是 PreparationReader 的私有解析实现，不安装为公开头，不依赖 memory/observation、ROS 或模型服务。

公开返回类型仍位于 preparation_reader.hpp，包括 PreparedCalibration、PreparedMessageSummary、PreparedDepthStorage 和 PreparedPair 的新增字段。字段语义见 [reader](preparation_reader.md)。

## 职责

- 从同一份已读取 manifest 字节计算 SHA-256；索引 reader 打开时缓存数据集事实及两路绑定配置。
- 用 from_chars 和有界整数读取精确恢复差值、门限、序号、记录时间与摘要长度，拒绝浮点/溢出或非规范十进制字符串。
- 原样保存原始 CameraInfo、实际选择配置、匹配/规范化状态、错误信息和静态/同步时序结果。原 D8 不被派生 D5 覆盖。
- source_nonfinite=true 时，JSON null 已无法区分原 NaN/Inf，original 为空且记录不可恢复原因；消息证据与选择失败状态保留。即使原始数值不可恢复，必要头字段仍须满足保存格式。
- 对保存的 matched/ready 声明复用 CameraInfo 检查和 CameraModel 重建，核对状态、完整网格和 K/D/R/P；坏候选本身不因此被当成损坏文件。
- 单帧 storage 与源深度解释只解析一次，后续 DepthFrame 解码直接消费同一描述。
- 消息摘要解析不读取源 bag；CDR 路径/文件长度由 reader 的实际路径边界核对。

## 边界与错误

私有 PreparationMetadataError 携带既有 PreparationReadCode，由两个公开读取入口统一转换成失败结果，无部分 pair。结构/类型损坏为 invalid_metadata，引用或声明矛盾为 invalid_reference，实际存储解释矛盾为 invalid_media，哈希设施失败为 io_error。

历史无逐帧 storage 的米制 NPY 记录使用 legacy_metric_npy=true，后续实际读取核对 NPY 头和像素；这包括 v5 及其合法 v7 精简转换副本。未记录存储且无媒体的失败帧不会被猜成 NPY。

SHA-256 通过项目已有 OpenSSL EVP 设施实现，Crypto 为 reader 的私有依赖。核心构建也需要 libssl-dev；公开头不包含 OpenSSL 或 JSON。manifest 指纹只标识该文件的原始字节，不是整个 preparation 或源 bag 的完整性证明。

## 校验与复用

readPair 负责当前引用集合、路径、媒体和已保存位姿链检查；本解析实现不重新配对或重选标定。完整 TF 数值重算一致性继续由正式 Observation 校验负责。基于 reader 结果的 C++ 测试组装函数不读取任何来源 JSONL，独立 Python 对照仅用于测试。详见 [本轮验收与自查](../../reviews/preparation_reader_assembly_metadata_verification.md)。
