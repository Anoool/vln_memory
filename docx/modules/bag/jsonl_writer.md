# jsonl_writer.hpp / jsonl_writer.cpp

## 职责与入口

[头](../../../include/vln_memory/bag/builder/jsonl_writer.hpp)、[实现](../../../src/bag/builder/jsonl_writer.cpp) 只序列化，不选消息或计算几何。nlohmann_json 是 builder 的公开依赖，旧传感器核心不依赖它。

stampJson(SensorStamp) → sec/nanosec 对象；sourceJson(BagMessageRef) → 精确身份/路径/记录时间/摘要；cameraInfoJson(BagCameraInfoRecord) → 原始参数/错误；tfTimeJson(TfTimeCheck) → 查询状态和 before/after 源引用；issueJson(BagIssue) → 稳定问题字段。

writeJsonLine(OutputFiles&,relative_path,json object) 添加 schema_version=bag_preparation/1 并写单行；writeJsonFile 写完整 JSON 临时文件。对象必须有效，字符串由库转义，原始换行不能破坏 JSONL 分隔。

## 数值与引用

原始 sec/nanosec 保持整数分量；int64 记录时间/时差、uint64 来源序号写十进制字符串。相对路径定位外部 CDR/PNG/NPY，不把像素塞 JSON。

CameraInfo/TF 非有限原值写 null 并带 source_nonfinite；原始 CDR 保留实际位模式。Depth 像素 NaN 只在 NPY；无界 max_depth_m 的 JSON null 表示未设置上界。

tfTimeJson 的 covered 仅说明原始端点可用，明确 pose_computed=false。正式 Observation/实体/关系 schema 不由本文件提前伪造。

## 使用与验证

与同一 OutputFiles 共享预算，close 成功后才允许发布目录。独立 Python 读取并验证原始证据摘要、时间、引用和重复导出一致性，见 [输出验证](tests/test_bag_output.md)。

## 严格时间与压缩来源

当前 BagBuilder 写出的 pair 只含原始同刻两图，delta_ns 为字符串 "0"；summary/manifest 的同步状态与提取状态分开。frames 增加源消息类型/format，压缩源没有 source_step/source_is_bigendian，写 null；坏负载仍可 metadata_available=true。底层 stamp/整数序号序列化规则不变。
