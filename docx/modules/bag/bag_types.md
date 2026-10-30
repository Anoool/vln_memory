# bag_types.hpp

## 文件职责

[源码](../../../include/vln_memory/bag/builder/bag_types.hpp) 提供无 ROS 的共享身份/元数据/错误/运行结果。内联小函数没有独立 cpp；相机、TF、配对专有类型仍留在各自头文件。

## 类型与格式

- BagMessageRef：message_id/topic；file_index；topic_ordinal、message_order（uint64）；bag_record_time_ns（int64）；serialized_bytes；payload_sha256。ID 包含输入内容身份、分卷、稳定话题索引及该分卷同话题序号。不是设备发布序号。
- BagImageRecord：source、原始 SensorStamp、frame_id、width/height/encoding/step/is_bigendian/data_bytes、deserialization_error；不含像素。反序列化失败的默认元数据不能当原始值，JSON 的 metadata_available=false、stamp=null。
- BagSourceFile/BagTopicRecord：实际路径/字节/SHA-256；话题类型/序列化/QoS、声明与实际扫描数量、是否选择。
- BagIssue：stage/code/message/message_id/pair_id，标识可空，机器按 code 处理。
- BagBuildError：继承 runtime_error，code() 给稳定失败码；底层失败向顶层传播。
- BagBuildResult：status/code/message、output_directory/work_directory、图像/解码/配对/准备/问题数量。ok() 只表示完整发布（可带问题），不代表正式 Observation 或几何有效。

## 所有权和使用

string/vector 均拥有数据；SourceStamp 为整数原值。buildBag 返回结果不携带整包像素。失败工作目录保留供检查，不是发布目录。详细调用见 [BagBuilder 接口](../../interfaces/bag_builder_api.md)。
