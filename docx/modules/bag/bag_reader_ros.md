# bag_reader_ros.hpp / bag_reader_ros.cpp

## 职责与调用

[头](../../../include/vln_memory/bag/builder/bag_reader_ros.hpp)、[实现](../../../src/bag/builder/bag_reader_ros.cpp) 使用官方 SequentialReader 读取未压缩 SQLite 目录（含分卷）或单独 .db3；固定三种实际消息类型，绑定话题过滤。不是在线订阅/回放。

scanBag(options, OutputFiles&) → 拥有型 BagScan：源身份/文件/话题、全部来源消息、图像元数据、原始 CameraInfo、TF 索引、问题及统计。首遍将所有配置消息原始 CDR 保存到 evidence/raw；messages.jsonl 使空 TFMessage、坏 CDR 也有来源行。CameraInfo/TF 逐项写原始字段及校验/插入状态。

readBagImages(options, scan, BagImageConsumer) → void：重开、同约定扫描并核对每条来源/摘要，图像同步回调。Image 指针仅在回调期间有效；nullptr 表示首遍坏 CDR。回调不能保存借用指针。输出立即落盘，像素不跨回调缓存。

verifyBagSources(options, scan) → void：复核 SHA-256、字节数、metadata，拒绝活动 SQLite 日志和输入变化。失败抛 BagBuildError，顶层不发布。

## 时间与来源

使用 header.stamp 配对/查询 TF；bag time_stamp 原样记录，不能称作曝光或独立发布时刻。metadata 原有分卷顺序保留，切换不保证时间递增；file/topic 查找表和当前文件缓存避免逐消息扫描/系统路径解析。

显式 seek 最小 int64 记录时间，避免默认零起点漏掉负记录。Humble MetadataIo 用 uint64 解码 starting_time，无法读回 Writer 写出的负值；SignedMetadataIo 只在内存副本中通过官方结构解析，再恢复根和分卷的有符号起点，不改输入。依据 [官方实现](https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage/src/rosbag2_storage/metadata_io.cpp)。

## 失败与预算

绑定缺失/类型错误、缺分卷、压缩/非 SQLite、超限、声明计数不符或来源改变均拒绝。坏 Image 可保留并继续；无法反序列化 CameraInfo/TF 时版本/历史边界不可信，整次失败。图像回退超过 max_header_reorder_ns 失败，不自动跨 epoch 排序。

第一遍仍完整反序列化 Image，临时有 data 分配；整体是两遍顺序 I/O，源文件指纹有额外读取成本。无真实设备录包验收承诺，磁盘测试用官方 Writer 构造。

框架审查后源文件完整 SHA-256 读取保留在初始身份、首遍结束、第二遍结束三处；第二遍开始用大小/日志/metadata 快检，过程中逐消息核对内容摘要。删除重复的整包 SHA 扫描，未降低最终完整输入校验。
