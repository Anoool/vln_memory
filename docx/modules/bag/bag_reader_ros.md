# bag_reader_ros.hpp / bag_reader_ros.cpp

## 职责与调用

[头](../../../include/vln_memory/bag/builder/bag_reader_ros.hpp)、[实现](../../../src/bag/builder/bag_reader_ros.cpp) 使用官方 SequentialReader 读取未压缩 SQLite 目录（含分卷）或单独 .db3；显式支持 Image、白名单 CompressedImage、CameraInfo 和 TFMessage，绑定话题过滤。不是在线订阅/回放。

scanBag(options, OutputFiles&) → 拥有型 BagScan：源身份/文件/话题、全部来源消息、图像元数据、原始 CameraInfo、TF 索引、问题及统计。首遍将所有配置消息原始 CDR 保存到 evidence/raw；messages.jsonl 使空 TFMessage、坏 CDR 也有来源行。CameraInfo/TF 逐项写原始字段及校验/插入状态。

readBagImages(options, scan, BagImageConsumer) → void：重开、同约定扫描并核对每条来源/摘要，图像同步回调。Image 指针仅在回调期间有效；nullptr 表示坏 CDR 或压缩负载错误；查看 record 的两类错误字段区分，负载坏时保留真实 header。回调不能保存借用指针。输出立即落盘，像素不跨回调缓存。

verifyBagSources(options, scan) → void：复核 SHA-256、字节数、metadata，拒绝活动 SQLite 日志和输入变化。失败抛 BagBuildError，顶层不发布。

## 时间与来源

使用 header.stamp 配对/查询 TF；bag time_stamp 原样记录，不能称作曝光或独立发布时刻。metadata 原有分卷顺序保留，切换不保证时间递增；file/topic 查找表和当前文件缓存避免逐消息扫描/系统路径解析。

显式 seek 最小 int64 记录时间，避免默认零起点漏掉负记录。Humble MetadataIo 用 uint64 解码 starting_time，无法读回 Writer 写出的负值；SignedMetadataIo 只在内存副本中通过官方结构解析，再恢复根和分卷的有符号起点，不改输入。依据 [官方实现](https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage/src/rosbag2_storage/metadata_io.cpp)。

## 失败与预算

绑定缺失/类型错误、缺分卷、bag 级压缩/非 SQLite、超限、声明计数不符或来源改变均拒绝。坏 Image 可保留并继续；无法反序列化 CameraInfo/TF 时版本/历史边界不可信，整次失败。图像回退超过 max_header_reorder_ns 失败，不自动跨 epoch 排序。

第一遍仍完整反序列化 Image，临时有 data 分配；整体是两遍顺序 I/O，源文件指纹有额外读取成本。无真实设备录包验收承诺，磁盘测试用官方 Writer 构造。

框架审查后源文件完整 SHA-256 读取保留在初始身份、首遍结束、第二遍结束三处；第二遍开始用大小/日志/metadata 快检，过程中逐消息核对内容摘要。删除重复的整包 SHA 扫描，未降低最终完整输入校验。

## 压缩输入接入

首遍完整反序列化原消息，但只从压缩头读取布局，不分配解压像素；第二遍使用 [image_decoder_ros](image_decoder_ros.md) 解码后回调 Image。来源/CDR/原 header/format 保留，不以解码后的字节数覆盖原负载大小。未知格式和坏负载按帧 pending，不改配；已读取的 CDR 若第二遍不可读，仍按 source_changed 失败。

readBagImages 回调中 record.message_type 和 compressed_format 表示来源；encoding 表示交给核心的 Image 编码。像素和临时 record 引用都只在回调期间有效。真实版本验收见 [重跑记录](../../reviews/rgbd_zero_tolerance_verification.md)。

## 2026-11-03 CameraInfo 统一路径

reader 用 copyRosCameraInfo 保留原始参数，再用 normalizeCameraInfo 按来源相机的显式 compatibility 生成派生参数。严格原始检查与派生可用性分别导出；仅派生失败写不可用标定 issue。共享标定话题在配置阶段要求两路兼容策略一致。全部来源、原始 header 和 CDR 保持不变。
