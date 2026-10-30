# 观测准备层公开接口手册

核查日期：2026-10-30。范围：当前 include/vln_memory/bag、src/bag、CMake 导出目标及安装消费者。本文按实际声明和实现编写；所有 C++ 类型/函数属于 vln_memory 命名空间。完整目标与审查结论分别见 [系统规划](../project/system_plan.md)、[本次核查报告](../reviews/observation_preparation_audit.md)。

当前项目验收仅覆盖原设计第 1、2 部分，范围对照见 [当前规划](../project/sections_1_2_scope.md)。本手册只描述其中已经实现的观测准备接口，不代表实体、关系、MemoryEngine 和 Viewer 已实现。

## 1. 能调用哪些接口

传感器接口继续处理已反序列化消息；新增 buildBag 可传入 bag 配置并输出 preparation 图像/JSONL。完整参数及文件契约见 [BagBuilder 接口](bag_builder_api.md)，它还不是正式 Observation 导出。

| 作用 | 公开头文件（include/vln_memory/bag/ 下） | 实现文件（src/bag/ 下） | CMake 目标（vln_memory:: 前缀） |
| --- | --- | --- | --- |
| 已解析话题格式 | common/sensor_names.hpp | header-only，无实现文件 | 各核心目标的公开头路径 |
| 共享时间/网格 | common/sensor_types.hpp | 无实现，纯数据 | 随各核心头文件提供 |
| 候选时间检查 | common/sensor_time.hpp | common/sensor_time.cpp | vln_memory_sensor_time |
| RGB 解码 | rgb/rgb.hpp | rgb/rgb.cpp | vln_memory_rgb |
| ROS RGB 适配 | rgb/rgb_ros.hpp | rgb/rgb_ros.cpp | vln_memory_rgb_ros |
| Depth 解码与注册声明核对 | depth/depth.hpp | depth/depth.cpp | vln_memory_depth |
| ROS Depth 适配 | depth/depth_ros.hpp | depth/depth_ros.cpp | vln_memory_depth_ros |
| 标定校验/图像关联 | camera_info/camera_info.hpp | camera_info/camera_info.cpp | vln_memory_camera_info |
| ROS CameraInfo 适配 | camera_info/camera_info_ros.hpp | camera_info/camera_info_ros.cpp | vln_memory_camera_info_ros |
| TF 历史索引/链/覆盖 | tf/tf.hpp | tf/tf.cpp | vln_memory_tf |
| ROS TF 接入 | tf/tf_ros.hpp | tf/tf_ros.cpp | vln_memory_tf_ros |
| RGB-D 自动配对 | builder/rgbd_sync.hpp | builder/rgbd_sync.cpp | vln_memory_rgbd_sync |
| 标定候选索引 | builder/camera_info_index.hpp | builder/camera_info_index.cpp | vln_memory_camera_info_index |
| rosbag 准备闭环 | builder/bag_builder.hpp | builder/bag_builder.cpp 等 | vln_memory_bag_builder |

来源路径示例：[RGB 头](../../include/vln_memory/bag/rgb/rgb.hpp)、[Depth 头](../../include/vln_memory/bag/depth/depth.hpp)、[CameraInfo 头](../../include/vln_memory/bag/camera_info/camera_info.hpp)、[TF 头](../../include/vln_memory/bag/tf/tf.hpp)。头文件声明接口；实现文件执行校验，不能把接口注释中的未来用途视为现有流程。

## 2. 通用格式、所有权及成功判断

| 项目 | 格式与约束 |
| --- | --- |
| SensorStamp | {int32_t sec, uint32_t nanosec}；nanosec<1000000000；负秒/零时刻合法；不是 UTC，不含时钟 ID |
| ImageGeometry | {uint32_t width,height; std::string frame_id}；拥有 frame 字符串，不含标定、时间或注册证明 |
| 图像布局 | 行优先；输入 step 为每行字节数，包含填充；data_size 必须精确等于 step*height |
| 输入图像视图 | const uint8_t* 与 string_view 借用；指针、长度、字符串必须在同步调用期间有效且不被修改 |
| 标准输出 | vector/string/array 拥有数据；源 ROS 消息释放后仍可使用。公开字段可变，修改后不能继续假定原校验仍成立 |
| 输入错误 | 返回类型化状态和原因；Rgb/Depth/CameraInfo 解码失败时 optional 数据为空 |
| 系统异常 | bad_alloc 等资源异常不转换为坏帧；TF 插入遇此类异常应丢弃本次构建索引 |
| 单位/旋转 | Depth 输出米制 optical-z；TF 平移约定米，四元数 xyzw；矩阵 row-major |
| 持久化 | 保存状态名称而非枚举序号；builder 已序列化 preparation JSONL；非有限原值为 null+标记，原始 CDR 保留证据，像素 NaN 保存在 NPY |

| 返回类型 | 正确成功判断 | 成功不代表什么 |
| --- | --- | --- |
| RgbResult / DepthResult / CameraInfoResult | ok() 后读取 frame / info | 不代表整条 Observation 可用 |
| SensorTimeCheck | matched() 后读取 delta_ns | 不证明硬件同步或已自动选择最近消息 |
| DepthAlignmentResult | alignedToRgb() | 不代表已做物理配准 |
| CameraInfoMatchResult | matched() | 不代表已生成有效裁剪内参或验证物理精度 |
| TfInsertResult | accepted()，code 区分 inserted/duplicate | 不代表整包 TF 完整 |
| TfChainReport | chainExists()，再查 coverage_status/intervals | 链存在不等于时间有交集 |
| TfTimeCheck | covered() | 不代表已经求出 map 位姿 |
| TfMessageReport | ok()；还可读 inserted/duplicates/issues | processed 单独不是成功，可能部分条目失败 |

默认构造的三个解码结果均无 frame/info，即便 code=none 也不是成功。不要统一写 code==none 代替上述谓词。

## 3. 公共名称与时间检查

[sensor_names.hpp](../../include/vln_memory/bag/common/sensor_names.hpp) 提供 header-only 的 `bool isResolvedTopicName(std::string_view topic) noexcept`，无分配、不保存输入引用。接受已解析的绝对 ASCII 话题名，拒绝空 token/空白/NUL/数字开头 token/相对名/超过 247 字节；不执行 remap，不证明消息类型或来源真实性。TF 核心、TF ROS binding、CameraInfo 共用，详细规则见 [同名说明](../common/sensor_names.md)。

### 时间检查

提供方：[sensor_time.hpp](../../include/vln_memory/bag/common/sensor_time.hpp)，实现：[sensor_time.cpp](../../src/bag/common/sensor_time.cpp)。

```cpp
SensorTimeCheck checkSensorTimes(
  const SensorStamp & reference, const SensorStamp & candidate,
  const SensorTimeOptions & options);
```

| 参数/结果 | 详细格式 |
| --- | --- |
| reference / candidate | 原始采样时刻，通常分别传 rgb.stamp、depth.stamp |
| options.clock_relation | SensorClockRelation::{unknown,same_clock,different_clocks}，默认 unknown；必须由采集配置证明 same_clock |
| options.max_delta_ns | int64_t 非负纳秒，默认 0 表示严格相等 |
| 返回 code | matched/outside_tolerance/invalid_stamp/invalid_tolerance/clock_unknown/clock_mismatch |
| 返回 delta_ns | optional<int64_t>，candidate-reference；合法同域时即使超差仍保留；未知/不同域或坏输入为空 |

不修改 stamp、不校正时钟、不选择消息、不建立同步队列。CameraInfo 逐帧检查也使用此函数，但 reference 是图像、candidate 是 CameraInfo。

## 4. RGB

提供方：[rgb.hpp](../../include/vln_memory/bag/rgb/rgb.hpp)，实现：[rgb.cpp](../../src/bag/rgb/rgb.cpp)。

```cpp
RgbResult decodeRgb(const RgbImageView & input, const RgbLimits & limits = {});
RgbResult decodeRosRgb(const sensor_msgs::msg::Image & image, const RgbLimits & limits = {});
```

后一个由 [rgb_ros.hpp](../../include/vln_memory/bag/rgb/rgb_ros.hpp) / [rgb_ros.cpp](../../src/bag/rgb/rgb_ros.cpp) 提供。

| RgbImageView 字段 | 格式/约束 |
| --- | --- |
| stamp / frame_id | SensorStamp / string_view；原样保留；空 frame 允许解码，但不能据此定位 |
| width,height | uint32_t，均>0 |
| encoding | string_view，仅 rgb8、bgr8、mono8 |
| step / is_bigendian | uint32_t 行字节数 / uint8_t 仅 0 或 1；8 位通道无需交换字节序 |
| data / data_size | const uint8_t* / size_t，必须是同一块有效缓冲区与长度 |
| limits | max_pixels 默认 16*1024*1024，max_input_bytes 默认 128*1024*1024，均>0 |

返回 RgbResult{RgbErrorCode code; string message; optional<RgbFrame> frame}。成功帧包含：

- stamp/frame_id/width/height：采样身份与尺寸；
- encoding 固定 rgb8，step=width*3，pixels 为 vector<uint8_t>，大小=step*height、逐像素 R,G,B；
- source_encoding/source_step/source_is_bigendian：输入格式依据。

不缩放、不裁剪、不保存 PNG；不支持 CompressedImage、Bayer、rgba8 等其他格式。ROS 重载只映射真实 Image 字段，不读取 bag 或订阅话题。

## 5. Depth

提供方：[depth.hpp](../../include/vln_memory/bag/depth/depth.hpp)，实现：[depth.cpp](../../src/bag/depth/depth.cpp)。ROS 入口为 [depth_ros.hpp](../../include/vln_memory/bag/depth/depth_ros.hpp) / [depth_ros.cpp](../../src/bag/depth/depth_ros.cpp)。

```cpp
DepthResult decodeDepth(
  const DepthImageView & input, const DepthOptions & options,
  const DepthLimits & limits = {});
DepthResult decodeRosDepth(
  const sensor_msgs::msg::Image & image, const DepthOptions & options,
  const DepthLimits & limits = {});
DepthAlignmentResult checkDepthRgbAlignment(
  const DepthFrame & depth, const ImageGeometry & rgb);
```

DepthImageView 与 RGB 视图拥有相同字段名称和基础类型，encoding 仅支持 16UC1（uint16）或 32FC1（IEEE float32）。允许大小端、奇数行填充和未对齐的输入地址。

| DepthOptions 字段 | 格式/含义 |
| --- | --- |
| input_unit | DepthUnit，必填 meters 或 millimeters；默认 unspecified 会失败，不按编码猜单位 |
| convention | DepthConvention，必填 optical_z；unspecified、radial_range 不支持 |
| alignment | DepthAlignment，直接填写 unknown / not_aligned / aligned_to_rgb |
| rgb_grid / alignment_basis | ImageGeometry / string，并列保存目标网格和注册依据 |
| min_depth_m / max_depth_m | double，min 有限且>=0，max>min；默认 0/+Inf，正有效值按闭区间过滤 |
| limits | 与 RGB 同名字段及相同默认上限；只限制本次转换 |

alignment=unknown/not_aligned 时 rgb_grid 必须为空。aligned_to_rgb 时，rgb_grid 必须与深度网格宽高/frame 相同，且 alignment_basis 非空；alignment_basis 来自注册流/设备配置，不能拿尺寸相同作依据。

返回 DepthResult{code,message,optional<DepthFrame> frame}。DepthFrame 持有：

| 字段 | 返回格式 |
| --- | --- |
| stamp,frame_id,width,height | 原始身份和网格 |
| encoding,step,unit,convention | 32FC1、width*4、meters、optical_z |
| depth_m | vector<float>，width*height，逐行紧密排列；无效值 quiet NaN，不是可直接写入 ROS Image 的字节缓冲区 |
| source_* | encoding、numeric_type、unit、scale_to_m、step、is_bigendian，防止重复换算 |
| alignment / rgb_grid / alignment_basis / min_depth_m / max_depth_m | 并列保存本次注册声明、目标网格、依据和过滤策略 |
| 扁平质量字段 | valid_pixels、invalid_pixels、zero_pixels、negative_pixels、nonfinite_pixels、out_of_range_pixels、unrepresentable_pixels 为 size_t；valid_ratio 为 double，均直接位于 DepthFrame |

零/负数/非有限/过滤外/不可表示像素分别统计。全无效图像也可 ok()，此时 valid_ratio=0；上层必须决定是否进入 pending。

checkDepthRgbAlignment 返回 {DepthAlignmentStatus status; string message}，核对声明与实际 RGB 元数据；unknown、not_aligned、尺寸或 frame 不符均不会返回 aligned。它不重投影、不检查像素/时间/标定。下游调用方式是 ImageGeometry{rgb.width,rgb.height,rgb.frame_id}。

## 6. CameraInfo

提供方：[camera_info.hpp](../../include/vln_memory/bag/camera_info/camera_info.hpp)，实现：[camera_info.cpp](../../src/bag/camera_info/camera_info.cpp)。ROS 入口：[camera_info_ros.hpp](../../include/vln_memory/bag/camera_info/camera_info_ros.hpp) / [camera_info_ros.cpp](../../src/bag/camera_info/camera_info_ros.cpp)。

```cpp
CameraInfoResult decodeCameraInfo(CameraInfoData input);
CameraInfoResult decodeRosCameraInfo(
  const sensor_msgs::msg::CameraInfo & message, std::string_view topic);
CameraInfoMatchResult checkCameraInfoImage(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options);
```

| CameraInfoData 字段 | 输入/输出格式 |
| --- | --- |
| stamp / topic / frame_id | SensorStamp / string / string；topic 为已解析绝对话题，由 bag 元数据提供；frame 无前导 /、无空白 |
| width,height | uint32_t 标定分辨率，不一定等于实际图像尺寸 |
| distortion_model / d | string / vector<double>；plumb_bob(5)、rational_polynomial(8)、equidistant(4)，系数数量精确匹配 |
| k,r,p | array<double,9>、array<double,9>、array<double,12>，row-major；全部有限 |
| binning_x,binning_y | uint32_t，0 按 1 解释，保留原值 |
| roi | CameraRoi{uint32_t x_offset,y_offset,width,height; bool do_rectify}，未 binning 的完整网格坐标 |

按值传入意味着左值复制、std::move 转移；结果 info 完全拥有数据。K[0]==0 返回 uncalibrated；要求正焦距、K/P 标准结构和有效 R。当前即便 raw 图像也要求完整有效 R/P；不能只填 K 并把 R/P 留零。检查是结构/数值校验，不证明物理标定精度。

返回 CameraInfoResult{CameraInfoErrorCode code; string message; optional<CameraInfoData> info}，输出保留完整原始模型，没有生成 effective K/P、去畸变映射或投影器。

| 关联参数 | 格式/约束 |
| --- | --- |
| image | CameraImageReference{string topic; uint32_t width,height; string frame_id; SensorStamp stamp}，逐项来自实际候选图像，不含像素 |
| options.camera_info_topic / image_topic / optical_frame / binding_basis | 四个 string，分别为标定话题、图像话题、光学 frame 和绑定依据，直接位于配置顶层 |
| options.image_domain | raw 或 rectified，默认 unspecified 不可用 |
| time_policy | synchronized（默认）或 static_calibration |
| options.clock_relation / max_delta_ns | SensorClockRelation / int64_t 纳秒；synchronized 要求同域与非负容差通过；static_calibration 不比较时差，binding_basis 必须说明固定段依据 |

匹配结果字段：code、message、calibration_code、expected_width/expected_height、optional<SensorTimeCheck> timing、used_static_calibration。若标定校验失败，calibration_code 说明具体原因；timing 只在真正运行时间检查时有值，符号为 CameraInfo-image。

尺寸由 ROI.width/bx、ROI.height/by 推出；默认零 ROI 用完整标定网格。当前要求整除，不猜舍入。rectified + roi.do_rectify=true 返回 unsupported_rectified_roi。raw K/P 原值仍保留，后续投影不能拿未调整的完整分辨率内参直接用于裁剪图像。

CameraInfoBinding 已移除，图像尺寸/frame 和匹配配置均直接访问顶层字段；旧版调用者应按 [迁移表](../modules/camera_info/camera_info.md#10-2026-10-29-可读性重构) 修改并重新编译。原始 ROI、采样时刻和结果诊断类型保留。

本接口不从多条 CameraInfo 自动选版本；动态有效区间和固定标定的配置证明由 builder 管理。

## 7. TF 核心

提供方：[tf.hpp](../../include/vln_memory/bag/tf/tf.hpp)，实现：[tf.cpp](../../src/bag/tf/tf.cpp)。

```cpp
explicit TfIndex(TfScope scope, TfLimits limits = {});
TfInsertResult TfIndex::insert(TfTransform transform);
TfChainReport TfIndex::inspectChain(
  const std::string & target, const std::string & source,
  const TfCoverageOptions & options) const;
TfTimeCheck TfIndex::checkTime(
  const std::string & target, const std::string & source,
  const SensorStamp & stamp, const TfTimeOptions & options) const;
```

| 输入结构 | 字段与约束 |
| --- | --- |
| TfScope | string clock_id,segment_id，必须非空；由采集配置声明，一个索引一段，不含数据段起止时间 |
| TfLimits | size_t max_edges=4096、max_samples=1000000；必须>0，构造函数存储配置，insert/查询时检查有效性 |
| TfTransform | TfKind kind；SensorStamp stamp；string parent_frame,child_frame；array<double,3> translation_m；array<double,4> rotation_xyzw；string source_topic,source_message_id；size_t source_transform_index |
| TfKind | dynamic 或 static_transform，不能从 stamp 是否为零推断 |
| TfTransform 来源字段 | string source_topic,source_message_id；size_t source_transform_index。source_topic 必须通过公共完整名称规则（绝对名称、合法 token、最长 247 字节）；source_message_id 非空。坏来源返回 missing_provenance，索引不变 |
| 几何 | parent!=child；有效 frame；有限平移；四元数 norm 与 1 差<=1e-3；保留原值 |
| TfCoverageOptions | int64_t max_interpolation_gap_ns，非负；默认 0 仅允许精确样本 |
| TfTimeOptions | string clock_id,segment_id；int64_t max_interpolation_gap_ns；查询必须与索引 clock/segment 相同，间隔非负 |

变换保存 T_parent_child，把 child 中的点变到 parent。target/source 为查询终点/起点，例如 target=map、source=rgb_optical。

insert 返回 {TfInsertCode code; string message}；accepted() 接受 inserted 或 duplicate。相同动态同刻/重复静态几何去重，q 与 -q 视为同旋转。父节点、静动态类型、静态几何、同刻动态几何冲突及环会阻断相关节点；以后查询祖先触及阻断节点会失败。普通坏输入不增加样本；遇内存异常应终止索引构建。

| TfChainReport 字段 | 结果格式 |
| --- | --- |
| code,message,clock_id,segment_id,target_frame,source_frame,failed_frame | 拓扑/配置状态、上下文与定位信息 |
| frames | vector<string>，从 source 到 target |
| edges | vector<TfChainEdge>；每边 parent_frame,child_frame,kind,inverted,sample_count,optional<TfTimeInterval> dynamic_bounds |
| inverted | source→target 路径沿 parent→child 时 true，几何计算时需取逆 |
| dynamic_bounds | 首末动态样本包络，可能有洞；静态为空 |
| coverage_status | not_evaluated/static_only/available/no_common_time |
| intervals | vector<TfTimeInterval>；每项 {SensorStamp start,end} 为有序闭区间，取所有动态边可插值区间的交集 |

静态/identity 链 static_only 且 intervals 为空，不代表无覆盖；动态链 no_common_time 则代表无公共时段。未见过的同名 source/target 不生成 identity。没有 latest 回退，实际零时刻按零处理。

checkTime 返回 TfTimeCheck：code,message,stamp,chain_code,failed_frame,frames，以及 vector<TfSelectedEdge> edges。每条选择包含 bool inverted、TfTransform before、optional<TfTransform> after；精确/静态只有 before，允许插值时有 before/after。失败时 edges 清空；当前没有插值后或组合后的 pose 字段。

附属接口：scope()→const TfScope&（依赖索引生命周期）、edgeCount()/sampleCount()→size_t（唯一计数）。所有返回报告拥有字符串和所选样本；先完成 insert 再并发只读查询，不能并发写入。TfIndex 是内存索引，没有磁盘索引或自动分段。

## 8. ROS TF 接入

提供方：[tf_ros.hpp](../../include/vln_memory/bag/tf/tf_ros.hpp)，实现：[tf_ros.cpp](../../src/bag/tf/tf_ros.cpp)。

```cpp
TfMessageReport ingestRosTf(
  TfIndex & index, const tf2_msgs::msg::TFMessage & message,
  std::string_view topic, std::string_view message_id,
  const TfTopicBinding & binding = {});
```

| 参数/结果 | 具体格式 |
| --- | --- |
| index | 当前段的可写索引，函数会插入有效条目 |
| message | 已反序列化 TFMessage；内部每条 TransformStamped 使用自己的 header.stamp |
| topic | 实际 bag 话题，必须与 binding 中一个话题精确匹配 |
| message_id | bag ID/稳定消息序号等来源标识，非空；ROS 消息本身不提供这个 ID |
| binding | TfTopicBinding{string dynamic_topic="/tf",static_topic="/tf_static"}；允许显式 remap，两个绝对话题须不同 |
| 返回 code | processed / invalid_binding / unexpected_topic / missing_message_id |
| inserted,duplicates | size_t，本消息的插入、重复数 |
| issues | vector<TfMessageIssue>；每项 transform_index、TfInsertCode code、message |
| entry_codes | vector<TfInsertCode>；按原 transforms 顺序保存每条插入/重复/拒绝 |

code=processed 可以有 issues，只有 ok() 才表示所有条目通过。消息允许部分成功，不是整包事务回滚；调用方不得忽略 issues。空消息是零条处理，不生成坐标边。完整话题类型校验/反序列化由 builder 的实际 bag reader 负责，TF 适配本身仍不打开 bag。

## 9. 名称转换接口

以下接口均接收对应枚举、返回 const char* 静态名称，noexcept，不需要释放；未知枚举返回 unknown。它们提供报告字段，不写任何文件。

| 提供头/实现模块 | 函数及参数类型 |
| --- | --- |
| rgb/rgb.hpp / rgb.cpp | rgbErrorCodeName(RgbErrorCode) |
| depth/depth.hpp / depth.cpp | depthErrorCodeName(DepthErrorCode)、depthUnitName(DepthUnit)、depthNumericTypeName(DepthNumericType)、depthAlignmentStatusName(DepthAlignmentStatus) |
| common/sensor_time.hpp / sensor_time.cpp | sensorTimeCodeName(SensorTimeCode) |
| camera_info/camera_info.hpp / camera_info.cpp | cameraInfoErrorCodeName(CameraInfoErrorCode)、cameraInfoMatchCodeName(CameraInfoMatchCode) |
| tf/tf.hpp / tf.cpp | tfInsertCodeName(TfInsertCode)、tfChainCodeName(TfChainCode)、tfCoverageStatusName(TfCoverageStatus)、tfTimeCodeName(TfTimeCode) |
| tf/tf_ros.hpp / tf_ros.cpp | tfMessageCodeName(TfMessageCode) |

## 10. 跨模块引用顺序与尚缺接口

仍可手工组合四模块；现在 buildBag 已自动完成 bag 读取、时间配对、两图各自标定选择/TF 原时刻检查、媒体和 preparation JSONL。详细流程见 [实际 BagBuilder 接口](bag_builder_api.md)。

完整 Observation 仍缺 TF 数值插值/求逆/组合、有效投影模型、关键帧及正式观测组装；断点恢复也未实现。已有入口为 buildBag 函数，不存在假定的 BagBuilder::build 类方法或 ObservationWriter。

builder 已为坏帧、无效深度/标定/TF 等分配 pending 与质量状态；未知时钟配置不能进行关联。单帧 RgbFrame/DepthFrame 本身仍不携带来源，BagMessageRef 统一维护 topic/message_id，BagBuildOptions 维护 clock/segment。map_id/geometry_revision 与正式观测几何仍需下一阶段落实，不能只凭解码 ok() 发布可定位观测。

## 11. 下游构建与最小调用示例

```cmake
cmake_minimum_required(VERSION 3.16)
project(memory_api_consumer LANGUAGES CXX)
find_package(vln_memory CONFIG REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE vln_memory::vln_memory_rgb
  vln_memory::vln_memory_depth vln_memory::vln_memory_sensor_time
  vln_memory::vln_memory_camera_info vln_memory::vln_memory_tf)
```

ROS 接入则链接所需 *_ros 目标，消息依赖随目标传递；仅包含 *_ros.hpp 不等于读取 rosbag。当前 find_package 不提供组件选择接口，不要假定 COMPONENTS bag/builder 等存在。

```cpp
#include <vln_memory/bag/rgb/rgb.hpp>
#include <vln_memory/bag/common/sensor_time.hpp>
#include <iostream>
#include <vector>

int main()
{
  using namespace vln_memory;
  const std::vector<std::uint8_t> bytes{30, 20, 10};
  RgbImageView input;
  input.stamp = {10, 0};
  input.frame_id = "rgb_optical";
  input.width = 1;
  input.height = 1;
  input.encoding = "bgr8";
  input.step = 3;
  input.data = bytes.data();
  input.data_size = bytes.size();
  const auto rgb = decodeRgb(input);
  if (!rgb.ok()) {std::cerr << rgb.message; return 1;}
  const auto timing = checkSensorTimes(rgb.frame->stamp, {10, 5000000},
    {SensorClockRelation::same_clock, 10000000});
  if (!timing.matched()) {return 1;}
  std::cout << static_cast<int>(rgb.frame->pixels[0]) << "; delta_ns=" << *timing.delta_ns << '\n';
  return 0;
}
```

更完整的现有调用实例：[RGB/Depth/标定组合](../../test/install_consumer/camera_info_consumer.cpp)、[TF/图像元数据](../../test/install_consumer/tf_consumer.cpp)、[ROS TF](../../test/install_consumer/tf_ros_consumer.cpp)。这些是接口消费者，不是完整 bag 处理器。模块细节见 [文档导航](../README.md)。

## 12. 枚举取值清单（按当前公开头核对）

下表同步自当前头文件；解释与限制以前文各接口为准。

| 枚举 | 当前取值 |
| --- | --- |
| CameraInfoErrorCode | none, invalid_topic, invalid_frame, invalid_stamp, invalid_dimensions, uncalibrated, nonfinite_parameters, unsupported_distortion_model, invalid_distortion, invalid_intrinsics, invalid_rectification, invalid_projection, invalid_roi, unsupported_binning |
| CameraImageDomain | unspecified, raw, rectified |
| CameraInfoTimePolicy | synchronized, static_calibration |
| CameraInfoMatchCode | matched, invalid_camera_info, invalid_options, invalid_image, topic_mismatch, frame_mismatch, size_mismatch, unsupported_rectified_roi, time_mismatch |
| SensorClockRelation | unknown, same_clock, different_clocks |
| SensorTimeCode | matched, outside_tolerance, invalid_stamp, invalid_tolerance, clock_unknown, clock_mismatch |
| DepthUnit | unspecified, meters, millimeters |
| DepthConvention | unspecified, optical_z, radial_range |
| DepthNumericType | unknown, uint16, float32 |
| DepthAlignment | unknown, not_aligned, aligned_to_rgb |
| DepthErrorCode | none, invalid_limits, invalid_dimensions, invalid_stamp, unsupported_encoding, invalid_endianness, invalid_step, size_overflow, size_limit_exceeded, invalid_data, missing_unit, unsupported_unit, missing_convention, unsupported_convention, invalid_range, invalid_alignment |
| DepthAlignmentStatus | unknown, not_aligned, aligned, invalid_reference, inconsistent_declaration, size_mismatch, frame_mismatch |
| RgbErrorCode | none, invalid_limits, invalid_dimensions, invalid_stamp, unsupported_encoding, invalid_endianness, invalid_step, size_overflow, size_limit_exceeded, invalid_data |
| TfKind | dynamic, static_transform |
| TfInsertCode | inserted, duplicate, invalid_scope, invalid_limits, invalid_kind, invalid_frame, self_transform, invalid_stamp, invalid_translation, invalid_quaternion, missing_provenance, limit_exceeded, parent_conflict, kind_conflict, static_conflict, duplicate_stamp_conflict, cycle_detected, edge_blocked |
| TfChainCode | connected, invalid_options, invalid_frame, unknown_frame, disconnected, topology_conflict |
| TfCoverageStatus | not_evaluated, static_only, available, no_common_time |
| TfTimeCode | covered, invalid_options, invalid_stamp, invalid_scope, clock_mismatch, segment_mismatch, chain_unavailable, outside_range, gap_too_large |
| TfMessageCode | processed, invalid_binding, unexpected_topic, missing_message_id |

## 13. TF 字段迁移（2026-10-30）

TF 来源和查询/报告上下文已改为顶层字段，TfSource 不再提供。完整迁移表与仍保留的语义类型见 [TF 说明](../modules/tf/tf.md#12-2026-10-30-可读性重构与字段迁移)。旧式 options={scope,coverage} 需改为逐字段填写 clock_id、segment_id、max_interpolation_gap_ns；调用方必须重新编译。

TF 适配器补充 entry_codes，和源 transforms 顺序一一对应，逐条保留插入/重复/拒绝状态；其他索引/冲突规则不变。
