# BagBuilder 接口与文件契约（可配置最近配对版本）

更新：2026-11-03，builder_version=5。以下为实际已实现接口，命名空间均为 vln_memory。[实施记录](../project/bag_builder_v1_implementation.md) 说明范围与实际验证；[原方案](../project/rosbag_integration_plan.md) 保留设计依据。

## 1. 从哪里调用

| 接口 | 提供文件（include/vln_memory/bag/builder/） | 参数/返回 | 用途 |
| --- | --- | --- | --- |
| buildBag | bag_builder.hpp；实现 src/bag/builder/bag_builder.cpp | const BagBuildOptions& → BagBuildResult | 完整两遍提取/发布，顶层错误转结构化结果 |
| loadBagBuildOptions | bag_config.hpp/.cpp | YAML 路径 string → BagBuildOptions | 严格加载；错误抛 BagBuildError |
| validateBagBuildOptions | bag_config.hpp/.cpp | const options& → vector<string> | 无 I/O 汇总配置问题 |
| pairRgbDepth | rgbd_sync.hpp/.cpp | 两个 RgbdStreamIndex、RgbdSyncOptions → RgbdSyncResult | 无 ROS 同段时间配对 |
| CameraInfoIndex::select | camera_info_index.hpp/.cpp | CameraImageReference、CameraInfoMatchOptions → CameraInfoSelection | 借用索引，选择及最终匹配 |
| scanBag | bag_reader_ros.hpp/.cpp | options、OutputFiles& → BagScan | 第一遍/原始证据/完整轻量索引 |
| readBagImages | bag_reader_ros.hpp/.cpp | options、const BagScan&、BagImageConsumer → void | 第二遍，回调只临时借用 Image |
| verifyBagSources | bag_reader_ros.hpp/.cpp | options、const BagScan& → void | 文件/metadata/日志及跨遍内容校验 |
| writeRgb/writeDepth | media_writer.hpp/.cpp | OutputFiles&、安全 ID、拥有型标准帧 → string 相对路径 | PNG/米制 NPY |
| writeJsonLine/writeJsonFile | jsonl_writer.hpp/.cpp | OutputFiles&、相对路径、json object → void | 带 schema_version 的 JSON 输出 |

bag_types.hpp 为 header-only 共享身份/错误/结果；CLI 是 src/apps/bag_builder_main.cpp。除 buildBag 外的底层 I/O/配置入口可抛 BagBuildError/标准异常；只有 buildBag 统一返回失败状态。

## 2. 链接与最小使用

```cmake
find_package(vln_memory REQUIRED)
target_link_libraries(your_target PRIVATE vln_memory::vln_memory_bag_builder)
```

```cpp
#include <vln_memory/bag/builder/bag_builder.hpp>

/** @brief 从明确配置提取准备数据；真实服务应记录 result 的错误和质量计数。 */
int main(int argc, char ** argv)
{
  if (argc != 2) {return 2;}
  const auto options = vln_memory::loadBagBuildOptions(argv[1]);
  const auto result = vln_memory::buildBag(options);
  if (!result.ok()) {return 1;}
  return 0;
}
```

YAML 需按 [配置说明](../config/vln_memory.md) 填真实绑定/单位/时钟依据。接口返回 completed_with_issues 时仍可消费完整准备数据，但需检查每条质量/状态。

独立核心链接 vln_memory::vln_memory_rgbd_sync 或 vln_memory::vln_memory_camera_info_index，不需要 ROS/YAML/JSON。BagBuilder 仅 ROS 构建导出，公开传递消息、rosbag2/rclcpp 及 JSON 所需依赖。

## 3. 输入与所有权

| 参数 | 格式与约束 |
| --- | --- |
| bag_uri | 未压缩 SQLite ROS 2 目录（metadata.yaml + 所有分卷）或独立 .db3；只读关闭状态，不可边录边提取；发布当前支持 Linux/Ubuntu/WSL |
| output_directory | 不存在的新目录；不能与输入路径包含彼此；失败只保留独立 .partial 工作目录 |
| clock/segment | 单个明确同钟段和采集依据。两路 header 须已处于同一可比较时间基准，最大重排量由 max_header_reorder_ns 声明 |
| rgb_camera/depth_camera | CameraInfoMatchOptions：实际 image_topic/camera_info_topic、optical_frame、binding_basis、raw/rectified、synchronized/static_calibration、独立 max_delta_ns |
| depth | DepthOptions：米/毫米、optical_z、范围、注册状态/依据及声明网格；不从编码或尺寸猜测 |
| sync_tolerance_ns | 非负 int64 纳秒最大绝对时差（含边界）；0 严格，正值为 RGB 基准单调一对一最近候选。省略/API 默认 0，示例显式 30000000（30 ms 试用值） |
| tf_max_gap_ns | 可选非负 int64 纳秒。省略时按所需链的动态样本估计，显式 0 只接受精确动态点 |
| tf_max_translation_step_m / tf_max_rotation_step_rad | 可选有限正值，限制相邻插值端点的平移量/最短旋转角 |
| limits | 分别限制消息/图像/标定/TF 条目与样本、序列化单消息、像素/源图像字节、全部输出字节 |

BagScan 拥有小元数据/TF 历史。CameraInfoIndex 借用其 records，不可在使用中修改。BagImageConsumer 收到 const Image* 只在回调内有效，不能缓存；nullptr 表示 CDR 或压缩负载无法解析；用 deserialization_error/payload_error_code 区分，负载坏时仍可有真实 header。标准帧由既有解码模块拥有，写出后释放。

## 4. 返回与错误

BagBuildResult：status 为 failed/completed/completed_with_issues；code/message 解释整体结果；output_directory 是成功发布路径；失败 work_directory 可供排查。计数包括 rgb_frames/depth_frames/decoded_frames/pairs/prepared_pairs/issue_count。

常见整体失败码：invalid_config、unsupported_input、missing_metadata、missing_storage_file、topic_type_mismatch、missing_topic、limit_exceeded、ambiguous_time_segment、unreadable_calibration、unreadable_tf、source_changed、message_count_mismatch、output_exists、io_error。资源分配错误为 resource_error，其他标准异常为 io_or_runtime_error。失败计数可能只反映已执行部分，不能当全包统计。

可保留的单条问题：坏 Image 布局/编码/时间、CameraInfo 参数无效、TF 单条数值/拓扑冲突。坏 Image 原始字节仍保存；坏 TF 条目被现有索引拒绝/阻断，不能“最后一条覆盖”。无法解析整个标定/TF 消息会使版本/历史边界不可信，整次失败。

## 5. 产物契约 bag_preparation/1

| 文件 | 一条记录是什么 | 其他模块如何引用 |
| --- | --- | --- |
| bag_manifest.json | 数据集类型、完成状态、bag_id、源分卷/指纹、话题清单、有效配置、TF 覆盖、文件清单 | 先确认 artifact_kind=bag_preparation；不等同正式 memory manifest |
| messages.jsonl | 每条配置消息的身份/记录时间/序号/摘要/原始 CDR 路径 | message_id 是所有其他来源引用的根，包括坏 CDR 和空 TFMessage |
| frames.jsonl | 每个 RGB/Depth 消息的原始元数据、decode_code、标准路径、标定/历史 TF/质量 | source.message_id；metadata_available=false 时 stamp=null；media_path=null 表示没有标准图像；source_data_bytes 为源 Image 像素或 CompressedImage 负载字节数，numeric_quality_passed 只表示数值质量 |
| rgbd_pairs.jsonl | 一对两图 ID/原始 stamp/有符号 delta_ns、标定 ID、策略、准备状态及原因 | pair_id；frames 中各自查询端点仍为自身采样时刻 |
| camera_info.jsonl | 每条原始完整 K/D/R/P/ROI/binning 和来源/校验结果 | calibration_id=来源 message_id，重复广播来源不合并消失 |
| tf.jsonl | 每条 TransformStamped 原始父子/时间/几何/来源条目/insert_code | source.message_id + transform_index；静态与动态显式区分 |
| reports/issues.jsonl | stage/code/message 与消息/配对引用 | 按稳定码处理，无需解析中文/英文文本 |
| reports/pending.jsonl | 坏帧/数值深度质量、不可准备配对、未配对 RGB、未使用 Depth | unused_depth 的 is_bad_frame=false；需要另查解码/质量 |
| reports/summary.json | 实际数量、有效时间索引数量、配对/未配对/问题及 TF 统计 | 两流配对守恒针对 valid_time_*，不含无法解析的 Image |
| evidence/raw/*.cdr | 选定消息完整原始序列化字节 | messages 中 topic/type（类型由 manifest inventory 提供）/格式/摘要共同解释 |
| evidence/rgb/*.png | 标准 RGB8，无裁剪/缩放 | 保留原图像素坐标 |
| evidence/depth/*.npy | height×width、C 顺序、little-endian float32 米制 optical-z、NaN 无效 | 用 numpy.load(allow_pickle=False) 或标准 NPY reader |

每条 JSONL 都有 schema_version。源 stamp 以 sec/nanosec 数字表达；int64 记录时间/时差及 uint64 来源序号用十进制字符串，解析者不能先转 double。非有限标定/TF 原值写 null 并标记，原始 CDR 保留真实字节。路径相对发布根目录。

## 6. 准备状态与几何边界

prepared_for_geometry 要求两图解码、各自标定/有效模型、各自的 map 相机位姿、RGB 时刻底盘位姿、注册声明核对和深度质量检查通过。pending 的 reasons 列出失败项，不自动改配。

后续读取 preparation 时按实际操作判断输入是否齐备。Depth→map 需要有效 Depth 像素、该帧模型和相机位姿。Depth→RGB 再需要配对、RGB 模型、RGB 相机位姿和共同时间及地图上下文。注册依据或底盘位姿不足的记录仍可用于满足上述条件的单点验算，保留 pending 原因及候选像素的限制。关键帧选择单独核对底盘位姿。

builder_version=5 输出数值位姿。frames 中 map_camera_tf 和 map_body_tf 保留原始时间检查及端点引用，新增 pose_code、pose_failed_frame 和可空 pose。pose 包含 target_frame、source_frame、translation_m、rotation_xyzw，表示 T_target_source。code 仍为时间状态，pose_code 为独立求解状态。

pose_computed 在 Depth 帧表示相机位姿成功，在 RGB 帧表示相机和底盘均成功，在配对表示两路相机与 RGB 时刻底盘均成功。摘要的 posed_frames、posed_pairs 给出数量。manifest 的 pose_computed 和摘要的 numeric_poses_computed 表示至少一帧所需位姿成功。配对的准备状态还检查解码、模型、注册和深度质量。effective_model_computed 逐帧表示实际模型成功，逐对表示两图均成功，完整数量看 camera_models。关键帧及正式 observations.jsonl 待实现。

## 7. 检查方法

构建并运行 CTest；磁盘构造回归产物在 build/.../test_artifacts/latest.txt 指向目录，带 demo_config.yaml。独立 NumPy 校验与安装消费者见 [测试说明](../modules/bag/tests/test_bag_builder.md)。真实设备 bag 的物理同步/注册/标定仍待单独验收。

注册配置须使 RGB/Depth 的 optical_frame 与声明目标一致；共享 CameraInfo 只接受明确的 aligned_to_rgb 声明。manifest 保留独立的全部资源阈值，容量以十进制字符串存储。

第一版实际测试/安装/内存检查边界及可查看样例见 [验收记录](../reviews/bag_builder_v1_verification.md)。

## 8. 时间配对版本与兼容

v3 引入以下配对契约，v5 继续沿用。JSON schema 为 bag_preparation/1。v2 的零配置继续有效，BagBuildOptions 默认仍为 0。本次允许显式正容差，不能再假设全部 exact=true。没有修改现有结构布局或 pairRgbDepth 签名；下游应使用匹配的新版头和库。

| 位置 | 字段格式 | 实际语义 |
| --- | --- | --- |
| pair | rgb_stamp/depth_stamp：{sec: int32, nanosec: uint32} | 两图原始采样时间；RGB 为索引参考 |
| pair | delta_ns：十进制 int64 字符串 | Depth−RGB，可为负，绝对值不超过门限 |
| pair | exact：bool | 仅 delta_ns==0 为 true |
| pair | policy：string；sync_tolerance_ns：十进制字符串 | 实际规则和显式门限 |
| CLI/manifest/summary | timestamp_match_policy；sync_tolerance_ns | 与 pair 同策略/门限；manifest 另保留 effective_options |
| summary | exact_pairs/approximate_pairs、max_abs_delta_ns | 按实际差统计；空最大值 0 不表示测得同步 |
| result/CLI/manifest/summary | timestamp_match_status：string | not_checked、empty_stream、no_exact_pairs（零模式无对）、no_pairs（正门限无对）、partial、all_paired |
| pair/CLI/manifest/summary | capture_sync_status："unknown" | 当前没有设备曝光同步证明 |

0 策略为 exact_header_one_to_one_v1，正值为 rgb_reference_monotonic_nearest_one_to_one_v1。正容差下实际相等的一对仍 exact=true。算法为 RGB 顺序、Depth 单调、一对一贪心最近候选；等距取较早 Depth，同刻取较小来源顺序，不保证全局最多配对或最小总误差。

新增 rgbd_sync.hpp/.cpp 公共函数：const char * rgbdSyncPolicyName(std::int64_t max_delta_ns) noexcept。输入是整数纳秒门限，返回上述静态字符串；负输入为 unknown，不分配内存、无需释放。正常调用前先校验配置，builder/CLI 共用该入口。

两图标定与 TF 各用自身原时间，body 用 RGB 时间；不把 Depth 当作发生在 RGB 时刻。近似匹配没有运动补偿，prepared_for_geometry 仅通过已有准备检查，不是最终几何精度结论。

零配对保留提取证据：零模式问题 no_exact_rgbd_pairs，正模式 no_rgbd_pairs，空有效流为 empty_stream。result.ok() 只表示完整发布，必须另查对数/策略/差值/质量。sync_tolerance_violation/strict_sync_violation 是内部不变量错误，整次失败。

### RGB-D 时间基准与配对门限

配对先核对两路索引的 clock_id、segment_id，以及显式 same_clock 声明和 clock_basis，再计算 delta_ns = t_depth − t_rgb。配置为 30 ms 时，候选满足 |delta_ns| ≤ 30,000,000 ns。BagBuilder 在一个数据集内对两路索引和历史 TF 查询使用同一个声明的 clock/segment，保留原始 header、实际差值和配对策略。

两台设备可使用各自的硬件时钟。只有 header 已映射到同一时间基准，或者有可验证的时钟转换关系，两个数值的差才有采样时间意义。BagBuilder 当前消费已处于同一声明时间基准的 header。clock_basis 记录这一前提的采集依据，30 ms 门限度量该前提下的时差。设备曝光同步状态单独记录，当前 capture_sync_status 为 unknown。

复用同一可信 preparation 的配对时，输入组织层按 pair 中的消息 ID 读取两帧，从同一 manifest 继承 bag_id 和 clock/segment，核对两帧与各自标定、位姿的引用，并沿用保存的配对结果和门限。若输入来自独立数据源，需先验证时间关系和地图配准依据。仅凭同名 map 和相近的 header 数值无法建立这些关系。

## 9. 图像格式与来源

支持 Image 和声明过的 CompressedImage（JPEG RGB8/BGR8、16UC1 compressedDepth PNG），格式/预算/错误见 [image_decoder_ros](../modules/bag/image_decoder_ros.md)。首遍不解压，第二遍逐帧解码。

BagImageRecord 保留 message_type、compressed_format、payload_error_code/payload_error。frames 的 source_step/source_is_bigendian 在压缩源为 null；source_encoding 是中间 Image 编码，压缩源声明在 source_format。坏压缩体保留 header，坏 CDR 才无 metadata。

实际测试、全包时间核查与短片段媒体验证见 [最近配对验收](../reviews/rgbd_nearest_pairing_verification.md)；[零容差验收](../reviews/rgbd_zero_tolerance_verification.md) 保留此前 v2 的历史结果。

## 10. 相机兼容与有效模型（builder_version=4）

JSON schema 继续 bag_preparation/1，新增字段保留旧原始值；C++ 结构/ROS 适配签名已扩展，下游须用配套头/库重新编译。v3 时间配对行为不变。

每路相机新增 compatibility：strict（省略/API 默认）或 plumb_bob_zero_tail_d8_v1。后者仅接受有限且末三项严格为零的 plumb_bob D8，在副本中派生 D5 并严格校验。共享 CameraInfo 话题要求两路使用相同策略。

camera_info.jsonl 的 d/k/r/p 和 validation_code/validation_message 仍描述原始参数；新增 compatibility、normalization_applied、normalized_code、effective_d（失败 null）。规范化成功不再作为不可用标定问题重复写 issues。

frames.jsonl 新增 camera_model：code/message，成功时包含实际 width/height/frame_id、image_domain、distortion_model、D/K/R/P、normalization_applied、ray_convention=original_optical_frame_z_one。原始 calibration_id 继续关联 camera_info.jsonl，图像时间不变。失败明确 effective_model_computed=false，不输出零模型。

camera_models 是成功生成模型的帧数，CLI/result/summary/manifest 一致。配对失败原因新增 rgb_camera_model_unavailable / depth_camera_model_unavailable；TF/注册缺失时可以已有有效模型而 pair 仍 pending。

接口与坐标约定见 [CameraModel](../modules/camera_info/camera_model.md)，实现与真实验证见 [实施说明](../project/camera_model_implementation.md) 和 [验收](../reviews/camera_model_verification.md)。
