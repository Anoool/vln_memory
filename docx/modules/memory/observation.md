# 正式 Observation 记录

实现于 2026-11-06。公开头为 include/vln_memory/memory/observation.hpp，校验实现为 src/memory/observation.cpp，JSON 实现为 src/memory/observation_json.cpp。目标为 vln_memory::vln_memory_observation，核心与 ROS 构建均提供。

## 交付与职责

本模块落实正式记录类型、身份、字段关联/内部一致性校验、pending 汇总及 observation/1 JSON 往返。记录独立拥有字符串、模型、原始标定、TF 端点和消息摘要，不持有像素、借用指针或 preparation reader。

依赖现有 CameraModel、Depth 和 TF 原语，JSON 库为私有依赖；没有 builder、ROS 或模型服务链接依赖。实施前方案及自查见 [实施方案](../../project/observation_implementation_plan.md)。

生产流程通过 [观测组装](../engine/observation_builder.md) 从已回读配对构造正式记录，并由 [观测数据集发布](../engine/observation_dataset.md) 统一写出 JSONL、批次及来源清单。完整记忆版本另需实体、关系与索引。

## 数据与拥有关系

全部 struct 位于命名空间顶层，复用已有传感器类型：

| 类型/字段组 | 含义与约束 |
| --- | --- |
| Observation 身份 | bag_id、pair_id、两条原始消息 ID、clock_id/segment_id 构成稳定身份；revision 为正 uint64 |
| 地图 | map_id、geometry_revision、map_frame、body_frame、map_basis 显式给出；geometry_revision 为不透明非空字符串 |
| 时间 | stamp 等于 rgb_image.stamp；JSON 显式保存 stamp/rgb_stamp/depth_stamp；delta_ns 为 Depth−RGB，门限必须覆盖实际差值 |
| 来源 | source_manifest_sha256 标识媒体/消息所属来源数据集；clock_basis/segment_basis 保存声明依据 |
| 图像 | RGB/Depth 各自 topic、frame、width/height、stamp、模型与可选媒体路径；不合并成未经证明的共同模型 |
| ObservationCalibration | calibration_id、source_message_id、完整原始 CameraInfo（含 D8/ROI/binning/原始时刻）及实际选择参数 |
| 三个位姿 | camera_pose_map 用 RGB 时刻；depth_camera_pose_map 用 Depth 时刻；body_pose_map 用 RGB 时刻；复用 TfPoseResult，保存完整端点 |
| ObservationDepthStorage | 实际文件 format/dtype/unit/scale/invalid_rule、源 encoding/unit/scale、范围和 valid/invalid 计数；不含像素 |
| ObservationEvidence | 源消息 ID、topic、分卷索引、topic 顺序、全局顺序、bag 记录时刻、字节数、SHA-256 和可选 CDR 路径 |
| 状态与 pending | 来源 prepared_for_geometry/pending 及原始原因独立保留；正式 pending_reasons 还包含缺项和未核实声明 |

无媒体时图像宽高可以同时为 0；不能有依赖该网格的媒体或可解释标定/模型。不可解释的原始标定仍保留在来源 preparation，通过消息摘要与来源数据集回查；本类型的可选嵌入标定表示能够按指定参数绑定的原始标定。

## 公开接口

- makeObservationId(observation)：六个来源分量采用长度前缀编码，按 UTF-8 字节长度定义。不使用 std::hash、不增加哈希依赖，不因地图/轨迹 revision 或媒体存储变化而变更。身份分量不超过 4096 字节。ID 是不透明键，不作为文件名。
- collectObservationPendingReasons(observation)：按来源顺序保留原因，合并已有自定义原因，再补充缺项/未知声明的规范原因，去重且不修改输入。
- validateObservation(observation)：返回 ObservationCheck，首次错误包含稳定 code/message；不修改原始字段。
- serializeObservation(observation)：先共用校验，再返回单行规范 JSON。失败 json 为空。
- deserializeObservation(json, max_json_bytes)：严格解析并共用校验，失败 observation 为空。默认输入上限 16 MiB，嵌套深度最大 64；该上限不限制调用前字符串本身的分配。

常用消费方式：

```cpp
auto parsed = vln_memory::deserializeObservation(line);
if (!parsed.ok()) {
  // 记录 observationCodeName(parsed.code) 和 parsed.message。
  return;
}
const auto & observation = *parsed.observation;
// 按操作检查具体媒体、模型、位姿和 pending；ok() 只表示记录合法。
auto encoded = vln_memory::serializeObservation(observation);
```

组装方完成字段后应显式设置 id=makeObservationId(record)，再将 pending_reasons 设置为 collectObservationPendingReasons(record)。解析器不自动修复缺失原因、身份或错误时间。

## 校验链路

1. 身份、版本、地图/来源/时钟依据和图像元数据。
2. 复用 checkSensorTimes 验证整数时差、门限和 exact/approximate；零/负时间合法。
3. 建立唯一消息摘要索引，核对图像、标定与 TF 的源 ID/topic。
4. 校验媒体相对路径、深度格式/单位/比例/范围与计数。
5. 复用 buildCameraModel 从完整原始标定及实际选择参数重建，对比全部 K/D/R/P、网格、图像域及 normalization 标记。
6. 逐端点检查坐标链方向和来源，复用 TfIndex 与显式 TF 间隔/跳变限制重新求解，比较查询时刻、端点选择、平移和旋转。四元数 q 与 −q 表示同一旋转。
7. 检查同步/注册声明及其依据，核对全部规范 pending 原因。

TF 平移/模型比较采用 1e-10 的绝对/相对容差；该容差只针对保存和重算的数值一致性。TF 原始四元数及模型结构仍由现有原语校验；物理误差不采用这个阈值验收。

## 媒体与声明

RGB 媒体语义为 png_rgb8。深度支持：

| 文件 | format/dtype/unit/scale | invalid_rule |
| --- | --- | --- |
| 源 16UC1 PNG | png/uint16，实际单位 m 或 mm，比例 1 或 0.001，须与源单位/比例一致 | zero_or_out_of_range_to_nan |
| 米制 NPY | npy/float32/m/1，源单位和原始比例另外保留 | nan |

max_depth_m=null 表示未声明上界；NaN/Inf 不能作为字段值写入 JSON。valid+invalid 必须等于图像像素数，全无效深度可合法保存并包含 no_valid_depth。

路径相对于 source_manifest_sha256 标识的数据集根，消费端负责显式映射该根。结构校验拒绝绝对路径、盘符、反斜线、空路径组件及 . / .. 组件；不访问文件、不解析符号链接、不核验真实内容。

capture_sync_status 的词汇为 unknown、verified_synchronized、not_synchronized，后两项须有 basis。depth_alignment 为 unknown、not_aligned、aligned_to_rgb，aligned_to_rgb 须有 basis 及共同光学 frame/尺寸；not_aligned 可按已有 Depth 契约保留空 basis，并保留其 pending。相同网格与非空依据只表示声明一致，不能证明物理注册/同步。原始未知状态继续原样保留。

## JSON 契约

schema_version=observation/1，固定 length_unit=m、rotation_convention=xyzw、depth_convention=optical_z。所有固定对象拒绝未知/缺失键；可选项显式为 null，重复键和不支持的 schema 被拒绝。

sec/nanosec、尺寸等小整数使用有界 JSON 整数；revision、纳秒差/门限、源记录索引/顺序/长度与 TF 条目索引使用规范十进制字符串。拒绝浮点替代整数、前导加号/零、溢出和矩阵多余元素。媒体、像素和真实源字节不内嵌在 JSON 中。

pending 的规范原因包括 missing_rgb_media、missing_depth_media、missing_rgb_calibration、missing_depth_calibration、missing_rgb_model、missing_depth_model、missing_rgb_map_pose、missing_depth_map_pose、missing_body_map_pose、no_valid_depth、depth_rgb_alignment_unverified、depth_rgb_not_aligned、capture_sync_unverified、capture_not_synchronized。来源原因与自定义原因也必须保留。

## 错误、复杂度与验证边界

错误码为 invalid_identity/context/time/media/calibration/pose/reference/pending/json 及 unsupported_schema（均有 observationCodeName 稳定名称）。成功 code=ready 表示结构合法，可包含 pending。校验不以非空哈希/依据证明源文件或物理质量。

令 E 为消息证据数、T 为 TF 端点数、R 为原因数。证据索引约 O(E log E)，端点查询/重算沿用 TfIndex，pending 合并 O(R log R)；空间用于拥有型元数据及临时索引，不复制 RGB/Depth 像素。JSON 阶段另需与单记录大小成比例的 DOM/字符串空间。

回归与真实样本验收见 [observation 验收](../../reviews/observation_verification.md)。
