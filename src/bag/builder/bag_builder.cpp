#include <vln_memory/bag/builder/bag_builder.hpp>

#include "preparation_metadata.hpp"
#include "preparation_storage.hpp"
#include "../../engine/observation_dataset_format.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <filesystem>
#include <map>
#include <set>
#include <string_view>
#include <utility>

#include <vln_memory/bag/builder/bag_reader_ros.hpp>
#include <vln_memory/bag/builder/jsonl_writer.hpp>
#include <vln_memory/bag/builder/rgbd_sync.hpp>
#include <vln_memory/bag/depth/depth_ros.hpp>
#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <vln_memory/bag/rgb/rgb_ros.hpp>

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;

/** @brief 仅保留后续组装所需小字段，图像和 TF 端点 JSON 写完即释放。 */
struct PreparedFrame
{
  std::string message_id;
  std::string decode_code;
  std::string media_path;
  std::string calibration_id;
  std::string calibration_code;
  bool camera_matched{false};
  bool model_ready{false};
  bool camera_pose_ready{false};
  bool body_pose_ready{false};
  bool alignment_checked{false};
  bool aligned{false};
  bool depth_quality{true};
  CameraImageReference image;
  std::optional<CameraModel> model;
  std::optional<TfPoseResult> camera_pose;
  std::optional<TfPoseResult> body_pose;
  std::optional<PreparedDepthStorage> depth_storage;
  DepthAlignment depth_alignment{DepthAlignment::unknown};
  std::string alignment_basis;
};

/** @brief 单帧转换的小型返回，不含拥有型像素；details 保存统计/原始语义。 */
struct ConvertedFrame
{
  std::string code{"deserialize_failed"};
  std::string message;
  std::string media_path;
  bool alignment_checked{false};
  bool aligned{false};
  bool depth_quality{true};
  Json details = Json::object();
  std::optional<PreparedDepthStorage> depth_storage;
};

/** @brief 初始化固定 JSONL 文件，零条消息/问题仍有明确的空文件契约。 */
void initializeOutput(OutputFiles & output)
{
  for (const auto * path : {"messages.jsonl", "frames.jsonl", "rgbd_pairs.jsonl", "camera_info.jsonl",
      "tf.jsonl", "reports/issues.jsonl", "reports/pending.jsonl", detail::kPreparationMediaIndexFile})
  {
    output.ensureLines(path);
  }
}

/** @brief 构造纯时间索引；坏 CDR/非法 stamp 不冒充零时刻有效消息。 */
RgbdStreamIndex streamIndex(const BagBuildOptions & options, const BagScan & scan, const std::string & topic)
{
  RgbdStreamIndex stream;
  stream.topic = topic;
  stream.clock_id = options.clock_id;
  stream.segment_id = options.segment_id;
  for (const auto & image : scan.images) {
    if (image.source.topic == topic && image.deserialization_error.empty() && image.stamp.nanosec < 1000000000U) {
      stream.frames.push_back({image.source.message_id, image.source.message_order, image.stamp});
    }
  }
  return stream;
}

/** @brief 执行无 ROS 配对核心；算法失败终止，不输出半份配对结果。 */
RgbdSyncReport synchronize(const BagBuildOptions & options, const BagScan & scan)
{
  RgbdSyncOptions sync;
  sync.clock_relation = options.clock_relation;
  sync.clock_basis = options.clock_basis;
  sync.max_delta_ns = options.sync_tolerance_ns;  // 显式门限；只选择来源，不修改两图原始 stamp。
  sync.max_frames = options.max_frames;
  auto result = pairRgbDepth(
    streamIndex(options, scan, options.rgb_camera.image_topic),
    streamIndex(options, scan, options.depth_camera.image_topic), sync);
  if (!result.ok()) {throw BagBuildError(rgbdSyncCodeName(result.code), result.message);}
  if (result.report->max_abs_delta_ns > static_cast<std::uint64_t>(options.sync_tolerance_ns)) {
    throw BagBuildError("sync_tolerance_violation", "RGB-D pairing exceeded the configured time tolerance");
  }
  if (options.sync_tolerance_ns == 0 && result.report->exact_count != result.report->pairs.size()) {
    throw BagBuildError("strict_sync_violation", "Non-exact RGB-D pair passed zero tolerance");
  }
  return std::move(*result.report);
}

/** @brief 只描述原 header 配对覆盖，不把解码、标定或设备曝光同步混入此状态。 */
std::string timestampMatchStatus(const RgbdSyncReport & sync, std::int64_t tolerance_ns)
{
  if (sync.rgb_count == 0 || sync.depth_count == 0) {return "empty_stream";}
  if (sync.pairs.empty()) {
    if (tolerance_ns == 0) {return "no_exact_pairs";}
    return "no_pairs";
  }
  if (sync.pairs.size() == sync.rgb_count && sync.pairs.size() == sync.depth_count) {return "all_paired";}
  return "partial";
}

/** @brief 真实配对 RGB 网格按 Depth 来源索引，无像素复制和每图全表扫描。 */
std::map<std::string, ImageGeometry> pairedRgbGrids(const BagScan & scan, const RgbdSyncReport & sync)
{
  std::map<std::string, const BagImageRecord *> images;
  for (const auto & image : scan.images) {
    images.emplace(image.source.message_id, &image);
  }
  std::map<std::string, ImageGeometry> grids;
  for (const auto & pair : sync.pairs) {
    const auto & image = *images.at(pair.rgb.message_id);
    grids.emplace(pair.depth.message_id, ImageGeometry{image.width, image.height, image.frame_id});
  }
  return grids;
}

/** @brief RGB 解码成功即写 PNG，返回值不携带像素；失败保持已有核心错误码。 */
ConvertedFrame convertRgb(
  const BagImageRecord & image, const sensor_msgs::msg::Image & message,
  const BagBuildOptions & options, OutputFiles & output)
{
  ConvertedFrame result;
  const auto decoded = decodeRosRgb(message, options.rgb_limits);
  result.code = rgbErrorCodeName(decoded.code);
  result.message = decoded.message;
  if (decoded.ok()) {
    result.media_path = writeRgb(output, image.source.message_id, *decoded.frame);
    result.details = {{"encoding", "rgb8"}, {"source_encoding", decoded.frame->source_encoding}};
  }
  return result;
}

/** @brief 深度解码/保存并检查真正配对网格，NaN 不进入 JSON 数值字段。 */
ConvertedFrame convertDepth(
  const BagImageRecord & image, const sensor_msgs::msg::Image & message,
  const BagBuildOptions & options, const std::map<std::string, ImageGeometry> & rgb_grids, OutputFiles & output)
{
  ConvertedFrame result;
  const auto decoded = decodeRosDepth(message, options.depth, options.depth_limits);
  result.code = depthErrorCodeName(decoded.code);
  result.message = decoded.message;
  if (!decoded.ok()) {return result;}
  const auto & frame = *decoded.frame;
  const bool store_source_png = options.depth_storage == BagDepthStorage::source_png16 &&
    frame.source_numeric_type == DepthNumericType::uint16;
  if (store_source_png) {
    result.media_path = writeDepthPng16(output, image.source.message_id, message);
  } else {
    result.media_path = writeDepth(output, image.source.message_id, frame);
  }
  result.depth_quality = frame.valid_pixels > 0 && frame.valid_ratio >= options.min_valid_depth_ratio;
  result.details = {{"encoding", "32FC1"}, {"unit", "meters"}, {"convention", "optical_z"},
    {"source_encoding", frame.source_encoding}, {"source_unit", depthUnitName(frame.source_unit)},
    {"source_numeric_type", depthNumericTypeName(frame.source_numeric_type)},
    {"source_scale_to_m", frame.source_scale_to_m}, {"valid_pixels", frame.valid_pixels},
    {"invalid_pixels", frame.invalid_pixels}, {"valid_ratio", frame.valid_ratio},
    {"zero_pixels", frame.zero_pixels}, {"negative_pixels", frame.negative_pixels},
    {"nonfinite_pixels", frame.nonfinite_pixels}, {"out_of_range_pixels", frame.out_of_range_pixels},
    {"unrepresentable_pixels", frame.unrepresentable_pixels}, {"alignment_basis", frame.alignment_basis}};
  result.details["storage"] = {
    {"format", store_source_png ? "png" : "npy"},
    {"dtype", store_source_png ? "uint16" : "float32"},
    {"unit", store_source_png ? depthUnitName(frame.source_unit) : "m"},
    {"scale_to_m", store_source_png ? frame.source_scale_to_m : 1.0},
    {"invalid_rule", store_source_png ? "zero_or_out_of_range_to_nan" : "nan"},
    {"min_depth_m", frame.min_depth_m}, {"max_depth_m", frame.max_depth_m}};
  result.depth_storage = PreparedDepthStorage{store_source_png ? "png" : "npy",
    store_source_png ? "uint16" : "float32", store_source_png ? frame.source_unit : DepthUnit::meters,
    store_source_png ? frame.source_scale_to_m : 1.0,
    store_source_png ? "zero_or_out_of_range_to_nan" : "nan", frame.source_encoding, frame.source_unit,
    frame.source_scale_to_m, frame.min_depth_m,
    std::isfinite(frame.max_depth_m) ? std::optional<double>(frame.max_depth_m) : std::nullopt,
    frame.valid_pixels, frame.invalid_pixels, false};
  const auto grid = rgb_grids.find(image.source.message_id);
  result.details["alignment_code"] = "not_checked_no_pair";
  if (grid != rgb_grids.end()) {
    const auto alignment = checkDepthRgbAlignment(frame, grid->second);
    result.alignment_checked = true;
    result.aligned = alignment.alignedToRgb();
    result.details["alignment_code"] = depthAlignmentStatusName(alignment.status);
    result.details["alignment_message"] = alignment.message;
  }
  return result;
}

/** @brief 导出实际模型及其明确坐标约定；失败仅写状态，不输出零内参。 */
Json cameraModelJson(const CameraModelResult & result)
{
  Json record{{"code", cameraGeometryCodeName(result.code)}, {"message", result.message}};
  if (!result.ok()) {return record;}
  const auto & model = *result.model;
  record["width"] = model.width;
  record["height"] = model.height;
  record["frame_id"] = model.frame_id;
  record["image_domain"] = "raw";
  if (model.image_domain == CameraImageDomain::rectified) {record["image_domain"] = "rectified";}
  record["distortion_model"] = model.distortion_model;
  record["d"] = model.d;
  record["k"] = model.k;
  record["r"] = model.r;
  record["p"] = model.p;
  record["normalization_applied"] = model.normalization_applied;
  record["ray_convention"] = "original_optical_frame_z_one";
  return record;
}

/** @brief 生成相机模型，并按各图原始采样时刻求 map 位姿。 */
void attachGeometryChecks(
  const BagImageRecord & image, bool is_rgb, const BagBuildOptions & options,
  const BagScan & scan, const CameraInfoIndex & calibrations, PreparedFrame & prepared, Json & record,
  bool retain_observation)
{
  const CameraInfoMatchOptions * binding = &options.depth_camera;
  if (is_rgb) {binding = &options.rgb_camera;}
  const CameraImageReference reference{image.source.topic, image.width, image.height, image.frame_id, image.stamp};
  const auto selected = calibrations.select(reference, *binding);
  prepared.calibration_id = selected.calibration_id;
  prepared.calibration_code = selected.code;
  prepared.camera_matched = selected.matched();
  record["calibration"] = {{"calibration_id", selected.calibration_id}, {"code", selected.code},
    {"message", selected.message}, {"used_static_calibration", selected.match.used_static_calibration},
    {"calibration_code", cameraInfoErrorCodeName(selected.match.calibration_code)}};
  if (selected.match.timing) {
    auto & timing = record["calibration"]["timing"];
    timing = {{"code", sensorTimeCodeName(selected.match.timing->code)}, {"delta_ns", nullptr}};
    if (selected.match.timing->delta_ns) {timing["delta_ns"] = std::to_string(*selected.match.timing->delta_ns);}
  }
  if (selected.info != nullptr) {
    auto model = buildCameraModel(*selected.info, reference, *binding);
    if (model.model) {model.model->normalization_applied = selected.normalization_applied;}
    prepared.model_ready = model.ok();
    record["camera_model"] = cameraModelJson(model);
    if (retain_observation && model.ok()) {prepared.model = std::move(model.model);}
    record["effective_model_computed"] = prepared.model_ready;
  }
  TfTimeOptions time;
  time.clock_id = options.clock_id;
  time.segment_id = options.segment_id;
  time.max_interpolation_gap_ns = options.tf_max_gap_ns.value();
  auto camera_tf = scan.tf_index.lookupPose(
    options.map_frame, image.frame_id, image.stamp, time, options.tf_jump_limits);
  prepared.camera_pose_ready = camera_tf.ok();
  record["map_camera_tf"] = tfPoseJson(camera_tf);
  if (retain_observation && camera_tf.ok()) {prepared.camera_pose = std::move(camera_tf);}
  if (is_rgb) {
    auto body_tf = scan.tf_index.lookupPose(
      options.map_frame, options.body_frame, image.stamp, time, options.tf_jump_limits);
    prepared.body_pose_ready = body_tf.ok();
    record["map_body_tf"] = tfPoseJson(body_tf);
    if (retain_observation && body_tf.ok()) {prepared.body_pose = std::move(body_tf);}
  }
  record["pose_computed"] = prepared.camera_pose_ready && (!is_rgb || prepared.body_pose_ready);
}

/** @brief 单帧原始布局与数值转换结果；坏 CDR 明确没有可用 header。 */
Json frameRecord(const BagImageRecord & image, bool is_rgb, bool metadata_available, const ConvertedFrame & converted)
{
  Json record{{"source", sourceJson(image.source)}, {"kind", "depth"},
    {"stamp", stampJson(image.stamp)}, {"frame_id", image.frame_id}, {"width", image.width}, {"height", image.height},
    {"source_encoding", image.encoding}, {"source_step", image.step}, {"source_is_bigendian", image.is_bigendian},
    {"source_data_bytes", image.data_bytes}, {"source_message_type", image.message_type},
    {"source_format", nullptr},
    {"decode_code", converted.code}, {"decode_message", converted.message}, {"media_path", nullptr},
    {"details", converted.details}, {"pose_computed", false}, {"effective_model_computed", false},
    {"camera_model", {{"code", "not_computed"}, {"message", "No usable calibration/image"}}}};
  if (image.message_type == "sensor_msgs/msg/CompressedImage") {
    record["source_format"] = image.compressed_format;
    record["source_step"] = nullptr;
    record["source_is_bigendian"] = nullptr;  // 压缩源消息没有 Image 的行宽/端序字段。
  }
  record["metadata_available"] = metadata_available;
  record["numeric_quality_passed"] = !converted.media_path.empty() && converted.depth_quality;
  if (!metadata_available) {record["stamp"] = nullptr;}
  if (is_rgb) {record["kind"] = "rgb";}
  return record;
}

/** @brief 每个坏帧/全无效深度独立可查；未配对策略与像素质量分别报告。 */
void reportConversionIssues(
  const BagImageRecord & image, bool metadata_available, const ConvertedFrame & converted,
  OutputFiles & output, std::vector<BagIssue> & issues, bool write_preparation)
{
  const auto * pending_path = write_preparation ? "reports/pending.jsonl" : "evidence/pending.jsonl";
  if (converted.media_path.empty()) {
    if (metadata_available) {
      issues.push_back({"decode", converted.code, converted.message, image.source.message_id, {}});
    }
    writeJsonLine(
      output, pending_path, {{"kind", "invalid_frame"},
        {"message_id", image.source.message_id}, {"reason", converted.code}});
  } else if (!converted.depth_quality) {
    issues.push_back(
      {"quality", "insufficient_valid_depth", "Decoded depth fails numeric quality threshold",
        image.source.message_id, {}});
    writeJsonLine(
      output, pending_path, {{"kind", "invalid_depth_quality"},
        {"message_id", image.source.message_id}, {"reason", "insufficient_valid_depth"}});
  }
}

/** @brief 分开转换、几何检查和报告，像素在转换辅助函数结束时释放。 */
PreparedFrame processImage(
  const BagImageRecord & image, const sensor_msgs::msg::Image * message,
  const BagBuildOptions & options, const BagScan & scan, const CameraInfoIndex & calibrations,
  const std::map<std::string, ImageGeometry> & rgb_grids, OutputFiles & output, std::vector<BagIssue> & issues,
  bool write_preparation)
{
  const bool is_rgb = image.source.topic == options.rgb_camera.image_topic;
  ConvertedFrame converted;
  if (message != nullptr) {
    if (is_rgb) {converted = convertRgb(image, *message, options, output);} else {
      converted = convertDepth(image, *message, options, rgb_grids, output);
    }
  } else if (!image.deserialization_error.empty()) {
    converted.message = image.deserialization_error;
  } else {
    converted.code = image.payload_error_code;
    converted.message = image.payload_error;
  }
  const bool metadata_available = image.deserialization_error.empty();
  PreparedFrame prepared;
  prepared.message_id = image.source.message_id;
  prepared.decode_code = converted.code;
  prepared.media_path = converted.media_path;
  prepared.depth_quality = converted.depth_quality;
  prepared.alignment_checked = converted.alignment_checked;
  prepared.aligned = converted.aligned;
  auto record = frameRecord(image, is_rgb, metadata_available, converted);
  if (!converted.media_path.empty()) {
    record["media_path"] = converted.media_path;
    attachGeometryChecks(image, is_rgb, options, scan, calibrations, prepared, record, !write_preparation);
  }
  reportConversionIssues(image, metadata_available, converted, output, issues, write_preparation);
  if (write_preparation) {
    writeJsonLine(output, "frames.jsonl", std::move(record));
  } else {
    prepared.image = {image.source.topic, image.width, image.height, image.frame_id, image.stamp};
    if (!is_rgb && !converted.media_path.empty()) {
      const auto & details = converted.details;
      prepared.depth_storage = std::move(converted.depth_storage);
      if (converted.aligned) {prepared.depth_alignment = DepthAlignment::aligned_to_rgb;}
      else if (details.at("alignment_code") == "not_aligned") {
        prepared.depth_alignment = DepthAlignment::not_aligned;
      }
      prepared.alignment_basis = details.at("alignment_basis");
    }
  }
  return prepared;
}

/** @brief 针对已固定配对汇总质量原因，不因坏帧重配，也不以单位位姿填缺失。 */
std::vector<std::string> pairReasons(const PreparedFrame & rgb, const PreparedFrame & depth)
{
  std::vector<std::string> reasons;
  if (rgb.media_path.empty()) {reasons.push_back("rgb_decode_failed");}
  if (depth.media_path.empty()) {reasons.push_back("depth_decode_failed");}
  if (!rgb.camera_matched) {reasons.push_back("rgb_calibration_unavailable");}
  if (!depth.camera_matched) {reasons.push_back("depth_calibration_unavailable");}
  if (!rgb.model_ready) {reasons.push_back("rgb_camera_model_unavailable");}
  if (!depth.model_ready) {reasons.push_back("depth_camera_model_unavailable");}
  if (!rgb.camera_pose_ready) {reasons.push_back("rgb_tf_unavailable");}
  if (!depth.camera_pose_ready) {reasons.push_back("depth_tf_unavailable");}
  if (!rgb.body_pose_ready) {reasons.push_back("body_tf_unavailable");}
  if (!depth.alignment_checked || !depth.aligned) {reasons.push_back("depth_rgb_alignment_unverified");}
  if (!depth.depth_quality) {reasons.push_back("insufficient_valid_depth");}
  return reasons;
}

/** @brief 写配对/待处理，引用 frames.jsonl 的具体两图和标定；不会输出正式 Observation。 */
void writePairs(
  const BagBuildOptions & options, const RgbdSyncReport & sync,
  const std::map<std::string, PreparedFrame> & frames,
  OutputFiles & output, BagBuildResult & result, std::vector<BagIssue> & issues)
{
  for (const auto & pair : sync.pairs) {
    const auto & rgb = frames.at(pair.rgb.message_id);
    const auto & depth = frames.at(pair.depth.message_id);
    const auto pair_id = "p_" + pair.rgb.message_id;
    const auto reasons = pairReasons(rgb, depth);
    const bool pose_ready = rgb.camera_pose_ready && depth.camera_pose_ready && rgb.body_pose_ready;
    if (pose_ready) {++result.posed_pairs;}
    Json record{{"pair_id", pair_id}, {"rgb_message_id", pair.rgb.message_id},
      {"depth_message_id", pair.depth.message_id}, {"rgb_stamp", stampJson(pair.rgb.stamp)},
      {"depth_stamp", stampJson(pair.depth.stamp)}, {"delta_ns", std::to_string(pair.delta_ns)},
      {"exact", pair.delta_ns == 0}, {"policy", rgbdSyncPolicyName(options.sync_tolerance_ns)},
      {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)}, {"capture_sync_status", "unknown"},
      {"rgb_calibration_id", rgb.calibration_id}, {"depth_calibration_id", depth.calibration_id},
      {"status", "pending"}, {"reasons", reasons}, {"pose_computed", pose_ready},
      {"effective_model_computed", rgb.model_ready && depth.model_ready}};
    if (reasons.empty()) {record["status"] = "prepared_for_geometry"; ++result.prepared_pairs;} else {
      writeJsonLine(output, "reports/pending.jsonl", record);
      issues.push_back({"prepare", "pair_pending", "Pair has failed sensor preparation checks", {}, pair_id});
    }
    writeJsonLine(output, "rgbd_pairs.jsonl", std::move(record));
  }
  result.pairs = sync.pairs.size();
}

/** @brief 未配对 RGB 有明确原因；unused Depth 单独记录，不伪装成坏帧。 */
void writeUnmatched(
  const RgbdSyncReport & sync, OutputFiles & output, std::vector<BagIssue> & issues, const char * pending_path)
{
  for (const auto & unmatched : sync.unmatched_rgb) {
    const auto code = rgbdUnmatchedReasonName(unmatched.reason);
    writeJsonLine(
      output, pending_path, {{"kind", "unmatched_rgb"},
        {"message_id", unmatched.rgb.message_id}, {"stamp", stampJson(unmatched.rgb.stamp)}, {"reason", code}});
    issues.push_back({"sync", code, "RGB has no selected depth", unmatched.rgb.message_id, {}});
  }
  for (const auto & depth : sync.unused_depth) {
    writeJsonLine(
      output, pending_path, {{"kind", "unused_depth"}, {"message_id", depth.message_id},
        {"stamp", stampJson(depth.stamp)}, {"reason", "not_selected_by_policy"}, {"is_bad_frame", false}});
  }
}

/** @brief 容量使用精确十进制字符串，分别保留各处理阶段实际上限。 */
Json effectiveLimits(const BagBuildOptions & options)
{
  return {{"max_messages", std::to_string(options.max_messages)}, {"max_frames", std::to_string(options.max_frames)},
    {"max_calibrations", std::to_string(options.max_calibrations)}, {"max_tf_entries", std::to_string(
        options.max_tf_entries)},
    {"max_serialized_bytes", std::to_string(options.max_serialized_bytes)},
    {"max_output_bytes", std::to_string(options.max_output_bytes)},
    {"rgb_max_pixels", std::to_string(options.rgb_limits.max_pixels)},
    {"rgb_max_input_bytes", std::to_string(options.rgb_limits.max_input_bytes)},
    {"depth_max_pixels", std::to_string(options.depth_limits.max_pixels)},
    {"depth_max_input_bytes", std::to_string(options.depth_limits.max_input_bytes)},
    {"max_tf_edges", std::to_string(options.tf_limits.max_edges)},
    {"max_tf_samples", std::to_string(options.tf_limits.max_samples)}};
}

/** @brief 记录实际有效配置，不写 API 凭据；null max 深度表示无上界。 */
Json effectiveOptions(const BagBuildOptions & options)
{
  const auto camera = [](const CameraInfoMatchOptions & value) {
      Json result{{"image_topic", value.image_topic}, {"camera_info_topic", value.camera_info_topic},
        {"optical_frame", value.optical_frame}, {"binding_basis", value.binding_basis},
        {"compatibility", cameraInfoCompatibilityName(value.compatibility)},
        {"max_delta_ns", std::to_string(value.max_delta_ns)}, {"image_domain", "raw"}, {"time_policy", "synchronized"}};
      if (value.image_domain == CameraImageDomain::rectified) {result["image_domain"] = "rectified";}
      if (value.time_policy == CameraInfoTimePolicy::static_calibration) {result["time_policy"] = "static_calibration";}
      return result;
    };
  Json depth{{"input_unit", depthUnitName(options.depth.input_unit)}, {"convention", "optical_z"},
    {"min_depth_m", options.depth.min_depth_m}, {"max_depth_m", options.depth.max_depth_m},
    {"alignment_basis", options.depth.alignment_basis}, {"alignment", "unknown"},
    {"rgb_grid", {{"width", options.depth.rgb_grid.width}, {"height", options.depth.rgb_grid.height},
        {"frame_id", options.depth.rgb_grid.frame_id}}}};
  if (options.depth.alignment == DepthAlignment::aligned_to_rgb) {
    depth["alignment"] = "aligned_to_rgb";
  } else if (options.depth.alignment == DepthAlignment::not_aligned) {depth["alignment"] = "not_aligned";}
  return {{"clock_id", options.clock_id}, {"clock_basis", options.clock_basis}, {"clock_relation", "same_clock"},
    {"segment_id", options.segment_id}, {"segment_basis", options.segment_basis},
    {"map_frame", options.map_frame}, {"body_frame", options.body_frame},
    {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)},
    {"tf_max_gap_ns", std::to_string(options.tf_max_gap_ns.value())},
    {"tf_max_translation_step_m", options.tf_jump_limits.translation_m ?
        Json(*options.tf_jump_limits.translation_m) : Json(nullptr)},
    {"tf_max_rotation_step_rad", options.tf_jump_limits.rotation_rad ?
        Json(*options.tf_jump_limits.rotation_rad) : Json(nullptr)},
    {"max_header_reorder_ns", std::to_string(options.max_header_reorder_ns)},
    {"rgb_camera", camera(options.rgb_camera)}, {"depth_camera", camera(options.depth_camera)}, {"depth", depth},
    {"depth_storage", bagDepthStorageName(options.depth_storage)},
    {"save_raw_cdr", options.save_raw_cdr},
    {"min_valid_depth_ratio", options.min_valid_depth_ratio}, {"tf_topic", options.tf_topic},
    {"tf_static_topic", options.tf_static_topic}, {"limits", effectiveLimits(options)}};
}

/** @brief 分段/所需链的共同覆盖仅计算一次，逐帧查询不重扫全量历史。 */
Json coverageReports(const BagBuildOptions & options, const BagScan & scan)
{
  std::set<std::string> sources{options.body_frame, options.rgb_camera.optical_frame,
    options.depth_camera.optical_frame};
  Json reports = Json::array();
  for (const auto & source : sources) {
    const auto chain = scan.tf_index.inspectChain(options.map_frame, source, {options.tf_max_gap_ns.value()});
    Json intervals = Json::array();
    for (const auto & interval : chain.intervals) {
      intervals.push_back({{"start", stampJson(interval.start)}, {"end", stampJson(interval.end)}});
    }
    reports.push_back(
      {{"target_frame", options.map_frame}, {"source_frame", source},
        {"code", tfChainCodeName(chain.code)}, {"coverage_status", tfCoverageStatusName(chain.coverage_status)},
        {"frames", chain.frames}, {"failed_frame", chain.failed_frame}, {"intervals", intervals}});
  }
  return reports;
}

/** @brief 汇总输入、算法和质量数量；配对守恒只针对有效时间引用输入。 */
Json summaryRecord(
  const BagBuildOptions & options, const BagScan & scan,
  const RgbdSyncReport & sync, const BagBuildResult & result)
{
  return {{"status", bagBuildStatusName(result.status)}, {"messages_scanned", scan.messages.size()},
    {"rgb_frames", result.rgb_frames}, {"depth_frames", result.depth_frames}, {"decoded_frames", result.decoded_frames},
    {"valid_time_rgb", sync.rgb_count}, {"valid_time_depth", sync.depth_count}, {"pairs", result.pairs},
    {"exact_pairs", sync.exact_count}, {"approximate_pairs", sync.pairs.size() - sync.exact_count},
    {"timestamp_match_policy", rgbdSyncPolicyName(options.sync_tolerance_ns)},
    {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)},
    {"timestamp_match_status", result.timestamp_match_status}, {"capture_sync_status", "unknown"},
    {"unmatched_rgb", sync.unmatched_rgb.size()}, {"unused_depth", sync.unused_depth.size()},
    {"max_abs_delta_ns", std::to_string(sync.max_abs_delta_ns)}, {"prepared_pairs", result.prepared_pairs},
    {"pending_pairs", result.pairs - result.prepared_pairs}, {"issue_count", result.issue_count},
    {"tf_raw_entries", scan.tf_entries}, {"tf_inserted", scan.tf_inserted}, {"tf_duplicates", scan.tf_duplicates},
    {"tf_unique_samples", scan.tf_index.sampleCount()}, {"numeric_poses_computed", result.posed_frames > 0},
    {"posed_frames", result.posed_frames}, {"posed_pairs", result.posed_pairs},
    {"effective_models_computed", result.camera_models > 0}, {"camera_models", result.camera_models},
    {"keyframes_selected", false}};
}

/** @brief 写完整数据集说明及来源，字段/版本明确区分 preparation 和正式 memory。 */
void writeManifest(
  const BagBuildOptions & options, const BagScan & scan, const BagBuildResult & result,
  bool automatic_tf_gap, std::string_view media_index_sha256, OutputFiles & output)
{
  Json sources = Json::array();
  for (const auto & file : scan.source_files) {
    sources.push_back({{"path", file.path}, {"bytes", std::to_string(file.bytes)}, {"sha256", file.sha256}});
  }
  Json topics = Json::array();
  for (const auto & topic : scan.topics) {
    topics.push_back(
      {{"name", topic.name}, {"type", topic.type}, {"serialization_format", topic.serialization_format},
        {"offered_qos_profiles", topic.offered_qos_profiles}, {"declared_count", std::to_string(topic.declared_count)},
        {"scanned_count", std::to_string(topic.scanned_count)}, {"selected", topic.selected}});
  }
  writeJsonFile(
    output, "bag_manifest.json", {{"artifact_kind", "bag_preparation"}, {"builder_version", 7},
      {"status", bagBuildStatusName(result.status)}, {"bag_id", scan.bag_id}, {"input_scope", scan.input_scope},
      {"timestamp_match_policy", rgbdSyncPolicyName(options.sync_tolerance_ns)},
      {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)},
      {"timestamp_match_status", result.timestamp_match_status}, {"capture_sync_status", "unknown"},
      {"metadata_sha256", scan.metadata_sha256}, {"source_files", sources}, {"topics", topics},
      {"media_integrity", {{"schema_version", detail::kPreparationMediaIndexSchema},
          {"baseline", "exported"}, {"index_sha256", media_index_sha256}}},
      {"effective_options", effectiveOptions(options)}, {"tf_coverage", coverageReports(options, scan)},
      {"tf_gap_policy", automatic_tf_gap ? "twice_lower_median_v1" : "explicit"},
      {"files", {"messages.jsonl", "frames.jsonl", "rgbd_pairs.jsonl", "camera_info.jsonl", "tf.jsonl",
          "reports/issues.jsonl", "reports/pending.jsonl", "reports/summary.json",
          detail::kPreparationMediaIndexFile}},
      {"pose_computed", result.posed_frames > 0}, {"effective_model_computed", result.camera_models > 0},
      {"camera_models", result.camera_models}, {"keyframes_selected", false}});
}

/** @brief 校验实际保存的原始/标准文件引用，避免在引用断裂时发布。 */
void validateReferences(
  const fs::path & root, const BagScan & scan, const std::map<std::string,
  PreparedFrame> & frames)
{
  for (const auto & message : scan.messages) {
    if (message.raw_cdr_saved &&
      !fs::is_regular_file(root / ("evidence/raw/" + message.message_id + ".cdr"))) {
      throw BagBuildError("broken_reference", "Missing raw evidence file");
    }
  }
  for (const auto & frame : frames) {
    if (!frame.second.media_path.empty() && !fs::is_regular_file(root / frame.second.media_path)) {
      throw BagBuildError("broken_reference", "Missing normalized image evidence");
    }
  }
}

/** @brief 从正式位姿引用中收集原始 TF 消息，只保留实际用到的端点。 */
void addPoseSources(std::set<std::string> & ids, const std::optional<TfPoseResult> & pose)
{
  if (!pose) {return;}
  for (const auto & edge : pose->time.edges) {
    ids.insert(edge.before.source_message_id);
    if (edge.after) {ids.insert(edge.after->source_message_id);}
  }
}

/** @brief 为正式观测保留选中的原始 CameraInfo 及其配置，有效模型另存。 */
PreparedCalibration directCalibration(
  const PreparedFrame & frame, const CameraInfoMatchOptions & options,
  const std::map<std::string, const BagCameraInfoRecord *> & calibrations)
{
  PreparedCalibration result;
  result.selection = options;
  if (frame.camera_matched) {
    result.selection_code = "matched";
    result.calibration_id = frame.calibration_id;
    result.source_message_id = frame.calibration_id;
    result.original = calibrations.at(frame.calibration_id)->data;
  }
  return result;
}

/** @brief 固定直出来源身份及生效设置，摘要不依赖之后才写入的观测 JSONL。 */
std::string writeDirectSource(
  const BagBuildOptions & options, const BagScan & scan, const fs::path & work, OutputFiles & output)
{
  Json sources = Json::array();
  for (const auto & file : scan.source_files) {
    sources.push_back({{"path", file.path}, {"bytes", std::to_string(file.bytes)}, {"sha256", file.sha256}});
  }
  writeJsonFile(output, detail::kDirectObservationSource, {{"schema_version", "observation_source/1"},
    {"bag_id", scan.bag_id}, {"input_scope", scan.input_scope}, {"metadata_sha256", scan.metadata_sha256},
    {"source_files", sources}, {"effective_options", effectiveOptions(options)}});
  return detail::preparationFileSha256(work / detail::kDirectObservationSource);
}

/** @brief 只对实测的 camera_record 及其 30 ms 配对附加验收结论。 */
bool hasCameraRecordRgbdAudit(const BagScan & scan, const BagBuildOptions & options)
{
  return options.sync_tolerance_ns == 30000000 && options.depth.input_unit == DepthUnit::millimeters &&
    options.rgb_camera.image_topic == "/camera/color/image_raw/compressed" &&
    options.depth_camera.image_topic == "/camera/depth/image_raw/compressedDepth" &&
    options.rgb_camera.optical_frame == "camera_color_optical_frame" &&
    options.depth_camera.optical_frame == "camera_color_optical_frame" &&
    scan.source_files.size() == 1 &&
    scan.source_files.front().sha256 == "5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d";
}

/** @brief 按已固定配对和候选顺序组装正式记录；只借用当前运行中的元数据。 */
void writeDirectObservations(
  const BagBuildOptions & options, const ObservationBuildContext & context, const BuildInputOptions & selection,
  const BagScan & scan, const RgbdSyncReport & sync, const std::map<std::string, PreparedFrame> & frames,
  const fs::path & work, OutputFiles & output, BagBuildResult & result, std::vector<BagIssue> & issues)
{
  const auto source_digest = writeDirectSource(options, scan, work, output);
  const bool audited_rgbd = hasCameraRecordRgbdAudit(scan, options);
  std::map<std::string, const BagMessageRef *> messages;
  for (const auto & message : scan.messages) {messages.emplace(message.message_id, &message);}
  std::map<std::string, const BagCameraInfoRecord *> calibrations;
  for (const auto & calibration : scan.calibrations) {calibrations.emplace(calibration.source.message_id, &calibration);}

  std::vector<BuildInputFrame> candidates;
  candidates.reserve(sync.pairs.size());
  for (const auto & pair : sync.pairs) {
    const auto & rgb = frames.at(pair.rgb.message_id);
    const auto & depth = frames.at(pair.depth.message_id);
    BuildInputFrame item;
    item.source_index = candidates.size();
    item.pair_id = "p_" + pair.rgb.message_id;
    item.bag_id = scan.bag_id;
    item.map_frame = options.map_frame;
    item.clock_id = options.clock_id;
    item.segment_id = options.segment_id;
    item.rgb_message_id = pair.rgb.message_id;
    item.depth_message_id = pair.depth.message_id;
    item.rgb_stamp = pair.rgb.stamp;
    item.depth_stamp = pair.depth.stamp;
    item.delta_ns = pair.delta_ns;
    item.capture_sync_status = "unknown";
    item.pending_reasons = pairReasons(rgb, depth);
    item.pair_status = item.pending_reasons.empty() ? "prepared_for_geometry" : "pending";
    if (rgb.body_pose) {item.body_pose_map = rgb.body_pose->pose;}
    if (item.pending_reasons.empty()) {++result.prepared_pairs;}
    else {issues.push_back({"prepare", "pair_pending", "Pair has failed sensor preparation checks", {}, item.pair_id});}
    if (rgb.camera_pose_ready && depth.camera_pose_ready && rgb.body_pose_ready) {++result.posed_pairs;}
    if (rgb.media_path.empty() || depth.media_path.empty()) {
      item.eligibility_reasons.push_back("missing_rgbd_media");
    }
    if (selection.purpose != BuildInputPurpose::rgb &&
      (!depth.depth_storage || !depth.model || !depth.camera_pose)) {
      item.eligibility_reasons.push_back("missing_depth_geometry");
    }
    if (selection.purpose == BuildInputPurpose::prepared_rgbd && !item.pending_reasons.empty()) {
      item.eligibility_reasons.push_back("pair_not_prepared");
    }
    item.eligible = item.eligibility_reasons.empty();
    candidates.push_back(std::move(item));
  }
  result.pairs = candidates.size();
  selectBuildInputFrames(candidates, selection);
  for (const auto & item : candidates) {
    if (!item.selected) {
      if (item.selection_reason != "below_motion_threshold") {
        const auto & reason = item.eligible ? item.selection_reason : item.eligibility_reasons.front();
        issues.push_back({"selection", reason, "Pair excluded from direct observations", {}, item.pair_id});
      }
      continue;
    }
    const auto & rgb = frames.at(item.rgb_message_id);
    const auto & depth = frames.at(item.depth_message_id);
    PreparedPair pair;
    pair.bag_id = scan.bag_id;
    pair.pair_id = item.pair_id;
    pair.map_frame = options.map_frame;
    pair.body_frame = options.body_frame;
    pair.time_scope = {options.clock_id, options.segment_id};
    pair.clock_basis = options.clock_basis;
    pair.segment_basis = options.segment_basis;
    pair.source_manifest_sha256 = source_digest;
    pair.delta_ns = item.delta_ns;
    pair.sync_tolerance_ns = options.sync_tolerance_ns;
    pair.timestamp_match_policy = rgbdSyncPolicyName(options.sync_tolerance_ns);
    pair.capture_sync_status = item.capture_sync_status;
    pair.status = item.pair_status;
    pair.pending_reasons = item.pending_reasons;
    pair.rgb_message_id = item.rgb_message_id;
    pair.depth_message_id = item.depth_message_id;
    pair.rgb_image = rgb.image;
    pair.depth_image = depth.image;
    pair.rgb_media_relative = rgb.media_path;
    pair.depth_media_relative = depth.media_path;
    pair.rgb_model = rgb.model;
    pair.depth_model = depth.model;
    pair.rgb_pose = rgb.camera_pose;
    pair.depth_pose = depth.camera_pose;
    pair.body_pose = rgb.body_pose;
    pair.rgb_calibration = directCalibration(rgb, options.rgb_camera, calibrations);
    pair.depth_calibration = directCalibration(depth, options.depth_camera, calibrations);
    pair.depth_storage = depth.depth_storage;
    pair.depth_alignment = depth.depth_alignment;
    pair.alignment_basis = depth.alignment_basis;
    if (audited_rgbd && pair.depth_alignment == DepthAlignment::unknown) {
      pair.alignment_basis = "camera_record RGB-D audit: the low-motion sample did not meet the "
        "5 px edge criterion (RGB messages 2701, 3374, 3360). See the BagBuilder audit.";
      pair.capture_sync_basis = "camera_record RGB-D audit: 30 ms original-header pairing and separate "
        "optical poses; dual-time TF reduces some edge offsets, exposure synchronization is unverified.";
    }
    pair.tf_max_gap_ns = *options.tf_max_gap_ns;
    pair.tf_jump_limits = options.tf_jump_limits;

    std::set<std::string> ids{pair.rgb_message_id, pair.depth_message_id};
    if (rgb.camera_matched) {ids.insert(rgb.calibration_id);}
    if (depth.camera_matched) {ids.insert(depth.calibration_id);}
    addPoseSources(ids, pair.rgb_pose);
    addPoseSources(ids, pair.depth_pose);
    addPoseSources(ids, pair.body_pose);
    for (const auto & id : ids) {
      const auto & source = *messages.at(id);
      PreparedMessageSummary summary{source.message_id, source.topic, source.file_index, source.topic_ordinal,
        source.message_order, source.bag_record_time_ns, source.serialized_bytes, source.payload_sha256, std::nullopt};
      if (source.raw_cdr_saved) {summary.raw_path = "evidence/raw/" + id + ".cdr";}
      pair.message_summaries.push_back(std::move(summary));
    }
    std::sort(pair.message_summaries.begin(), pair.message_summaries.end(), [](const auto & a, const auto & b) {
      return std::tie(a.message_order, a.message_id) < std::tie(b.message_order, b.message_id);
    });
    const auto built = buildObservation(pair, context);
    if (!built.ok()) {throw BagBuildError("invalid_observation", item.pair_id + ": " + built.message);}
    const auto json = serializeObservation(*built.observation);
    if (!json.ok()) {throw BagBuildError("invalid_observation", item.pair_id + ": " + json.message);}
    output.appendLine(detail::kObservationDatasetRows, json.json);
    ++result.observations;
    if (!built.observation->pending_reasons.empty()) {++result.pending_observations;}
  }
}

/** @brief 写最终清单，绑定根相对观测行、源清单与媒体索引。 */
void writeDirectManifest(
  const ObservationBuildContext & context, const BuildInputOptions & selection,
  const BagBuildResult & result, const fs::path & work, OutputFiles & output)
{
  writeJsonFile(output, detail::kDirectObservationManifest, {{"artifact_kind", "observation_dataset"},
    {"schema_version", detail::kDirectObservationVersion}, {"observation_schema_version", kObservationSchemaVersion},
    {"map_id", context.map_id}, {"map_frame", context.map_frame}, {"map_basis", context.map_basis},
    {"geometry_revision", context.geometry_revision}, {"revision", std::to_string(context.revision)},
    {"observations", result.observations}, {"pending", result.pending_observations},
    {"batch_size", selection.batch_size}, {"source_manifest_sha256",
      detail::preparationFileSha256(work / detail::kDirectObservationSource)},
    {"file_sha256", {{"observations", detail::preparationFileSha256(work / detail::kObservationDatasetRows)},
      {"media", detail::preparationFileSha256(work / detail::kDirectObservationMediaIndex)}}}});
}

/** @brief 将扫描、配对、逐图转换和发布串联；大对象只在本次运行生命周期内拥有。 */
void runBuild(
  BagBuildOptions options, BagBuildResult & result,
  const ObservationBuildContext * context, const BuildInputOptions & selection)
{
  const auto final_path = detail::resolveOutputDirectory(options.output_directory);
  const auto work = detail::createWorkDirectory(options.bag_uri, final_path);
  result.work_directory = work.string();
  OutputFiles output(work, options.max_output_bytes);
  const bool write_preparation = context == nullptr;
  const auto * media_index = write_preparation ?
    detail::kPreparationMediaIndexFile : detail::kDirectObservationMediaIndex;
  if (write_preparation) {initializeOutput(output);}
  else {
    output.ensureLines(detail::kObservationDatasetRows);
    output.ensureLines(media_index);
  }
  auto scan = scanBag(options, output, write_preparation);
  const bool automatic_tf_gap = !options.tf_max_gap_ns.has_value();
  if (automatic_tf_gap) {
    options.tf_max_gap_ns = scan.tf_index.inferInterpolationGapNs(options.map_frame,
      {options.body_frame, options.rgb_camera.optical_frame, options.depth_camera.optical_frame});
  }
  const auto sync = synchronize(options, scan);
  result.timestamp_match_status = timestampMatchStatus(sync, options.sync_tolerance_ns);
  const auto rgb_grids = pairedRgbGrids(scan, sync);
  const CameraInfoIndex calibrations(scan.calibrations);
  std::map<std::string, PreparedFrame> frames;
  auto issues = std::move(scan.issues);
  std::set<std::string> paired_rgb;
  if (!write_preparation) {
    for (const auto & pair : sync.pairs) {paired_rgb.insert(pair.rgb.message_id);}
  }
  readBagImages(
    options, scan, [&](const auto & image, const auto * message) {
      const bool is_rgb = image.source.topic == options.rgb_camera.image_topic;
      if (is_rgb) {++result.rgb_frames;} else {++result.depth_frames;}
      if (!write_preparation && !(is_rgb ? paired_rgb.count(image.source.message_id) :
        rgb_grids.count(image.source.message_id))) {return;}
      auto prepared = processImage(
        image, message, options, scan, calibrations, rgb_grids, output, issues, write_preparation);
      if (!prepared.media_path.empty()) {
        ++result.decoded_frames;
        const auto digest = detail::preparationFileSha256(work / prepared.media_path);
        output.appendLine(media_index,
          detail::preparationMediaRow(image.source.message_id, prepared.media_path, digest).dump());
      }
      if (prepared.model_ready) {++result.camera_models;}
      if (prepared.camera_pose_ready &&
        (image.source.topic != options.rgb_camera.image_topic || prepared.body_pose_ready))
      {
        ++result.posed_frames;
      }
      frames.emplace(image.source.message_id, std::move(prepared));
    });
  if (result.rgb_frames == 0 || result.depth_frames == 0) {
    issues.push_back({"scan", "empty_image_stream", "A configured image stream contains no messages", {}, {}});
  }
  if (result.timestamp_match_status == "no_exact_pairs") {
    issues.push_back({"sync", "no_exact_rgbd_pairs", "No equal original RGB/Depth header timestamps", {}, {}});
  } else if (result.timestamp_match_status == "no_pairs") {
    issues.push_back({"sync", "no_rgbd_pairs", "No RGB/Depth pairs within the configured tolerance", {}, {}});
  }
  if (write_preparation) {
    writePairs(options, sync, frames, output, result, issues);
  } else {
    writeDirectObservations(options, *context, selection, scan, sync, frames, work, output, result, issues);
  }
  writeUnmatched(sync, output, issues, write_preparation ? "reports/pending.jsonl" : "evidence/pending.jsonl");
  for (const auto & issue : issues) {
    writeJsonLine(output, write_preparation ? "reports/issues.jsonl" : "evidence/issues.jsonl", issueJson(issue));
  }
  result.issue_count = issues.size();
  result.status = BagBuildStatus::completed;
  if (!issues.empty()) {result.status = BagBuildStatus::completed_with_issues;}
  validateReferences(work, scan, frames);
  auto summary = summaryRecord(options, scan, sync, result);
  if (!write_preparation) {
    summary["observations"] = result.observations;
    summary["pending_observations"] = result.pending_observations;
    summary["keyframes_selected"] = (selection.distance_m || selection.yaw_rad) && result.observations > 0;
  }
  writeJsonFile(output, write_preparation ? "reports/summary.json" : "evidence/summary.json", summary);
  output.close();
  if (write_preparation) {
    writeManifest(options, scan, result, automatic_tf_gap, detail::preparationFileSha256(work / media_index), output);
  } else {
    writeDirectManifest(*context, selection, result, work, output);
  }
  detail::publishDirectory(work, final_path);
  result.output_directory = final_path.string();
  result.work_directory.clear();
  result.code = "ok";
}

/** @brief 顶层将可预期/系统错误转为失败结果，已存在输出不变且失败不标记完成。 */
BagBuildResult build(
  const BagBuildOptions & options, const ObservationBuildContext * context, const BuildInputOptions & selection)
{
  BagBuildResult result;
  try {
    const auto errors = validateBagBuildOptions(options);
    if (!errors.empty()) {throw BagBuildError("invalid_config", errors.front());}
    if (context) {
      /** @brief 即便没有选中观测，也不能发布无效的地图上下文。 */
      const auto text = [](std::string_view value) {
          return value.find_first_not_of(' ') != std::string_view::npos &&
            std::none_of(value.begin(), value.end(), [](unsigned char byte) {return byte < 32 || byte == 127;});
        };
      if (!text(context->map_id) || context->map_id.size() > 4096 ||
        !text(context->geometry_revision) || context->geometry_revision.size() > 4096 ||
        !text(context->map_basis) || context->map_frame != options.map_frame || context->revision == 0 ||
        !std::all_of(context->additional_pending_reasons.begin(), context->additional_pending_reasons.end(), text)) {
        throw BagBuildError("invalid_config", "Direct output requires valid map ID, geometry revision, map frame and basis");
      }
      std::vector<BuildInputFrame> empty;
      selectBuildInputFrames(empty, selection);
    }
    runBuild(options, result, context, selection);
  } catch (const BagBuildError & error) {
    result.status = BagBuildStatus::failed;
    result.code = error.code();
    result.message = error.what();
  } catch (const std::bad_alloc & error) {
    result.status = BagBuildStatus::failed;
    result.code = "resource_error";
    result.message = error.what();
  } catch (const std::exception & error) {
    result.status = BagBuildStatus::failed;
    result.code = "io_or_runtime_error";
    result.message = error.what();
  }
  return result;
}

}  // namespace

BagBuildResult buildBag(const BagBuildOptions & options)
{
  return build(options, nullptr, {});
}

BagBuildResult buildBagObservations(
  const BagBuildOptions & options, const ObservationBuildContext & context, const BuildInputOptions & selection)
{
  return build(options, &context, selection);
}

}  // namespace vln_memory
