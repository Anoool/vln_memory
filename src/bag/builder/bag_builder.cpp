#include "vln_memory/bag/builder/bag_builder.hpp"

#include <chrono>
#include <filesystem>
#include <map>
#include <set>
#include <utility>

#include "vln_memory/bag/builder/bag_reader_ros.hpp"
#include "vln_memory/bag/builder/jsonl_writer.hpp"
#include "vln_memory/bag/builder/rgbd_sync.hpp"
#include "vln_memory/bag/depth/depth_ros.hpp"
#include "vln_memory/bag/rgb/rgb_ros.hpp"

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
  bool tf_covered{false};
  bool body_tf_covered{false};
  bool alignment_checked{false};
  bool aligned{false};
  bool depth_quality{true};
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
};

/** @brief 检查路径包含关系，按组件比较，不能用字符串前缀误判 sibling。 */
bool containsPath(const fs::path & parent, const fs::path & candidate)
{
  auto next = candidate.begin();
  for (const auto & part : parent) {
    if (next == candidate.end() || part != *next) {return false;}
    ++next;
  }
  return true;
}

/** @brief 输出与输入不重叠、旧结果不覆盖；创建相同文件系统上的唯一工作目录。 */
fs::path createWorkDirectory(const BagBuildOptions & options)
{
  const auto input = fs::weakly_canonical(options.bag_uri);
  const auto output = fs::weakly_canonical(options.output_directory);
  if (containsPath(input, output) || containsPath(output, input)) {
    throw BagBuildError("invalid_output", "Input and output paths must not contain each other");
  }
  if (fs::exists(output) || fs::is_symlink(output)) {throw BagBuildError("output_exists", "Output already exists");}
  fs::create_directories(output.parent_path());
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto work = fs::path(output.string() + ".partial-" + std::to_string(tick) + "-" + std::to_string(attempt));
    if (fs::create_directory(work)) {return work;}
  }
  throw BagBuildError("invalid_output", "Cannot allocate unique work directory");
}

/** @brief 初始化固定 JSONL 文件，零条消息/问题仍有明确的空文件契约。 */
void initializeOutput(OutputFiles & output)
{
  for (const auto * path : {"messages.jsonl", "frames.jsonl", "rgbd_pairs.jsonl", "camera_info.jsonl",
    "tf.jsonl", "reports/issues.jsonl", "reports/pending.jsonl"})
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
  sync.max_delta_ns = options.sync_tolerance_ns;
  sync.max_frames = options.max_frames;
  auto result = pairRgbDepth(streamIndex(options, scan, options.rgb_camera.image_topic),
    streamIndex(options, scan, options.depth_camera.image_topic), sync);
  if (!result.ok()) {throw BagBuildError(rgbdSyncCodeName(result.code), result.message);}
  return std::move(*result.report);
}

/** @brief 真实配对 RGB 网格按 Depth 来源索引，无像素复制和每图全表扫描。 */
std::map<std::string, ImageGeometry> pairedRgbGrids(const BagScan & scan, const RgbdSyncReport & sync)
{
  std::map<std::string, const BagImageRecord *> images;
  for (const auto & image : scan.images) {images.emplace(image.source.message_id, &image);}
  std::map<std::string, ImageGeometry> grids;
  for (const auto & pair : sync.pairs) {
    const auto & image = *images.at(pair.rgb.message_id);
    grids.emplace(pair.depth.message_id, ImageGeometry{image.width, image.height, image.frame_id});
  }
  return grids;
}

/** @brief RGB 解码成功即写 PNG，返回值不携带像素；失败保持已有核心错误码。 */
ConvertedFrame convertRgb(const BagImageRecord & image, const sensor_msgs::msg::Image & message,
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
ConvertedFrame convertDepth(const BagImageRecord & image, const sensor_msgs::msg::Image & message,
  const BagBuildOptions & options, const std::map<std::string, ImageGeometry> & rgb_grids, OutputFiles & output)
{
  ConvertedFrame result;
  const auto decoded = decodeRosDepth(message, options.depth, options.depth_limits);
  result.code = depthErrorCodeName(decoded.code);
  result.message = decoded.message;
  if (!decoded.ok()) {return result;}
  const auto & frame = *decoded.frame;
  result.media_path = writeDepth(output, image.source.message_id, frame);
  result.depth_quality = frame.valid_pixels > 0 && frame.valid_ratio >= options.min_valid_depth_ratio;
  result.details = {{"encoding", "32FC1"}, {"unit", "meters"}, {"convention", "optical_z"},
    {"source_encoding", frame.source_encoding}, {"source_unit", depthUnitName(frame.source_unit)},
    {"source_numeric_type", depthNumericTypeName(frame.source_numeric_type)},
    {"source_scale_to_m", frame.source_scale_to_m}, {"valid_pixels", frame.valid_pixels},
    {"invalid_pixels", frame.invalid_pixels}, {"valid_ratio", frame.valid_ratio},
    {"zero_pixels", frame.zero_pixels}, {"negative_pixels", frame.negative_pixels},
    {"nonfinite_pixels", frame.nonfinite_pixels}, {"out_of_range_pixels", frame.out_of_range_pixels},
    {"unrepresentable_pixels", frame.unrepresentable_pixels}, {"alignment_basis", frame.alignment_basis}};
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

/** @brief 选择标定、查询原始时刻 TF 并写入元数据；不计算/伪造位姿。 */
void attachGeometryChecks(const BagImageRecord & image, bool is_rgb, const BagBuildOptions & options,
  const BagScan & scan, const CameraInfoIndex & calibrations, PreparedFrame & prepared, Json & record)
{
  const auto & camera = options.depth_camera;
  const CameraInfoMatchOptions * binding = &camera;
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
  TfTimeOptions time;
  time.clock_id = options.clock_id;
  time.segment_id = options.segment_id;
  time.max_interpolation_gap_ns = options.tf_max_gap_ns;
  const auto camera_tf = scan.tf_index.checkTime(options.map_frame, image.frame_id, image.stamp, time);
  prepared.tf_covered = camera_tf.covered();
  record["map_camera_tf"] = tfTimeJson(camera_tf);
  if (is_rgb) {
    const auto body_tf = scan.tf_index.checkTime(options.map_frame, options.body_frame, image.stamp, time);
    prepared.body_tf_covered = body_tf.covered();
    record["map_body_tf"] = tfTimeJson(body_tf);
  }
}

/** @brief 处理单张图，核心转换/几何元数据检查分开；仅小型 PreparedFrame 跨回调持有。 */
PreparedFrame processImage(const BagImageRecord & image, const sensor_msgs::msg::Image * message,
  const BagBuildOptions & options, const BagScan & scan, const CameraInfoIndex & calibrations,
  const std::map<std::string, ImageGeometry> & rgb_grids, OutputFiles & output, std::vector<BagIssue> & issues)
{
  const bool is_rgb = image.source.topic == options.rgb_camera.image_topic;
  ConvertedFrame converted;
  if (message != nullptr) {
    if (is_rgb) {converted = convertRgb(image, *message, options, output);}
    else {converted = convertDepth(image, *message, options, rgb_grids, output);}
  } else {converted.message = image.deserialization_error;}
  PreparedFrame prepared;
  prepared.message_id = image.source.message_id;
  prepared.decode_code = converted.code;
  prepared.media_path = converted.media_path;
  prepared.depth_quality = converted.depth_quality;
  prepared.alignment_checked = converted.alignment_checked;
  prepared.aligned = converted.aligned;
  Json record{{"source", sourceJson(image.source)}, {"kind", "depth"},
    {"stamp", stampJson(image.stamp)}, {"frame_id", image.frame_id}, {"width", image.width}, {"height", image.height},
    {"source_encoding", image.encoding}, {"source_step", image.step}, {"source_is_bigendian", image.is_bigendian},
    {"decode_code", converted.code}, {"decode_message", converted.message}, {"media_path", nullptr},
    {"details", converted.details}, {"pose_computed", false}, {"effective_model_computed", false}};
  record["metadata_available"] = message != nullptr;
  record["quality_passed"] = !converted.media_path.empty() && converted.depth_quality;
  if (message == nullptr) {record["stamp"] = nullptr;}
  if (is_rgb) {record["kind"] = "rgb";}
  if (!converted.media_path.empty()) {
    record["media_path"] = converted.media_path;
    attachGeometryChecks(image, is_rgb, options, scan, calibrations, prepared, record);
  } else if (message != nullptr) {
    issues.push_back({"decode", converted.code, converted.message, image.source.message_id, {}});
  }
  writeJsonLine(output, "frames.jsonl", std::move(record));
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
  if (!rgb.tf_covered) {reasons.push_back("rgb_tf_unavailable");}
  if (!depth.tf_covered) {reasons.push_back("depth_tf_unavailable");}
  if (!rgb.body_tf_covered) {reasons.push_back("body_tf_unavailable");}
  if (!depth.alignment_checked || !depth.aligned) {reasons.push_back("depth_rgb_alignment_unverified");}
  if (!depth.depth_quality) {reasons.push_back("insufficient_valid_depth");}
  return reasons;
}

/** @brief 写配对/待处理，引用 frames.jsonl 的具体两图和标定；不会输出正式 Observation。 */
void writePairs(const RgbdSyncReport & sync, const std::map<std::string, PreparedFrame> & frames,
  OutputFiles & output, BagBuildResult & result, std::vector<BagIssue> & issues)
{
  for (const auto & pair : sync.pairs) {
    const auto & rgb = frames.at(pair.rgb.message_id);
    const auto & depth = frames.at(pair.depth.message_id);
    const auto pair_id = "p_" + pair.rgb.message_id;
    const auto reasons = pairReasons(rgb, depth);
    Json record{{"pair_id", pair_id}, {"rgb_message_id", pair.rgb.message_id},
      {"depth_message_id", pair.depth.message_id}, {"rgb_stamp", stampJson(pair.rgb.stamp)},
      {"depth_stamp", stampJson(pair.depth.stamp)}, {"delta_ns", std::to_string(pair.delta_ns)},
      {"exact", pair.delta_ns == 0}, {"policy", "rgb_monotonic_one_to_one_nearest_v1"},
      {"rgb_calibration_id", rgb.calibration_id}, {"depth_calibration_id", depth.calibration_id},
      {"status", "pending"}, {"reasons", reasons}, {"pose_computed", false}, {"effective_model_computed", false}};
    if (reasons.empty()) {record["status"] = "prepared_for_geometry"; ++result.prepared_pairs;}
    else {
      writeJsonLine(output, "reports/pending.jsonl", record);
      issues.push_back({"prepare", "pair_pending", "Pair has failed sensor preparation checks", {}, pair_id});
    }
    writeJsonLine(output, "rgbd_pairs.jsonl", std::move(record));
  }
  result.pairs = sync.pairs.size();
}

/** @brief 未配对 RGB 有明确原因；unused Depth 单独记录，不伪装成坏帧。 */
void writeUnmatched(const RgbdSyncReport & sync, OutputFiles & output, std::vector<BagIssue> & issues)
{
  for (const auto & unmatched : sync.unmatched_rgb) {
    const auto code = rgbdUnmatchedReasonName(unmatched.reason);
    writeJsonLine(output, "reports/pending.jsonl", {{"kind", "unmatched_rgb"},
      {"message_id", unmatched.rgb.message_id}, {"stamp", stampJson(unmatched.rgb.stamp)}, {"reason", code}});
    issues.push_back({"sync", code, "RGB has no selected depth", unmatched.rgb.message_id, {}});
  }
  for (const auto & depth : sync.unused_depth) {
    writeJsonLine(output, "reports/pending.jsonl", {{"kind", "unused_depth"}, {"message_id", depth.message_id},
      {"stamp", stampJson(depth.stamp)}, {"reason", "not_selected_by_policy"}, {"is_bad_frame", false}});
  }
}

/** @brief 记录实际有效配置，不写 API 凭据；null max 深度表示无上界。 */
Json effectiveOptions(const BagBuildOptions & options)
{
  const auto camera = [](const CameraInfoMatchOptions & value) {
    Json result{{"image_topic", value.image_topic}, {"camera_info_topic", value.camera_info_topic},
      {"optical_frame", value.optical_frame}, {"binding_basis", value.binding_basis},
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
  if (options.depth.alignment == DepthAlignment::aligned_to_rgb) {depth["alignment"] = "aligned_to_rgb";}
  else if (options.depth.alignment == DepthAlignment::not_aligned) {depth["alignment"] = "not_aligned";}
  return {{"clock_id", options.clock_id}, {"clock_basis", options.clock_basis}, {"clock_relation", "same_clock"},
    {"segment_id", options.segment_id}, {"segment_basis", options.segment_basis},
    {"map_frame", options.map_frame}, {"body_frame", options.body_frame},
    {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)}, {"tf_max_gap_ns", std::to_string(options.tf_max_gap_ns)},
    {"max_header_reorder_ns", std::to_string(options.max_header_reorder_ns)},
    {"rgb_camera", camera(options.rgb_camera)}, {"depth_camera", camera(options.depth_camera)}, {"depth", depth},
    {"min_valid_depth_ratio", options.min_valid_depth_ratio}, {"tf_topic", options.tf_topic},
    {"tf_static_topic", options.tf_static_topic}};
}

/** @brief 分段/所需链的共同覆盖仅计算一次，逐帧查询不重扫全量历史。 */
Json coverageReports(const BagBuildOptions & options, const BagScan & scan)
{
  std::set<std::string> sources{options.body_frame, options.rgb_camera.optical_frame, options.depth_camera.optical_frame};
  Json reports = Json::array();
  for (const auto & source : sources) {
    const auto chain = scan.tf_index.inspectChain(options.map_frame, source, {options.tf_max_gap_ns});
    Json intervals = Json::array();
    for (const auto & interval : chain.intervals) {
      intervals.push_back({{"start", stampJson(interval.start)}, {"end", stampJson(interval.end)}});
    }
    reports.push_back({{"target_frame", options.map_frame}, {"source_frame", source},
      {"code", tfChainCodeName(chain.code)}, {"coverage_status", tfCoverageStatusName(chain.coverage_status)},
      {"frames", chain.frames}, {"failed_frame", chain.failed_frame}, {"intervals", intervals}});
  }
  return reports;
}

/** @brief 汇总输入、算法和质量数量；配对守恒只针对有效时间引用输入。 */
Json summaryRecord(const BagScan & scan, const RgbdSyncReport & sync, const BagBuildResult & result)
{
  return {{"status", bagBuildStatusName(result.status)}, {"messages_scanned", scan.messages.size()},
    {"rgb_frames", result.rgb_frames}, {"depth_frames", result.depth_frames}, {"decoded_frames", result.decoded_frames},
    {"valid_time_rgb", sync.rgb_count}, {"valid_time_depth", sync.depth_count}, {"pairs", result.pairs},
    {"exact_pairs", sync.exact_count}, {"approximate_pairs", sync.pairs.size() - sync.exact_count},
    {"unmatched_rgb", sync.unmatched_rgb.size()}, {"unused_depth", sync.unused_depth.size()},
    {"max_abs_delta_ns", std::to_string(sync.max_abs_delta_ns)}, {"prepared_pairs", result.prepared_pairs},
    {"pending_pairs", result.pairs - result.prepared_pairs}, {"issue_count", result.issue_count},
    {"tf_raw_entries", scan.tf_entries}, {"tf_inserted", scan.tf_inserted}, {"tf_duplicates", scan.tf_duplicates},
    {"tf_unique_samples", scan.tf_index.sampleCount()}, {"numeric_poses_computed", false},
    {"effective_models_computed", false}, {"keyframes_selected", false}};
}

/** @brief 写完整数据集说明及来源，字段/版本明确区分 preparation 和正式 memory。 */
void writeManifest(const BagBuildOptions & options, const BagScan & scan, const BagBuildResult & result, OutputFiles & output)
{
  Json sources = Json::array();
  for (const auto & file : scan.source_files) {
    sources.push_back({{"path", file.path}, {"bytes", std::to_string(file.bytes)}, {"sha256", file.sha256}});
  }
  Json topics = Json::array();
  for (const auto & topic : scan.topics) {
    topics.push_back({{"name", topic.name}, {"type", topic.type}, {"serialization_format", topic.serialization_format},
      {"offered_qos_profiles", topic.offered_qos_profiles}, {"declared_count", std::to_string(topic.declared_count)},
      {"scanned_count", std::to_string(topic.scanned_count)}, {"selected", topic.selected}});
  }
  writeJsonFile(output, "bag_manifest.json", {{"artifact_kind", "bag_preparation"}, {"builder_version", 1},
    {"status", bagBuildStatusName(result.status)}, {"bag_id", scan.bag_id}, {"input_scope", scan.input_scope},
    {"metadata_sha256", scan.metadata_sha256}, {"source_files", sources}, {"topics", topics},
    {"effective_options", effectiveOptions(options)}, {"tf_coverage", coverageReports(options, scan)},
    {"files", {"messages.jsonl", "frames.jsonl", "rgbd_pairs.jsonl", "camera_info.jsonl", "tf.jsonl",
      "reports/issues.jsonl", "reports/pending.jsonl", "reports/summary.json"}},
    {"pose_computed", false}, {"effective_model_computed", false}, {"keyframes_selected", false}});
}

/** @brief 校验实际保存的原始/标准文件引用，避免在引用断裂时发布。 */
void validateReferences(const fs::path & root, const BagScan & scan, const std::map<std::string, PreparedFrame> & frames)
{
  for (const auto & message : scan.messages) {
    if (!fs::is_regular_file(root / ("evidence/raw/" + message.message_id + ".cdr"))) {
      throw BagBuildError("broken_reference", "Missing raw evidence file");
    }
  }
  for (const auto & frame : frames) {
    if (!frame.second.media_path.empty() && !fs::is_regular_file(root / frame.second.media_path)) {
      throw BagBuildError("broken_reference", "Missing normalized image evidence");
    }
  }
}

/** @brief 将扫描、配对、逐图转换和发布串联；大对象只在本次运行生命周期内拥有。 */
void runBuild(const BagBuildOptions & options, BagBuildResult & result)
{
  const auto work = createWorkDirectory(options);
  result.work_directory = work.string();
  OutputFiles output(work, options.max_output_bytes);
  initializeOutput(output);
  auto scan = scanBag(options, output);
  const auto sync = synchronize(options, scan);
  const auto rgb_grids = pairedRgbGrids(scan, sync);
  const CameraInfoIndex calibrations(scan.calibrations);
  std::map<std::string, PreparedFrame> frames;
  auto issues = std::move(scan.issues);
  readBagImages(options, scan, [&](const auto & image, const auto * message) {
    auto prepared = processImage(image, message, options, scan, calibrations, rgb_grids, output, issues);
    if (image.source.topic == options.rgb_camera.image_topic) {++result.rgb_frames;}
    else {++result.depth_frames;}
    if (!prepared.media_path.empty()) {++result.decoded_frames;}
    frames.emplace(image.source.message_id, std::move(prepared));
  });
  if (result.rgb_frames == 0 || result.depth_frames == 0) {
    issues.push_back({"scan", "empty_image_stream", "A configured image stream contains no messages", {}, {}});
  }
  writePairs(sync, frames, output, result, issues);
  writeUnmatched(sync, output, issues);
  for (const auto & issue : issues) {writeJsonLine(output, "reports/issues.jsonl", issueJson(issue));}
  result.issue_count = issues.size();
  result.status = BagBuildStatus::completed;
  if (!issues.empty()) {result.status = BagBuildStatus::completed_with_issues;}
  validateReferences(work, scan, frames);
  verifyBagSources(options, scan);
  writeJsonFile(output, "reports/summary.json", summaryRecord(scan, sync, result));
  writeManifest(options, scan, result, output);
  output.close();
  const auto final_path = fs::weakly_canonical(options.output_directory);
  if (fs::exists(final_path) || fs::is_symlink(final_path)) {throw BagBuildError("output_exists", "Output appeared during build");}
  fs::rename(work, final_path);
  result.output_directory = final_path.string();
  result.work_directory.clear();
  result.code = "ok";
}

}  // namespace

/** @brief 顶层将可预期/系统错误转为失败结果，已存在输出不变且失败不标记完成。 */
BagBuildResult buildBag(const BagBuildOptions & options)
{
  BagBuildResult result;
  try {
    const auto errors = validateBagBuildOptions(options);
    if (!errors.empty()) {throw BagBuildError("invalid_config", errors.front());}
    runBuild(options, result);
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

}  // namespace vln_memory
