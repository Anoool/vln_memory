#include <vln_memory/engine/build_input.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <nlohmann/json.hpp>
#include <vln_memory/bag/builder/bag_types.hpp>
#include "../bag/builder/preparation_storage.hpp"
#include "build_input_internal.hpp"

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;

/** @brief 拒绝未知用途、零批次和非有限/非正阈值，任何选择发生前调用。 */
void validateOptions(const BuildInputOptions & options)
{
  if (std::string(buildInputPurposeName(options.purpose)) == "unknown" || options.batch_size == 0) {
    throw std::invalid_argument("Unknown purpose or zero batch_size");
  }
  for (const auto & value : {options.distance_m, options.yaw_rad, options.max_step_m, options.max_step_yaw_rad}) {
    if (value && (!std::isfinite(*value) || *value <= 0)) {
      throw std::invalid_argument("Sampling thresholds must be finite and positive");
    }
  }
  const auto pi = std::acos(-1.0);
  if ((options.yaw_rad && *options.yaw_rad > pi) ||
    (options.max_step_yaw_rad && *options.max_step_yaw_rad > pi)) {
    throw std::invalid_argument("Yaw thresholds must not exceed pi");
  }
}

/** @brief 取 map Z 轴 yaw；数值位姿无效或底盘前向投影退化时返回空。 */
std::optional<double> poseYaw(const TfPose & pose)
{
  if (!transformTfPoint(pose, {0, 0, 0})) {return std::nullopt;}
  const auto & q = pose.rotation_xyzw;
  const double x = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  const double y = 2 * (q[0] * q[1] + q[3] * q[2]);
  if (std::hypot(x, y) < 1e-12) {return std::nullopt;}
  return std::atan2(y, x);
}

/** @brief 三维 map 平移的欧氏距离，单位米。 */
double distance(const TfPose & first, const TfPose & second)
{
  const auto & a = first.translation_m;
  const auto & b = second.translation_m;
  return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** @brief 返回绕 map Z 的最短角差，处理正负 pi 环绕。 */
double yawDifference(double first, double second)
{
  return std::abs(std::atan2(std::sin(first - second), std::cos(first - second)));
}

/** @brief 检查步进或关键帧阈值，配置的任一条件达到即满足。 */
bool reaches(
  const TfPose & current, double yaw, const TfPose & reference, double reference_yaw,
  const std::optional<double> & meters, const std::optional<double> & radians)
{
  return (meters && distance(current, reference) >= *meters) ||
         (radians && yawDifference(yaw, reference_yaw) >= *radians);
}

/** @brief 提取读取器已核验的来源和用途要求，不复制图像/深度像素。 */
void describePair(BuildInputFrame & frame, const PreparedPair & pair, BuildInputPurpose purpose)
{
  frame.bag_id = pair.bag_id;
  frame.map_frame = pair.map_frame;
  frame.clock_id = pair.time_scope.clock_id;
  frame.segment_id = pair.time_scope.segment_id;
  frame.capture_sync_status = pair.capture_sync_status;
  if (pair.body_pose && pair.body_pose->ok()) {frame.body_pose_map = pair.body_pose->pose;}
  if (purpose == BuildInputPurpose::rgb || purpose == BuildInputPurpose::prepared_rgbd) {
    if (!pair.rgb_media) {frame.eligibility_reasons.push_back("missing_rgb_media");}
  }
  if (purpose == BuildInputPurpose::depth_map || purpose == BuildInputPurpose::prepared_rgbd) {
    if (!pair.depth) {frame.eligibility_reasons.push_back("missing_depth");}
    if (!pair.depth_model) {frame.eligibility_reasons.push_back("missing_depth_model");}
    if (!pair.depth_pose || !pair.depth_pose->ok()) {
      frame.eligibility_reasons.push_back("missing_depth_map_pose");
    }
  }
  if (purpose == BuildInputPurpose::prepared_rgbd) {
    if (!pair.rgb_model) {frame.eligibility_reasons.push_back("missing_rgb_model");}
    if (!pair.rgb_pose || !pair.rgb_pose->ok()) {
      frame.eligibility_reasons.push_back("missing_rgb_map_pose");
    }
    if (pair.status != "prepared_for_geometry") {frame.eligibility_reasons.push_back("pair_not_prepared");}
  }
  frame.eligible = frame.eligibility_reasons.empty();
}

/** @brief JSON 时间保持整数 sec/nanosec，纳秒差不经浮点转换。 */
Json stampJson(const SensorStamp & stamp)
{
  return {{"sec", stamp.sec}, {"nanosec", stamp.nanosec}};
}

/** @brief 将一条轻量决策保存为可回查记录；所有可选值显式写 null。 */
Json frameJson(const BuildInputFrame & frame)
{
  Json value{{"source_index", frame.source_index}, {"pair_id", frame.pair_id},
    {"bag_id", frame.bag_id}, {"map_frame", frame.map_frame}, {"clock_id", frame.clock_id},
    {"segment_id", frame.segment_id}, {"rgb_message_id", frame.rgb_message_id},
    {"depth_message_id", frame.depth_message_id}, {"rgb_stamp", stampJson(frame.rgb_stamp)},
    {"depth_stamp", stampJson(frame.depth_stamp)}, {"delta_ns", std::to_string(frame.delta_ns)},
    {"pair_status", frame.pair_status}, {"capture_sync_status", frame.capture_sync_status},
    {"pending_reasons", frame.pending_reasons}, {"read_code", detail::preparationReadCodeName(frame.read_code)},
    {"read_message", frame.read_message}, {"eligible", frame.eligible},
    {"eligibility_reasons", frame.eligibility_reasons}, {"selected", frame.selected},
    {"selection_reason", frame.selection_reason}, {"selection_index", nullptr},
    {"batch_index", nullptr}, {"body_pose_map", nullptr}};
  if (frame.selection_index) {value["selection_index"] = *frame.selection_index;}
  if (frame.batch_index) {value["batch_index"] = *frame.batch_index;}
  if (frame.body_pose_map) {
    value["body_pose_map"] = {{"translation_m", frame.body_pose_map->translation_m},
      {"rotation_xyzw", frame.body_pose_map->rotation_xyzw}};
  }
  return value;
}

/** @brief 完整写入清单；关闭后检查错误，半文件不能成为正式产物。 */
void writeManifest(const fs::path & path, const Json & value)
{
  std::ofstream stream(path);
  stream << value.dump(2) << '\n';
  stream.close();
  if (!stream) {throw std::runtime_error("Cannot write " + path.string());}
}

}  // namespace

/** @brief 转成稳定错误名称，用于持久清单；不把读取成功当作业务可用。 */
const char * detail::preparationReadCodeName(PreparationReadCode code)
{
  switch (code) {
    case PreparationReadCode::ready: return "ready";
    case PreparationReadCode::unsupported_dataset: return "unsupported_dataset";
    case PreparationReadCode::pair_not_found: return "pair_not_found";
    case PreparationReadCode::invalid_metadata: return "invalid_metadata";
    case PreparationReadCode::invalid_reference: return "invalid_reference";
    case PreparationReadCode::invalid_media: return "invalid_media";
    case PreparationReadCode::io_error: return "io_error";
    case PreparationReadCode::source_changed: return "source_changed";
  }
  return "unknown";
}

/** @brief 保存实际生效的阈值和角度单位，空阈值显式关闭。 */
Json detail::buildInputOptionsJson(const BuildInputOptions & options)
{
  Json value{{"purpose", buildInputPurposeName(options.purpose)}, {"batch_size", options.batch_size},
    {"distance_m", nullptr}, {"yaw_rad", nullptr}, {"max_step_m", nullptr},
    {"max_step_yaw_rad", nullptr}};
  if (options.distance_m) {value["distance_m"] = *options.distance_m;}
  if (options.yaw_rad) {value["yaw_rad"] = *options.yaw_rad;}
  if (options.max_step_m) {value["max_step_m"] = *options.max_step_m;}
  if (options.max_step_yaw_rad) {value["max_step_yaw_rad"] = *options.max_step_yaw_rad;}
  return value;
}

/** @brief 返回不依赖枚举数值的用途名称。 */
const char * buildInputPurposeName(BuildInputPurpose purpose) noexcept
{
  switch (purpose) {
    case BuildInputPurpose::rgb: return "rgb";
    case BuildInputPurpose::depth_map: return "depth_map";
    case BuildInputPurpose::prepared_rgbd: return "prepared_rgbd";
  }
  return "unknown";
}

/** @brief 先验证上下文，再稳定排序并以最后关键帧采样；只保留轻量位姿。 */
void selectBuildInputFrames(std::vector<BuildInputFrame> & frames, const BuildInputOptions & options)
{
  validateOptions(options);
  std::set<std::string> ids;
  std::optional<std::tuple<std::string, std::string, std::string, std::string>> context;
  for (const auto & frame : frames) {
    if (frame.pair_id.empty() || !ids.insert(frame.pair_id).second ||
      frame.rgb_stamp.nanosec >= 1000000000U || frame.depth_stamp.nanosec >= 1000000000U) {
      throw std::invalid_argument("Invalid/duplicate pair ID or timestamp");
    }
    if (!frame.eligible) {continue;}
    const auto current = std::make_tuple(frame.bag_id, frame.map_frame, frame.clock_id, frame.segment_id);
    if (frame.bag_id.empty() || frame.map_frame.empty() || frame.clock_id.empty() ||
      frame.segment_id.empty() || (context && *context != current)) {
      throw std::invalid_argument("Candidates must share explicit bag/map/clock/segment context");
    }
    context = current;
  }
  /** @brief 仅比较 RGB 整数时间，同刻由 stable_sort 保留源序。 */
  const auto earlier = [](const BuildInputFrame & a, const BuildInputFrame & b) {
      return std::tie(a.rgb_stamp.sec, a.rgb_stamp.nanosec) <
             std::tie(b.rgb_stamp.sec, b.rgb_stamp.nanosec);
    };
  std::stable_sort(frames.begin(), frames.end(), earlier);
  const bool sample = options.distance_m || options.yaw_rad;
  const bool need_pose = sample || options.max_step_m || options.max_step_yaw_rad;
  const bool need_yaw = options.yaw_rad || options.max_step_yaw_rad;
  std::optional<TfPose> anchor;
  std::optional<TfPose> previous;
  double anchor_yaw = 0;
  double previous_yaw = 0;
  bool discontinuity = false;
  std::size_t selected = 0;
  for (auto & frame : frames) {
    frame.selected = false;
    frame.selection_index.reset();
    frame.batch_index.reset();
    frame.selection_reason = "ineligible";
    if (discontinuity) {frame.selection_reason = "trajectory_discontinuity"; continue;}
    if (!frame.eligible) {
      anchor.reset();
      previous.reset();
      continue;
    }
    double yaw = 0;
    if (need_pose) {
      std::optional<double> heading;
      if (frame.body_pose_map && need_yaw) {heading = poseYaw(*frame.body_pose_map);}
      if (!frame.body_pose_map || !transformTfPoint(*frame.body_pose_map, {0, 0, 0}) ||
        (need_yaw && !heading)) {
        frame.selection_reason = "missing_or_invalid_body_pose";
        anchor.reset();
        previous.reset();
        continue;
      }
      if (heading) {yaw = *heading;}
      if (previous && ((options.max_step_m && distance(*frame.body_pose_map, *previous) > *options.max_step_m) ||
        (options.max_step_yaw_rad && yawDifference(yaw, previous_yaw) > *options.max_step_yaw_rad))) {
        frame.selection_reason = "pose_jump";
        discontinuity = true;
        continue;
      }
      previous = frame.body_pose_map;
      previous_yaw = yaw;
    }
    if (!sample) {frame.selection_reason = "sampling_disabled";}
    else if (!anchor) {frame.selection_reason = "first_after_gap";}
    else if (reaches(*frame.body_pose_map, yaw, *anchor, anchor_yaw, options.distance_m, options.yaw_rad)) {
      frame.selection_reason = "motion_threshold";
    } else {
      frame.selection_reason = "below_motion_threshold";
      continue;
    }
    frame.selected = true;
    frame.selection_index = selected;
    frame.batch_index = selected / options.batch_size;
    ++selected;
    if (need_pose) {anchor = frame.body_pose_map; anchor_yaw = yaw;}
  }
}

/** @brief 遍历当前索引、完整核验媒体，按与 build_input 相同的规则选择。 */
detail::BuildInputScan detail::scanBuildInput(
  const PreparationReader & reader, const BuildInputOptions & options, bool retain_file_snapshots)
{
  validateOptions(options);
  BuildInputScan result;
  auto & selection = result.selection;
  selection.frames.reserve(reader.pairs().size());
  for (const auto & info : reader.pairs()) {
    BuildInputFrame frame;
    frame.source_index = selection.frames.size();
    frame.pair_id = info.pair_id;
    frame.rgb_message_id = info.rgb_message_id;
    frame.depth_message_id = info.depth_message_id;
    frame.rgb_stamp = info.rgb_stamp;
    frame.depth_stamp = info.depth_stamp;
    frame.delta_ns = info.delta_ns;
    frame.pair_status = info.status;
    frame.pending_reasons = info.pending_reasons;
    auto loaded = reader.loadPair(info.pair_id);
    frame.read_code = loaded.code;
    frame.read_message = loaded.message;
    if (loaded.code == PreparationReadCode::source_changed) {
      throw BagBuildError("source_changed", loaded.message);
    }
    if (loaded.ok()) {
      describePair(frame, *loaded.pair, options.purpose);
      if (loaded.pair->depth) {++result.verified_depth_payloads;}
      if (retain_file_snapshots) {
        for (auto & snapshot : loaded.pair->file_snapshots) {
          result.verified_files.push_back(std::move(snapshot));
        }
      }
    } else {
      frame.eligibility_reasons.push_back("read_failed");
    }
    selection.frames.push_back(std::move(frame));
  }
  selectBuildInputFrames(selection.frames, options);
  for (const auto & frame : selection.frames) {
    if (frame.selected) {
      ++selection.selected;
      selection.batches = *frame.batch_index + 1;
    }
  }
  return result;
}

/** @brief 保存输入决策和原始配对批次，供单独发布和观测数据集共同使用。 */
void detail::writeBuildInputSelection(
  const fs::path & path, const fs::path & root,
  const BuildInputResult & selection, const BuildInputOptions & options)
{
  Json rows = Json::array();
  Json batches = Json::array();
  std::size_t eligible = 0;
  for (const auto & frame : selection.frames) {
    rows.push_back(frameJson(frame));
    if (frame.eligible) {++eligible;}
    if (!frame.selected) {continue;}
    if (*frame.batch_index == batches.size()) {batches.push_back(Json::array());}
    batches.at(*frame.batch_index).push_back(frame.pair_id);
  }
  writeManifest(path, {{"schema_version", "build_input/1"}, {"artifact_kind", "build_input_selection"},
    {"preparation_root", root.string()}, {"options", detail::buildInputOptionsJson(options)},
    {"order", "rgb_stamp_stable_source_order"}, {"candidates", selection.frames.size()},
    {"eligible", eligible}, {"selected", selection.selected}, {"frames", std::move(rows)},
    {"batches", std::move(batches)}});
}

/** @brief 逐对核验、采样与原子发布；全局错误保留失败上下文且不发布目录。 */
BuildInputResult buildInputManifest(
  const fs::path & preparation, const fs::path & output, const BuildInputOptions & options)
{
  BuildInputResult result;
  try {
    validateOptions(options);
    auto opened = openPreparation(preparation);
    if (!opened.ok()) {
      result.code = detail::preparationReadCodeName(opened.code);
      result.message = opened.message;
      return result;
    }
    const auto & reader = *opened.reader;
    const auto final_path = detail::resolveOutputDirectory(output);
    result.work_directory = detail::createWorkDirectory(reader.root(), final_path);
    auto scanned = detail::scanBuildInput(reader, options, true);
    result.frames = std::move(scanned.selection.frames);
    result.selected = scanned.selection.selected;
    result.batches = scanned.selection.batches;
    detail::writeBuildInputSelection(
      result.work_directory / "build_input_manifest.json", reader.root(), result, options);
    const auto source = reader.verifyUnchanged(scanned.verified_files);
    if (!source.ok()) {throw BagBuildError(detail::preparationReadCodeName(source.code), source.message);}
    detail::publishDirectory(result.work_directory, final_path);
    result.output_directory = final_path;
    result.work_directory.clear();
    result.code = "ok";
    result.completed = true;
  } catch (const BagBuildError & error) {
    result.code = error.code();
    result.message = error.what();
  } catch (const std::invalid_argument & error) {
    result.code = "invalid_options_or_context";
    result.message = error.what();
  } catch (const std::exception & error) {
    result.code = "io_or_runtime_error";
    result.message = error.what();
  }
  return result;
}

}  // namespace vln_memory
