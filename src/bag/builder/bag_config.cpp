#include "vln_memory/bag/builder/bag_config.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <yaml-cpp/yaml.h>

#include "vln_memory/bag/common/sensor_names.hpp"

namespace vln_memory
{
namespace
{
/** @brief 只接受非空白声明，避免空格被当作时钟/标定依据。 */
bool hasText(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") != std::string::npos;
}

/** @brief 检查 frame 原始命名，不擅自去掉前导斜杠。 */
bool validFrame(const std::string & value)
{
  if (value.empty() || value.front() == '/') {return false;}
  for (const char character : value) {
    if (static_cast<unsigned char>(character) <= ' ' || character == 127) {return false;}
  }
  return true;
}

/** @brief 严格检查 YAML 字段表及重复项，不允许拼错参数后悄悄使用默认值。 */
void checkKeys(const YAML::Node & map, const std::set<std::string> & allowed)
{
  if (!map.IsMap()) {throw BagBuildError("invalid_config", "Expected a YAML mapping");}
  std::set<std::string> seen;
  for (const auto & field : map) {
    const auto key = field.first.as<std::string>();
    if (allowed.count(key) == 0 || !seen.insert(key).second) {
      throw BagBuildError("invalid_config", "Unknown or duplicate field: " + key);
    }
  }
}

/** @brief 读取可选类型化标量；不存在时保留默认值，错误类型交给加载边界报告。 */
template<typename Value>
void readScalar(const YAML::Node & map, const char * key, Value & value)
{
  if (map[key]) {value = map[key].as<Value>();}
}

/** @brief 解析非负十进制容量，拒绝负值、溢出和浮点数，不通过有符号转换绕过上限。 */
std::uint64_t readCapacity(const YAML::Node & node)
{
  const auto text = node.as<std::string>();
  std::uint64_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    throw BagBuildError("invalid_config", "Capacity must be an unsigned decimal integer");
  }
  return value;
}

/** @brief 读取 size_t 容量并核对目标平台范围；不存在时不修改输出。 */
void readSize(const YAML::Node & map, const char * key, std::size_t & value)
{
  if (!map[key]) {return;}
  const auto parsed = readCapacity(map[key]);
  if (parsed > std::numeric_limits<std::size_t>::max()) {
    throw BagBuildError("invalid_config", "Capacity does not fit size_t");
  }
  value = static_cast<std::size_t>(parsed);
}

/** @brief 尺寸同样严格按非负整数解析，避免 -1 被无符号转换为超大宽高。 */
void readDimension(const YAML::Node & map, const char * key, std::uint32_t & value)
{
  if (!map[key]) {return;}
  const auto parsed = readCapacity(map[key]);
  if (parsed > std::numeric_limits<std::uint32_t>::max()) {
    throw BagBuildError("invalid_config", "Image dimension exceeds uint32 range");
  }
  value = static_cast<std::uint32_t>(parsed);
}

/** @brief 加载单个相机的显式绑定/图像域/标定策略；不从话题后缀猜测。 */
void readCamera(const YAML::Node & node, CameraInfoMatchOptions & camera)
{
  checkKeys(
    node, {"image_topic", "camera_info_topic", "optical_frame", "binding_basis",
      "image_domain", "time_policy", "max_delta_ns"});
  readScalar(node, "image_topic", camera.image_topic);
  readScalar(node, "camera_info_topic", camera.camera_info_topic);
  readScalar(node, "optical_frame", camera.optical_frame);
  readScalar(node, "binding_basis", camera.binding_basis);
  readScalar(node, "max_delta_ns", camera.max_delta_ns);
  const auto domain = node["image_domain"].as<std::string>();
  if (domain == "raw") {camera.image_domain = CameraImageDomain::raw;} else if (domain == "rectified") {
    camera.image_domain = CameraImageDomain::rectified;
  } else {throw BagBuildError("invalid_config", "image_domain must be raw or rectified");}
  const auto policy = node["time_policy"].as<std::string>();
  if (policy == "synchronized") {
    camera.time_policy = CameraInfoTimePolicy::synchronized;
  } else if (policy == "static_calibration") {camera.time_policy = CameraInfoTimePolicy::static_calibration;} else {
    throw BagBuildError("invalid_config", "Unsupported calibration time_policy");
  }
}

/** @brief 加载深度含义和声明；只有显式 aligned_to_rgb 才接受目标网格。 */
void readDepth(const YAML::Node & node, DepthOptions & depth)
{
  checkKeys(
    node, {"input_unit", "convention", "alignment", "alignment_basis", "rgb_width",
      "rgb_height", "rgb_frame", "min_depth_m", "max_depth_m"});
  const auto unit = node["input_unit"].as<std::string>();
  if (unit == "meters") {depth.input_unit = DepthUnit::meters;} else if (unit == "millimeters") {
    depth.input_unit = DepthUnit::millimeters;
  } else {throw BagBuildError("invalid_config", "Depth input_unit must be meters or millimeters");}
  if (node["convention"].as<std::string>() != "optical_z") {
    throw BagBuildError("invalid_config", "Only optical_z depth is supported");
  }
  depth.convention = DepthConvention::optical_z;
  const auto alignment = node["alignment"].as<std::string>();
  if (alignment == "unknown") {depth.alignment = DepthAlignment::unknown;} else if (alignment == "not_aligned") {
    depth.alignment = DepthAlignment::not_aligned;
  } else if (alignment == "aligned_to_rgb") {depth.alignment = DepthAlignment::aligned_to_rgb;} else {
    throw BagBuildError("invalid_config", "Unsupported depth alignment");
  }
  readScalar(node, "alignment_basis", depth.alignment_basis);
  readDimension(node, "rgb_width", depth.rgb_grid.width);
  readDimension(node, "rgb_height", depth.rgb_grid.height);
  readScalar(node, "rgb_frame", depth.rgb_grid.frame_id);
  readScalar(node, "min_depth_m", depth.min_depth_m);
  readScalar(node, "max_depth_m", depth.max_depth_m);
}

/** @brief 加载容量与图像/TF 预算；限制为正，最终统一校验。 */
void readLimits(const YAML::Node & node, BagBuildOptions & options)
{
  checkKeys(
    node, {"max_messages", "max_frames", "max_calibrations", "max_serialized_bytes",
      "max_output_bytes", "max_tf_entries", "max_pixels", "max_input_bytes", "max_tf_edges", "max_tf_samples"});
  readSize(node, "max_messages", options.max_messages);
  readSize(node, "max_frames", options.max_frames);
  readSize(node, "max_calibrations", options.max_calibrations);
  readSize(node, "max_tf_entries", options.max_tf_entries);
  readSize(node, "max_serialized_bytes", options.max_serialized_bytes);
  readSize(node, "max_pixels", options.rgb_limits.max_pixels);
  readSize(node, "max_input_bytes", options.rgb_limits.max_input_bytes);
  options.depth_limits.max_pixels = options.rgb_limits.max_pixels;
  options.depth_limits.max_input_bytes = options.rgb_limits.max_input_bytes;
  readSize(node, "max_tf_edges", options.tf_limits.max_edges);
  readSize(node, "max_tf_samples", options.tf_limits.max_samples);
  if (node["max_output_bytes"]) {options.max_output_bytes = readCapacity(node["max_output_bytes"]);}
}

/** @brief 校验一个相机绑定，所有时间阈值独立；静态标定仍要求明确依据。 */
void checkCamera(const CameraInfoMatchOptions & camera, std::vector<std::string> & errors)
{
  if (!isResolvedTopicName(camera.image_topic) || !isResolvedTopicName(camera.camera_info_topic) ||
    !validFrame(camera.optical_frame) || !hasText(camera.binding_basis))
  {
    errors.push_back("Camera topics, optical_frame and binding_basis must be explicit");
  }
  if (camera.image_domain != CameraImageDomain::raw && camera.image_domain != CameraImageDomain::rectified) {
    errors.push_back("Camera image_domain must be raw or rectified");
  }
  if (camera.time_policy != CameraInfoTimePolicy::synchronized &&
    camera.time_policy != CameraInfoTimePolicy::static_calibration)
  {
    errors.push_back("Invalid CameraInfo time_policy");
  }
  if (camera.max_delta_ns < 0 || camera.clock_relation != SensorClockRelation::same_clock) {
    errors.push_back("Camera matching requires same_clock and nonnegative max_delta_ns");
  }
}

/** @brief 检查深度单位/注册声明，不从图像尺寸或编码推断语义。 */
void checkDepth(const DepthOptions & depth, std::vector<std::string> & errors)
{
  if ((depth.input_unit != DepthUnit::meters && depth.input_unit != DepthUnit::millimeters) ||
    depth.convention != DepthConvention::optical_z)
  {
    errors.push_back("Explicit meters/millimeters and optical_z are required");
  }
  if (!std::isfinite(depth.min_depth_m) || depth.min_depth_m < 0 ||
    std::isnan(depth.max_depth_m) || depth.max_depth_m <= depth.min_depth_m)
  {
    errors.push_back("Invalid depth range");
  }
  if (depth.alignment == DepthAlignment::aligned_to_rgb) {
    if (!hasText(depth.alignment_basis) || depth.rgb_grid.width == 0 || depth.rgb_grid.height == 0 ||
      !validFrame(depth.rgb_grid.frame_id))
    {
      errors.push_back("Registered depth requires explicit grid/frame and alignment_basis");
    }
  } else if (depth.alignment == DepthAlignment::unknown || depth.alignment == DepthAlignment::not_aligned) {
    if (depth.rgb_grid.width != 0 || depth.rgb_grid.height != 0 || !depth.rgb_grid.frame_id.empty()) {
      errors.push_back("Unregistered/unknown depth must not claim an RGB grid");
    }
  } else {errors.push_back("Invalid depth alignment enum");}
}

}  // namespace

/** @brief 汇集配置错误，无文件访问；输入/输出目录冲突由 reader/顶层进一步检查。 */
std::vector<std::string> validateBagBuildOptions(const BagBuildOptions & options)
{
  std::vector<std::string> errors;
  if (options.bag_uri.empty() || options.output_directory.empty() || options.storage_id != "sqlite3") {
    errors.push_back("Input/output paths and sqlite3 storage_id are required");
  }
  if (!hasText(options.clock_id) || !hasText(options.clock_basis) || !hasText(options.segment_id) ||
    !hasText(options.segment_basis) || options.clock_relation != SensorClockRelation::same_clock)
  {
    errors.push_back("One explicit same-clock segment and its acquisition basis are required");
  }
  if (!validFrame(options.map_frame) || !validFrame(options.body_frame) || options.sync_tolerance_ns < 0 ||
    options.tf_max_gap_ns < 0 || options.max_header_reorder_ns < 0)
  {
    errors.push_back("Invalid frames or negative time thresholds");
  }
  checkCamera(options.rgb_camera, errors);
  checkCamera(options.depth_camera, errors);
  checkDepth(options.depth, errors);
  const std::set<std::string> distinct{options.rgb_camera.image_topic, options.depth_camera.image_topic,
    options.rgb_camera.camera_info_topic, options.depth_camera.camera_info_topic, options.tf_topic,
    options.tf_static_topic};
  std::size_t expected_topics = 6;
  if (options.rgb_camera.camera_info_topic == options.depth_camera.camera_info_topic) {expected_topics = 5;}
  if (distinct.size() != expected_topics || !isResolvedTopicName(options.tf_topic) ||
    !isResolvedTopicName(options.tf_static_topic))
  {
    errors.push_back("Distinct typed stream/TF topics required; only CameraInfo topics may be shared");
  }
  if (options.depth.alignment == DepthAlignment::aligned_to_rgb &&
    (options.depth.rgb_grid.frame_id != options.rgb_camera.optical_frame ||
    options.depth_camera.optical_frame != options.rgb_camera.optical_frame))
  {
    errors.push_back("Registered depth and both camera bindings must name the declared RGB optical frame");
  }
  if (options.rgb_camera.camera_info_topic == options.depth_camera.camera_info_topic &&
    options.depth.alignment != DepthAlignment::aligned_to_rgb)
  {
    errors.push_back("Sharing CameraInfo requires an explicit registered RGB grid/basis");
  }
  if (!std::isfinite(options.min_valid_depth_ratio) || options.min_valid_depth_ratio < 0 ||
    options.min_valid_depth_ratio > 1 || options.max_messages == 0 || options.max_frames == 0 ||
    options.max_calibrations == 0 || options.max_tf_entries == 0 || options.max_serialized_bytes == 0 ||
    options.max_output_bytes == 0 ||
    options.rgb_limits.max_pixels == 0 || options.rgb_limits.max_input_bytes == 0 ||
    options.depth_limits.max_pixels == 0 || options.depth_limits.max_input_bytes == 0 ||
    options.tf_limits.max_edges == 0 || options.tf_limits.max_samples == 0)
  {
    errors.push_back("Invalid quality threshold or zero resource limit");
  }
  return errors;
}

/** @brief 加载作用域清晰的 bag 配置；路径覆盖后由 buildBag 再统一校验。 */
BagBuildOptions loadBagBuildOptions(const std::string & yaml_path)
{
  try {
    const auto root = YAML::LoadFile(yaml_path);
    checkKeys(root, {"bag"});
    const auto node = root["bag"];
    checkKeys(
      node, {"input", "output", "storage_id", "clock_id", "clock_basis", "segment_id",
        "segment_basis", "clock_relation", "map_frame", "body_frame", "sync_tolerance_ns", "tf_max_gap_ns",
        "max_header_reorder_ns", "rgb_camera", "depth_camera", "depth", "tf_topic", "tf_static_topic",
        "min_valid_depth_ratio", "limits"});
    BagBuildOptions options;
    readScalar(node, "input", options.bag_uri);
    readScalar(node, "output", options.output_directory);
    readScalar(node, "storage_id", options.storage_id);
    readScalar(node, "clock_id", options.clock_id);
    readScalar(node, "clock_basis", options.clock_basis);
    readScalar(node, "segment_id", options.segment_id);
    readScalar(node, "segment_basis", options.segment_basis);
    if (node["clock_relation"].as<std::string>() != "same_clock") {
      throw BagBuildError("invalid_config", "First version requires explicit same_clock");
    }
    options.clock_relation = SensorClockRelation::same_clock;
    readScalar(node, "map_frame", options.map_frame);
    readScalar(node, "body_frame", options.body_frame);
    readScalar(node, "sync_tolerance_ns", options.sync_tolerance_ns);
    readScalar(node, "tf_max_gap_ns", options.tf_max_gap_ns);
    readScalar(node, "max_header_reorder_ns", options.max_header_reorder_ns);
    readScalar(node, "tf_topic", options.tf_topic);
    readScalar(node, "tf_static_topic", options.tf_static_topic);
    readScalar(node, "min_valid_depth_ratio", options.min_valid_depth_ratio);
    readCamera(node["rgb_camera"], options.rgb_camera);
    readCamera(node["depth_camera"], options.depth_camera);
    options.rgb_camera.clock_relation = options.clock_relation;
    options.depth_camera.clock_relation = options.clock_relation;
    readDepth(node["depth"], options.depth);
    if (node["limits"]) {readLimits(node["limits"], options);}
    return options;
  } catch (const YAML::Exception & error) {
    throw BagBuildError("invalid_config", error.what());
  }
}

}  // namespace vln_memory
