#include <vln_memory/memory/observation.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <nlohmann/json.hpp>

namespace vln_memory
{
namespace
{
using Json = nlohmann::json;
using Code = ObservationCode;

struct ParseError : std::runtime_error
{
  /** @brief JSON 格式或类型不满足固定 schema 时停止，不返回部分对象。 */
  explicit ParseError(const std::string & message) : std::runtime_error(message) {}
};

/** @brief 抛出统一的严格解析错误。 */
void require(bool condition, const std::string & message)
{
  if (!condition) {throw ParseError(message);}
}

/** @brief 固定 schema 禁止缺失键和未知键，防止往返时悄悄丢失信息。 */
void members(const Json & value, std::initializer_list<const char *> names)
{
  require(value.is_object() && value.size() == names.size(), "Unexpected object fields");
  for (const auto * name : names) {require(value.contains(name), std::string("Missing field: ") + name);}
}

/** @brief 小整数按真实有符号/无符号类型检查，拒绝浮点和溢出转换。 */
template<typename T>
T integer(const Json & value)
{
  static_assert(sizeof(T) <= 4, "Use decimal strings for large integers");
  require(value.is_number_integer(), "Expected integer");
  if (value.is_number_unsigned()) {
    const auto number = value.get<std::uint64_t>();
    require(number <= static_cast<std::uint64_t>(std::numeric_limits<T>::max()), "Integer out of range");
    return static_cast<T>(number);
  }
  const auto number = value.get<std::int64_t>();
  require(number >= static_cast<std::int64_t>(std::numeric_limits<T>::lowest()) &&
    number <= static_cast<std::int64_t>(std::numeric_limits<T>::max()), "Integer out of range");
  return static_cast<T>(number);
}

/** @brief int64/uint64 均采用规范十进制字符串，不接受浮点、前导零或加号。 */
template<typename T>
T decimal(const Json & value)
{
  require(value.is_string(), "Expected decimal string");
  const auto & encoded = value.get_ref<const std::string &>();
  T result{};
  const auto parsed = std::from_chars(encoded.data(), encoded.data() + encoded.size(), result);
  require(parsed.ec == std::errc{} && parsed.ptr == encoded.data() + encoded.size() &&
    std::to_string(result) == encoded, "Invalid/out-of-range decimal string");
  return result;
}

/** @brief JSON 里的几何量必须为有限数值，不能接受 bool、null 或字符串。 */
double number(const Json & value)
{
  require(value.is_number(), "Expected finite number");
  const auto result = value.get<double>();
  require(std::isfinite(result), "Nonfinite number");
  return result;
}

/** @brief 固定长度矩阵拒绝多余或缺失元素。 */
template<std::size_t N>
std::array<double, N> matrix(const Json & value)
{
  require(value.is_array() && value.size() == N, "Wrong matrix length");
  std::array<double, N> result{};
  for (std::size_t i = 0; i < N; ++i) {result[i] = number(value.at(i));}
  return result;
}

/** @brief 解析有限的变长畸变数组。 */
std::vector<double> distortion(const Json & value)
{
  require(value.is_array(), "Expected distortion array");
  std::vector<double> result;
  for (const auto & item : value) {result.push_back(number(item));}
  return result;
}

/** @brief 从固定词汇表解析枚举，拒绝默认为首个枚举的隐式回退。 */
template<typename T>
T enumValue(const Json & value, std::initializer_list<std::pair<const char *, T>> choices)
{
  const auto encoded = value.get<std::string>();
  for (const auto & choice : choices) {
    if (encoded == choice.first) {return choice.second;}
  }
  throw ParseError("Unknown enum: " + encoded);
}

/** @brief 相机图像域的稳定名称；无效值只会在写前校验失败。 */
const char * domainName(CameraImageDomain value)
{
  if (value == CameraImageDomain::raw) {return "raw";}
  if (value == CameraImageDomain::rectified) {return "rectified";}
  return "unspecified";
}

/** @brief 注册声明使用固定名称，不把 unknown 提升为 aligned。 */
const char * alignmentName(DepthAlignment value)
{
  if (value == DepthAlignment::aligned_to_rgb) {return "aligned_to_rgb";}
  if (value == DepthAlignment::not_aligned) {return "not_aligned";}
  return "unknown";
}

/** @brief 声明的单位只接受源格式支持的米或毫米。 */
DepthUnit parseUnit(const Json & value)
{
  return enumValue<DepthUnit>(value, {{"m", DepthUnit::meters}, {"mm", DepthUnit::millimeters}});
}

/** @brief 图像域按 schema 明确区分 raw/rectified。 */
CameraImageDomain parseDomain(const Json & value)
{
  return enumValue<CameraImageDomain>(value,
    {{"raw", CameraImageDomain::raw}, {"rectified", CameraImageDomain::rectified}});
}

/** @brief 原始 ROS 秒和纳秒保留为有界整数。 */
Json stampJson(const SensorStamp & value)
{
  return {{"sec", value.sec}, {"nanosec", value.nanosec}};
}

/** @brief 解析时间，语义范围由共用观测校验检查。 */
SensorStamp parseStamp(const Json & value)
{
  members(value, {"sec", "nanosec"});
  return {integer<std::int32_t>(value.at("sec")), integer<std::uint32_t>(value.at("nanosec"))};
}

/** @brief 图像 stamp 放在顶层 rgb_stamp/depth_stamp，避免重复保存两个副本。 */
Json imageJson(const CameraImageReference & value)
{
  return {{"topic", value.topic}, {"width", value.width}, {"height", value.height}, {"frame_id", value.frame_id}};
}

/** @brief 按各自顶层采样时刻绑定图像网格。 */
CameraImageReference parseImage(const Json & value, const SensorStamp & stamp)
{
  members(value, {"topic", "width", "height", "frame_id"});
  return {value.at("topic").get<std::string>(), integer<std::uint32_t>(value.at("width")),
    integer<std::uint32_t>(value.at("height")), value.at("frame_id").get<std::string>(), stamp};
}

/** @brief 保留完整有效模型，不压缩成单一 pinhole 内参视图。 */
Json modelJson(const CameraModel & value)
{
  return {{"width", value.width}, {"height", value.height}, {"frame_id", value.frame_id},
    {"image_domain", domainName(value.image_domain)}, {"distortion_model", value.distortion_model},
    {"d", value.d}, {"k", value.k}, {"r", value.r}, {"p", value.p},
    {"normalization_applied", value.normalization_applied}};
}

/** @brief 解析拥有型有效模型，后续从原始标定重新推导校验。 */
CameraModel parseModel(const Json & value)
{
  members(value, {"width", "height", "frame_id", "image_domain", "distortion_model",
    "d", "k", "r", "p", "normalization_applied"});
  CameraModel result;
  result.width = integer<std::uint32_t>(value.at("width"));
  result.height = integer<std::uint32_t>(value.at("height"));
  result.frame_id = value.at("frame_id").get<std::string>();
  result.image_domain = parseDomain(value.at("image_domain"));
  result.distortion_model = value.at("distortion_model").get<std::string>();
  result.d = distortion(value.at("d"));
  result.k = matrix<9>(value.at("k"));
  result.r = matrix<9>(value.at("r"));
  result.p = matrix<12>(value.at("p"));
  result.normalization_applied = value.at("normalization_applied").get<bool>();
  return result;
}

/** @brief 原始 CameraInfo 的 ROI/binning/D8 与原始时间独立保存。 */
Json cameraInfoJson(const CameraInfoData & value)
{
  return {{"stamp", stampJson(value.stamp)}, {"topic", value.topic}, {"frame_id", value.frame_id},
    {"width", value.width}, {"height", value.height}, {"distortion_model", value.distortion_model},
    {"d", value.d}, {"k", value.k}, {"r", value.r}, {"p", value.p},
    {"binning_x", value.binning_x}, {"binning_y", value.binning_y},
    {"roi", {{"x_offset", value.roi.x_offset}, {"y_offset", value.roi.y_offset},
      {"width", value.roi.width}, {"height", value.roi.height}, {"do_rectify", value.roi.do_rectify}}}};
}

/** @brief 严格解析原始标定字段，禁止矩阵长度或整数范围静默变化。 */
CameraInfoData parseCameraInfo(const Json & value)
{
  members(value, {"stamp", "topic", "frame_id", "width", "height", "distortion_model",
    "d", "k", "r", "p", "binning_x", "binning_y", "roi"});
  CameraInfoData result;
  result.stamp = parseStamp(value.at("stamp"));
  result.topic = value.at("topic").get<std::string>();
  result.frame_id = value.at("frame_id").get<std::string>();
  result.width = integer<std::uint32_t>(value.at("width"));
  result.height = integer<std::uint32_t>(value.at("height"));
  result.distortion_model = value.at("distortion_model").get<std::string>();
  result.d = distortion(value.at("d"));
  result.k = matrix<9>(value.at("k"));
  result.r = matrix<9>(value.at("r"));
  result.p = matrix<12>(value.at("p"));
  result.binning_x = integer<std::uint32_t>(value.at("binning_x"));
  result.binning_y = integer<std::uint32_t>(value.at("binning_y"));
  const auto & roi = value.at("roi");
  members(roi, {"x_offset", "y_offset", "width", "height", "do_rectify"});
  result.roi = {integer<std::uint32_t>(roi.at("x_offset")), integer<std::uint32_t>(roi.at("y_offset")),
    integer<std::uint32_t>(roi.at("width")), integer<std::uint32_t>(roi.at("height")),
    roi.at("do_rectify").get<bool>()};
  return result;
}

/** @brief 保存标定绑定、静态复用或同步策略及其依据。 */
Json selectionJson(const CameraInfoMatchOptions & value)
{
  std::string clock = "unknown";
  if (value.clock_relation == SensorClockRelation::same_clock) {clock = "same_clock";}
  if (value.clock_relation == SensorClockRelation::different_clocks) {clock = "different_clocks";}
  return {{"camera_info_topic", value.camera_info_topic}, {"image_topic", value.image_topic},
    {"optical_frame", value.optical_frame}, {"binding_basis", value.binding_basis},
    {"image_domain", domainName(value.image_domain)},
    {"time_policy", value.time_policy == CameraInfoTimePolicy::static_calibration ? "static_calibration" : "synchronized"},
    {"clock_relation", clock}, {"max_delta_ns", std::to_string(value.max_delta_ns)},
    {"compatibility", cameraInfoCompatibilityName(value.compatibility)}};
}

/** @brief 恢复标定选择参数，保持静态标定时间与图像时间分开。 */
CameraInfoMatchOptions parseSelection(const Json & value)
{
  members(value, {"camera_info_topic", "image_topic", "optical_frame", "binding_basis",
    "image_domain", "time_policy", "clock_relation", "max_delta_ns", "compatibility"});
  CameraInfoMatchOptions result;
  result.camera_info_topic = value.at("camera_info_topic").get<std::string>();
  result.image_topic = value.at("image_topic").get<std::string>();
  result.optical_frame = value.at("optical_frame").get<std::string>();
  result.binding_basis = value.at("binding_basis").get<std::string>();
  result.image_domain = parseDomain(value.at("image_domain"));
  result.time_policy = enumValue<CameraInfoTimePolicy>(value.at("time_policy"),
    {{"static_calibration", CameraInfoTimePolicy::static_calibration}, {"synchronized", CameraInfoTimePolicy::synchronized}});
  result.clock_relation = enumValue<SensorClockRelation>(value.at("clock_relation"),
    {{"unknown", SensorClockRelation::unknown}, {"same_clock", SensorClockRelation::same_clock},
      {"different_clocks", SensorClockRelation::different_clocks}});
  result.max_delta_ns = decimal<std::int64_t>(value.at("max_delta_ns"));
  result.compatibility = enumValue<CameraInfoCompatibility>(value.at("compatibility"),
    {{"strict", CameraInfoCompatibility::strict}, {"plumb_bob_zero_tail_d8_v1", CameraInfoCompatibility::plumb_bob_zero_tail_d8}});
  return result;
}

/** @brief 原始标定和选取信息整体编码。 */
Json calibrationJson(const ObservationCalibration & value)
{
  return {{"calibration_id", value.calibration_id}, {"source_message_id", value.source_message_id},
    {"original", cameraInfoJson(value.original)}, {"selection", selectionJson(value.selection)}};
}

/** @brief 解析标定及源消息引用。 */
ObservationCalibration parseCalibration(const Json & value)
{
  members(value, {"calibration_id", "source_message_id", "original", "selection"});
  return {value.at("calibration_id").get<std::string>(), value.at("source_message_id").get<std::string>(),
    parseCameraInfo(value.at("original")), parseSelection(value.at("selection"))};
}

/** @brief 不将无上界深度范围作为 Infinity 写入 JSON。 */
Json storageJson(const ObservationDepthStorage & value)
{
  Json result{{"format", value.format}, {"dtype", value.dtype}, {"unit", depthUnitName(value.unit)},
    {"scale_to_m", value.scale_to_m}, {"invalid_rule", value.invalid_rule},
    {"source_encoding", value.source_encoding}, {"source_unit", depthUnitName(value.source_unit)},
    {"source_scale_to_m", value.source_scale_to_m}, {"min_depth_m", value.min_depth_m},
    {"max_depth_m", nullptr}, {"valid_pixels", std::to_string(value.valid_pixels)},
    {"invalid_pixels", std::to_string(value.invalid_pixels)}};
  if (value.max_depth_m) {result["max_depth_m"] = *value.max_depth_m;}
  return result;
}

/** @brief 媒体本身的单位/比例与源消息单位分别恢复。 */
ObservationDepthStorage parseStorage(const Json & value)
{
  members(value, {"format", "dtype", "unit", "scale_to_m", "invalid_rule", "source_encoding",
    "source_unit", "source_scale_to_m", "min_depth_m", "max_depth_m", "valid_pixels", "invalid_pixels"});
  ObservationDepthStorage result;
  result.format = value.at("format").get<std::string>();
  result.dtype = value.at("dtype").get<std::string>();
  result.unit = parseUnit(value.at("unit"));
  result.scale_to_m = number(value.at("scale_to_m"));
  result.invalid_rule = value.at("invalid_rule").get<std::string>();
  result.source_encoding = value.at("source_encoding").get<std::string>();
  result.source_unit = parseUnit(value.at("source_unit"));
  result.source_scale_to_m = number(value.at("source_scale_to_m"));
  result.min_depth_m = number(value.at("min_depth_m"));
  if (!value.at("max_depth_m").is_null()) {result.max_depth_m = number(value.at("max_depth_m"));}
  result.valid_pixels = decimal<std::uint64_t>(value.at("valid_pixels"));
  result.invalid_pixels = decimal<std::uint64_t>(value.at("invalid_pixels"));
  return result;
}

/** @brief 保存原始 TF 端点，不只保存重算后的位姿。 */
Json transformJson(const TfTransform & value)
{
  return {{"kind", value.kind == TfKind::dynamic ? "dynamic" : "static"},
    {"stamp", stampJson(value.stamp)}, {"parent_frame", value.parent_frame}, {"child_frame", value.child_frame},
    {"translation_m", value.translation_m}, {"rotation_xyzw", value.rotation_xyzw},
    {"source_topic", value.source_topic}, {"source_message_id", value.source_message_id},
    {"source_transform_index", std::to_string(value.source_transform_index)}};
}

/** @brief 解析 TF 端点及其消息内条目索引。 */
TfTransform parseTransform(const Json & value)
{
  members(value, {"kind", "stamp", "parent_frame", "child_frame", "translation_m", "rotation_xyzw",
    "source_topic", "source_message_id", "source_transform_index"});
  TfTransform result;
  result.kind = enumValue<TfKind>(value.at("kind"), {{"dynamic", TfKind::dynamic}, {"static", TfKind::static_transform}});
  result.stamp = parseStamp(value.at("stamp"));
  result.parent_frame = value.at("parent_frame").get<std::string>();
  result.child_frame = value.at("child_frame").get<std::string>();
  result.translation_m = matrix<3>(value.at("translation_m"));
  result.rotation_xyzw = matrix<4>(value.at("rotation_xyzw"));
  result.source_topic = value.at("source_topic").get<std::string>();
  result.source_message_id = value.at("source_message_id").get<std::string>();
  result.source_transform_index = decimal<std::size_t>(value.at("source_transform_index"));
  return result;
}

/** @brief 存在的 pose 已由写前校验保证 ready，空 pose 由外层 null 表达。 */
Json poseJson(const TfPoseResult & value)
{
  Json edges = Json::array();
  for (const auto & edge : value.time.edges) {
    Json item{{"inverted", edge.inverted}, {"before", transformJson(edge.before)}, {"after", nullptr}};
    if (edge.after) {item["after"] = transformJson(*edge.after);}
    edges.push_back(std::move(item));
  }
  return {{"stamp", stampJson(value.time.stamp)}, {"frames", value.time.frames},
    {"edges", std::move(edges)}, {"translation_m", value.pose->translation_m}, {"rotation_xyzw", value.pose->rotation_xyzw}};
}

/** @brief 解析一条存在的完整求解记录，后续校验重新计算端点与位姿。 */
TfPoseResult parsePose(const Json & value)
{
  members(value, {"stamp", "frames", "edges", "translation_m", "rotation_xyzw"});
  TfPoseResult result;
  result.code = TfPoseCode::ready;
  result.time.code = TfTimeCode::covered;
  result.time.chain_code = TfChainCode::connected;
  result.time.stamp = parseStamp(value.at("stamp"));
  result.time.frames = value.at("frames").get<std::vector<std::string>>();
  require(value.at("edges").is_array(), "Expected TF edges");
  for (const auto & item : value.at("edges")) {
    members(item, {"inverted", "before", "after"});
    TfSelectedEdge edge;
    edge.inverted = item.at("inverted").get<bool>();
    edge.before = parseTransform(item.at("before"));
    if (!item.at("after").is_null()) {edge.after = parseTransform(item.at("after"));}
    result.time.edges.push_back(std::move(edge));
  }
  result.pose = TfPose{matrix<3>(value.at("translation_m")), matrix<4>(value.at("rotation_xyzw"))};
  return result;
}

/** @brief 原始来源大整数全部写十进制字符串，CDR 省略时明确 null。 */
Json evidenceJson(const ObservationEvidence & value)
{
  Json result{{"message_id", value.message_id}, {"topic", value.topic},
    {"file_index", std::to_string(value.file_index)}, {"topic_ordinal", std::to_string(value.topic_ordinal)},
    {"message_order", std::to_string(value.message_order)}, {"bag_record_time_ns", std::to_string(value.bag_record_time_ns)},
    {"serialized_bytes", std::to_string(value.serialized_bytes)}, {"payload_sha256", value.payload_sha256},
    {"raw_path", nullptr}};
  if (value.raw_path) {result["raw_path"] = *value.raw_path;}
  return result;
}

/** @brief 解析消息摘要，不读取或伪造原始 CDR。 */
ObservationEvidence parseEvidence(const Json & value)
{
  members(value, {"message_id", "topic", "file_index", "topic_ordinal", "message_order",
    "bag_record_time_ns", "serialized_bytes", "payload_sha256", "raw_path"});
  ObservationEvidence result;
  result.message_id = value.at("message_id").get<std::string>();
  result.topic = value.at("topic").get<std::string>();
  result.file_index = decimal<std::uint64_t>(value.at("file_index"));
  result.topic_ordinal = decimal<std::uint64_t>(value.at("topic_ordinal"));
  result.message_order = decimal<std::uint64_t>(value.at("message_order"));
  result.bag_record_time_ns = decimal<std::int64_t>(value.at("bag_record_time_ns"));
  result.serialized_bytes = decimal<std::uint64_t>(value.at("serialized_bytes"));
  result.payload_sha256 = value.at("payload_sha256").get<std::string>();
  if (!value.at("raw_path").is_null()) {result.raw_path = value.at("raw_path").get<std::string>();}
  return result;
}

/** @brief 单条记录的固定字段编码，不添加当前时钟以保证可重复往返。 */
Json observationJson(const Observation & o)
{
  Json value{{"schema_version", kObservationSchemaVersion}, {"length_unit", "m"},
    {"rotation_convention", "xyzw"}, {"depth_convention", "optical_z"}, {"rgb_format", "png_rgb8"},
    {"id", o.id}, {"revision", std::to_string(o.revision)}, {"bag_id", o.bag_id}, {"pair_id", o.pair_id},
    {"rgb_message_id", o.rgb_message_id}, {"depth_message_id", o.depth_message_id},
    {"source_manifest_sha256", o.source_manifest_sha256}, {"clock_id", o.clock_id}, {"segment_id", o.segment_id},
    {"clock_basis", o.clock_basis}, {"segment_basis", o.segment_basis}, {"map_id", o.map_id},
    {"geometry_revision", o.geometry_revision}, {"map_frame", o.map_frame}, {"body_frame", o.body_frame},
    {"map_basis", o.map_basis}, {"stamp", stampJson(o.stamp)}, {"rgb_stamp", stampJson(o.rgb_image.stamp)},
    {"depth_stamp", stampJson(o.depth_image.stamp)}, {"rgb_image", imageJson(o.rgb_image)}, {"depth_image", imageJson(o.depth_image)},
    {"delta_ns", std::to_string(o.delta_ns)}, {"sync_tolerance_ns", std::to_string(o.sync_tolerance_ns)},
    {"time_match", o.time_match}, {"pairing_policy", o.pairing_policy}, {"source_pair_status", o.source_pair_status},
    {"source_pending_reasons", o.source_pending_reasons}, {"capture_sync_status", o.capture_sync_status},
    {"capture_sync_basis", o.capture_sync_basis}, {"depth_alignment", alignmentName(o.depth_alignment)},
    {"alignment_basis", o.alignment_basis}, {"rgb_media", nullptr}, {"depth_media", nullptr},
    {"depth_storage", nullptr}, {"rgb_calibration", nullptr}, {"depth_calibration", nullptr},
    {"rgb_model", nullptr}, {"depth_model", nullptr}, {"tf_max_gap_ns", std::to_string(o.tf_max_gap_ns)},
    {"tf_jump_limits", {{"translation_m", nullptr}, {"rotation_rad", nullptr}}},
    {"camera_pose_map", nullptr}, {"depth_camera_pose_map", nullptr}, {"body_pose_map", nullptr},
    {"evidence", Json::array()}, {"pending_reasons", o.pending_reasons}};
  if (o.rgb_media) {value["rgb_media"] = *o.rgb_media;}
  if (o.depth_media) {value["depth_media"] = *o.depth_media;}
  if (o.depth_storage) {value["depth_storage"] = storageJson(*o.depth_storage);}
  if (o.rgb_calibration) {value["rgb_calibration"] = calibrationJson(*o.rgb_calibration);}
  if (o.depth_calibration) {value["depth_calibration"] = calibrationJson(*o.depth_calibration);}
  if (o.rgb_model) {value["rgb_model"] = modelJson(*o.rgb_model);}
  if (o.depth_model) {value["depth_model"] = modelJson(*o.depth_model);}
  if (o.tf_jump_limits.translation_m) {value["tf_jump_limits"]["translation_m"] = *o.tf_jump_limits.translation_m;}
  if (o.tf_jump_limits.rotation_rad) {value["tf_jump_limits"]["rotation_rad"] = *o.tf_jump_limits.rotation_rad;}
  if (o.camera_pose_map) {value["camera_pose_map"] = poseJson(*o.camera_pose_map);}
  if (o.depth_camera_pose_map) {value["depth_camera_pose_map"] = poseJson(*o.depth_camera_pose_map);}
  if (o.body_pose_map) {value["body_pose_map"] = poseJson(*o.body_pose_map);}
  for (const auto & item : o.evidence) {value["evidence"].push_back(evidenceJson(item));}
  return value;
}

/** @brief 固定字段解码，不使用默认值掩盖来源或版本缺失。 */
Observation parseObservation(const Json & value)
{
  members(value, {"schema_version", "length_unit", "rotation_convention", "depth_convention", "rgb_format",
    "id", "revision", "bag_id", "pair_id", "rgb_message_id", "depth_message_id", "source_manifest_sha256",
    "clock_id", "segment_id", "clock_basis", "segment_basis", "map_id", "geometry_revision", "map_frame", "body_frame", "map_basis",
    "stamp", "rgb_stamp", "depth_stamp", "rgb_image", "depth_image", "delta_ns", "sync_tolerance_ns", "time_match", "pairing_policy",
    "source_pair_status", "source_pending_reasons", "capture_sync_status", "capture_sync_basis", "depth_alignment", "alignment_basis",
    "rgb_media", "depth_media", "depth_storage", "rgb_calibration", "depth_calibration", "rgb_model", "depth_model", "tf_max_gap_ns",
    "tf_jump_limits", "camera_pose_map", "depth_camera_pose_map", "body_pose_map", "evidence", "pending_reasons"});
  require(value.at("length_unit") == "m" && value.at("rotation_convention") == "xyzw" &&
    value.at("depth_convention") == "optical_z" && value.at("rgb_format") == "png_rgb8", "Unsupported geometry/media convention");
  Observation o;
  o.id = value.at("id").get<std::string>();
  o.revision = decimal<std::uint64_t>(value.at("revision"));
  o.bag_id = value.at("bag_id").get<std::string>();
  o.pair_id = value.at("pair_id").get<std::string>();
  o.rgb_message_id = value.at("rgb_message_id").get<std::string>();
  o.depth_message_id = value.at("depth_message_id").get<std::string>();
  o.source_manifest_sha256 = value.at("source_manifest_sha256").get<std::string>();
  o.clock_id = value.at("clock_id").get<std::string>();
  o.segment_id = value.at("segment_id").get<std::string>();
  o.clock_basis = value.at("clock_basis").get<std::string>();
  o.segment_basis = value.at("segment_basis").get<std::string>();
  o.map_id = value.at("map_id").get<std::string>();
  o.geometry_revision = value.at("geometry_revision").get<std::string>();
  o.map_frame = value.at("map_frame").get<std::string>();
  o.body_frame = value.at("body_frame").get<std::string>();
  o.map_basis = value.at("map_basis").get<std::string>();
  o.stamp = parseStamp(value.at("stamp"));
  o.rgb_image = parseImage(value.at("rgb_image"), parseStamp(value.at("rgb_stamp")));
  o.depth_image = parseImage(value.at("depth_image"), parseStamp(value.at("depth_stamp")));
  o.delta_ns = decimal<std::int64_t>(value.at("delta_ns"));
  o.sync_tolerance_ns = decimal<std::int64_t>(value.at("sync_tolerance_ns"));
  o.time_match = value.at("time_match").get<std::string>();
  o.pairing_policy = value.at("pairing_policy").get<std::string>();
  o.source_pair_status = value.at("source_pair_status").get<std::string>();
  o.source_pending_reasons = value.at("source_pending_reasons").get<std::vector<std::string>>();
  o.capture_sync_status = value.at("capture_sync_status").get<std::string>();
  o.capture_sync_basis = value.at("capture_sync_basis").get<std::string>();
  o.depth_alignment = enumValue<DepthAlignment>(value.at("depth_alignment"),
    {{"unknown", DepthAlignment::unknown}, {"not_aligned", DepthAlignment::not_aligned}, {"aligned_to_rgb", DepthAlignment::aligned_to_rgb}});
  o.alignment_basis = value.at("alignment_basis").get<std::string>();
  if (!value.at("rgb_media").is_null()) {o.rgb_media = value.at("rgb_media").get<std::string>();}
  if (!value.at("depth_media").is_null()) {o.depth_media = value.at("depth_media").get<std::string>();}
  if (!value.at("depth_storage").is_null()) {o.depth_storage = parseStorage(value.at("depth_storage"));}
  if (!value.at("rgb_calibration").is_null()) {o.rgb_calibration = parseCalibration(value.at("rgb_calibration"));}
  if (!value.at("depth_calibration").is_null()) {o.depth_calibration = parseCalibration(value.at("depth_calibration"));}
  if (!value.at("rgb_model").is_null()) {o.rgb_model = parseModel(value.at("rgb_model"));}
  if (!value.at("depth_model").is_null()) {o.depth_model = parseModel(value.at("depth_model"));}
  o.tf_max_gap_ns = decimal<std::int64_t>(value.at("tf_max_gap_ns"));
  const auto & limits = value.at("tf_jump_limits");
  members(limits, {"translation_m", "rotation_rad"});
  if (!limits.at("translation_m").is_null()) {o.tf_jump_limits.translation_m = number(limits.at("translation_m"));}
  if (!limits.at("rotation_rad").is_null()) {o.tf_jump_limits.rotation_rad = number(limits.at("rotation_rad"));}
  if (!value.at("camera_pose_map").is_null()) {o.camera_pose_map = parsePose(value.at("camera_pose_map"));}
  if (!value.at("depth_camera_pose_map").is_null()) {o.depth_camera_pose_map = parsePose(value.at("depth_camera_pose_map"));}
  if (!value.at("body_pose_map").is_null()) {o.body_pose_map = parsePose(value.at("body_pose_map"));}
  require(value.at("evidence").is_array(), "Expected evidence array");
  for (const auto & item : value.at("evidence")) {o.evidence.push_back(parseEvidence(item));}
  o.pending_reasons = value.at("pending_reasons").get<std::vector<std::string>>();
  return o;
}
}  // namespace

/** @brief 先做完整校验，拒绝 JSON 库把 NaN/Inf 静默写成 null。 */
ObservationJsonResult serializeObservation(const Observation & observation)
{
  const auto checked = validateObservation(observation);
  if (!checked.ok()) {return {checked.code, checked.message, {}};}
  try {
    return {Code::ready, "", observationJson(observation).dump()};
  } catch (const Json::exception & error) {
    return {Code::invalid_json, error.what(), {}};
  }
}

/** @brief 先限定输入，再以重复键/深度检查解析，最终复用相同观测校验。 */
ObservationParseResult deserializeObservation(std::string_view encoded, std::size_t max_json_bytes)
{
  if (encoded.empty() || max_json_bytes == 0 || encoded.size() > max_json_bytes) {
    return {Code::invalid_json, "Empty/oversized JSON or zero input limit", {}};
  }
  try {
    std::vector<std::set<std::string>> keys;
    /** @brief 每个对象独立记录键，遇到重复键或过深嵌套即终止。 */
    const auto callback = [&keys](int depth, Json::parse_event_t event, Json & value) {
        require(depth <= 64, "JSON nesting exceeds 64");
        if (event == Json::parse_event_t::object_start) {keys.emplace_back();}
        if (event == Json::parse_event_t::key) {
          require(!keys.empty() && keys.back().insert(value.get<std::string>()).second, "Duplicate JSON key");
        }
        if (event == Json::parse_event_t::object_end) {keys.pop_back();}
        return true;
      };
    const auto value = Json::parse(encoded.begin(), encoded.end(), callback);
    if (!value.is_object() || !value.contains("schema_version") || value.at("schema_version") != kObservationSchemaVersion) {
      return {Code::unsupported_schema, "Expected observation/1 object", {}};
    }
    auto observation = parseObservation(value);
    const auto checked = validateObservation(observation);
    if (!checked.ok()) {return {checked.code, checked.message, {}};}
    return {Code::ready, "", std::move(observation)};
  } catch (const ParseError & error) {
    return {Code::invalid_json, error.what(), {}};
  } catch (const Json::exception & error) {
    return {Code::invalid_json, error.what(), {}};
  }
}

}  // namespace vln_memory
