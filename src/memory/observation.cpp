#include <vln_memory/memory/observation.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include <vln_memory/bag/common/sensor_names.hpp>

namespace vln_memory
{
namespace
{
using Code = ObservationCode;
using EvidenceIndex = std::map<std::string, const ObservationEvidence *>;

struct CheckError : std::runtime_error
{
  Code code;
  /** @brief 保存稳定校验码，供顶层转换成无部分结果的报告。 */
  CheckError(Code value, const std::string & message) : std::runtime_error(message), code(value) {}
};

/** @brief 在首次不一致处停止当前记录校验。 */
void require(bool condition, Code code, const std::string & message)
{
  if (!condition) {throw CheckError(code, message);}
}

/** @brief 接受非空文字，拒绝控制字符和仅 ASCII 空白的声明。 */
bool text(std::string_view value)
{
  bool content = false;
  for (const auto c : value) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte < 32 || byte == 127) {return false;}
    if (c != ' ') {content = true;}
  }
  return content;
}

/** @brief 身份分量长度有界；长度前缀允许普通分隔符而不产生拼接碰撞。 */
bool identity(std::string_view value)
{
  return value.size() <= 4096 && text(value);
}

/** @brief 与 CameraInfo/TF 一致，frame 拒绝前导斜杠、空白及控制字符。 */
bool frameName(const std::string & value)
{
  if (value.empty() || value.front() == '/') {return false;}
  for (const char c : value) {
    if (c <= ' ' || c == 127) {return false;}
  }
  return true;
}

/** @brief 跨平台相对证据路径，不依赖宿主平台对 Windows 盘符的解释。 */
bool relativePath(const std::string & path)
{
  if (!text(path) || path.front() == '/' || path.back() == '/' ||
    path.find('\\') != std::string::npos || path.find(':') != std::string::npos) {return false;}
  std::size_t start = 0;
  while (start < path.size()) {
    const auto end = path.find('/', start);
    const auto part = path.substr(start, end == std::string::npos ? end : end - start);
    if (part.empty() || part == "." || part == "..") {return false;}
    if (end == std::string::npos) {break;}
    start = end + 1;
  }
  return true;
}

/** @brief SHA-256 仅检查摘要格式；不以此宣称已核验源字节。 */
bool digest(const std::string & value)
{
  if (value.size() != 64) {return false;}
  for (const char c : value) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) {return false;}
  }
  return true;
}

/** @brief 比较整数时间，保留零和负秒。 */
bool sameStamp(const SensorStamp & a, const SensorStamp & b)
{
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

/** @brief 对有界几何结果使用相对/绝对容差，并明确拒绝非有限数。 */
bool close(double a, double b)
{
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a - b) <= 1e-10 * std::max({1.0, std::abs(a), std::abs(b)});
}

/** @brief 比较固定/动态长度参数数组，不忽略有效模型中的任一参数。 */
template<typename Values>
bool sameValues(const Values & a, const Values & b)
{
  if (a.size() != b.size()) {return false;}
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!close(a[i], b[i])) {return false;}
  }
  return true;
}

/** @brief 根据原始标定重建后核对有效网格和完整投影参数。 */
bool sameModel(const CameraModel & a, const CameraModel & b)
{
  return a.width == b.width && a.height == b.height && a.frame_id == b.frame_id &&
         a.image_domain == b.image_domain && a.distortion_model == b.distortion_model &&
         a.normalization_applied == b.normalization_applied && sameValues(a.k, b.k) &&
         sameValues(a.d, b.d) && sameValues(a.r, b.r) && sameValues(a.p, b.p);
}

/** @brief 建立唯一消息索引，检查摘要、话题与可选 CDR 路径。 */
EvidenceIndex checkEvidence(const Observation & observation)
{
  EvidenceIndex index;
  for (const auto & item : observation.evidence) {
    require(identity(item.message_id) && isResolvedTopicName(item.topic) &&
      digest(item.payload_sha256) && item.serialized_bytes > 0,
      Code::invalid_reference, "Invalid evidence identity/topic/digest/length");
    require(!item.raw_path || relativePath(*item.raw_path), Code::invalid_reference, "Invalid raw evidence path");
    require(index.emplace(item.message_id, &item).second, Code::invalid_reference, "Duplicate evidence message ID");
  }
  return index;
}

/** @brief 引用必须落到当前观测的消息摘要集合，且角色话题一致。 */
void checkReference(const EvidenceIndex & index, const std::string & id, const std::string & topic)
{
  const auto found = index.find(id);
  require(found != index.end() && found->second->topic == topic, Code::invalid_reference,
    "Missing evidence or topic mismatch: " + id);
}

/** @brief 校验图像元数据；无媒体的坏帧可保留未知的零尺寸及 pending。 */
void checkImage(const CameraImageReference & image)
{
  require(isResolvedTopicName(image.topic) && frameName(image.frame_id) &&
    ((image.width == 0 && image.height == 0) || (image.width > 0 && image.height > 0)),
    Code::invalid_context, "Invalid image metadata");
  require(image.stamp.nanosec < 1000000000U, Code::invalid_time, "Invalid image timestamp");
}

/** @brief 校验标定来源与实际绑定，用已有 CameraModel 实现重新派生并核对。 */
void checkCalibration(
  const std::optional<ObservationCalibration> & calibration,
  const std::optional<CameraModel> & model, const CameraImageReference & image,
  const EvidenceIndex & evidence)
{
  require(!model || calibration.has_value(), Code::invalid_calibration, "Camera model lacks raw calibration");
  if (!calibration) {return;}
  const auto & c = *calibration;
  require(identity(c.calibration_id), Code::invalid_calibration, "Empty calibration identity");
  checkReference(evidence, c.source_message_id, c.original.topic);
  require(c.selection.camera_info_topic == c.original.topic, Code::invalid_calibration,
    "Calibration topic differs from selection");
  require(c.selection.max_delta_ns >= 0 && text(c.selection.binding_basis) &&
    (c.selection.clock_relation == SensorClockRelation::unknown ||
      c.selection.clock_relation == SensorClockRelation::same_clock ||
      c.selection.clock_relation == SensorClockRelation::different_clocks),
    Code::invalid_calibration, "Invalid calibration clock/tolerance/basis");
  if (!model) {
    const auto match = checkCameraInfoImage(c.original, image, c.selection);
    require(match.matched(), Code::invalid_calibration, "Calibration/image binding: " + match.message);
    return;
  }
  const auto rebuilt = buildCameraModel(c.original, image, c.selection);
  require(rebuilt.ok(), Code::invalid_calibration, "Calibration/model binding: " + rebuilt.message);
  require(sameModel(*model, *rebuilt.model), Code::invalid_calibration,
    "Effective camera model differs from raw calibration and selection");
}

/** @brief 按单位精确解释编码比例，不从编码猜测源物理单位。 */
bool unitScale(DepthUnit unit, double scale)
{
  return (unit == DepthUnit::meters && scale == 1.0) ||
         (unit == DepthUnit::millimeters && scale == 0.001);
}

/** @brief 检查 PNG/NPY、源单位、过滤范围和质量计数，避免双重缩放。 */
void checkStorage(const Observation & observation)
{
  require(observation.depth_media.has_value() == observation.depth_storage.has_value(),
    Code::invalid_media, "Depth path and storage contract must occur together");
  if (observation.rgb_media) {
    require(relativePath(*observation.rgb_media) && observation.rgb_image.width > 0,
      Code::invalid_media, "Invalid RGB media path/grid");
  }
  if (!observation.depth_storage) {return;}
  const auto & s = *observation.depth_storage;
  require(relativePath(*observation.depth_media) && observation.depth_image.width > 0,
    Code::invalid_media, "Invalid depth media path/grid");
  const bool png = s.format == "png" && s.dtype == "uint16" && s.invalid_rule == "zero_or_out_of_range_to_nan" &&
    s.source_encoding == "16UC1" && s.unit == s.source_unit && s.scale_to_m == s.source_scale_to_m;
  const bool npy = s.format == "npy" && s.dtype == "float32" && s.unit == DepthUnit::meters &&
    s.scale_to_m == 1.0 && s.invalid_rule == "nan";
  require((png || npy) && unitScale(s.unit, s.scale_to_m) &&
    unitScale(s.source_unit, s.source_scale_to_m) &&
    (s.source_encoding == "16UC1" || s.source_encoding == "32FC1"),
    Code::invalid_media, "Depth format/dtype/unit/scale/invalid rule contradict");
  require(std::isfinite(s.min_depth_m) && s.min_depth_m >= 0 &&
    (!s.max_depth_m || (std::isfinite(*s.max_depth_m) && *s.max_depth_m > 0 &&
      *s.max_depth_m >= s.min_depth_m)), Code::invalid_media, "Invalid depth range");
  const auto pixels = static_cast<std::uint64_t>(observation.depth_image.width) * observation.depth_image.height;
  require(s.valid_pixels <= pixels && s.invalid_pixels == pixels - s.valid_pixels,
    Code::invalid_media, "Depth quality counts differ from image grid");
}

/** @brief 核验 TF 端点的角色、方向和消息引用，并交给共享索引检查数值/拓扑。 */
void insertEndpoint(
  TfIndex & index, const TfTransform & endpoint, const EvidenceIndex & evidence,
  const std::string & source, const std::string & target, bool inverted)
{
  checkReference(evidence, endpoint.source_message_id, endpoint.source_topic);
  require(endpoint.parent_frame == (inverted ? source : target) &&
    endpoint.child_frame == (inverted ? target : source), Code::invalid_pose, "TF endpoint does not join saved chain");
  const auto inserted = index.insert(endpoint);
  require(inserted.accepted(), Code::invalid_pose, "TF endpoint: " + inserted.message);
}

/** @brief 用保存的 TF 端点/限制重新求解，核对独立查询时间、坐标链和数值。 */
void checkPose(
  const std::optional<TfPoseResult> & saved, const Observation & observation,
  const std::string & source, const SensorStamp & stamp, const EvidenceIndex & evidence)
{
  if (!saved) {return;}
  const auto & s = *saved;
  require(s.code == TfPoseCode::ready && s.pose && s.time.code == TfTimeCode::covered &&
    s.time.chain_code == TfChainCode::connected && sameStamp(s.time.stamp, stamp) &&
    s.failed_frame.empty() && s.time.failed_frame.empty() &&
    !s.time.frames.empty() && s.time.frames.front() == source &&
    s.time.frames.back() == observation.map_frame && s.time.frames.size() - 1 == s.time.edges.size() &&
    transformTfPoint(*s.pose, {0, 0, 0}).has_value(), Code::invalid_pose, "Invalid pose time/frame/status/value");
  if (s.time.edges.empty()) {
    require(source == observation.map_frame && sameValues(s.pose->translation_m, std::array<double, 3>{0, 0, 0}) &&
      close(s.pose->rotation_xyzw[0], 0.0) && close(s.pose->rotation_xyzw[1], 0.0) &&
      close(s.pose->rotation_xyzw[2], 0.0) && close(std::abs(s.pose->rotation_xyzw[3]), 1.0),
      Code::invalid_pose, "Invalid identity pose");
    return;
  }
  TfIndex index({observation.clock_id, observation.segment_id});
  for (std::size_t i = 0; i < s.time.edges.size(); ++i) {
    const auto & edge = s.time.edges[i];
    insertEndpoint(index, edge.before, evidence, s.time.frames[i], s.time.frames[i + 1], edge.inverted);
    if (edge.after) {
      require(edge.before.kind == TfKind::dynamic && edge.after->kind == TfKind::dynamic,
        Code::invalid_pose, "Only dynamic TF may interpolate");
      insertEndpoint(index, *edge.after, evidence, s.time.frames[i], s.time.frames[i + 1], edge.inverted);
    }
  }
  const auto solved = index.lookupPose(observation.map_frame, source, stamp,
    {observation.clock_id, observation.segment_id, observation.tf_max_gap_ns}, observation.tf_jump_limits);
  require(solved.ok() && solved.time.frames == s.time.frames &&
    sameValues(solved.pose->translation_m, s.pose->translation_m),
    Code::invalid_pose, "Saved pose does not match TF evidence/limits");
  double dot = 0;
  for (std::size_t i = 0; i < 4; ++i) {dot += solved.pose->rotation_xyzw[i] * s.pose->rotation_xyzw[i];}
  require(close(std::abs(dot), 1.0), Code::invalid_pose, "Saved rotation differs from TF evidence");
  for (std::size_t i = 0; i < s.time.edges.size(); ++i) {
    const auto & a = s.time.edges[i];
    const auto & b = solved.time.edges[i];
    require(a.inverted == b.inverted && a.before.source_message_id == b.before.source_message_id &&
      a.before.source_transform_index == b.before.source_transform_index && a.after.has_value() == b.after.has_value(),
      Code::invalid_pose, "TF endpoint selection differs");
    if (a.after) {
      require(a.after->source_message_id == b.after->source_message_id &&
        a.after->source_transform_index == b.after->source_transform_index,
        Code::invalid_pose, "TF after endpoint selection differs");
    }
  }
}

/** @brief 检查注册/同步声明；网格一致仅支持声明一致性，不证明物理质量。 */
void checkDeclarations(const Observation & observation)
{
  require(observation.capture_sync_status == "unknown" ||
    observation.capture_sync_status == "verified_synchronized" || observation.capture_sync_status == "not_synchronized",
    Code::invalid_context, "Unknown capture sync status");
  require(observation.capture_sync_status == "unknown" || text(observation.capture_sync_basis),
    Code::invalid_context, "Capture sync declaration lacks basis");
  require(observation.depth_alignment == DepthAlignment::unknown ||
    observation.depth_alignment == DepthAlignment::not_aligned ||
    observation.depth_alignment == DepthAlignment::aligned_to_rgb, Code::invalid_context, "Unknown alignment enum");
  require(observation.depth_alignment != DepthAlignment::aligned_to_rgb || text(observation.alignment_basis),
    Code::invalid_context, "Alignment declaration lacks basis");
  if (observation.depth_alignment == DepthAlignment::aligned_to_rgb) {
    require(observation.rgb_image.width > 0 && observation.rgb_image.width == observation.depth_image.width &&
      observation.rgb_image.height == observation.depth_image.height &&
      observation.rgb_image.frame_id == observation.depth_image.frame_id,
      Code::invalid_context, "Aligned declaration has different optical grid");
  }
}

/** @brief 首次出现的原因保持源序，后续规范原因不重复追加。 */
void appendReason(std::vector<std::string> & reasons, std::set<std::string> & seen, const std::string & reason)
{
  if (seen.insert(reason).second) {reasons.push_back(reason);}
}

}  // namespace

/** @brief 返回稳定静态名称，无分配。 */
const char * observationCodeName(ObservationCode code) noexcept
{
  switch (code) {
    case ObservationCode::ready: return "ready";
    case ObservationCode::invalid_identity: return "invalid_identity";
    case ObservationCode::invalid_context: return "invalid_context";
    case ObservationCode::invalid_time: return "invalid_time";
    case ObservationCode::invalid_media: return "invalid_media";
    case ObservationCode::invalid_calibration: return "invalid_calibration";
    case ObservationCode::invalid_pose: return "invalid_pose";
    case ObservationCode::invalid_reference: return "invalid_reference";
    case ObservationCode::invalid_pending: return "invalid_pending";
    case ObservationCode::invalid_json: return "invalid_json";
    case ObservationCode::unsupported_schema: return "unsupported_schema";
  }
  return "unknown";
}

/** @brief 不用可碰撞分隔拼接或不稳定 std::hash，长度元组明确保存身份边界。 */
std::string makeObservationId(const Observation & observation)
{
  std::string id = "obs1:";
  for (const auto * value : {&observation.bag_id, &observation.pair_id, &observation.rgb_message_id,
    &observation.depth_message_id, &observation.clock_id, &observation.segment_id}) {
    if (!identity(*value)) {throw std::invalid_argument("Invalid observation source identity");}
    id += std::to_string(value->size()) + ":" + *value;
  }
  return id;
}

/** @brief 缺项与来源原因合并；本函数不改变任何状态声明或证据。 */
std::vector<std::string> collectObservationPendingReasons(const Observation & observation)
{
  std::vector<std::string> reasons;
  std::set<std::string> seen;
  for (const auto & reason : observation.source_pending_reasons) {appendReason(reasons, seen, reason);}
  for (const auto & reason : observation.pending_reasons) {appendReason(reasons, seen, reason);}
  if (!observation.rgb_media) {appendReason(reasons, seen, "missing_rgb_media");}
  if (!observation.depth_media) {appendReason(reasons, seen, "missing_depth_media");}
  if (!observation.rgb_calibration) {appendReason(reasons, seen, "missing_rgb_calibration");}
  if (!observation.depth_calibration) {appendReason(reasons, seen, "missing_depth_calibration");}
  if (!observation.rgb_model) {appendReason(reasons, seen, "missing_rgb_model");}
  if (!observation.depth_model) {appendReason(reasons, seen, "missing_depth_model");}
  if (!observation.camera_pose_map) {appendReason(reasons, seen, "missing_rgb_map_pose");}
  if (!observation.depth_camera_pose_map) {appendReason(reasons, seen, "missing_depth_map_pose");}
  if (!observation.body_pose_map) {appendReason(reasons, seen, "missing_body_map_pose");}
  if (observation.depth_storage && observation.depth_storage->valid_pixels == 0) {
    appendReason(reasons, seen, "no_valid_depth");
  }
  if (observation.depth_alignment == DepthAlignment::unknown) {appendReason(reasons, seen, "depth_rgb_alignment_unverified");}
  if (observation.depth_alignment == DepthAlignment::not_aligned) {appendReason(reasons, seen, "depth_rgb_not_aligned");}
  if (observation.capture_sync_status == "unknown") {appendReason(reasons, seen, "capture_sync_unverified");}
  if (observation.capture_sync_status == "not_synchronized") {appendReason(reasons, seen, "capture_not_synchronized");}
  return reasons;
}

/** @brief 顺序检查基础契约及引用/重算，失败保留具体字段语义而不返回半合法对象。 */
ObservationCheck validateObservation(const Observation & observation)
{
  try {
    for (const auto * value : {&observation.bag_id, &observation.pair_id, &observation.rgb_message_id,
      &observation.depth_message_id, &observation.clock_id, &observation.segment_id}) {
      require(identity(*value), Code::invalid_identity, "Invalid observation identity component");
    }
    require(observation.id == makeObservationId(observation) && observation.revision > 0 &&
      observation.rgb_message_id != observation.depth_message_id, Code::invalid_identity, "Observation identity/revision mismatch");
    require(digest(observation.source_manifest_sha256) && identity(observation.map_id) &&
      identity(observation.geometry_revision) && frameName(observation.map_frame) && frameName(observation.body_frame) &&
      text(observation.map_basis) && text(observation.clock_basis) && text(observation.segment_basis),
      Code::invalid_context, "Missing map/time/source context or basis");
    checkImage(observation.rgb_image);
    checkImage(observation.depth_image);
    const auto time = checkSensorTimes(observation.rgb_image.stamp, observation.depth_image.stamp,
      {SensorClockRelation::same_clock, observation.sync_tolerance_ns});
    require(time.matched() && *time.delta_ns == observation.delta_ns &&
      sameStamp(observation.stamp, observation.rgb_image.stamp) &&
      observation.time_match == (observation.delta_ns == 0 ? "exact" : "approximate") &&
      text(observation.pairing_policy), Code::invalid_time, "Reference stamp/delta/tolerance/time match contradict");
    require(observation.tf_max_gap_ns >= 0, Code::invalid_pose, "Negative TF interpolation gap");
    for (const auto & limit : {observation.tf_jump_limits.translation_m, observation.tf_jump_limits.rotation_rad}) {
      require(!limit || (std::isfinite(*limit) && *limit >= 0), Code::invalid_pose, "Invalid TF jump limit");
    }
    const auto evidence = checkEvidence(observation);
    checkReference(evidence, observation.rgb_message_id, observation.rgb_image.topic);
    checkReference(evidence, observation.depth_message_id, observation.depth_image.topic);
    checkStorage(observation);
    checkCalibration(observation.rgb_calibration, observation.rgb_model, observation.rgb_image, evidence);
    checkCalibration(observation.depth_calibration, observation.depth_model, observation.depth_image, evidence);
    checkPose(observation.camera_pose_map, observation, observation.rgb_image.frame_id, observation.rgb_image.stamp, evidence);
    checkPose(observation.depth_camera_pose_map, observation, observation.depth_image.frame_id, observation.depth_image.stamp, evidence);
    checkPose(observation.body_pose_map, observation, observation.body_frame, observation.stamp, evidence);
    checkDeclarations(observation);
    require((observation.source_pair_status == "pending" && !observation.source_pending_reasons.empty()) ||
      (observation.source_pair_status == "prepared_for_geometry" && observation.source_pending_reasons.empty()),
      Code::invalid_pending, "Source preparation status and reasons contradict");
    std::set<std::string> actual;
    for (const auto & reason : observation.pending_reasons) {
      require(text(reason) && actual.insert(reason).second, Code::invalid_pending, "Invalid/duplicate pending reason");
    }
    for (const auto & reason : collectObservationPendingReasons(observation)) {
      require(text(reason) && actual.count(reason), Code::invalid_pending, "Unreported pending reason: " + reason);
    }
    return {Code::ready, ""};
  } catch (const CheckError & error) {
    return {error.code, error.what()};
  }
}

}  // namespace vln_memory
