#include <vln_memory/memory/observation.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace
{
using namespace vln_memory;
using Json = nlohmann::json;

/** @brief Release 构建也执行的检查，失败时报告具体契约。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 构造声明明确的测试消息摘要；不声称摘要对应真实物理采集。 */
ObservationEvidence evidence(const std::string & id, const std::string & topic)
{
  ObservationEvidence result;
  result.message_id = id;
  result.topic = topic;
  result.serialized_bytes = 100;
  result.payload_sha256 = std::string(64, 'a');
  return result;
}

/** @brief 原始 D8 测试标定，静态复用时刻与图像时刻不同。 */
ObservationCalibration calibration(const std::string & id, const std::string & topic, const std::string & image_topic)
{
  ObservationCalibration result;
  result.calibration_id = "calibration_" + id;
  result.source_message_id = id;
  auto & info = result.original;
  info.stamp = {-2, 1};
  info.topic = topic;
  info.frame_id = "camera";
  info.width = 2;
  info.height = 2;
  info.distortion_model = "plumb_bob";
  info.d.assign(8, 0);
  info.k = {2, 0, 0.5, 0, 2, 0.5, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {2, 0, 0.5, 0, 0, 2, 0.5, 0, 0, 0, 1, 0};
  auto & choice = result.selection;
  choice.camera_info_topic = topic;
  choice.image_topic = image_topic;
  choice.optical_frame = "camera";
  choice.binding_basis = "Synthetic fixed camera and segment";
  choice.image_domain = CameraImageDomain::raw;
  choice.time_policy = CameraInfoTimePolicy::static_calibration;
  choice.clock_relation = SensorClockRelation::same_clock;
  choice.compatibility = CameraInfoCompatibility::plumb_bob_zero_tail_d8;
  return result;
}

/** @brief 用共享 TfIndex 产生可独立重算的完整测试位姿。 */
TfTransform transform(const std::string & id, const std::string & child, double x)
{
  TfTransform value;
  value.kind = TfKind::static_transform;
  value.stamp = {-1, 0};
  value.parent_frame = "map";
  value.child_frame = child;
  value.translation_m = {x, 0, 0};
  value.source_topic = "/tf_static";
  value.source_message_id = id;
  return value;
}

/** @brief 完整 pending 夹具：两个原始时刻、D8、PNG16、独立相机/底盘 TF。 */
Observation complete()
{
  Observation o;
  o.bag_id = "bag";
  o.pair_id = "pair";
  o.rgb_message_id = "rgb";
  o.depth_message_id = "depth";
  o.source_manifest_sha256 = std::string(64, 'b');
  o.clock_id = "clock";
  o.segment_id = "segment";
  o.clock_basis = "Synthetic common clock";
  o.segment_basis = "Single synthetic segment";
  o.map_id = "test_map";
  o.geometry_revision = "trajectory_v1";
  o.map_frame = "map";
  o.body_frame = "body";
  o.map_basis = "Synthetic known transforms";
  o.rgb_image = {"/rgb", 2, 2, "camera", {0, 0}};
  o.depth_image = {"/depth", 2, 2, "camera", {0, 20000000}};
  o.stamp = o.rgb_image.stamp;
  o.delta_ns = 20000000;
  o.sync_tolerance_ns = 30000000;
  o.time_match = "approximate";
  o.pairing_policy = "test_explicit_pair";
  o.source_pair_status = "pending";
  o.source_pending_reasons = {"depth_rgb_alignment_unverified"};
  o.rgb_media = "evidence/rgb/rgb.png";
  o.depth_media = "evidence/depth/depth.png";
  o.depth_storage = ObservationDepthStorage{"png", "uint16", DepthUnit::millimeters, 0.001,
    "zero_or_out_of_range_to_nan", "16UC1", DepthUnit::millimeters, 0.001, 0, std::nullopt, 4, 0};
  o.rgb_calibration = calibration("cal_rgb", "/rgb_info", "/rgb");
  o.depth_calibration = calibration("cal_depth", "/depth_info", "/depth");
  o.rgb_model = buildCameraModel(o.rgb_calibration->original, o.rgb_image, o.rgb_calibration->selection).model;
  o.depth_model = buildCameraModel(o.depth_calibration->original, o.depth_image, o.depth_calibration->selection).model;
  TfIndex index({o.clock_id, o.segment_id});
  expect(index.insert(transform("tf_camera", "camera", 1)).accepted(), "Camera TF fixture");
  expect(index.insert(transform("tf_body", "body", 2)).accepted(), "Body TF fixture");
  const TfTimeOptions time{o.clock_id, o.segment_id, 0};
  o.camera_pose_map = index.lookupPose("map", "camera", o.rgb_image.stamp, time);
  o.depth_camera_pose_map = index.lookupPose("map", "camera", o.depth_image.stamp, time);
  o.body_pose_map = index.lookupPose("map", "body", o.stamp, time);
  o.evidence = {evidence("rgb", "/rgb"), evidence("depth", "/depth"), evidence("cal_rgb", "/rgb_info"),
    evidence("cal_depth", "/depth_info"), evidence("tf_camera", "/tf_static"), evidence("tf_body", "/tf_static")};
  o.id = makeObservationId(o);
  o.pending_reasons = collectObservationPendingReasons(o);
  return o;
}

/** @brief 结构合法的缺项记录，保留已知来源和明确缺项原因。 */
Observation partial()
{
  auto o = complete();
  o.rgb_media.reset();
  o.depth_media.reset();
  o.depth_storage.reset();
  o.rgb_calibration.reset();
  o.depth_calibration.reset();
  o.rgb_model.reset();
  o.depth_model.reset();
  o.camera_pose_map.reset();
  o.depth_camera_pose_map.reset();
  o.body_pose_map.reset();
  o.pending_reasons = collectObservationPendingReasons(o);
  return o;
}

/** @brief 严格往返须保持规范 JSON、原始 D8 与 pending，不丢失整数。 */
void roundTrip(const Observation & o)
{
  const auto checked = validateObservation(o);
  expect(checked.ok(), observationCodeName(checked.code) + std::string(": ") + checked.message);
  const auto encoded = serializeObservation(o);
  expect(encoded.ok(), encoded.message);
  const auto decoded = deserializeObservation(encoded.json);
  expect(decoded.ok(), decoded.message);
  const auto repeated = serializeObservation(*decoded.observation);
  expect(repeated.ok() && repeated.json == encoded.json, "Round-trip changed canonical record");
  expect(decoded.observation->pending_reasons == o.pending_reasons, "Pending changed");
  expect(decoded.observation->revision == o.revision, "uint64 revision changed");
}

/** @brief 无效 C++ 对象必须同时被校验与序列化拒绝，写失败无 JSON。 */
void rejected(const Observation & o, ObservationCode code)
{
  const auto checked = validateObservation(o);
  const auto written = serializeObservation(o);
  expect(!checked.ok() && checked.code == code, "Wrong check result: " + checked.message);
  expect(!written.ok() && written.json.empty() && written.code == code, "Partial/invalid JSON returned");
}

/** @brief 损坏 JSON 失败时不返回部分观测。 */
void badJson(const Json & value)
{
  const auto result = deserializeObservation(value.dump());
  expect(!result.ok() && !result.observation, "Malformed JSON accepted");
}

/** @brief 检查来源身份独立于几何版本，且元组边界不发生拼接碰撞。 */
void identityCases()
{
  auto a = complete();
  roundTrip(a);
  const auto original = a.id;
  a.geometry_revision = "trajectory_v2";
  a.revision = 2;
  expect(makeObservationId(a) == original, "Geometry update changed observation ID");
  roundTrip(a);
  a.depth_message_id = "other_depth";
  expect(makeObservationId(a) != original, "Depth pairing change kept ID");
  a = complete();
  a.clock_id = "other_clock";
  expect(makeObservationId(a) != original, "Clock change kept ID");
  a = complete();
  a.bag_id = "x:2:y";
  a.pair_id = "z";
  const auto first = makeObservationId(a);
  a.bag_id = "x";
  a.pair_id = "2:y:z";
  expect(makeObservationId(a) != first, "Length tuple collision");
  a = complete();
  a.id = "obs_fabricated";
  rejected(a, ObservationCode::invalid_identity);
}

/** @brief 零/负时间、大范围时差以及所有大整数不经 double。 */
void timeCases()
{
  auto o = partial();
  roundTrip(o);
  o.rgb_image.stamp = {std::numeric_limits<std::int32_t>::min(), 0};
  o.depth_image.stamp = {std::numeric_limits<std::int32_t>::max(), 999999999};
  o.stamp = o.rgb_image.stamp;
  const auto matched = checkSensorTimes(o.rgb_image.stamp, o.depth_image.stamp,
    {SensorClockRelation::same_clock, std::numeric_limits<std::int64_t>::max()});
  expect(matched.matched(), "Extreme timestamp fixture");
  o.delta_ns = *matched.delta_ns;
  o.sync_tolerance_ns = std::numeric_limits<std::int64_t>::max();
  o.revision = std::numeric_limits<std::uint64_t>::max();
  o.evidence.front().bag_record_time_ns = std::numeric_limits<std::int64_t>::min();
  o.evidence.front().topic_ordinal = std::numeric_limits<std::uint64_t>::max();
  roundTrip(o);
  o.depth_image.stamp = o.rgb_image.stamp;
  o.delta_ns = 0;
  o.sync_tolerance_ns = 0;
  o.time_match = "exact";
  roundTrip(o);
  o.time_match = "approximate";
  rejected(o, ObservationCode::invalid_time);
  o = complete();
  o.stamp.nanosec = 1;
  rejected(o, ObservationCode::invalid_time);
  o = complete();
  o.delta_ns = -o.delta_ns;
  rejected(o, ObservationCode::invalid_time);
  o = complete();
  o.sync_tolerance_ns = 1;
  rejected(o, ObservationCode::invalid_time);
}

/** @brief 存储、引用、原始标定与有效模型均有破坏性边界检查。 */
void metadataCases()
{
  auto o = complete();
  for (auto * c : {&*o.rgb_calibration, &*o.depth_calibration}) {
    c->original.width = 8;
    c->original.height = 8;
    c->original.roi = {2, 2, 4, 4, false};
    c->original.binning_x = 2;
    c->original.binning_y = 2;
  }
  o.rgb_model = buildCameraModel(o.rgb_calibration->original, o.rgb_image, o.rgb_calibration->selection).model;
  o.depth_model = buildCameraModel(o.depth_calibration->original, o.depth_image, o.depth_calibration->selection).model;
  expect(o.rgb_model && o.rgb_model->k[0] == 1 && o.rgb_model->width == 2, "ROI/binning fixture");
  roundTrip(o);
  o = complete();
  o.rgb_calibration->original.stamp = o.rgb_image.stamp;
  o.depth_calibration->original.stamp = o.depth_image.stamp;
  o.rgb_calibration->selection.time_policy = CameraInfoTimePolicy::synchronized;
  o.depth_calibration->selection.time_policy = CameraInfoTimePolicy::synchronized;
  roundTrip(o);
  o = complete();
  o.depth_storage->format = "npy";
  o.depth_storage->dtype = "float32";
  o.depth_storage->unit = DepthUnit::meters;
  o.depth_storage->scale_to_m = 1;
  o.depth_storage->invalid_rule = "nan";
  o.depth_media = "evidence/depth/depth.npy";
  roundTrip(o);
  o.depth_storage->source_encoding = "32FC1";
  o.depth_storage->source_unit = DepthUnit::meters;
  o.depth_storage->source_scale_to_m = 1;
  o.depth_storage->max_depth_m = 3;
  roundTrip(o);
  o.depth_storage->scale_to_m = 0.001;
  rejected(o, ObservationCode::invalid_media);
  o = complete();
  o.depth_storage->max_depth_m = std::numeric_limits<double>::infinity();
  rejected(o, ObservationCode::invalid_media);
  o = complete();
  o.depth_storage->valid_pixels = std::numeric_limits<std::uint64_t>::max();
  rejected(o, ObservationCode::invalid_media);
  for (const auto * path : {"../rgb.png", "/rgb.png", "C:\\rgb.png", "a//b.png", "a/./b.png", "a/../b.png"}) {
    o = complete();
    o.rgb_media = path;
    rejected(o, ObservationCode::invalid_media);
  }
  o = complete();
  o.rgb_model->k[0] += 1;
  rejected(o, ObservationCode::invalid_calibration);
  o = complete();
  o.rgb_calibration->original.k[0] = std::numeric_limits<double>::quiet_NaN();
  rejected(o, ObservationCode::invalid_calibration);
  o = complete();
  o.rgb_calibration->selection.time_policy = CameraInfoTimePolicy::synchronized;
  rejected(o, ObservationCode::invalid_calibration);
  o = complete();
  o.rgb_calibration->selection.clock_relation = static_cast<SensorClockRelation>(99);
  rejected(o, ObservationCode::invalid_calibration);
  o = complete();
  o.rgb_calibration->selection.max_delta_ns = -1;
  rejected(o, ObservationCode::invalid_calibration);
  o = complete();
  o.evidence.pop_back();
  rejected(o, ObservationCode::invalid_reference);
  o = complete();
  o.evidence.push_back(o.evidence.front());
  rejected(o, ObservationCode::invalid_reference);
  o = complete();
  o.evidence.front().raw_path = "evidence/raw/rgb.cdr";
  roundTrip(o);
}

/** @brief 缺项可读，但不能漏报对应原因或更改未核实声明。 */
void pendingCases()
{
  auto o = partial();
  roundTrip(o);
  o.pending_reasons.erase(std::find(o.pending_reasons.begin(), o.pending_reasons.end(), "missing_body_map_pose"));
  rejected(o, ObservationCode::invalid_pending);
  o.pending_reasons = collectObservationPendingReasons(o);
  roundTrip(o);
  o = complete();
  o.source_pair_status = "prepared_for_geometry";
  rejected(o, ObservationCode::invalid_pending);
  o = complete();
  o.depth_alignment = DepthAlignment::not_aligned;
  o.alignment_basis.clear();
  o.pending_reasons = collectObservationPendingReasons(o);
  roundTrip(o);
  o.depth_alignment = DepthAlignment::aligned_to_rgb;
  rejected(o, ObservationCode::invalid_context);
  o = complete();
  o.capture_sync_status = "verified_synchronized";
  rejected(o, ObservationCode::invalid_context);
  o = complete();
  o.depth_alignment = DepthAlignment::aligned_to_rgb;
  o.alignment_basis = "Synthetic registered-grid declaration";
  o.capture_sync_status = "verified_synchronized";
  o.capture_sync_basis = "Synthetic shared exposure declaration";
  o.source_pair_status = "prepared_for_geometry";
  o.source_pending_reasons.clear();
  o.pending_reasons.clear();
  roundTrip(o);
  o.depth_image.width = 3;
  rejected(o, ObservationCode::invalid_media);
}

/** @brief 独立时刻、端点、求解数值与显式插值限制必须一致。 */
void poseCases()
{
  auto o = complete();
  for (auto & value : o.camera_pose_map->pose->rotation_xyzw) {value = -value;}
  roundTrip(o);
  o = complete();
  o.depth_camera_pose_map->time.stamp = o.rgb_image.stamp;
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.camera_pose_map->pose->translation_m[0] += 1;
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.camera_pose_map->time.edges[0].before.translation_m[0] += 1;
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.camera_pose_map->pose->rotation_xyzw = {0, 0, 0, 0};
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.camera_pose_map->time.edges[0].inverted = true;
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.body_frame = "map";
  o.body_pose_map->time.frames = {"map"};
  o.body_pose_map->time.edges.clear();
  o.body_pose_map->pose = TfPose{};
  roundTrip(o);
  o.body_pose_map->pose->rotation_xyzw[0] = 0.0001;
  rejected(o, ObservationCode::invalid_pose);
  o = complete();
  o.rgb_image.stamp = {0, 500000000};
  o.depth_image.stamp = {0, 520000000};
  o.stamp = o.rgb_image.stamp;
  o.tf_max_gap_ns = 1000000000;
  auto before = transform("before", "camera", 0);
  before.kind = TfKind::dynamic;
  before.source_topic = "/tf";
  before.stamp = {0, 0};
  auto after = before;
  after.source_message_id = "after";
  after.stamp = {1, 0};
  after.translation_m[0] = 1;
  TfIndex index({o.clock_id, o.segment_id});
  expect(index.insert(before).accepted() && index.insert(after).accepted(), "Dynamic fixture");
  expect(index.insert(transform("tf_body", "body", 2)).accepted(), "Body fixture");
  const TfTimeOptions time{o.clock_id, o.segment_id, o.tf_max_gap_ns};
  o.camera_pose_map = index.lookupPose("map", "camera", o.rgb_image.stamp, time);
  o.depth_camera_pose_map = index.lookupPose("map", "camera", o.depth_image.stamp, time);
  o.body_pose_map = index.lookupPose("map", "body", o.stamp, time);
  o.evidence.push_back(evidence("before", "/tf"));
  o.evidence.push_back(evidence("after", "/tf"));
  roundTrip(o);
  o.tf_max_gap_ns = 1;
  rejected(o, ObservationCode::invalid_pose);
  o.tf_max_gap_ns = 1000000000;
  o.tf_jump_limits.translation_m = 0.5;
  rejected(o, ObservationCode::invalid_pose);
}

/** @brief JSON 固定 schema、重复键、类型、精度及资源边界。 */
void jsonCases()
{
  const auto encoded = serializeObservation(complete());
  expect(encoded.ok(), encoded.message);
  const auto original = Json::parse(encoded.json);
  auto value = original;
  value["extra"] = 1;
  badJson(value);
  value = original;
  value["schema_version"] = "observation/999";
  const auto unsupported = deserializeObservation(value.dump());
  expect(unsupported.code == ObservationCode::unsupported_schema && !unsupported.observation, "Unknown schema accepted");
  value = original;
  value["rgb_image"]["width"] = 2.0;
  badJson(value);
  value = original;
  value["rgb_stamp"]["sec"] = std::numeric_limits<std::uint64_t>::max();
  badJson(value);
  value = original;
  value["revision"] = 1;
  badJson(value);
  value = original;
  value["revision"] = "18446744073709551616";
  badJson(value);
  value = original;
  value["delta_ns"] = "+20000000";
  badJson(value);
  value = original;
  value["rgb_model"]["k"].push_back(1);
  badJson(value);
  value = original;
  value["rgb_calibration"]["selection"]["compatibility"] = "guessed";
  badJson(value);
  const auto duplicate = std::string("{\"id\":\"duplicate\",") + encoded.json.substr(1);
  expect(!deserializeObservation(duplicate).ok(), "Duplicate JSON key accepted");
  expect(!deserializeObservation(encoded.json, 1).ok(), "Input byte limit ignored");
  expect(!deserializeObservation(encoded.json, 0).ok(), "Zero input limit ignored");
  expect(!deserializeObservation(std::string(70, '[') + "0" + std::string(70, ']')).ok(), "Deep JSON accepted");
  expect(!deserializeObservation(encoded.json + " trailing").ok(), "Trailing data accepted");
}
}  // namespace

/** @brief 只链接正式模块即可运行全部契约与严格编解码回归。 */
int main()
{
  try {
    identityCases();
    timeCases();
    metadataCases();
    pendingCases();
    poseCases();
    jsonCases();
    std::cout << "Observation contract/JSON checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
