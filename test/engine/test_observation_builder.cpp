#include <vln_memory/engine/observation_builder.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace vln_memory;

/** @brief 使断言在 Release 模式下同样执行。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 仅给出合法 pending 所需的元数据，刻意使用负时间和独立光学 frame。 */
PreparedPair pendingPair()
{
  PreparedPair pair;
  pair.bag_id = "bag";
  pair.pair_id = "pair";
  pair.rgb_message_id = "rgb";
  pair.depth_message_id = "depth";
  pair.source_manifest_sha256 = std::string(64, 'a');
  pair.map_frame = "map";
  pair.body_frame = "body";
  pair.time_scope = {"clock", "segment"};
  pair.clock_basis = "Synthetic common clock";
  pair.segment_basis = "Synthetic single segment";
  pair.rgb_image = {"/rgb", 2, 2, "rgb_optical", {-1, 900000000}};
  pair.depth_image = {"/depth", 2, 2, "depth_optical", {-1, 899999997}};
  pair.delta_ns = -3;
  pair.sync_tolerance_ns = 3;
  pair.timestamp_match_policy = "rgb_reference";
  pair.capture_sync_status = "unknown";
  pair.status = "pending";
  pair.pending_reasons = {"source_unavailable"};
  pair.message_summaries = {
    {"rgb", "/rgb", 0, 0, 0, -10, 64, std::string(64, 'b'), std::nullopt},
    {"depth", "/depth", 0, 0, 1, -9, 64, std::string(64, 'c'), std::nullopt}};
  return pair;
}

/** @brief 检查合法缺项、独立拥有数据、稳定身份和失败结果。 */
void assemblyCases(const ObservationBuildContext & context)
{
  auto pair = pendingPair();
  const auto built = buildObservation(pair, context);
  expect(built.ok() && !built.preparation_error && !built.observation_error, built.message);
  const auto & observation = *built.observation;
  expect(observation.stamp.sec == -1 && observation.stamp.nanosec == 900000000 &&
    observation.depth_image.stamp.nanosec == 899999997 && observation.delta_ns == -3 &&
    observation.time_match == "approximate", "Original RGB/Depth timing changed");
  expect(observation.pending_reasons.front() == "source_unavailable" &&
    std::count(observation.pending_reasons.begin(), observation.pending_reasons.end(), "needs_check") == 1 &&
    !observation.depth_media && !observation.depth_storage && !observation.body_pose_map,
    "Pending reasons or absent data changed");
  const auto serialized = serializeObservation(observation);
  expect(serialized.ok() && deserializeObservation(serialized.json).ok(), "Pending JSON must round-trip");

  auto revised = context;
  revised.geometry_revision = "geometry-v2";
  revised.revision = 2;
  const auto updated = buildObservation(pair, revised);
  expect(updated.ok() && updated.observation->id == observation.id &&
    updated.observation->geometry_revision == "geometry-v2" && updated.observation->revision == 2,
    "Geometry revision changed observation identity");
  revised.map_frame = "other_map";
  const auto mismatch = buildObservation(pair, revised);
  expect(!mismatch.ok() && !mismatch.preparation_error && mismatch.observation_error == ObservationCode::invalid_context,
    "Mismatched map must fail in observation layer");

  pair.depth_media_relative = "depth/example.png";
  const auto missing_storage = buildObservation(pair, context);
  expect(!missing_storage.ok() && missing_storage.observation_error == ObservationCode::invalid_media,
    "Depth without storage interpretation must fail");
  pair.depth_media_relative.reset();
  pair.message_summaries.pop_back();
  const auto missing_evidence = buildObservation(pair, context);
  expect(!missing_evidence.ok() && missing_evidence.observation_error == ObservationCode::invalid_reference &&
    observation.evidence.size() == 2, "Invalid reference or shared result ownership");
  pair.pair_id.clear();
  const auto invalid_identity = buildObservation(pair, context);
  expect(!invalid_identity.ok() && invalid_identity.observation_error == ObservationCode::invalid_identity,
    "Invalid source identity must return a structured error");
}

/** @brief 已匹配的校正双目标定能随 pending 记录保留，模型及其失败语义不改写。 */
void calibrationCases(const ObservationBuildContext & context)
{
  auto pair = pendingPair();
  auto & rgb = pair.rgb_calibration;
  rgb.calibration_id = "cal_rgb";
  rgb.source_message_id = "cal_rgb";
  rgb.selection_code = "matched";
  rgb.selection = {"/rgb_info", "/rgb", "rgb_optical", "Synthetic rectified stereo",
    CameraImageDomain::rectified, CameraInfoTimePolicy::synchronized, SensorClockRelation::same_clock};
  auto & raw = rgb.original.emplace();
  raw.stamp = pair.rgb_image.stamp;
  raw.topic = "/rgb_info";
  raw.frame_id = "rgb_optical";
  raw.width = 2;
  raw.height = 2;
  raw.distortion_model = "plumb_bob";
  raw.d.assign(5, 0);
  raw.k = {2, 0, 0.5, 0, 2, 0.5, 0, 0, 1};
  raw.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  raw.p = {2, 0, 0.5, -0.25, 0, 2, 0.5, 0, 0, 0, 1, 0};
  pair.message_summaries.push_back({"cal_rgb", "/rgb_info", 0, 0, 2, -8, 128, std::string(64, 'd'), std::nullopt});
  expect(checkCameraInfoImage(raw, pair.rgb_image, rgb.selection).matched() &&
    buildCameraModel(raw, pair.rgb_image, rgb.selection).code == CameraGeometryCode::unsupported_projection,
    "Fixture must match calibration while failing projection");
  const auto built = buildObservation(pair, context);
  expect(built.ok() && built.observation->rgb_calibration && !built.observation->rgb_model &&
    built.observation->rgb_calibration->original.p[3] == -0.25 &&
    std::count(built.observation->pending_reasons.begin(), built.observation->pending_reasons.end(), "missing_rgb_model") == 1,
    "Matched raw calibration was lost when model was unavailable: " + built.message);
  const auto encoded = serializeObservation(*built.observation);
  expect(encoded.ok() && deserializeObservation(encoded.json).ok(), "Matched raw calibration must round-trip");

  rgb.selection.optical_frame = "wrong_optical";
  const auto invalid_binding = buildObservation(pair, context);
  expect(!invalid_binding.ok() && invalid_binding.observation_error == ObservationCode::invalid_calibration,
    "Missing model must not hide a bad raw calibration binding");
  rgb.original.reset();
  const auto no_original = buildObservation(pair, context);
  expect(!no_original.ok() && no_original.observation_error == ObservationCode::invalid_calibration,
    "Contradictory matched state was accepted");
}
}  // namespace

/** @brief 运行无磁盘、无 ROS 的正式单对组装回归。 */
int main()
{
  try {
    const ObservationBuildContext context{"project_map", "geometry-v1", "map", "Synthetic map", 1,
      {"needs_check", "needs_check"}};
    assemblyCases(context);
    calibrationCases(context);
    std::cout << "ObservationBuilder core checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
