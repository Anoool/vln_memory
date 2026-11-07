#include <vln_memory/engine/observation_builder.hpp>

#include <filesystem>
#include <stdexcept>
#include <utility>

namespace vln_memory
{
namespace
{

/** @brief 保留正式校验错误，不返回部分观测。 */
ObservationBuildResult observationFailure(ObservationCode code, std::string message)
{
  return {std::nullopt, code, std::move(message), std::nullopt};
}

/** @brief 转换 Reader 已核验的根相对路径，缺项保持为空。 */
std::optional<std::string> relativePath(const std::optional<std::filesystem::path> & path)
{
  if (!path) {return std::nullopt;}
  return path->generic_string();
}

/** @brief 嵌入已匹配的原始标定；其他候选仍可通过来源消息摘要回查。 */
std::optional<ObservationCalibration> observationCalibration(const PreparedCalibration & value)
{
  if (value.selection_code != "matched") {return std::nullopt;}
  return ObservationCalibration{value.calibration_id, value.source_message_id, *value.original, value.selection};
}

}  // namespace

/** @brief 按原始配对事实映射一条观测，再调用正式记录的身份、pending 和校验逻辑。 */
ObservationBuildResult buildObservation(const PreparedPair & pair, const ObservationBuildContext & context)
{
  if (context.map_frame != pair.map_frame) {
    return observationFailure(ObservationCode::invalid_context, "Map frame differs from preparation");
  }
  if (pair.depth_media_relative && !pair.depth_storage) {
    return observationFailure(ObservationCode::invalid_media, "Depth media lacks storage interpretation");
  }
  for (const auto * calibration : {&pair.rgb_calibration, &pair.depth_calibration}) {
    if (calibration->selection_code == "matched" && !calibration->original) {
      return observationFailure(ObservationCode::invalid_calibration, "Matched calibration lacks original parameters");
    }
  }

  Observation observation;
  observation.revision = context.revision;
  observation.map_id = context.map_id;
  observation.geometry_revision = context.geometry_revision;
  observation.map_basis = context.map_basis;
  observation.bag_id = pair.bag_id;
  observation.pair_id = pair.pair_id;
  observation.rgb_message_id = pair.rgb_message_id;
  observation.depth_message_id = pair.depth_message_id;
  observation.source_manifest_sha256 = pair.source_manifest_sha256;
  observation.clock_id = pair.time_scope.clock_id;
  observation.segment_id = pair.time_scope.segment_id;
  observation.clock_basis = pair.clock_basis;
  observation.segment_basis = pair.segment_basis;
  observation.map_frame = pair.map_frame;
  observation.body_frame = pair.body_frame;
  observation.stamp = pair.rgb_image.stamp;
  observation.rgb_image = pair.rgb_image;
  observation.depth_image = pair.depth_image;
  observation.delta_ns = pair.delta_ns;
  observation.sync_tolerance_ns = pair.sync_tolerance_ns;
  observation.time_match = pair.delta_ns == 0 ? "exact" : "approximate";
  observation.pairing_policy = pair.timestamp_match_policy;
  observation.source_pair_status = pair.status;
  observation.source_pending_reasons = pair.pending_reasons;
  observation.capture_sync_status = pair.capture_sync_status;
  observation.capture_sync_basis = pair.capture_sync_basis;
  observation.depth_alignment = pair.depth_alignment;
  observation.alignment_basis = pair.alignment_basis;
  observation.rgb_media = relativePath(pair.rgb_media_relative);
  observation.depth_media = relativePath(pair.depth_media_relative);
  if (observation.depth_media) {
    const auto & storage = *pair.depth_storage;
    observation.depth_storage = ObservationDepthStorage{storage.format, storage.dtype, storage.unit,
      storage.scale_to_m, storage.invalid_rule, storage.source_encoding, storage.source_unit,
      storage.source_scale_to_m, storage.min_depth_m, storage.max_depth_m, storage.valid_pixels,
      storage.invalid_pixels};
  }
  observation.rgb_calibration = observationCalibration(pair.rgb_calibration);
  observation.depth_calibration = observationCalibration(pair.depth_calibration);
  observation.rgb_model = pair.rgb_model;
  observation.depth_model = pair.depth_model;
  observation.tf_max_gap_ns = pair.tf_max_gap_ns;
  observation.tf_jump_limits = pair.tf_jump_limits;
  observation.camera_pose_map = pair.rgb_pose;
  observation.depth_camera_pose_map = pair.depth_pose;
  observation.body_pose_map = pair.body_pose;
  observation.evidence.reserve(pair.message_summaries.size());
  for (const auto & message : pair.message_summaries) {
    observation.evidence.push_back({message.message_id, message.topic, message.file_index,
      message.topic_ordinal, message.message_order, message.bag_record_time_ns, message.serialized_bytes,
      message.payload_sha256, relativePath(message.raw_path)});
  }
  try {
    observation.id = makeObservationId(observation);
  } catch (const std::invalid_argument & error) {
    return observationFailure(ObservationCode::invalid_identity, error.what());
  }
  observation.pending_reasons = context.additional_pending_reasons;
  observation.pending_reasons = collectObservationPendingReasons(observation);
  auto checked = validateObservation(observation);
  if (!checked.ok()) {return observationFailure(checked.code, std::move(checked.message));}
  return {std::nullopt, std::nullopt, "", std::move(observation)};
}

/** @brief 复用已有 Reader 索引，并区分来源读取失败与正式校验失败。 */
ObservationBuildResult buildObservation(
  const PreparationReader & reader, std::string_view pair_id, const ObservationBuildContext & context)
{
  auto loaded = reader.loadPair(pair_id);
  if (!loaded.ok()) {return {loaded.code, std::nullopt, std::move(loaded.message), std::nullopt};}
  return buildObservation(*loaded.pair, context);
}

}  // namespace vln_memory
