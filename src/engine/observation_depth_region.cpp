#include <vln_memory/engine/observation_depth_region.hpp>

#include <utility>

#include "observation_source_internal.hpp"

namespace vln_memory
{
namespace
{
using Code = ObservationDepthRegionCode;

/** @brief 比较模型的来源网格和全部用于几何重算的参数。 */
bool sameModel(const CameraModel & a, const CameraModel & b)
{
  return a.frame_id == b.frame_id && a.width == b.width && a.height == b.height &&
    a.image_domain == b.image_domain && a.distortion_model == b.distortion_model &&
    a.normalization_applied == b.normalization_applied && a.k == b.k && a.d == b.d && a.r == b.r && a.p == b.p;
}

/** @brief 比较保存的光学深度存储解释及已核验的像素计数。 */
bool sameStorage(const PreparedDepthStorage & a, const ObservationDepthStorage & b)
{
  return a.format == b.format && a.dtype == b.dtype && a.unit == b.unit && a.scale_to_m == b.scale_to_m &&
    a.invalid_rule == b.invalid_rule && a.source_encoding == b.source_encoding && a.source_unit == b.source_unit &&
    a.source_scale_to_m == b.source_scale_to_m && a.min_depth_m == b.min_depth_m &&
    a.max_depth_m == b.max_depth_m && a.valid_pixels == b.valid_pixels && a.invalid_pixels == b.invalid_pixels;
}

/** @brief 确保投影使用的媒体、帧和几何仍对应已发布观测的来源。 */
bool matchesSource(const PreparedPair & pair, const Observation & observation)
{
  return detail::matchesPreparationIdentity(pair, observation) &&
    pair.clock_basis == observation.clock_basis && pair.segment_basis == observation.segment_basis &&
    detail::sameImageReference(pair.depth_image, observation.depth_image) &&
    pair.depth_media_relative && pair.depth_media_relative->generic_string() == *observation.depth_media &&
    pair.depth_storage && sameStorage(*pair.depth_storage, *observation.depth_storage) &&
    observation.depth_calibration &&
    pair.depth_calibration.calibration_id == observation.depth_calibration->calibration_id &&
    pair.depth_calibration.source_message_id == observation.depth_calibration->source_message_id &&
    pair.depth_model && sameModel(*pair.depth_model, *observation.depth_model) &&
    pair.depth_pose && detail::sameRecordedPose(*pair.depth_pose, *observation.depth_camera_pose_map) &&
    detail::sameObservationEvidence(pair, observation);
}

/** @brief 保留 preparation 读回和变更检查的具体错误码。 */
ObservationDepthRegionResult readFailure(PreparationReadCode code, std::string message)
{
  return {Code::preparation_read_failed, std::move(message), code, std::nullopt, std::nullopt};
}

/** @brief 构建无法进入投影阶段的结果。 */
ObservationDepthRegionResult rejected(Code code, std::string message)
{
  return {code, std::move(message), std::nullopt, std::nullopt, std::nullopt};
}

}  // namespace

/** @brief 回读与观测一致的深度像素，使用观测中 Depth 时刻的历史几何完成区域投影。 */
ObservationDepthRegionResult projectObservationDepthRegion(
  const PreparationReader & reader, const Observation & observation,
  const DepthRegion & region, const DepthRegionOptions & options)
{
  if (reader.manifestSha256() != observation.source_manifest_sha256 || reader.mapFrame() != observation.map_frame) {
    return rejected(Code::source_mismatch, "Observation does not identify this preparation manifest and map frame");
  }
  const auto check = validateObservation(observation);
  if (!check.ok()) {
    return {Code::invalid_observation, check.message, std::nullopt, check.code, std::nullopt};
  }
  if (!observation.depth_media || !observation.depth_storage) {
    return rejected(Code::missing_depth, "Observation has no depth media and storage contract");
  }
  if (!observation.depth_model) {return rejected(Code::missing_depth_model, "Observation has no depth model");}
  if (!observation.depth_camera_pose_map) {return rejected(Code::missing_depth_pose, "Observation has no depth pose");}

  auto loaded = reader.loadPair(observation.pair_id);
  if (!loaded.ok()) {return readFailure(loaded.code, std::move(loaded.message));}
  const auto & pair = *loaded.pair;
  if (!pair.depth || !matchesSource(pair, observation)) {
    return rejected(Code::source_mismatch, "Preparation depth data or geometry differs from the observation");
  }
  /** @brief 两种选区都在同一份已核验的深度、模型和历史位姿上求解。 */
  const auto project = [&](const auto & shape) {
      return projectDepthRegionToMap(*pair.depth, *observation.depth_model, *observation.depth_camera_pose_map,
        observation.map_frame, shape, options);
    };
  auto projection = std::visit(project, region);
  const auto stable = reader.verifyUnchanged(pair.file_snapshots);
  if (!stable.ok()) {return readFailure(stable.code, stable.message);}
  return {Code::evaluated, "", std::nullopt, std::nullopt, std::move(projection)};
}

/** @brief 只使用直出观测中经过校验的模型、位姿与深度。 */
ObservationDepthRegionResult projectObservationDepthRegion(
  const ObservationDatasetReader & dataset, std::string_view observation_id,
  const DepthRegion & region, const DepthRegionOptions & options)
{
  const auto loaded = dataset.loadObservation(observation_id);
  if (!loaded.ok()) {return rejected(Code::invalid_observation, loaded.message);}
  const auto & observation = *loaded.observation;
  if (!observation.depth_media || !observation.depth_storage) {
    return rejected(Code::missing_depth, "Observation has no depth media and storage contract");
  }
  if (!observation.depth_model) {return rejected(Code::missing_depth_model, "Observation has no depth model");}
  if (!observation.depth_camera_pose_map) {return rejected(Code::missing_depth_pose, "Observation has no depth pose");}
  const auto data = dataset.loadDepth(observation_id);
  if (!data.ok()) {return rejected(Code::source_mismatch, data.message);}
  auto projection = std::visit([&](const auto & shape) {
      return projectDepthRegionToMap(*data.depth, *observation.depth_model, *observation.depth_camera_pose_map,
        observation.map_frame, shape, options);
    }, region);
  return {Code::evaluated, "", std::nullopt, std::nullopt, std::move(projection)};
}

}  // namespace vln_memory
