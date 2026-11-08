#include <vln_memory/engine/entity_builder.hpp>

#include <algorithm>
#include <utility>

#include <vln_memory/engine/observation_depth_region.hpp>

#include "observation_source_internal.hpp"

namespace vln_memory
{
namespace
{
using Code = EntityBuildCode;
using Projection = ObservationDepthRegionCode;

/** @brief 标签和确认引用必须含非空文字，按输入保留其字节。 */
bool validText(std::string_view text)
{
  return !text.empty() && text.size() <= 4096 &&
    std::any_of(text.begin(), text.end(), [](unsigned char byte) {return byte > ' ';}) &&
    std::none_of(text.begin(), text.end(), [](unsigned char byte) {return byte < ' ' || byte == 0x7f;});
}

/** @brief 在未解码像素的路径上核对观测引用对应同一保存配对。 */
std::optional<EntityBuildResult> pendingReferenceError(const PreparationReader & reader, const Observation & observation)
{
  if (reader.manifestSha256() != observation.source_manifest_sha256 || reader.mapFrame() != observation.map_frame) {
    return EntityBuildResult{Code::source_mismatch, "Observation does not identify this preparation and map frame", std::nullopt};
  }
  const auto loaded = reader.loadPairMetadata(observation.pair_id);
  if (!loaded.ok()) {return EntityBuildResult{Code::preparation_read_failed, loaded.message, std::nullopt};}
  const auto & pair = *loaded.pair;
  if (!detail::matchesPreparationIdentity(pair, observation) ||
    !detail::sameImageReference(pair.depth_image, observation.depth_image)) {
    return EntityBuildResult{Code::source_mismatch, "Observation identity differs from the saved pair", std::nullopt};
  }
  return std::nullopt;
}

/** @brief 将缺项变成待定位原因；完整性和结构错误由创建入口拒绝。 */
const char * missingGeometry(Projection code) noexcept
{
  switch (code) {
    case Projection::missing_depth: return "missing_depth";
    case Projection::missing_depth_model: return "missing_depth_model";
    case Projection::missing_depth_pose: return "missing_depth_pose";
    default: return nullptr;
  }
}

/** @brief 绑定观测修订及其 preparation 指纹，具体证据由创建入口填入。 */
EntityObservationRef observationRef(const Observation & observation)
{
  return {observation.id, observation.revision, observation.source_manifest_sha256, std::nullopt, std::nullopt};
}

/** @brief 分配实体身份与初始状态，结果独立拥有观测证据。 */
EntityBuildResult finishEntity(
  const Observation & observation, std::string_view label, EntityKind kind, EntityObservationRef reference,
  const char * pending_reason)
{
  auto id = allocateEntityId();
  if (!id) {return {Code::id_unavailable, "Cannot allocate an entity ID", std::nullopt};}
  Entity entity;
  entity.entity_id = std::move(*id);
  entity.kind = kind;
  entity.semantic_label = label;
  entity.map_id = observation.map_id;
  entity.map_frame = observation.map_frame;
  entity.geometry_revision = observation.geometry_revision;
  entity.observations.push_back(std::move(reference));
  if (pending_reason) {
    entity.location_pending_reason = pending_reason;
  } else {
    entity.status = EntityStatus::located;
    entity.location_observation = EntityLocationRef{observation.id, observation.revision};
  }
  return {Code::created, "", std::move(entity)};
}

/** @brief 物品仅附加深度选区证据，复用统一的实体初始化。 */
EntityBuildResult finishObject(
  const Observation & observation, std::string_view label, std::optional<ObjectSelectionEvidence> evidence,
  const char * pending_reason)
{
  auto ref = observationRef(observation);
  ref.object_selection = std::move(evidence);
  return finishEntity(observation, label, EntityKind::object, std::move(ref), pending_reason);
}

/** @brief 确认记录须指向同一目标标签、观测修订、来源和地图几何上下文。 */
bool matchesRegionConfirmation(
  const RegionInteriorConfirmation & confirmation, const Observation & observation, std::string_view label)
{
  return validText(confirmation.evidence_ref) && confirmation.semantic_label == label &&
    confirmation.observation_id == observation.id && confirmation.observation_revision == observation.revision &&
    confirmation.source_manifest_sha256 == observation.source_manifest_sha256 &&
    confirmation.map_id == observation.map_id && confirmation.map_frame == observation.map_frame &&
    confirmation.geometry_revision == observation.geometry_revision;
}

/** @brief 核对底盘时刻所属的 RGB 帧、底盘帧、原始位姿端点及观测消息证据。 */
bool matchesRegionSource(const PreparedPair & pair, const Observation & observation)
{
  if (!detail::matchesPreparationIdentity(pair, observation) || pair.body_frame != observation.body_frame ||
    pair.clock_basis != observation.clock_basis || pair.segment_basis != observation.segment_basis ||
    !detail::sameImageReference(pair.rgb_image, observation.rgb_image) ||
    !detail::sameImageReference(pair.depth_image, observation.depth_image) ||
    !detail::sameObservationEvidence(pair, observation)) {return false;}
  // 观测修订明确缺位姿时，保留缺项；不能由 preparation 中的位姿代填。
  return !observation.body_pose_map ||
    (pair.body_pose && detail::sameRecordedPose(*pair.body_pose, *observation.body_pose_map));
}

}  // namespace

/** @brief 从实际投影结果或明确的缺几何状态构建一条新物品记录。 */
EntityBuildResult createObjectEntity(
  const PreparationReader & reader, const Observation & observation, std::string_view semantic_label,
  const std::optional<DepthRegion> & selection, const DepthRegionOptions & options)
{
  if (!validText(semantic_label)) {return {Code::invalid_label, "Invalid semantic label", std::nullopt};}
  if (!selection) {
    const auto checked = validateObservation(observation);
    if (!checked.ok()) {return {Code::invalid_observation, checked.message, std::nullopt};}
    if (auto error = pendingReferenceError(reader, observation)) {return std::move(*error);}
    return finishObject(observation, semantic_label, std::nullopt, "missing_selection");
  }
  if (selection->valueless_by_exception()) {
    return {Code::invalid_selection, "Missing selection shape", std::nullopt};
  }
  // 仅保留代表点和质量计数，证据不要求复制全部簇点。
  auto point_options = options;
  point_options.include_points = false;
  const auto projected = projectObservationDepthRegion(reader, observation, *selection, point_options);
  if (const auto * pending = missingGeometry(projected.code)) {
    if (auto error = pendingReferenceError(reader, observation)) {return std::move(*error);}
    const auto code = checkDepthRegionSelection(
      observation.depth_image.width, observation.depth_image.height, *selection, point_options);
    if (code != DepthRegionCode::ready) {return {Code::invalid_selection, depthRegionCodeName(code), std::nullopt};}
    return finishObject(observation, semantic_label,
      ObjectSelectionEvidence{*selection, point_options, std::nullopt, 0, 0, 0, std::nullopt}, pending);
  }
  switch (projected.code) {
    case Projection::invalid_observation: return {Code::invalid_observation, projected.message, std::nullopt};
    case Projection::source_mismatch: return {Code::source_mismatch, projected.message, std::nullopt};
    case Projection::preparation_read_failed: return {Code::preparation_read_failed, projected.message, std::nullopt};
    default: break;
  }
  if (!projected.projection) {return {Code::projection_failed, "No projection result", std::nullopt};}
  const auto & result = *projected.projection;
  const auto * reason = depthRegionCodeName(result.code);
  switch (result.code) {
    case DepthRegionCode::invalid_options:
    case DepthRegionCode::invalid_region:
    case DepthRegionCode::region_too_large:
      return {Code::invalid_selection, reason, std::nullopt};
    case DepthRegionCode::invalid_input:
    case DepthRegionCode::projection_failed:
      return {Code::projection_failed, reason, std::nullopt};
    default: break;
  }
  const auto leading_count = result.clusters.empty() ? 0 : result.clusters.front().point_count;
  return finishObject(observation, semantic_label,
    ObjectSelectionEvidence{*selection, point_options, result.code, result.region_pixels,
      result.valid_points, leading_count, result.representative}, result.ok() ? nullptr : reason);
}

/** @brief 以来源一致且经确认在内部的底盘原点创建空间，缺证据时保持待定位。 */
EntityBuildResult createRegionEntity(
  const PreparationReader & reader, const Observation & observation, std::string_view semantic_label,
  const std::optional<RegionInteriorConfirmation> & confirmation)
{
  if (!validText(semantic_label)) {return {Code::invalid_label, "Invalid semantic label", std::nullopt};}
  const auto checked = validateObservation(observation);
  if (!checked.ok()) {return {Code::invalid_observation, checked.message, std::nullopt};}
  if (confirmation && !matchesRegionConfirmation(*confirmation, observation, semantic_label)) {
    return {Code::invalid_region_confirmation, "Confirmation reference or region, observation, and map scope mismatch",
      std::nullopt};
  }
  if (reader.manifestSha256() != observation.source_manifest_sha256 || reader.mapFrame() != observation.map_frame) {
    return {Code::source_mismatch, "Observation does not identify this preparation and map frame", std::nullopt};
  }
  const auto loaded = reader.loadPairMetadata(observation.pair_id);
  if (!loaded.ok()) {return {Code::preparation_read_failed, loaded.message, std::nullopt};}
  if (!matchesRegionSource(*loaded.pair, observation)) {
    return {Code::source_mismatch, "Observation identity or body pose differs from the saved pair", std::nullopt};
  }
  const auto stable = reader.verifyUnchanged();
  if (!stable.ok()) {return {Code::preparation_read_failed, stable.message, std::nullopt};}

  auto ref = observationRef(observation);
  if (!confirmation) {
    return finishEntity(observation, semantic_label, EntityKind::region, std::move(ref), "missing_region_confirmation");
  }
  ref.region_interior = RegionInteriorEvidence{confirmation->evidence_ref, std::nullopt};
  if (!observation.body_pose_map) {
    return finishEntity(observation, semantic_label, EntityKind::region, std::move(ref), "missing_body_map_pose");
  }
  const auto & body = *loaded.pair->body_pose;
  ref.region_interior->body_anchor = RegionBodyAnchor{
    loaded.pair->body_frame, body.time.stamp, body.pose->translation_m};
  return finishEntity(observation, semantic_label, EntityKind::region, std::move(ref), nullptr);
}

}  // namespace vln_memory
