#pragma once

#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/bag/depth/depth_region_projection.hpp>
#include <vln_memory/memory/observation.hpp>
#include <vln_memory/engine/observation_dataset_reader.hpp>

namespace vln_memory
{

enum class ObservationDepthRegionCode
{
  evaluated, invalid_observation, source_mismatch, preparation_read_failed,
  missing_depth, missing_depth_model, missing_depth_pose
};

struct ObservationDepthRegionResult
{
  ObservationDepthRegionCode code{ObservationDepthRegionCode::invalid_observation};
  std::string message;
  std::optional<PreparationReadCode> read_code;
  std::optional<ObservationCode> observation_code;
  std::optional<DepthRegionResult> projection;

  /** @brief 来源、投影和选簇均成功时返回 true。 */
  bool ok() const noexcept {return code == ObservationDepthRegionCode::evaluated && projection && projection->ok();}
};

/**
 * @brief 从正式观测核对原 preparation、恢复米制深度，投影深度原图区域。
 * @note 使用该帧自己的模型和历史位姿，校验来源指纹、配对身份、时钟段和深度证据。
 *       合法的 RGB/Depth 注册或曝光同步 pending 不影响深度原图到 map。
 */
ObservationDepthRegionResult projectObservationDepthRegion(
  const PreparationReader & reader, const Observation & observation,
  const DepthRegion & region, const DepthRegionOptions & options = {});

/** @brief 从直出证据按观测 ID 回读深度并投影深度原图选区。 */
ObservationDepthRegionResult projectObservationDepthRegion(
  const ObservationDatasetReader & dataset, std::string_view observation_id,
  const DepthRegion & region, const DepthRegionOptions & options = {});

}  // namespace vln_memory
