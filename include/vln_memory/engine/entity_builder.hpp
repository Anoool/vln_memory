#pragma once

#include <optional>
#include <string>
#include <string_view>

#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/memory/entity.hpp>
#include <vln_memory/memory/observation.hpp>

namespace vln_memory
{

enum class EntityBuildCode
{
  created, invalid_label, invalid_observation, source_mismatch, preparation_read_failed,
  invalid_selection, projection_failed, id_unavailable, invalid_region_confirmation
};

struct EntityBuildResult
{
  EntityBuildCode code{EntityBuildCode::invalid_observation};
  std::string message;
  std::optional<Entity> entity;

  /** @brief 合法待定位实体也算创建成功；错误时没有半成品。 */
  bool ok() const noexcept {return code == EntityBuildCode::created && entity.has_value();}
};

/**
 * @brief 以正式观测创建一个物品；可选的 bbox/polygon 必须位于深度原图。
 * @note 核对已保存配对及来源，复用区域投影。无选区、缺几何或选区深度不足时保留待定位实体。
 *       返回拥有型记录，每次成功创建分配新 ID；持久存储和重复创建的幂等性由未来工作库负责。
 */
EntityBuildResult createObjectEntity(
  const PreparationReader & reader, const Observation & observation, std::string_view semantic_label,
  const std::optional<DepthRegion> & selection = std::nullopt, const DepthRegionOptions & options = {});

/** @brief 上游对新空间内部位置的确认，绑定目标标签与确切观测版本。 */
struct RegionInteriorConfirmation
{
  std::string semantic_label;
  std::string observation_id;
  std::uint64_t observation_revision{0};
  std::string source_manifest_sha256;
  std::string map_id;
  std::string map_frame;
  std::string geometry_revision;
  std::string evidence_ref;
};

/**
 * @brief 用经确认在区域内的历史底盘位姿创建空间实体。
 * @note 确认由上游提供；本入口核对确认范围、原始配对及位姿来源，不解码图像。
 *       缺确认或位姿时返回待定位实体。返回值拥有证据，不持有输入引用。
 */
EntityBuildResult createRegionEntity(
  const PreparationReader & reader, const Observation & observation, std::string_view semantic_label,
  const std::optional<RegionInteriorConfirmation> & confirmation = std::nullopt);

}  // namespace vln_memory
