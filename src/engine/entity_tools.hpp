#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include <nlohmann/json_fwd.hpp>
#include <vln_memory/engine/observed_entity_store.hpp>

namespace vln_memory
{
struct NearOptions
{
  std::string dimension{"xy"};
  double threshold_m{1.5};
};

/** @brief 序列化实体和关系，供工具返回及版本发布共用。 */
nlohmann::json describeEntity(const Entity & entity);
nlohmann::json describeRelation(const EntityRelation & relation);

/** @brief 序列化人工标注，供搜索结果和发布共用。 */
nlohmann::json describeAnnotation(const EntityAnnotation & annotation);

/** @brief 在同一观测集和 SQLite 工作库中执行实体或关系工具。 */
nlohmann::json runEntityTool(
  EntityStore & store, const ObservationDatasetReader & dataset, const std::filesystem::path & root,
  const NearOptions & near, std::string_view name, const nlohmann::json & arguments);
}  // namespace vln_memory
