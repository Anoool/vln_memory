#pragma once

#include <vln_memory/engine/observation_dataset_reader.hpp>
#include <vln_memory/memory/entity_store.hpp>

namespace vln_memory
{

/** @brief 在新路径创建与已验证 v2 观测集绑定的工作库。 */
EntityStoreOpenResult createObservedEntityStore(
  const std::filesystem::path & path, const ObservationDatasetReader & dataset);

/** @brief 从绑定的观测集回读实体的确切观测后，事务写入实体和证据。 */
EntityStoreResult insertObservedEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const Entity & entity);

}  // namespace vln_memory
