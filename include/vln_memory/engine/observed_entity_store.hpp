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

/** @brief 从已绑定观测集核验并追加证据，由调用方提供实体 ID 和当前 revision。 */
EntityStoreResult appendObservedEvidence(
  EntityStore & store, const ObservationDatasetReader & dataset, std::string_view entity_id,
  std::uint64_t expected_revision, const EntityObservationRef & evidence);

/** @brief 从绑定观测集核验定位依据，为待定位实体提交显式状态变更。 */
EntityStoreResult resolveEntityLocation(
  EntityStore & store, const ObservationDatasetReader & dataset, std::string_view entity_id,
  std::uint64_t expected_revision, const EntityLocationRef & selected);

}  // namespace vln_memory
