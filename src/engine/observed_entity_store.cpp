#include <vln_memory/engine/observed_entity_store.hpp>

namespace vln_memory
{
namespace
{

/** @brief 从已经打开的读取器提取持久身份，v1 仍可独立回读观测。 */
std::optional<EntityStoreContext> storeContext(const ObservationDatasetReader & dataset)
{
  const auto & info = dataset.info();
  if (info.schema_version != "observation_dataset/2" || !info.file_integrity_verified) {
    return std::nullopt;
  }
  return EntityStoreContext{info.dataset_manifest_sha256, info.source_manifest_sha256,
    info.map_id, info.map_frame, info.geometry_revision};
}

}  // namespace

/** @brief 使用读取器核验后的观测集身份创建数据库。 */
EntityStoreOpenResult createObservedEntityStore(
  const std::filesystem::path & path, const ObservationDatasetReader & dataset)
{
  const auto context = storeContext(dataset);
  if (!context) {return {{EntityStoreCode::dataset_mismatch, "Expected a verified v2 dataset"}, nullptr};}
  return createEntityStore(path, *context);
}

/** @brief 使用原数据集回读正式观测，禁止只用实体内部声明作为写入依据。 */
EntityStoreResult insertObservedEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const Entity & entity)
{
  const auto context = storeContext(dataset);
  if (!context || store.context().dataset_manifest_sha256 != context->dataset_manifest_sha256) {
    return {EntityStoreCode::dataset_mismatch, "Dataset differs from the store binding"};
  }
  if (entity.observations.size() != 1) {
    return {EntityStoreCode::invalid_reference, "Expected one observation for a new entity"};
  }
  const auto loaded = dataset.loadObservation(entity.observations.front().observation_id);
  if (loaded.code == ObservationDatasetReadCode::observation_not_found) {
    return {EntityStoreCode::invalid_reference, loaded.message};
  }
  if (!loaded.ok()) {return {EntityStoreCode::observation_read_failed, loaded.message};}
  return store.insertEntity(entity, *context, *loaded.observation);
}

}  // namespace vln_memory
