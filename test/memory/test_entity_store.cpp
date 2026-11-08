#include <vln_memory/engine/entity_builder.hpp>
#include <vln_memory/engine/observed_entity_store.hpp>

#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <sqlite3.h>

namespace
{
using namespace vln_memory;
using Code = EntityStoreCode;
namespace fs = std::filesystem;

/** @brief Release 构建也执行检查。 */
void expect(bool condition, std::string_view message)
{
  if (!condition) {throw std::runtime_error(std::string(message));}
}

/** @brief 为每次运行创建独立的 SQLite 文件路径，避开并发测试。 */
fs::path databasePath(const fs::path & parent)
{
  const auto id = allocateEntityId();
  expect(id.has_value(), "Cannot allocate scratch ID");
  return parent / (*id + ".sqlite3");
}

/** @brief 测试用 SQL 在第二次写入时触发失败。 */
void execute(const fs::path & file, const char * sql)
{
  sqlite3 * handle = nullptr;
  const int result = sqlite3_open_v2(file.string().c_str(), &handle, SQLITE_OPEN_READWRITE, nullptr);
  const std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(handle, sqlite3_close);
  expect(result == SQLITE_OK, "Cannot open fixture database");
  const int written = sqlite3_exec(db.get(), sql, nullptr, nullptr, nullptr);
  expect(written == SQLITE_OK, sqlite3_errmsg(db.get()));
}

/** @brief 验证写入、回滚、重复 ID 和重启后仍完整的原始引用。 */
void storeCases(const fs::path & parent)
{
  const EntityStoreContext binding{std::string(64, 'a'), std::string(64, 'b'), "map_id", "map", "geometry-v1"};
  Observation observation;
  observation.id = "fixture-observation";
  observation.revision = std::numeric_limits<std::uint64_t>::max();
  observation.source_manifest_sha256 = binding.source_manifest_sha256;
  observation.map_id = binding.map_id;
  observation.map_frame = binding.map_frame;
  observation.geometry_revision = binding.geometry_revision;
  observation.depth_image.width = 4;
  observation.depth_image.height = 4;
  const auto id = allocateEntityId();
  expect(id.has_value(), "Cannot allocate entity ID");
  Entity entity{*id, EntityKind::object, "测试物品", EntityStatus::pending_location, 1,
    binding.map_id, binding.map_frame, binding.geometry_revision,
    {{observation.id, observation.revision, observation.source_manifest_sha256,
      ObjectSelectionEvidence{DepthPolygon{{{0, 0}, {2, 0}, {0, 2}}}, {}, std::nullopt, 0, 0, 0, std::nullopt},
      std::nullopt}}, "missing_depth_pose"};

  const auto file = databasePath(parent);
  expect(!openEntityStore(file).ok() && !fs::exists(file), "Opening a missing store created a file");
  auto created = createEntityStore(file, binding);
  expect(created.ok(), created.message);
  auto & store = *created.store;
  expect(createEntityStore(file, binding).code == Code::already_exists, "Recreation was allowed");
  auto wrong = entity;
  wrong.observations.front().observation_revision = 1;
  expect(store.insertEntity(wrong, binding, observation).code == Code::invalid_reference, "Wrong revision accepted");
  wrong = entity;
  wrong.geometry_revision = "other-trajectory";
  expect(store.insertEntity(wrong, binding, observation).code == Code::invalid_reference, "Wrong trajectory accepted");
  wrong = entity;
  wrong.observations.front().source_manifest_sha256 = std::string(64, 'c');
  expect(store.insertEntity(wrong, binding, observation).code == Code::invalid_reference, "Wrong source accepted");
  auto other_dataset = binding;
  other_dataset.dataset_manifest_sha256 = std::string(64, 'd');
  expect(store.insertEntity(entity, other_dataset, observation).code == Code::dataset_mismatch,
    "Different dataset with the same observation was accepted");

  execute(file, "CREATE TRIGGER inject_failure BEFORE INSERT ON entity_observations "
    "BEGIN SELECT RAISE(ABORT, 'injected evidence failure'); END");
  const auto failed = store.insertEntity(entity, binding, observation);
  expect(failed.code == Code::storage_error && failed.message.find("injected evidence failure") != std::string::npos,
    "Test did not reach the second write");
  expect(store.loadEntity(entity.entity_id).code == Code::not_found, "A partial entity survived rollback");
  execute(file, "DROP TRIGGER inject_failure");
  const auto inserted = store.insertEntity(entity, binding, observation);
  expect(inserted.ok(), inserted.message);
  expect(store.insertEntity(entity, binding, observation).code == Code::already_exists, "Duplicate ID was accepted");

  created.store.reset();
  auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  const auto loaded = reopened.store->loadEntity(entity.entity_id);
  expect(loaded.ok() && loaded.entity.has_value(), loaded.message);
  const auto & restored = *loaded.entity;
  expect(restored.entity_id == entity.entity_id && restored.semantic_label == entity.semantic_label &&
    restored.revision == 1 && restored.status == EntityStatus::pending_location &&
    restored.location_pending_reason == entity.location_pending_reason &&
    restored.geometry_revision == binding.geometry_revision && restored.observations.size() == 1,
    "Pending entity fields changed after reopening");
  const auto & ref = restored.observations.front();
  expect(ref.observation_id == observation.id && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256 &&
    ref.object_selection && !ref.object_selection->representative, "Pending reference or selection was lost");
  const auto & vertices = std::get<DepthPolygon>(ref.object_selection->selection).vertices;
  expect(vertices.size() == 3 && vertices[1].u == 2 && vertices[2].v == 2, "Polygon was not preserved");
}

/** @brief 使用真正发布的观测、准备层和实体创建器完成事务回读。 */
void published(const fs::path & root, std::int64_t u, std::int64_t v, const fs::path & parent, bool confirm)
{
  const auto dataset = openObservationDataset(root);
  expect(dataset.ok(), dataset.message);
  const auto preparation = dataset.reader->openSourcePreparation();
  expect(preparation.ok(), preparation.message);
  const auto batch = dataset.reader->loadBatch(0);
  expect(batch.ok() && !batch.observations.empty(), batch.message);
  const auto & observation = batch.observations.front();
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const auto object = createObjectEntity(*preparation.reader, observation, "箱子", DepthBoundingBox{u, v, u + 1, v + 1}, options);
  expect(object.ok(), object.message);
  expect(object.entity->status == EntityStatus::located, "Selection did not produce a measured point");
  std::optional<RegionInteriorConfirmation> confirmation;
  if (confirm) {
    confirmation = RegionInteriorConfirmation{"房间", observation.id, observation.revision,
      observation.source_manifest_sha256, observation.map_id, observation.map_frame, observation.geometry_revision,
      "fixture/interior-confirmation"};
  }
  const auto region = createRegionEntity(*preparation.reader, observation, "房间", confirmation);
  expect(region.ok(), region.message);

  const auto file = databasePath(parent);
  auto created = createObservedEntityStore(file, *dataset.reader);
  expect(created.ok(), created.message);
  auto missing = *object.entity;
  missing.observations.front().observation_id = "missing-observation";
  expect(insertObservedEntity(*created.store, *dataset.reader, missing).code == Code::invalid_reference,
    "Store accepted an ID absent from the dataset");
  for (const auto * entity : {&*object.entity, &*region.entity}) {
    const auto inserted = insertObservedEntity(*created.store, *dataset.reader, *entity);
    expect(inserted.ok(), inserted.message);
  }
  created.store.reset();

  const auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  const auto restored_object = reopened.store->loadEntity(object.entity->entity_id);
  const auto restored_region = reopened.store->loadEntity(region.entity->entity_id);
  expect(restored_object.ok() && restored_object.entity, restored_object.message);
  expect(restored_region.ok() && restored_region.entity, restored_region.message);
  const auto & restored_ref = restored_object.entity->observations.front();
  expect(restored_ref.object_selection.has_value(), "Stored object lost its selection");
  const auto & stored_selection = *restored_ref.object_selection;
  const auto & expected = *object.entity->observations.front().object_selection->representative;
  const auto & box = std::get<DepthBoundingBox>(stored_selection.selection);
  expect(restored_object.entity->status == EntityStatus::located &&
    box.u_min == u && box.v_min == v && box.u_end == u + 1 && box.v_end == v + 1 &&
    stored_selection.options.min_cluster_points == 1 && stored_selection.valid_points == 1 &&
    stored_selection.representative && stored_selection.representative->point_map_m == expected.point_map_m &&
    stored_selection.representative->optical_z_m == expected.optical_z_m, "Stored object lost measured evidence");
  const auto & interior = restored_region.entity->observations.front().region_interior;
  if (confirm) {
    expect(restored_region.entity->status == EntityStatus::located && interior && interior->body_anchor &&
      interior->confirmation_ref == confirmation->evidence_ref &&
      interior->body_anchor->body_frame == observation.body_frame &&
      interior->body_anchor->stamp.sec == observation.stamp.sec &&
      interior->body_anchor->stamp.nanosec == observation.stamp.nanosec &&
      interior->body_anchor->point_map_m == observation.body_pose_map->pose->translation_m,
      "Stored region lost its confirmed anchor");
  } else {
    expect(restored_region.entity->status == EntityStatus::pending_location && !interior &&
      restored_region.entity->location_pending_reason == "missing_region_confirmation",
      "Stored region lost its pending reason");
  }
  std::cout << "Entity store round-trip passed for observation " << observation.id << '\n';
}

}  // namespace

/** @brief 核心回归或已发布观测的端到端验收入口。 */
int main(int argc, char ** argv)
{
  try {
    if (argc == 2) {storeCases(argv[1]);}
    else if ((argc == 6 || argc == 7) && std::string_view(argv[1]) == "--published") {
      published(argv[2], std::stoll(argv[3]), std::stoll(argv[4]), argv[5],
        argc == 7 && std::string_view(argv[6]) == "--fixture-confirmation");
    } else {throw std::invalid_argument("Expected DB_PARENT or --published DATASET U V DB_PARENT [--fixture-confirmation]");}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
