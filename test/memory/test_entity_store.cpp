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

/** @brief 恢复无定位引用的旧实体表，保留关联、修订和原始证据。 */
void restoreLegacyEntityTable(const fs::path & file)
{
  execute(file, R"sql(
    PRAGMA foreign_keys = OFF;
    BEGIN;
    DROP TABLE entity_relations;
    DROP TABLE pending_entities;
    DROP TABLE tool_calls;
    DROP TABLE build_progress;
    CREATE TABLE legacy_entities(
      entity_id TEXT PRIMARY KEY NOT NULL,
      kind TEXT NOT NULL CHECK(kind IN ('object','region')),
      label TEXT NOT NULL, revision INTEGER NOT NULL CHECK(revision > 0),
      status TEXT NOT NULL CHECK(status IN ('located','pending_location')),
      pending_reason TEXT,
      CHECK((status = 'located') = (pending_reason IS NULL))
    );
    INSERT INTO legacy_entities SELECT entity_id,kind,label,revision,status,pending_reason FROM entities;
    DROP TABLE entities;
    ALTER TABLE legacy_entities RENAME TO entities;
    PRAGMA user_version = 2;
    COMMIT;
  )sql");
}

/** @brief 用首版关联表格式构造含实际证据的 v1 库。 */
void restoreLegacyAssociationTable(const fs::path & file)
{
  execute(file, R"sql(
    BEGIN;
    ALTER TABLE entity_observations RENAME TO observations_v2;
    CREATE TABLE entity_observations(
      entity_id TEXT NOT NULL REFERENCES entities(entity_id),
      observation_id TEXT NOT NULL, observation_revision TEXT NOT NULL, source_manifest_sha256 TEXT NOT NULL,
      evidence_json TEXT NOT NULL,
      PRIMARY KEY(entity_id, observation_id, observation_revision)
    );
    INSERT INTO entity_observations
      SELECT entity_id,observation_id,observation_revision,source_manifest_sha256,evidence_json FROM observations_v2;
    DROP TABLE observations_v2;
    PRAGMA user_version = 1;
    COMMIT;
  )sql");
}

/** @brief 验证已定位实体的回读引用包含准确的观测 ID 与观测修订。 */
void expectLocationReference(const Entity & entity, const Observation & observation)
{
  const auto & location = entity.location_observation;
  expect(entity.status == EntityStatus::located && location &&
    location->observation_id == observation.id && location->observation_revision == observation.revision,
    "Location does not reference the expected observation revision");
}

/** @brief 验证补定位的引用、版本、回滚，以及选择后继续追加。 */
void locationResolveCases(
  const fs::path & file, EntityStore & store, const Entity & original,
  const Observation & first, const Observation & selected)
{
  const auto & binding = store.context();
  const EntityLocationRef target{selected.id, selected.revision};
  const EntityLocationRef first_ref{first.id, first.revision};
  expect(store.resolveEntityLocation(original.entity_id, 1, target, binding, selected).code ==
    Code::revision_conflict, "Stale location request was accepted");
  expect(store.resolveEntityLocation(original.entity_id, 2, first_ref, binding, first).code ==
    Code::location_unavailable, "Observation without geometry was accepted");
  auto incomplete = selected;
  incomplete.depth_camera_pose_map.reset();
  expect(store.resolveEntityLocation(original.entity_id, 2, target, binding, incomplete).code ==
    Code::location_unavailable, "Projection from observation without depth pose was accepted");
  auto foreign = selected;
  foreign.id = "another-entity-observation";
  const EntityLocationRef foreign_ref{foreign.id, foreign.revision};
  expect(store.resolveEntityLocation(original.entity_id, 2, foreign_ref, binding, foreign).code ==
    Code::invalid_reference, "Unlinked observation was accepted");
  const EntityLocationRef wrong_revision{selected.id, 1};
  expect(store.resolveEntityLocation(original.entity_id, 2, wrong_revision, binding, selected).code ==
    Code::invalid_reference, "Wrong observation revision was accepted");

  execute(file, "CREATE TRIGGER inject_location_failure AFTER UPDATE OF status ON entities "
    "BEGIN SELECT RAISE(FAIL, 'injected location failure'); END");
  const auto failed = store.resolveEntityLocation(original.entity_id, 2, target, binding, selected);
  expect(failed.code == Code::storage_error && failed.message.find("injected location failure") != std::string::npos,
    "Test did not reach the location update");
  const auto rolled_back = store.loadEntity(original.entity_id);
  expect(rolled_back.ok() && rolled_back.entity && rolled_back.entity->revision == 2 &&
    rolled_back.entity->status == EntityStatus::pending_location && !rolled_back.entity->location_observation &&
    rolled_back.entity->location_pending_reason == original.location_pending_reason &&
    rolled_back.entity->observations.size() == 2, "Failed resolution changed the entity");
  execute(file, "DROP TRIGGER inject_location_failure");

  const auto resolved = store.resolveEntityLocation(original.entity_id, 2, target, binding, selected);
  expect(resolved.code == Code::ready, resolved.message);
  expect(store.resolveEntityLocation(original.entity_id, 3, target, binding, selected).code ==
    Code::already_located, "Located entity accepted another resolution");

  execute(file, "UPDATE entities SET location_linked_at_revision=3");
  expect(store.loadEntity(original.entity_id).code == Code::invalid_store, "Dangling location reference accepted");
  execute(file, "UPDATE entities SET location_linked_at_revision=1");
  expect(store.loadEntity(original.entity_id).code == Code::invalid_store, "Location without geometry accepted");
  execute(file, "UPDATE entities SET location_linked_at_revision=2");
  const auto located = store.loadEntity(original.entity_id);
  expect(located.ok() && located.entity && located.entity->revision == 3 &&
    located.entity->observations.size() == 2 && !located.entity->location_pending_reason, located.message);
  expectLocationReference(*located.entity, selected);

  auto third = selected;
  third.id = "third-view";
  const EntityObservationRef next_ref{
    third.id, third.revision, third.source_manifest_sha256, std::nullopt, std::nullopt};
  const auto appended = store.appendEntityObservation(original.entity_id, 3, next_ref, store.context(), third);
  expect(appended.code == Code::ready, appended.message);
  const auto after = store.loadEntity(original.entity_id);
  expect(after.ok() && after.entity && after.entity->revision == 4 && after.entity->observations.size() == 3 &&
    after.entity->observations.back().observation_id == third.id, "Append after revision gap did not round-trip");
  expectLocationReference(*after.entity, selected);
}

/** @brief 验证迁移后的库可以追加证据，版本冲突和后半段写入失败不会留下部分结果。 */
void appendCases(
  const fs::path & file, EntityStore & store, const Entity & original, const Observation & first)
{
  auto second = first;
  second.id = "a-second-view";  // 按观测 ID 排序时会错误地放在首条前面。
  // 核心回归构造补定位所需字段。完整观测验证在发布目录回归中执行。
  second.depth_media = "fixture-depth.png";
  second.depth_storage.emplace();
  second.depth_model.emplace();
  second.depth_camera_pose_map.emplace().pose = TfPose{};
  const EntityObservationRef ref{second.id, second.revision, second.source_manifest_sha256,
    ObjectSelectionEvidence{DepthBoundingBox{0, 0, 2, 2}, {}, DepthRegionCode::ready, 4, 4, 4,
      DepthRegionPoint{{1, 1}, 1.25F, {0.1, 0.2, 1.25}}}, std::nullopt};
  auto stale = openEntityStore(file);
  expect(stale.ok(), stale.message);
  const auto before = stale.store->loadEntity(original.entity_id);
  expect(before.ok() && before.entity && before.entity->revision == 1, "Wrong base revision");
  const auto & binding = store.context();
  auto wrong_source = ref;
  wrong_source.source_manifest_sha256 = std::string(64, 'c');
  expect(store.appendEntityObservation(original.entity_id, 1, wrong_source, binding, second).code ==
    Code::invalid_reference, "Wrong evidence source was accepted");

  execute(file, "CREATE TRIGGER inject_revision_failure BEFORE UPDATE OF revision ON entities "
    "BEGIN SELECT RAISE(ABORT, 'injected revision failure'); END");
  const auto failed = store.appendEntityObservation(original.entity_id, 1, ref, binding, second);
  expect(failed.code == Code::storage_error && failed.message.find("injected revision failure") != std::string::npos,
    "Test did not reach the revision update");
  const auto rolled_back = store.loadEntity(original.entity_id);
  expect(rolled_back.ok() && rolled_back.entity && rolled_back.entity->revision == 1 &&
    rolled_back.entity->observations.size() == 1, "An association or revision survived rollback");
  execute(file, "DROP TRIGGER inject_revision_failure");

  const auto appended = store.appendEntityObservation(original.entity_id, 1, ref, binding, second);
  expect(appended.code == Code::ready, appended.message);
  expect(stale.store->appendEntityObservation(original.entity_id, before.entity->revision, ref,
    stale.store->context(), second).code == Code::revision_conflict, "Stale revision was accepted");
  const auto duplicate = store.appendEntityObservation(original.entity_id, 2, ref, binding, second);
  expect(duplicate.ok() && duplicate.code == Code::already_linked, "Identical evidence was not idempotent");
  auto changed = ref;
  changed.object_selection->representative->point_map_m[0] += 1.0;
  expect(store.appendEntityObservation(original.entity_id, 2, changed, binding, second).code ==
    Code::evidence_conflict, "Different evidence for the same observation overwrote the old evidence");

  auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  const auto loaded = reopened.store->loadEntity(original.entity_id);
  expect(loaded.ok() && loaded.entity && loaded.entity->revision == 2 && loaded.entity->observations.size() == 2,
    "Append did not survive reopening or retry changed the revision");
  const auto & entity = *loaded.entity;
  const auto & appended_ref = entity.observations[1];
  expect(entity.entity_id == original.entity_id && entity.status == original.status &&
    entity.location_pending_reason == original.location_pending_reason && !entity.location_observation &&
    entity.observations.front().observation_id == first.id && appended_ref.observation_id == second.id &&
    appended_ref.observation_revision == ref.observation_revision &&
    appended_ref.source_manifest_sha256 == ref.source_manifest_sha256 && appended_ref.object_selection &&
    appended_ref.object_selection->representative &&
    appended_ref.object_selection->representative->point_map_m == ref.object_selection->representative->point_map_m,
    "Association order, original entity state or new evidence was lost");
  locationResolveCases(file, *reopened.store, entity, first, second);
}

/** @brief 验证写入、回滚、重复 ID 和 v1 升级后仍完整的原始引用。 */
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
  restoreLegacyEntityTable(file);
  restoreLegacyAssociationTable(file);
  auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  const auto loaded = reopened.store->loadEntity(entity.entity_id);
  expect(loaded.ok() && loaded.entity.has_value(), loaded.message);
  const auto & restored = *loaded.entity;
  expect(restored.entity_id == entity.entity_id && restored.semantic_label == entity.semantic_label &&
    restored.revision == 1 && restored.status == EntityStatus::pending_location &&
    restored.location_pending_reason == entity.location_pending_reason && !restored.location_observation &&
    restored.geometry_revision == binding.geometry_revision && restored.observations.size() == 1,
    "Pending entity fields changed after reopening");
  const auto & ref = restored.observations.front();
  expect(ref.observation_id == observation.id && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256 &&
    ref.object_selection && !ref.object_selection->representative, "Pending reference or selection was lost");
  const auto & vertices = std::get<DepthPolygon>(ref.object_selection->selection).vertices;
  expect(vertices.size() == 3 && vertices[1].u == 2 && vertices[2].v == 2, "Polygon was not preserved");
  appendCases(file, *reopened.store, restored, observation);
}

/** @brief 用现有创建器产出的第二帧几何，为第一帧创建的原实体补定位。 */
void resolvePublishedEvidence(
  EntityStore & store, const ObservationDatasetReader & dataset, const Entity & pending, const Entity & candidate)
{
  expect(pending.status == EntityStatus::pending_location && candidate.location_observation,
    "Expected pending entity and a candidate location");
  const auto inserted = insertObservedEntity(store, dataset, pending);
  expect(inserted.ok(), inserted.message);
  const auto appended = appendObservedEvidence(
    store, dataset, pending.entity_id, 1, candidate.observations.front());
  expect(appended.code == Code::ready, appended.message);
  const auto before = store.loadEntity(pending.entity_id);
  expect(before.ok() && before.entity && before.entity->status == EntityStatus::pending_location &&
    before.entity->revision == 2 && before.entity->observations.size() == 2 && !before.entity->location_observation,
    "Appending a location candidate selected it before resolution");
  const auto resolved = resolveEntityLocation(store, dataset, pending.entity_id, 2, *candidate.location_observation);
  expect(resolved.code == Code::ready, resolved.message);
}

/** @brief 通过发布目录重开数据库，验证原 ID、两条观测及第二条定位依据。 */
void expectResolvedRoundTrip(
  EntityStore & store, const Entity & pending, const Observation & first, const Observation & second)
{
  const auto loaded = store.loadEntity(pending.entity_id);
  expect(loaded.ok() && loaded.entity && loaded.entity->entity_id == pending.entity_id &&
    loaded.entity->revision == 3 && loaded.entity->observations.size() == 2 &&
    loaded.entity->observations[0].observation_id == first.id &&
    loaded.entity->observations[1].observation_id == second.id && !loaded.entity->location_pending_reason,
    "Location update did not round-trip on the original entity");
  expectLocationReference(*loaded.entity, second);
}

/** @brief 用实际正式观测的深度投影和构造空间确认验证补定位写入。 */
void publishedResolution(
  const fs::path & file, const ObservationDatasetReader & dataset, const PreparationReader & preparation,
  const Observation & first, const Observation & second, const DepthBoundingBox & box, bool confirm)
{
  auto opened = openEntityStore(file);
  expect(opened.ok(), opened.message);
  const auto pending_object = createObjectEntity(preparation, first, "箱子");
  expect(pending_object.ok(), pending_object.message);
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const auto object_candidate = createObjectEntity(preparation, second, "箱子", box, options);
  expect(object_candidate.ok() && object_candidate.entity->status == EntityStatus::located,
    "Second observation lacks an object projection for this selection");
  resolvePublishedEvidence(*opened.store, dataset, *pending_object.entity, *object_candidate.entity);

  std::optional<Entity> pending_region;
  if (confirm) {
    auto initial = createRegionEntity(preparation, first, "房间");
    expect(initial.ok(), initial.message);
    pending_region = std::move(*initial.entity);
    const RegionInteriorConfirmation confirmation{"房间", second.id, second.revision,
      second.source_manifest_sha256, second.map_id, second.map_frame, second.geometry_revision,
      "fixture/second-interior-confirmation"};
    const auto region_candidate = createRegionEntity(preparation, second, "房间", confirmation);
    expect(region_candidate.ok(), region_candidate.message);
    resolvePublishedEvidence(*opened.store, dataset, *pending_region, *region_candidate.entity);
  }
  opened.store.reset();

  const auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  expectResolvedRoundTrip(*reopened.store, *pending_object.entity, first, second);
  if (pending_region) {expectResolvedRoundTrip(*reopened.store, *pending_region, first, second);}
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
  const auto & batches = dataset.reader->batches();
  expect(batch.observations.size() > 1 || batches.size() > 1, "At least two observations are required");
  const auto & second_id = batch.observations.size() > 1 ?
    batch.observations[1].id : batches[1].observation_ids.front();
  const auto second = dataset.reader->loadObservation(second_id);
  expect(second.ok(), second.message);
  const EntityObservationRef next_view{
    second.observation->id, second.observation->revision, second.observation->source_manifest_sha256,
    std::nullopt, std::nullopt};
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const auto object = createObjectEntity(*preparation.reader, observation, "箱子", DepthBoundingBox{u, v, u + 1, v + 1}, options);
  expect(object.ok(), object.message);
  expectLocationReference(*object.entity, observation);
  std::optional<RegionInteriorConfirmation> confirmation;
  if (confirm) {
    confirmation = RegionInteriorConfirmation{"房间", observation.id, observation.revision,
      observation.source_manifest_sha256, observation.map_id, observation.map_frame, observation.geometry_revision,
      "fixture/interior-confirmation"};
  }
  const auto region = createRegionEntity(*preparation.reader, observation, "房间", confirmation);
  expect(region.ok(), region.message);
  if (confirm) {expectLocationReference(*region.entity, observation);}
  else {expect(!region.entity->location_observation, "Unconfirmed region gained a location reference");}

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
    const auto appended = appendObservedEvidence(
      *created.store, *dataset.reader, entity->entity_id, entity->revision, next_view);
    expect(appended.code == Code::ready, appended.message);
  }
  const auto fresh_object = created.store->loadEntity(object.entity->entity_id);
  expect(fresh_object.ok() && fresh_object.entity, fresh_object.message);
  expectLocationReference(*fresh_object.entity, observation);
  created.store.reset();
  restoreLegacyEntityTable(file);

  const auto reopened = openEntityStore(file);
  expect(reopened.ok(), reopened.message);
  const auto restored_object = reopened.store->loadEntity(object.entity->entity_id);
  const auto restored_region = reopened.store->loadEntity(region.entity->entity_id);
  expect(restored_object.ok() && restored_object.entity, restored_object.message);
  expect(restored_region.ok() && restored_region.entity, restored_region.message);
  expectLocationReference(*restored_object.entity, observation);
  for (const auto * entity : {&*restored_object.entity, &*restored_region.entity}) {
    expect(entity->revision == 2 && entity->observations.size() == 2 &&
      entity->observations[0].observation_id == observation.id &&
      entity->observations[1].observation_id == next_view.observation_id &&
      entity->observations[1].observation_revision == next_view.observation_revision &&
      entity->observations[1].source_manifest_sha256 == next_view.source_manifest_sha256,
      "Published observation associations differ after reopening");
  }
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
    expectLocationReference(*restored_region.entity, observation);
    expect(interior && interior->body_anchor &&
      interior->confirmation_ref == confirmation->evidence_ref &&
      interior->body_anchor->body_frame == observation.body_frame &&
      interior->body_anchor->stamp.sec == observation.stamp.sec &&
      interior->body_anchor->stamp.nanosec == observation.stamp.nanosec &&
      interior->body_anchor->point_map_m == observation.body_pose_map->pose->translation_m,
      "Stored region lost its confirmed anchor");
  } else {
    expect(restored_region.entity->status == EntityStatus::pending_location && !interior &&
      !restored_region.entity->location_observation &&
      restored_region.entity->location_pending_reason == "missing_region_confirmation",
      "Stored region lost its pending reason");
  }
  publishedResolution(file, *dataset.reader, *preparation.reader, observation, *second.observation,
    DepthBoundingBox{u, v, u + 1, v + 1}, confirm);
  std::cout << "Entity location migration and resolution from the second published observation passed" << '\n';
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
