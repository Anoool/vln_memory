#include "entity_tools.hpp"

#include "observation_tools.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include <vln_memory/engine/entity_builder.hpp>

namespace vln_memory
{
namespace
{
using Json = nlohmann::json;
using Code = EntityStoreCode;
namespace fs = std::filesystem;

/** @brief 将工作库的失败映射为可由调用方识别的状态。 */
Json failure(const EntityStoreResult & result)
{
  const char * status = "storage_error";
  switch (result.code) {
    case Code::not_found: status = "entity_not_found"; break;
    case Code::revision_conflict: status = "revision_conflict"; break;
    case Code::evidence_conflict: status = "evidence_conflict"; break;
    case Code::invalid_reference: status = "invalid_reference"; break;
    case Code::invalid_entity: status = "invalid_entity"; break;
    case Code::dataset_mismatch: status = "dataset_mismatch"; break;
    case Code::location_unavailable: status = "location_unavailable"; break;
    default: break;
  }
  return {{"status", status}, {"message", result.message}};
}

/** @brief 校验修订号后用于乐观锁，避免小数和负数隐式转换。 */
std::uint64_t revision(const Json & args)
{
  const auto & value = args.at("expected_revision");
  if (!value.is_number_integer() || value <= 0) {
    throw std::invalid_argument("expected_revision must be a positive integer");
  }
  return value.get<std::uint64_t>();
}

/** @brief 解析关系或观测使用的真实 ID，找不到时提前退出。 */
Observation readObservation(const ObservationDatasetReader & dataset, const std::string & id)
{
  const auto loaded = dataset.loadObservation(id);
  if (!loaded.ok()) {throw std::invalid_argument("Cannot read observation " + id + ": " + loaded.message);}
  return *loaded.observation;
}

/** @brief 更新描述、状态和备注，attributes 按字段合并。 */
void applyFields(Json & details, const Json & fields)
{
  if (!fields.is_object()) {throw std::invalid_argument("set_fields must be an object");}
  if (fields.contains("description")) {details["description"] = fields.at("description").get<std::string>();}
  if (fields.contains("notes")) {details["notes"] = fields.at("notes").get<std::vector<std::string>>();}
  if (fields.contains("attributes")) {
    const auto & attributes = fields.at("attributes");
    if (!attributes.is_object()) {throw std::invalid_argument("attributes must be an object");}
    if (!details.contains("attributes")) {details["attributes"] = Json::object();}
    details["attributes"].update(attributes);
  }
  if (fields.contains("recognition_state")) {
    const auto state = fields.at("recognition_state").get<std::string>();
    if (state != "tentative" && state != "supported" && state != "needs_review") {
      throw std::invalid_argument("Invalid recognition_state");
    }
    details["recognition_state"] = state;
  }
}

/** @brief 将观测证据及实测代表点整理为工具返回值。 */
Json describeEntity(const Entity & entity)
{
  auto out = Json::parse(entity.details_json);
  out["id"] = entity.entity_id;
  out["entity_id"] = entity.entity_id;
  out["kind"] = entity.kind == EntityKind::object ? "object" : "region";
  out["semantic_label"] = entity.semantic_label;
  out["revision"] = entity.revision;
  out["region_id"] = entity.region_id ? Json(*entity.region_id) : Json(nullptr);
  const auto position = selectedEntityPoint(entity);
  out["position_map"] = position ? Json(*position) : Json(nullptr);
  if (position) {
    out["geometry_quality"] = entity.kind == EntityKind::object ? "visible_surface_estimate" : "confirmed_interior";
  }
  out["geometry_revision"] = entity.geometry_revision;
  auto refs = Json::array();
  for (const auto & item : entity.observations) {
    Json ref{{"observation_id", item.observation_id}, {"observation_revision", item.observation_revision}};
    if (item.requested_selector_json) {ref["selector"] = Json::parse(*item.requested_selector_json);}
    if (item.object_selection && item.object_selection->representative) {
      const auto & point = *item.object_selection->representative;
      ref["position_map"] = point.point_map_m;
      ref["depth_pixel_xy"] = {point.pixel.u, point.pixel.v};
    }
    if (item.region_interior) {
      ref["confirmation_ref"] = item.region_interior->confirmation_ref;
      if (item.region_interior->body_anchor) {ref["position_map"] = item.region_interior->body_anchor->point_map_m;}
    }
    refs.push_back(std::move(ref));
  }
  out["observation_refs"] = std::move(refs);
  return out;
}

/** @brief 根据关系键生成稳定 ID，保留证据和距离定义。 */
Json describeRelation(const EntityRelation & relation)
{
  std::string id = relation.type;
  for (const auto & endpoint : relation.entity_ids) {id += ":" + endpoint;}
  Json out{{"id", id}, {"type", relation.type}, {"entity_ids", relation.entity_ids},
    {"evidence_observation_ids", relation.evidence_observation_ids}, {"basis", relation.basis},
    {"recognition_state", relation.recognition_state}, {"revision", relation.revision},
    {"geometry_revision", relation.geometry_revision}};
  if (!relation.evidence_ref.empty()) {out["evidence_ref"] = relation.evidence_ref;}
  if (relation.distance_m && relation.threshold_m) {
    out["distance_m"] = *relation.distance_m;
    out["parameters"] = {{"dimension", relation.dimension}, {"geometry", "representative_point"},
      {"threshold_m", *relation.threshold_m}};
  }
  return out;
}

/** @brief 根据投影工具回传的原始深度像素，复用现有实体创建器生成可回查的证据。 */
Entity buildView(
  const ObservationDatasetReader & dataset, const fs::path & root, const Json & view,
  EntityKind kind, const std::string & label)
{
  const auto obs = readObservation(dataset, view.at("observation_id").get<std::string>());
  EntityBuildResult built;
  Json selector;
  std::string pending_reason;
  if (kind == EntityKind::region) {
    if (view.contains("selector") || view.contains("bbox_xyxy_px")) {
      throw std::invalid_argument("Region evidence uses a confirmed interior body pose");
    }
    std::optional<RegionInteriorConfirmation> confirmation;
    if (view.contains("confirmation_ref")) {
      confirmation = RegionInteriorConfirmation{label, obs.id, obs.revision, obs.source_manifest_sha256,
        obs.map_id, obs.map_frame, obs.geometry_revision, view.at("confirmation_ref").get<std::string>()};
    }
    built = createRegionEntity(dataset, obs.id, label, confirmation);
  } else {
    if (view.contains("confirmation_ref")) {throw std::invalid_argument("Interior confirmation is for regions");}
    if (view.contains("selector") && view.contains("bbox_xyxy_px")) {
      throw std::invalid_argument("Provide selector or bbox_xyxy_px");
    }
    if (view.contains("selector")) {selector = view.at("selector");}
    if (view.contains("bbox_xyxy_px")) {selector = {{"type", "bbox"}, {"xyxy_px", view.at("bbox_xyxy_px")}};}
    std::optional<DepthRegion> depth_box;
    DepthRegionOptions options;
    options.min_cluster_points = 1;
    if (!selector.is_null()) {
      selector["image"] = selector.value("image", std::string("rgb"));
      const auto projected = runObservationTool(dataset, root, "project_depth",
        {{"observation_id", obs.id}, {"selector", selector}});
      const auto status = projected.at("status").get<std::string>();
      if (status == "ok") {
        const auto p = projected.at("depth_pixel_xy").get<std::array<std::int64_t, 2>>();
        depth_box = DepthBoundingBox{p[0], p[1], p[0] + 1, p[1] + 1};
      } else if (status == "invalid_depth" || status == "missing_map_pose" || status == "missing_camera_model" ||
        status == "insufficient_support" || status == "ambiguous_region")
      {
        pending_reason = status;
      } else {
        throw std::invalid_argument("Cannot use observation selector: " + status);
      }
    }
    built = createObjectEntity(dataset, obs.id, label, depth_box, options);
  }
  if (!built.ok()) {throw std::invalid_argument(built.message);}
  auto entity = std::move(*built.entity);
  if (!selector.is_null()) {entity.observations.front().requested_selector_json = selector.dump();}
  if (!pending_reason.empty()) {entity.location_pending_reason = pending_reason;}
  return entity;
}

/** @brief 先准备所有视图，图像读取和投影在数据库写锁之外完成。 */
std::vector<Entity> buildViews(
  const ObservationDatasetReader & dataset, const fs::path & root,
  const Json & views, EntityKind kind, const std::string & label)
{
  if (!views.is_array()) {throw std::invalid_argument("add_observations must be an array");}
  std::vector<Entity> entities;
  for (const auto & view : views) {entities.push_back(buildView(dataset, root, view, kind, label));}
  return entities;
}

/** @brief 解析模型或人工提交的语义关系，验证每条观测 ID。 */
std::vector<EntityRelation> readRelations(
  const ObservationDatasetReader & dataset, const Json & args, const std::string & id)
{
  std::vector<EntityRelation> relations;
  const auto input = args.value("relations", Json::array());
  if (!input.is_array()) {throw std::invalid_argument("relations must be an array");}
  for (const auto & item : input) {
    EntityRelation relation;
    relation.type = item.at("type").get<std::string>();
    relation.entity_ids = item.at("entity_ids").get<std::vector<std::string>>();
    for (auto & endpoint : relation.entity_ids) {
      if (endpoint == "$self") {endpoint = id;}
    }
    if (std::find(relation.entity_ids.begin(), relation.entity_ids.end(), id) == relation.entity_ids.end()) {
      throw std::invalid_argument("Relation must reference the entity being written");
    }
    if (relation.type == "inside" && relation.entity_ids.front() != id) {
      throw std::invalid_argument("Write inside from its subject entity");
    }
    relation.basis = item.value("basis", std::string("model_observed"));
    if (relation.basis == "geometry_computed") {
      throw std::invalid_argument("Use spatial_query to calculate distance");
    }
    relation.recognition_state = item.value("recognition_state", std::string("supported"));
    relation.evidence_observation_ids = item.at("evidence_observation_ids").get<std::vector<std::string>>();
    for (const auto & evidence : relation.evidence_observation_ids) {readObservation(dataset, evidence);}
    relations.push_back(std::move(relation));
  }
  return relations;
}

/** @brief 写入全部关系，任何失败交由外层事务回滚。 */
EntityStoreResult writeRelations(EntityStore & store, const std::vector<EntityRelation> & relations)
{
  bool changed = false;
  for (const auto & relation : relations) {
    const auto saved = store.recordRelation(relation);
    if (!saved.ok()) {return saved;}
    changed = changed || saved.code == Code::ready;
  }
  return {changed ? Code::ready : Code::already_linked, ""};
}

/** @brief 在更新结果中回读当前实体和实际保存的关系。 */
Json describeWrite(EntityStore & store, const std::string & id, const std::vector<EntityRelation> & relations)
{
  const auto entity = store.loadEntity(id);
  if (!entity.ok()) {return failure(entity);}
  auto related = Json::array();
  for (const auto & requested : relations) {
    const auto saved = store.findRelation(requested.type, requested.entity_ids);
    if (saved) {related.push_back(describeRelation(*saved));}
  }
  return {{"status", "ok"}, {"entity", describeEntity(*entity.entity)}, {"relations", std::move(related)}};
}

/** @brief 以一条已定位观测创建永久实体，再在同一事务内写入其余证据。 */
Json createEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root, const Json & args)
{
  const auto kind = args.at("kind").get<std::string>();
  if (kind != "object" && kind != "region") {throw std::invalid_argument("Expected object or region");}
  const auto label = args.at("semantic_label").get<std::string>();
  if (label.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::invalid_argument("semantic_label must not be blank");
  }
  auto details = Json{{"recognition_state", "supported"}};
  applyFields(details, args);
  auto views = buildViews(dataset, root, args.at("add_observations"),
    kind == "object" ? EntityKind::object : EntityKind::region, label);
  if (views.empty()) {throw std::invalid_argument("Create requires at least one observation");}
  const auto anchor = std::find_if(views.begin(), views.end(), [](const Entity & view) {
      return view.status == EntityStatus::located;
    });
  if (anchor == views.end()) {
    const auto id = views.front().entity_id;
    auto pending = args;
    pending["location_pending_reason"] = views.front().location_pending_reason.value_or("location_unavailable");
    const auto saved = store.savePending(id, pending.dump());
    if (!saved.ok()) {return failure(saved);}
    return {{"status", "pending_location"}, {"pending_id", id}, {"reason", pending.at("location_pending_reason")}};
  }
  const auto id = anchor->entity_id;
  anchor->details_json = details.dump();
  const auto relations = readRelations(dataset, args, id);
  const auto result = store.writeAtomically([&]() -> EntityStoreResult {
    const auto created = insertObservedEntity(store, dataset, *anchor);
    if (!created.ok()) {return created;}
    std::uint64_t current_revision = 1;
    for (const auto & view : views) {
      if (&view == &*anchor) {continue;}
      const auto saved = appendObservedEvidence(store, dataset, id, current_revision, view.observations.front());
      if (!saved.ok()) {return saved;}
      if (saved.code == Code::ready) {++current_revision;}
    }
    return writeRelations(store, relations);
  });
  return result.ok() ? describeWrite(store, id, relations) : failure(result);
}

/** @brief 在期望修订上追加证据并更新字段，已有定位优先保留。 */
Json updateEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args)
{
  const auto id = args.at("entity_id").get<std::string>();
  const auto expected = revision(args);
  const auto current = store.loadEntity(id);
  if (!current.ok()) {return failure(current);}
  if (current.entity->revision != expected) {
    return failure({Code::revision_conflict, "Entity revision has changed"});
  }

  const auto fields = args.value("set_fields", Json::object());
  auto details = Json::parse(current.entity->details_json);
  applyFields(details, fields);
  for (const auto & field : fields.items()) {
    const auto & key = field.key();
    if (key != "semantic_label" && key != "description" && key != "attributes" &&
      key != "recognition_state" && key != "notes")
    {
      throw std::invalid_argument("Unsupported set_fields key: " + key);
    }
  }
  const auto label = fields.value("semantic_label", current.entity->semantic_label);
  const auto views = buildViews(dataset, root, args.value("add_observations", Json::array()),
    current.entity->kind, label);
  const auto position = selectedEntityPoint(*current.entity);
  if (position) {
    for (const auto & view : views) {
      const auto candidate = selectedEntityPoint(view);
      if (candidate && std::hypot((*position)[0] - (*candidate)[0], (*position)[1] - (*candidate)[1],
        (*position)[2] - (*candidate)[2]) > near.threshold_m)
      {
        details["recognition_state"] = "needs_review";
      }
    }
  }
  const auto relations = readRelations(dataset, args, id);
  const auto result = store.writeAtomically([&]() -> EntityStoreResult {
    const auto saved = store.updateEntityFields(id, expected, label, details.dump());
    if (!saved.ok()) {return saved;}
    auto current_revision = expected;
    if (saved.code == Code::ready) {++current_revision;}
    bool located = position.has_value();
    for (const auto & view : views) {
      const auto & ref = view.observations.front();
      const auto linked = appendObservedEvidence(store, dataset, id, current_revision, ref);
      if (!linked.ok()) {return linked;}
      if (linked.code == Code::ready) {++current_revision;}
      if (!located && view.status == EntityStatus::located) {
        const auto resolved = resolveEntityLocation(store, dataset, id, current_revision,
          {ref.observation_id, ref.observation_revision});
        if (!resolved.ok()) {return resolved;}
        ++current_revision;
        located = true;
      }
    }
    const auto related = writeRelations(store, relations);
    if (related.code == Code::ready && current_revision == expected) {
      return store.updateEntityFields(id, expected, label, details.dump(), true);
    }
    return related;
  });
  return result.ok() ? describeWrite(store, id, relations) : failure(result);
}

/** @brief 按名称、描述和可选空间过滤，返回位置和观测引用。 */
Json searchEntities(EntityStore & store, const Json & args)
{
  const auto query = args.value("query", std::string{});
  auto results = Json::array();
  for (const auto & id : store.listEntityIds()) {
    const auto loaded = store.loadEntity(id);
    if (!loaded.ok()) {return failure(loaded);}
    const auto & entity = *loaded.entity;
    if (args.contains("region_id") && entity.region_id != args.at("region_id").get<std::string>()) {continue;}
    auto record = describeEntity(entity);
    const auto description = record.value("description", std::string{});
    const auto name = record.value("display_name", std::string{});
    if (entity.semantic_label.find(query) != std::string::npos || description.find(query) != std::string::npos ||
      name.find(query) != std::string::npos)
    {
      results.push_back(std::move(record));
    }
  }
  return {{"status", "ok"}, {"entities", std::move(results)}};
}

/** @brief 复用有效关系；数值 near 可以从同一地图中的代表点即时计算。 */
Json spatialQuery(EntityStore & store, const NearOptions & defaults, const Json & args)
{
  const auto type = args.at("operation").get<std::string>();
  const auto ids = args.at("entity_ids").get<std::vector<std::string>>();
  const auto saved = store.findRelation(type, ids);
  std::vector<Entity> endpoints;
  for (const auto & id : ids) {
    const auto loaded = store.loadEntity(id);
    if (!loaded.ok()) {return failure(loaded);}
    endpoints.push_back(*loaded.entity);
  }
  const auto params = args.value("parameters", Json::object());
  const auto dimension = params.value("dimension", defaults.dimension);
  const auto threshold = params.value("threshold_m", defaults.threshold_m);
  if (type == "near" && ((dimension != "xy" && dimension != "xyz") ||
    !std::isfinite(threshold) || threshold <= 0 ||
    params.value("geometry", std::string("representative_point")) != "representative_point"))
  {
    throw std::invalid_argument("Near requires representative_point, dimension xy or xyz and positive threshold_m");
  }
  if (saved && saved->recognition_state == "supported" && saved->geometry_revision == store.context().geometry_revision &&
    (type != "near" || (saved->basis == "geometry_computed" && saved->dimension == dimension &&
    saved->threshold_m == threshold) || (saved->basis != "geometry_computed" && params.empty())))
  {
    auto relation = describeRelation(*saved);
    Json output{{"status", "ok"}, {"related", true}, {"reused", true}, {"entity_ids", ids},
      {"basis", saved->basis}, {"relation", relation}};
    if (saved->distance_m) {
      output["distance_m"] = *saved->distance_m;
      output["parameters"] = relation.at("parameters");
    }
    return output;
  }
  if (type != "near") {
    return {{"status", "ok"}, {"related", nullptr}, {"reused", false}, {"basis", "unconfirmed"}};
  }
  const auto a = selectedEntityPoint(endpoints[0]);
  const auto b = selectedEntityPoint(endpoints[1]);
  if (!a || !b) {return failure({Code::location_unavailable, "Both entities need map positions"});}
  const auto dz = dimension == "xyz" ? (*a)[2] - (*b)[2] : 0.0;
  const auto distance = std::hypot((*a)[0] - (*b)[0], (*a)[1] - (*b)[1], dz);
  if (!std::isfinite(distance)) {return failure({Code::invalid_entity, "Non-finite map distance"});}
  const bool related = distance <= threshold;
  Json output{{"status", "ok"}, {"related", related}, {"reused", false},
    {"entity_ids", ids}, {"basis", "geometry_computed"}, {"distance_m", distance},
    {"parameters", {{"dimension", dimension}, {"threshold_m", threshold}, {"geometry", "representative_point"}}}};
  if (related && (!saved || saved->basis == "geometry_computed")) {
    EntityRelation measured;
    measured.type = type;
    measured.entity_ids = ids;
    measured.basis = "geometry_computed";
    measured.distance_m = distance;
    measured.threshold_m = threshold;
    measured.dimension = dimension;
    const auto recorded = store.recordRelation(std::move(measured));
    if (!recorded.ok()) {return failure(recorded);}
    if (const auto current = store.findRelation(type, ids)) {
      output["relation"] = describeRelation(*current);
    }
  }
  return output;
}
}  // namespace

Json runEntityTool(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, std::string_view name, const Json & arguments)
{
  if (name == "search_entities") {return searchEntities(store, arguments);}
  if (name == "create_entity") {return createEntity(store, dataset, root, arguments);}
  if (name == "update_entity") {return updateEntity(store, dataset, root, near, arguments);}
  if (name == "spatial_query") {return spatialQuery(store, near, arguments);}
  return {{"status", "unknown_tool"}, {"message", "Unknown entity tool"}};
}
}  // namespace vln_memory
