#include "entity_tools.hpp"

#include "observation_tools.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <map>
#include <set>
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
std::uint64_t revision(const Json & args, const char * key = "expected_revision")
{
  const auto & value = args.at(key);
  if (!value.is_number_integer() || value <= 0) {
    throw std::invalid_argument(std::string(key) + " must be a positive integer");
  }
  return value.get<std::uint64_t>();
}

/** @brief 拒绝从其他操作误传的字段，避免部分执行或误撤回。 */
void checkKeys(const Json & args, std::initializer_list<std::string_view> fields,
  std::initializer_list<std::string_view> shared = {})
{
  if (!args.is_object()) {throw std::invalid_argument("Expected an object");}
  for (const auto & field : args.items()) {
    const std::string_view key = field.key();
    if (std::find(fields.begin(), fields.end(), key) == fields.end() &&
      std::find(shared.begin(), shared.end(), key) == shared.end()) {
      throw std::invalid_argument("Unexpected field: " + field.key());
    }
  }
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

}  // namespace

/** @brief 将实测代表点和观测证据整理为工具与发布共用记录。 */
Json describeEntity(const Entity & entity)
{
  auto out = Json::parse(entity.details_json);
  out["id"] = entity.entity_id;
  out["entity_id"] = entity.entity_id;
  out["kind"] = "region";
  if (entity.kind == EntityKind::object) {
    out["kind"] = "object";
  }
  out["semantic_label"] = entity.semantic_label;
  out["revision"] = entity.revision;
  out["region_id"] = nullptr;
  if (entity.region_id) {
    out["region_id"] = *entity.region_id;
  }
  const auto position = selectedEntityPoint(entity);
  out["position_map"] = nullptr;
  if (position) {
    out["position_map"] = *position;
  }
  if (position) {
    if (entity.kind == EntityKind::object) {
      out["geometry_quality"] = "visible_surface_estimate";
    } else {
      out["geometry_quality"] = "confirmed_interior";
    }
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

/** @brief 将拥有型人工标注整理成查询和发布共用的 JSON 记录。 */
Json describeAnnotation(const EntityAnnotation & annotation)
{
  Json result{{"entity_id", annotation.entity_id}, {"annotation_revision", annotation.annotation_revision},
    {"display_name", nullptr}, {"aliases", annotation.aliases}, {"note", nullptr}};
  if (annotation.display_name) {
    result["display_name"] = *annotation.display_name;
  }
  if (annotation.note) {
    result["note"] = *annotation.note;
  }
  return result;
}

namespace
{
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
  if (changed) {
    return {Code::ready, ""};
  }
  return {Code::already_linked, ""};
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

/** @brief 用已有距离阈值排除明显不一致的地图点。 */
bool locationConflict(const Entity & anchor, const std::vector<Entity> & views, double threshold)
{
  const auto point = selectedEntityPoint(anchor);
  if (!point) {return false;}
  for (const auto & view : views) {
    const auto candidate = selectedEntityPoint(view);
    if (candidate && std::hypot((*point)[0] - (*candidate)[0], (*point)[1] - (*candidate)[1],
      (*point)[2] - (*candidate)[2]) > threshold)
    {
      return true;
    }
  }
  return false;
}

/** @brief 以一条已定位观测创建实体，补定位时沿用待定位记录的 ID。 */
Json createEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args, std::string_view pending_id = {})
{
  const auto kind = args.at("kind").get<std::string>();
  if (kind != "object" && kind != "region") {throw std::invalid_argument("Expected object or region");}
  const auto label = args.at("semantic_label").get<std::string>();
  if (label.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::invalid_argument("semantic_label must not be blank");
  }
  auto details = Json{{"recognition_state", "supported"}};
  applyFields(details, args);
  auto entity_kind = EntityKind::region;
  if (kind == "object") {
    entity_kind = EntityKind::object;
  }
  auto views = buildViews(dataset, root, args.at("add_observations"), entity_kind, label);
  if (views.empty()) {throw std::invalid_argument("Create requires at least one observation");}
  const auto anchor = std::find_if(views.begin(), views.end(), [](const Entity & view) {
      return view.status == EntityStatus::located;
    });
  if (anchor == views.end()) {
    std::string id(pending_id);
    if (pending_id.empty()) {
      id = views.front().entity_id;
    }
    auto pending = args;
    pending["location_pending_reason"] = views.front().location_pending_reason.value_or("location_unavailable");
    const auto saved = store.savePending(id, pending.dump(), !pending_id.empty());
    if (!saved.ok()) {return failure(saved);}
    return {{"status", "pending_location"}, {"pending_id", id}, {"reason", pending.at("location_pending_reason")}};
  }
  if (locationConflict(*anchor, views, near.threshold_m)) {
    return failure({Code::evidence_conflict, "Observation positions conflict for this instance"});
  }
  if (!pending_id.empty()) {anchor->entity_id = pending_id;}
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
    const auto related = writeRelations(store, relations);
    if (!related.ok() || pending_id.empty()) {return related;}
    return store.removePending(pending_id);
  });
  if (result.ok()) {
    return describeWrite(store, id, relations);
  }
  return failure(result);
}

/** @brief 在待定位记录上优先使用本次提交的证据，保留其他帧的引用。 */
Json locatePendingEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args, Json pending)
{
  const auto & views = args.at("add_observations");
  if (!views.is_array() || views.empty()) {
    throw std::invalid_argument("Pending entity needs new observation evidence");
  }
  auto combined = views;
  for (const auto & view : pending.at("add_observations")) {
    const auto same = std::any_of(views.begin(), views.end(), [&](const auto & current) {
      return current.at("observation_id") == view.at("observation_id");
    });
    if (!same) {combined.push_back(view);}
  }
  const auto fields = args.value("set_fields", Json::object());
  applyFields(pending, fields);
  if (fields.contains("semantic_label")) {pending["semantic_label"] = fields.at("semantic_label");}
  pending["add_observations"] = std::move(combined);
  pending["relations"] = args.value("relations", Json::array());
  return createEntity(store, dataset, root, near, pending, args.at("entity_id").get<std::string>());
}

/** @brief 在期望修订上追加证据并更新字段，已有定位优先保留。 */
Json updateEntity(
  EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args)
{
  const auto id = args.at("entity_id").get<std::string>();
  const auto expected = revision(args);
  const auto current = store.loadEntity(id);
  if (current.code == Code::not_found) {
    if (const auto pending = store.findPending(id)) {
      if (expected != 1) {return failure({Code::revision_conflict, "Pending entity revision is 1"});}
      return locatePendingEntity(store, dataset, root, near, args, Json::parse(*pending));
    }
  }
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
  if (position && current.entity->kind == EntityKind::object) {
    for (const auto & view : views) {
      if (view.status != EntityStatus::located) {
        return failure({Code::location_unavailable, "New object evidence needs a located surface"});
      }
    }
  }
  if (locationConflict(*current.entity, views, near.threshold_m)) {
    return failure({Code::evidence_conflict, "Observation positions conflict for this instance"});
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
  if (result.ok()) {
    return describeWrite(store, id, relations);
  }
  return failure(result);
}

/** @brief 保存实体和选中的定位证据供纠错历史回读。 */
Json correctionSnapshot(const EntityStore & store, const Entity & entity)
{
  auto out = describeEntity(entity);
  out["status"] = "pending_location";
  if (entity.status == EntityStatus::located) {
    out["status"] = "located";
  }
  if (entity.location_pending_reason) {out["location_pending_reason"] = *entity.location_pending_reason;}
  if (entity.location_observation) {
    const auto & ref = *entity.location_observation;
    out["location_observation"] = {{"observation_id", ref.observation_id},
      {"observation_revision", ref.observation_revision}};
  }
  if (const auto annotation = store.findEntityAnnotation(entity.entity_id)) {
    out["annotation"] = describeAnnotation(*annotation);
  }
  return out;
}

/** @brief 汇集即将修改或退出活动集合的实体对应的原关系。 */
Json correctionRelations(const EntityStore & store, const std::vector<std::string> & affected)
{
  auto out = Json::array();
  for (const auto & relation : store.listRelations()) {
    for (const auto & end : relation.entity_ids) {
      if (std::find(affected.begin(), affected.end(), end) == affected.end()) {continue;}
      out.push_back(describeRelation(relation));
      break;
    }
  }
  return out;
}

/** @brief 合并到目标 ID。不同选区占用同一观测键时，由证据写入器返回冲突。 */
EntityStoreResult mergeEntities(EntityStore & store, const ObservationDatasetReader & dataset,
  const Entity & target, const Entity & source)
{
  if (target.kind != source.kind || target.entity_id == source.entity_id) {
    return {Code::invalid_entity, "Merge requires two distinct entities of the same kind"};
  }
  auto current = target.revision;
  for (const auto & ref : source.observations) {
    const auto linked = appendObservedEvidence(store, dataset, target.entity_id, current, ref);
    if (!linked.ok()) {return linked;}
    if (linked.code == Code::ready) {++current;}
  }
  if (!target.location_observation && source.location_observation) {
    const auto located = resolveEntityLocation(store, dataset, target.entity_id, current, *source.location_observation);
    if (!located.ok()) {return located;}
    ++current;
  }
  if (current == target.revision) {
    const auto bumped = store.updateEntityFields(
      target.entity_id, current, target.semantic_label, target.details_json, true);
    if (!bumped.ok()) {return bumped;}
  }
  return store.retireEntity(source.entity_id, source.revision);
}

/** @brief 判断观测本身是否保存地图代表点，供拆分选择首条证据。 */
bool locatedEvidence(const EntityObservationRef & ref)
{
  return (ref.object_selection && ref.object_selection->representative) ||
    (ref.region_interior && ref.region_interior->body_anchor);
}

/** @brief 复用首条观测建实体，再追加该组的其余证据。 */
EntityStoreResult createSplitPart(EntityStore & store, const ObservationDatasetReader & dataset,
  const Entity & source, const std::string & label, const std::vector<const EntityObservationRef *> & refs,
  std::vector<std::string> & children)
{
  auto first = std::find_if(refs.begin(), refs.end(), [](const auto * ref) {return locatedEvidence(*ref);});
  if (first == refs.end()) {first = refs.begin();}
  const auto id = allocateEntityId();
  if (!id) {return {Code::storage_error, "Cannot allocate entity ID"};}
  Entity child{*id, source.kind, label, EntityStatus::pending_location, 1,
    source.map_id, source.map_frame, source.geometry_revision, {**first}, "split_evidence_unlocated"};
  if (locatedEvidence(**first)) {
    child.status = EntityStatus::located;
    child.location_pending_reason.reset();
    child.location_observation = EntityLocationRef{(*first)->observation_id, (*first)->observation_revision};
  }
  const auto inserted = insertObservedEntity(store, dataset, child);
  if (!inserted.ok()) {return inserted;}
  auto revision = child.revision;
  for (const auto * ref : refs) {
    if (ref == *first) {continue;}
    const auto linked = appendObservedEvidence(store, dataset, *id, revision, *ref);
    if (!linked.ok()) {return linked;}
    if (linked.code == Code::ready) {++revision;}
  }
  children.push_back(*id);
  return {Code::ready, ""};
}

/** @brief 按请求的无交集观测分组创建新 ID，所有原证据必须恰好出现一次。 */
EntityStoreResult splitEntity(EntityStore & store, const ObservationDatasetReader & dataset,
  const Entity & source, const Json & parts, std::vector<std::string> & children)
{
  if (!parts.is_array() || parts.size() < 2) {throw std::invalid_argument("Split requires at least two parts");}
  std::map<std::string, const EntityObservationRef *> remaining;
  for (const auto & ref : source.observations) {remaining.emplace(ref.observation_id, &ref);}
  for (const auto & part : parts) {
    checkKeys(part, {"semantic_label", "observation_ids"});
    const auto label = part.at("semantic_label").get<std::string>();
    const auto ids = part.at("observation_ids").get<std::vector<std::string>>();
    if (ids.empty()) {throw std::invalid_argument("Each split part needs observation IDs");}
    std::vector<const EntityObservationRef *> refs;
    for (const auto & id : ids) {
      const auto match = remaining.find(id);
      if (match == remaining.end()) {
        return {Code::invalid_reference, "Split observations must belong to the source and appear once"};
      }
      refs.push_back(match->second);
      remaining.erase(match);
    }
    const auto created = createSplitPart(store, dataset, source, label, refs, children);
    if (!created.ok()) {return created;}
  }
  if (!remaining.empty()) {return {Code::invalid_reference, "Split must assign every source observation"};}
  return store.retireEntity(source.entity_id, source.revision);
}

/** @brief 沿用既有实体写入，并允许人工从已保存证据重新选择代表点。 */
Json correctFields(EntityStore & store, const ObservationDatasetReader & dataset,
  const fs::path & root, const NearOptions & near, const Json & args, const Entity & before)
{
  if (args.contains("set_fields") || args.contains("add_observations") || args.contains("relations")) {
    const auto reply = updateEntity(store, dataset, root, near, args);
    if (reply.at("status") != "ok") {return reply;}
  }
  if (args.contains("location_observation_id")) {
    const auto current = store.loadEntity(before.entity_id);
    if (!current.ok()) {return failure(current);}
    const auto obs = readObservation(dataset, args.at("location_observation_id").get<std::string>());
    const auto chosen = store.reselectEntityLocation(before.entity_id, current.entity->revision, {obs.id, obs.revision},
      store.context(), obs);
    if (!chosen.ok()) {return failure(chosen);}
  }
  const auto after = store.loadEntity(before.entity_id);
  if (!after.ok()) {return failure(after);}
  if (after.entity->revision == before.revision) {
    return {{"status", "invalid_request"}, {"message", "Correction made no changes"}};
  }
  return {{"status", "ok"}};
}

/** @brief 为每种纠错操作限制相应字段。 */
void checkCorrectionKeys(const Json & args, std::string_view operation)
{
  const std::initializer_list<std::string_view> common{"operation", "entity_id", "expected_revision", "reason"};
  if (operation == "update") {
    checkKeys(args, {"set_fields", "add_observations", "relations", "location_observation_id"}, common);
  } else if (operation == "merge") {
    checkKeys(args, {"source_entity_id", "source_expected_revision"}, common);
  } else if (operation == "split") {
    checkKeys(args, {"parts"}, common);
  } else if (operation == "retract") {
    checkKeys(args, {}, common);
  } else {
    throw std::invalid_argument("Expected update, retract, merge or split");
  }
}

/** @brief 在当前事务内执行一次完整纠错并记录改动前后的证据关联。 */
Json applyCorrection(EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args)
{
  const auto op = args.at("operation").get<std::string>();
  checkCorrectionKeys(args, op);
  const auto id = args.at("entity_id").get<std::string>();
  const auto reason = args.at("reason").get<std::string>();
  if (reason.size() > 4096 || reason.find_first_not_of(" \t\r\n") == std::string::npos) {
    throw std::invalid_argument("A correction reason of at most 4096 bytes is required");
  }
  const auto loaded = store.loadEntity(id);
  if (!loaded.ok()) {return failure(loaded);}
  const auto & current = *loaded.entity;
  if (current.revision != revision(args)) {return failure({Code::revision_conflict, "Entity revision has changed"});}

  auto before = Json::array({correctionSnapshot(store, current)});
  std::vector<std::string> affected{id};
  std::optional<Entity> source;
  if (op == "merge") {
    const auto other = store.loadEntity(args.at("source_entity_id").get<std::string>());
    if (!other.ok()) {return failure(other);}
    if (other.entity->revision != revision(args, "source_expected_revision")) {
      return failure({Code::revision_conflict, "Source entity revision has changed"});
    }
    source = std::move(*other.entity);
    affected.push_back(source->entity_id);
    before.push_back(correctionSnapshot(store, *source));
  }
  const auto relations = correctionRelations(store, affected);

  EntityStoreResult changed{Code::ready, ""};
  std::vector<std::string> remaining;
  std::vector<std::string> retired;
  if (op == "update") {
    const auto updated = correctFields(store, dataset, root, near, args, current);
    if (updated.at("status") != "ok") {return updated;}
    remaining.push_back(id);
  } else if (op == "merge") {
    changed = mergeEntities(store, dataset, current, *source);
    remaining.push_back(id);
    retired.push_back(source->entity_id);
  } else if (op == "split") {
    changed = splitEntity(store, dataset, current, args.at("parts"), remaining);
    retired.push_back(id);
  } else {
    // 其余操作已经由 checkCorrectionKeys 限定为 retract。
    changed = store.retireEntity(id, current.revision);
    retired.push_back(id);
  }
  if (!changed.ok()) {return failure(changed);}

  auto after = Json::array();
  for (const auto & active : remaining) {
    const auto stored = store.loadEntity(active);
    if (!stored.ok()) {return failure(stored);}
    after.push_back(correctionSnapshot(store, *stored.entity));
  }
  const auto correction_id = store.insertCorrection(Json{{"operation", op}, {"reason", reason},
    {"before", before}, {"after", after}, {"relations_before", relations}, {"retired_entity_ids", retired}}.dump());
  return {{"status", "ok"}, {"operation", op}, {"correction_id", correction_id},
    {"entities", std::move(after)}, {"retired_entity_ids", retired}};
}

/** @brief 将修改、撤回和历史封在同一个 SQLite 事务中。 */
Json correctEntity(EntityStore & store, const ObservationDatasetReader & dataset, const fs::path & root,
  const NearOptions & near, const Json & args)
{
  Json reply;
  const auto saved = store.writeAtomically([&]() -> EntityStoreResult {
    try {
      reply = applyCorrection(store, dataset, root, near, args);
    } catch (const Json::exception & error) {
      reply = {{"status", "invalid_request"}, {"message", error.what()}};
    } catch (const std::invalid_argument & error) {
      reply = {{"status", "invalid_request"}, {"message", error.what()}};
    }
    if (reply.at("status") == "ok") {return {Code::ready, ""};}
    return {Code::invalid_entity, "Correction was not applied"};
  });
  if (!saved.ok() && (reply.is_null() || reply.at("status") == "ok")) {return failure(saved);}
  return reply;
}

/** @brief 解析可部分更新的标注字段，复用同一实体 ID 上的独立修订。 */
Json annotateEntity(EntityStore & store, const Json & args)
{
  const auto id = args.at("entity_id").get<std::string>();
  const auto & revision = args.at("expected_annotation_revision");
  if (!revision.is_number_integer() || (!revision.is_number_unsigned() && revision.get<std::int64_t>() < 0)) {
    throw std::invalid_argument("expected_annotation_revision must be a nonnegative integer");
  }
  checkKeys(args, {"entity_id", "expected_annotation_revision", "display_name", "aliases", "note"});
  if (!args.contains("display_name") && !args.contains("aliases") && !args.contains("note")) {
    throw std::invalid_argument("Provide display_name, aliases or note");
  }
  auto annotation = store.findEntityAnnotation(id).value_or(EntityAnnotation{});
  annotation.entity_id = id;
  if (annotation.annotation_revision != revision.get<std::uint64_t>()) {
    if (annotation.annotation_revision == 0) {
      const auto entity = store.loadEntity(id);
      if (!entity.ok()) {return failure(entity);}
    }
    return failure({Code::revision_conflict, "Annotation revision has changed"});
  }
  for (const auto * key : {"display_name", "note"}) {
    if (!args.contains(key)) {continue;}
    auto * field = &annotation.note;
    if (std::string_view(key) == "display_name") {
      field = &annotation.display_name;
    }
    auto & target = *field;
    const auto & value = args.at(key);
    if (value.is_null()) {target.reset();}
    else {target = value.get<std::string>();}
  }
  if (args.contains("aliases")) {annotation.aliases = args.at("aliases").get<std::vector<std::string>>();}
  const auto saved = store.writeEntityAnnotation(annotation);
  if (!saved.ok()) {return failure(saved);}
  ++annotation.annotation_revision;
  return {{"status", "ok"}, {"annotation", describeAnnotation(annotation)}};
}

/** @brief 按人工名称、别名及原语义过滤，返回位置和观测引用。 */
Json searchEntities(EntityStore & store, const Json & args)
{
  const auto query = args.value("query", std::string{});
  std::map<std::string, EntityAnnotation> annotations;
  auto saved = store.listEntityAnnotations();
  for (auto & annotation : saved) {
    const auto id = annotation.entity_id;
    annotations.emplace(id, std::move(annotation));
  }
  auto results = Json::array();
  for (const auto & id : store.listEntityIds()) {
    const auto loaded = store.loadEntity(id);
    if (!loaded.ok()) {return failure(loaded);}
    const auto & entity = *loaded.entity;
    if (args.contains("region_id") && entity.region_id != args.at("region_id").get<std::string>()) {continue;}
    auto record = describeEntity(entity);
    const auto description = record.value("description", std::string{});
    const auto name = record.value("display_name", std::string{});
    const auto annotation = annotations.find(id);
    bool annotation_match = false;
    if (annotation != annotations.end()) {
      const auto & human = annotation->second;
      /** @brief 沿用实体搜索的子串匹配。 */
      const auto matches = [&](const std::string & value) {return value.find(query) != std::string::npos;};
      annotation_match = (human.display_name && matches(*human.display_name)) ||
        std::any_of(human.aliases.begin(), human.aliases.end(), matches);
      record["annotation"] = describeAnnotation(human);
    }
    if (annotation_match || entity.semantic_label.find(query) != std::string::npos ||
      description.find(query) != std::string::npos || name.find(query) != std::string::npos) {
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
  double dz = 0.0;
  if (dimension == "xyz") {
    dz = (*a)[2] - (*b)[2];
  }
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
  if (name == "create_entity") {return createEntity(store, dataset, root, near, arguments);}
  if (name == "update_entity") {return updateEntity(store, dataset, root, near, arguments);}
  if (name == "spatial_query") {return spatialQuery(store, near, arguments);}
  if (name == "memory_annotate") {return annotateEntity(store, arguments);}
  if (name == "correct_memory") {return correctEntity(store, dataset, root, near, arguments);}
  return {{"status", "unknown_tool"}, {"message", "Unknown entity tool"}};
}
}  // namespace vln_memory
