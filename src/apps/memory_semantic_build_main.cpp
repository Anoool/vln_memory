#include <vln_memory/engine/entity_builder.hpp>
#include <vln_memory/engine/observed_entity_store.hpp>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <openssl/sha.h>

namespace
{
using namespace vln_memory;
using Json = nlohmann::json;
namespace fs = std::filesystem;
using Code = EntityStoreCode;

/** @brief 读取指定的字符串字段，拒绝缺失或空值。 */
std::string requiredText(const Json & row, const char * key)
{
  auto value = row.at(key).get<std::string>();
  if (value.empty()) {throw std::invalid_argument(std::string("Empty field: ") + key);}
  return value;
}

/** @brief 严格解析整数，避免 JSON 浮点或超范围整数隐式转换。 */
std::int64_t integer(const Json & value)
{
  if (!value.is_number_integer() ||
    (value.is_number_unsigned() && value.get<std::uint64_t>() > std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("Expected a signed 64-bit integer");
  }
  return value.get<std::int64_t>();
}

/** @brief 恢复采样时间的整数 sec/nanosec 分量。 */
SensorStamp stamp(const Json & value)
{
  const auto sec = integer(value.at("sec"));
  const auto nanos = integer(value.at("nanosec"));
  if (sec < std::numeric_limits<std::int32_t>::min() || sec > std::numeric_limits<std::int32_t>::max() ||
    nanos < 0 || nanos >= 1000000000) {throw std::invalid_argument("Raw timestamp is out of range");}
  return {static_cast<std::int32_t>(sec), static_cast<std::uint32_t>(nanos)};
}

/** @brief 校验实体乐观锁所用的正整数。 */
std::uint64_t expectedRevision(const Json & request)
{
  const auto revision = integer(request.at("expected_revision"));
  if (revision < 1) {throw std::invalid_argument("Expected a positive entity revision");}
  return static_cast<std::uint64_t>(revision);
}

/** @brief 用宿主分配的实体键在同一观测集中稳定生成 ID，重试不会重复创建。 */
std::string stableEntityId(const std::string & dataset_digest, const std::string & key)
{
  if (key.size() > 256) {throw std::invalid_argument("entity_key exceeds 256 bytes");}
  const auto input = dataset_digest + ":" + key;
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(), digest);
  constexpr char hex[] = "0123456789abcdef";
  std::string id = "ent_";
  for (std::size_t i = 0; i < 16; ++i) {
    id += hex[digest[i] >> 4];
    id += hex[digest[i] & 0xf];
  }
  return id;
}

/** @brief 将实体工作库结果映射为调用方可重试或处理的状态。 */
const char * codeName(Code code)
{
  switch (code) {
    case Code::ready: return "ok";
    case Code::already_exists: return "already_exists";
    case Code::not_found: return "not_found";
    case Code::invalid_store: return "invalid_store";
    case Code::invalid_entity: return "invalid_entity";
    case Code::invalid_reference: return "invalid_reference";
    case Code::dataset_mismatch: return "dataset_mismatch";
    case Code::observation_read_failed: return "observation_read_failed";
    case Code::already_linked: return "already_linked";
    case Code::revision_conflict: return "revision_conflict";
    case Code::evidence_conflict: return "evidence_conflict";
    case Code::location_unavailable: return "location_unavailable";
    case Code::already_located: return "already_located";
    case Code::storage_error: return "storage_error";
  }
  return "storage_error";
}

/** @brief 构造稳定的 JSON 响应形状。 */
Json result(const EntityStoreResult & status)
{
  return {{"ok", status.ok()}, {"code", codeName(status.code)}, {"message", status.message}};
}

/** @brief 只接受显式声明为深度原图的半开像素框。 */
std::optional<DepthRegion> depthSelection(const Json & request)
{
  if (request.contains("rgb_bbox")) {throw std::invalid_argument("RGB boxes cannot be used as depth selections");}
  if (!request.contains("depth_bbox")) {return std::nullopt;}
  const auto & box = request.at("depth_bbox");
  if (!box.is_array() || box.size() != 4) {throw std::invalid_argument("depth_bbox requires four integers");}
  return DepthBoundingBox{integer(box.at(0)), integer(box.at(1)), integer(box.at(2)), integer(box.at(3))};
}

/** @brief 复用已核验的观测、几何创建器和事务库处理结构化指令。 */
class SemanticBuild
{
public:
  /** @brief 数据集在进程启动时索引一次，库和 preparation 按需打开。 */
  SemanticBuild(const ObservationDatasetReader & dataset, fs::path database, fs::path source)
  : dataset_(dataset), database_(std::move(database)), source_(std::move(source)) {}

  /** @brief 每个 JSONL 请求独立执行并返回当前状态。 */
  Json run(const Json & request)
  {
    const auto op = requiredText(request, "op");
    if (op == "get") {return entity(requiredText(request, "entity_id"));}
    if (op == "near") {
      const auto near = store().recordNear(requiredText(request, "subject_id"), requiredText(request, "target_id"),
        request.at("threshold_m").get<double>());
      auto output = result(near);
      if (near.ok()) {
        output["related"] = near.related;
        output["distance_m"] = near.distance_m;
        output["geometry_revision"] = store().context().geometry_revision;
      }
      return output;
    }
    if (op != "lookup" && op != "create" && op != "append" && op != "resolve_location" && op != "inside") {
      throw std::invalid_argument("Unknown semantic operation");
    }
    const auto loaded = observation(request);
    if (!loaded.ok()) {
      using ReadCode = ObservationDatasetReadCode;
      const char * code = "observation_read_failed";
      if (loaded.code == ReadCode::observation_not_found) {code = "observation_not_found";}
      if (loaded.code == ReadCode::ambiguous_observation) {code = "ambiguous_observation";}
      return {{"ok", false}, {"code", code}, {"message", loaded.message}};
    }
    const auto & obs = *loaded.observation;
    const EntityLocationRef ref{obs.id, obs.revision};
    if (op == "lookup") {
      return {{"ok", true}, {"observation_id", obs.id}, {"observation_revision", obs.revision},
        {"pending_reasons", obs.pending_reasons}};
    }
    if (op == "inside") {
      return result(store().recordInside(requiredText(request, "subject_id"), requiredText(request, "target_id"),
        ref, requiredText(request, "evidence_ref")));
    }
    if (op == "resolve_location") {
      const auto id = requiredText(request, "entity_id");
      const auto resolved = resolveEntityLocation(store(), dataset_, id, expectedRevision(request), ref);
      return resolved.ok() ? entity(id) : result(resolved);
    }
    return op == "create" ? create(request, obs) : append(request, obs);
  }

private:
  /** @brief 绑定发布源，再按 ID 或两路原始时间查找保存的配对。 */
  ObservationDatasetLoadResult observation(const Json & request)
  {
    if (request.contains("observation_id")) {
      if (request.contains("rgb_stamp") || request.contains("depth_stamp")) {
        throw std::invalid_argument("Provide observation_id or raw stamps");
      }
      return dataset_.loadObservation(requiredText(request, "observation_id"));
    }
    if (requiredText(request, "source_manifest_sha256") != dataset_.info().source_manifest_sha256) {
      throw std::invalid_argument("Replay source does not match the bound preparation");
    }
    return dataset_.loadObservationByStamps(stamp(request.at("rgb_stamp")), stamp(request.at("depth_stamp")));
  }

  /** @brief 打开绑定的 SQLite 库，首次写入可创建。 */
  EntityStore & store()
  {
    if (!store_) {
      if (database_.empty()) {throw std::invalid_argument("--store is required for entity operations");}
      auto opened = fs::exists(database_) ? openEntityStore(database_) : createObservedEntityStore(database_, dataset_);
      if (!opened.ok()) {throw std::runtime_error(opened.message);}
      if (opened.store->context().dataset_manifest_sha256 != dataset_.info().dataset_manifest_sha256) {
        throw std::invalid_argument("Work store belongs to a different observation dataset");
      }
      store_ = std::move(opened.store);
    }
    return *store_;
  }

  /** @brief 为已经绑定来源的观测生成实体证据。 */
  EntityBuildResult build(const Json & request, const Observation & obs, EntityKind kind, const std::string & label)
  {
    const auto box = depthSelection(request);
    if (!preparation_) {
      auto opened = dataset_.openSourcePreparation(source_);
      if (!opened.ok()) {throw std::runtime_error(opened.message);}
      preparation_ = std::move(opened.reader);
    }
    if (kind == EntityKind::object) {
      if (request.contains("confirmation_ref")) {throw std::invalid_argument("Interior confirmation requires a region");}
      return createObjectEntity(*preparation_, obs, label, box);
    }
    if (box) {throw std::invalid_argument("Depth selections require an object");}
    std::optional<RegionInteriorConfirmation> confirmation;
    if (request.contains("confirmation_ref")) {
      confirmation = RegionInteriorConfirmation{label, obs.id, obs.revision, obs.source_manifest_sha256,
        obs.map_id, obs.map_frame, obs.geometry_revision, requiredText(request, "confirmation_ref")};
    }
    return createRegionEntity(*preparation_, obs, label, confirmation);
  }

  /** @brief 创建时保存宿主键对应的永久实体 ID，重复请求返回该 ID。 */
  Json create(const Json & request, const Observation & obs)
  {
    const auto kind = requiredText(request, "kind");
    if (kind != "object" && kind != "region") {throw std::invalid_argument("Expected object or region");}
    const auto id = stableEntityId(dataset_.info().dataset_manifest_sha256, requiredText(request, "entity_key"));
    auto built = build(request, obs, kind == "object" ? EntityKind::object : EntityKind::region,
      requiredText(request, "label"));
    if (!built.ok()) {return {{"ok", false}, {"code", "entity_build_failed"}, {"message", built.message}};}
    built.entity->entity_id = id;
    const auto inserted = insertObservedEntity(store(), dataset_, *built.entity);
    if (inserted.ok()) {return entity(id);}
    auto output = result(inserted);
    if (inserted.code == Code::already_exists) {output["entity_id"] = id;}
    return output;
  }

  /** @brief 用调用方选定的已知实体追加一帧证据。 */
  Json append(const Json & request, const Observation & obs)
  {
    const auto id = requiredText(request, "entity_id");
    const auto revision = expectedRevision(request);
    const auto current = store().loadEntity(id);
    if (!current.ok()) {return result(current);}
    auto built = build(request, obs, current.entity->kind, current.entity->semantic_label);
    if (!built.ok()) {return {{"ok", false}, {"code", "entity_build_failed"}, {"message", built.message}};}
    const auto appended = appendObservedEvidence(store(), dataset_, id, revision, built.entity->observations.front());
    return appended.ok() ? entity(id) : result(appended);
  }

  /** @brief 回读实体身份、观测引用、当前定位和归属。 */
  Json entity(const std::string & id)
  {
    const auto loaded = store().loadEntity(id);
    if (!loaded.ok()) {return result(loaded);}
    const auto & item = *loaded.entity;
    auto observations = Json::array();
    for (const auto & ref : item.observations) {
      observations.push_back({{"observation_id", ref.observation_id}, {"observation_revision", ref.observation_revision}});
    }
    Json location = nullptr;
    if (item.location_observation) {
      location = {{"observation_id", item.location_observation->observation_id},
        {"observation_revision", item.location_observation->observation_revision}};
    }
    return {{"ok", true}, {"entity_id", item.entity_id}, {"label", item.semantic_label},
      {"kind", item.kind == EntityKind::object ? "object" : "region"},
      {"revision", item.revision}, {"status", item.status == EntityStatus::located ? "located" : "pending_location"},
      {"region_id", item.region_id ? Json(*item.region_id) : Json(nullptr)},
      {"location_observation", std::move(location)}, {"observations", std::move(observations)}};
  }

  const ObservationDatasetReader & dataset_;
  fs::path database_;
  fs::path source_;
  std::unique_ptr<EntityStore> store_;
  std::unique_ptr<PreparationReader> preparation_;
};

}  // namespace

/** @brief 从标准输入读取 JSONL 请求，每行向标准输出写一条结果。 */
int main(int argc, char ** argv)
{
  try {
    fs::path dataset_path, database, preparation;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "Usage: memory_semantic_build --dataset DIR [--store DB] [--preparation DIR] < requests.jsonl\n";
        return 0;
      }
      if (i + 1 == argc) {throw std::invalid_argument("Missing value for " + arg);}
      if (arg == "--dataset") {dataset_path = argv[++i];}
      else if (arg == "--store") {database = argv[++i];}
      else if (arg == "--preparation") {preparation = argv[++i];}
      else {throw std::invalid_argument("Unknown option " + arg);}
    }
    if (dataset_path.empty()) {throw std::invalid_argument("--dataset is required");}
    const auto opened = openObservationDataset(dataset_path);
    if (!opened.ok()) {throw std::runtime_error(opened.message);}
    SemanticBuild builder(*opened.reader, database, preparation);
    bool failed = false;
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.find_first_not_of(" \t\r") == std::string::npos) {continue;}
      Json output;
      try {
        output = builder.run(Json::parse(line));
      } catch (const std::exception & error) {
        output = {{"ok", false}, {"code", "invalid_request"}, {"message", error.what()}};
      }
      failed = failed || !output.at("ok").get<bool>();
      std::cout << output.dump(-1, ' ', false, Json::error_handler_t::replace) << std::endl;
    }
    return failed || std::cin.bad() ? 1 : 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
