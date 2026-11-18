#include "memory_publication.hpp"

#include <fstream>
#include <map>
#include <set>
#include <stdexcept>

#include "../bag/builder/preparation_metadata.hpp"
#include "../bag/builder/preparation_storage.hpp"
#include "entity_tools.hpp"
#include "observation_dataset_format.hpp"

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;
using ObservationRevisions = std::map<std::string, std::uint64_t>;

constexpr char kMediaIndex[] = "indexes/media_sha256.jsonl";
constexpr char kEntityIndex[] = "indexes/entities_by_observation.jsonl";
constexpr char kPendingEntities[] = "reports/pending_entities.jsonl";
constexpr char kCorrections[] = "reports/corrections.jsonl";

/** @brief 版本编号只用可跨路径保存的字符。 */
bool validVersionName(const std::string & version)
{
  return !version.empty() &&
    version.find_first_not_of("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_-.") == std::string::npos;
}

/** @brief 创建发布文件，打开、写入或关闭失败时抛出异常。 */
std::ofstream openOutput(const fs::path & path)
{
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path, std::ios::binary);
  return stream;
}

/** @brief 保留来源清单的原字节，复制并校验正式观测引用的每张图。 */
ObservationRevisions copyObservations(
  const ObservationDatasetInfo & expected, const fs::path & source, const fs::path & work)
{
  for (const auto * relative : {detail::kObservationDatasetRows, detail::kDirectObservationManifest,
      detail::kDirectObservationSource, detail::kDirectObservationMediaIndex}) {
    fs::copy_file(source / relative, work / relative);
  }
  const auto copied = openObservationDataset(work);
  if (!copied.ok()) {throw std::runtime_error(copied.message);}
  if (copied.reader->info().dataset_manifest_sha256 != expected.dataset_manifest_sha256) {
    throw std::runtime_error("Observation dataset changed during publication");
  }
  const auto manifest = detail::parsePreparationJson(
    detail::readPreparationDocument(work / detail::kDirectObservationManifest));
  const auto media = detail::readSealedMediaIndex(work / detail::kDirectObservationMediaIndex,
    manifest.at("file_sha256").at("media").get<std::string>());

  auto index = openOutput(work / kMediaIndex);
  std::set<std::string> copied_messages;
  /** @brief 只复制匹配此观测角色和封存索引的文件。 */
  const auto copyMedia = [&](const std::string & message, const std::string & path, const char * role) {
    const auto & entry = media.at(message);
    const fs::path relative(path);
    if (entry.path != path || relative.parent_path() != fs::path("evidence") / role ||
      !relative.has_filename() || relative.filename() == "." || relative.filename() == "..") {
      throw std::runtime_error("Invalid media path: " + path);
    }
    if (!copied_messages.insert(message).second) {return;}
    fs::copy_file(source / relative, work / relative);
    if (detail::preparationFileSha256(work / relative) != entry.sha256) {
      throw std::runtime_error("Media digest differs: " + path);
    }
    index << detail::preparationMediaRow(message, path, entry.sha256).dump() << '\n';
  };
  ObservationRevisions revisions;
  for (const auto & batch : copied.reader->batches()) {
    const auto result = copied.reader->loadBatch(batch.batch_index);
    if (!result.ok()) {throw std::runtime_error(result.message);}
    for (const auto & observation : result.observations) {
      copyMedia(observation.rgb_message_id, observation.rgb_media.value(), "rgb");
      copyMedia(observation.depth_message_id, observation.depth_media.value(), "depth");
      revisions.emplace(observation.id, observation.revision);
    }
  }
  index.close();
  return revisions;
}

/** @brief 在同一读事务中发布实体、关系、人工标注和待定位状态，核对观测引用。 */
Json exportStore(const fs::path & source, const fs::path & work,
  const ObservationDatasetInfo & dataset, const ObservationRevisions & revisions)
{
  auto opened = openEntityStoreSnapshot(source / "entities.sqlite3");
  if (!opened.ok()) {throw std::runtime_error(opened.message);}
  const auto & store = *opened.store;
  const auto & bound = store.context();
  if (bound.dataset_manifest_sha256 != dataset.dataset_manifest_sha256 ||
    bound.source_manifest_sha256 != dataset.source_manifest_sha256 || bound.map_id != dataset.map_id ||
    bound.map_frame != dataset.map_frame || bound.geometry_revision != dataset.geometry_revision) {
    throw std::runtime_error("Work store belongs to a different observation dataset");
  }

  auto entities_file = openOutput(work / "entities.jsonl");
  const auto entities = store.listEntityIds();
  std::map<std::string, std::vector<std::string>> by_observation;
  for (const auto & id : entities) {
    const auto loaded = store.loadEntity(id);
    if (!loaded.ok()) {throw std::runtime_error(loaded.message);}
    const auto & entity = *loaded.entity;
    for (const auto & ref : entity.observations) {
      const auto observation = revisions.find(ref.observation_id);
      if (observation == revisions.end() || observation->second != ref.observation_revision) {
        throw std::runtime_error("Entity evidence references a missing observation revision: " + id);
      }
      by_observation[ref.observation_id].push_back(id);
    }
    auto row = describeEntity(entity);
    row["status"] = "pending_location";
    if (entity.status == EntityStatus::located) {
      row["status"] = "located";
    }
    row["location_observation"] = nullptr;
    if (entity.location_observation) {
      const auto & location = *entity.location_observation;
      row["location_observation"] = {{"observation_id", location.observation_id},
        {"observation_revision", location.observation_revision}};
    }
    if (entity.location_pending_reason) {row["location_pending_reason"] = *entity.location_pending_reason;}
    entities_file << row.dump() << '\n';
  }
  entities_file.close();

  const auto relations = store.listRelations();
  auto relations_file = openOutput(work / "relations.jsonl");
  const std::set<std::string> entity_ids(entities.begin(), entities.end());
  for (const auto & relation : relations) {
    for (const auto & id : relation.entity_ids) {
      if (!entity_ids.count(id)) {throw std::runtime_error("Missing relation endpoint: " + id);}
    }
    for (const auto & id : relation.evidence_observation_ids) {
      if (!revisions.count(id)) {throw std::runtime_error("Missing relation observation: " + id);}
    }
    relations_file << describeRelation(relation).dump() << '\n';
  }
  relations_file.close();

  const auto annotations = store.listEntityAnnotations();
  auto annotations_file = openOutput(work / "annotations.jsonl");
  for (const auto & annotation : annotations) {
    if (!entity_ids.count(annotation.entity_id)) {
      throw std::runtime_error("Missing annotation entity: " + annotation.entity_id);
    }
    annotations_file << describeAnnotation(annotation).dump() << '\n';
  }
  annotations_file.close();

  const auto corrections = store.listCorrections();
  auto history = openOutput(work / kCorrections);
  for (const auto & [id, raw] : corrections) {
    auto correction = Json::parse(raw);
    correction["correction_id"] = id;
    history << correction.dump() << '\n';
  }
  history.close();

  const auto pending = store.listPending();
  auto report = openOutput(work / kPendingEntities);
  for (const auto & [id, request] : pending) {
    report << Json{{"pending_id", id}, {"request", Json::parse(request)}}.dump() << '\n';
  }
  report.close();

  auto index = openOutput(work / kEntityIndex);
  for (const auto & [id, linked] : by_observation) {
    index << Json{{"observation_id", id}, {"entity_ids", linked}}.dump() << '\n';
  }
  index.close();
  return {{"entities", entities.size()}, {"relations", relations.size()},
    {"observations", revisions.size()}, {"annotations", annotations.size()},
    {"pending_entities", pending.size()}, {"corrections", corrections.size()}};
}
}  // namespace

/** @brief 已发布目录必须有绑定同一观测目录的顶层清单。 */
std::optional<std::string> publishedMemoryVersion(const ObservationDatasetInfo & dataset, const fs::path & root)
{
  const auto path = root / "manifest.json";
  if (!fs::exists(fs::symlink_status(path))) {return std::nullopt;}
  const auto manifest = detail::parsePreparationJson(detail::readPreparationDocument(path));
  const auto version = manifest.at("memory_version").get<std::string>();
  if (manifest.at("artifact_kind") != "spatial_memory" || manifest.at("schema_version") != "spatial_memory/1" ||
    !validVersionName(version) || manifest.at("map_id") != dataset.map_id || manifest.at("frame_id") != dataset.map_frame ||
    manifest.at("geometry_revision") != dataset.geometry_revision ||
    manifest.at("dataset_manifest_sha256") != dataset.dataset_manifest_sha256 ||
    manifest.at("source_manifest_sha256") != dataset.source_manifest_sha256) {
    throw std::invalid_argument("Published memory manifest differs from its observation dataset");
  }
  return version;
}

/** @brief 复用同文件系统的原子改名，目标路径存在时拒绝覆盖。 */
Json publishMemoryVersion(
  const ObservationDatasetInfo & dataset, const fs::path & source, const fs::path & requested)
{
  const auto output = detail::resolveOutputDirectory(requested);
  const auto version = output.filename().string();
  if (!validVersionName(version)) {
    throw std::invalid_argument("Use letters, digits, underscores, hyphens or dots for the version directory name");
  }
  if (!fs::is_regular_file(source / "entities.sqlite3")) {
    throw std::invalid_argument("Publication needs an existing entities.sqlite3 work store");
  }
  const auto work = detail::createWorkDirectory(source, output);
  fs::create_directories(work / "evidence/rgb");
  fs::create_directories(work / "evidence/depth");
  fs::create_directory(work / "reports");
  fs::create_directory(work / "indexes");

  const auto observations = copyObservations(dataset, source, work);
  const auto counts = exportStore(source, work, dataset, observations);

  Json files = Json::object();
  for (const auto * relative : {"entities.jsonl", "relations.jsonl", "annotations.jsonl",
      detail::kObservationDatasetRows, detail::kDirectObservationManifest, detail::kDirectObservationSource,
      detail::kDirectObservationMediaIndex, kMediaIndex, kEntityIndex, kPendingEntities, kCorrections}) {
    files[relative] = detail::preparationFileSha256(work / relative);
  }
  for (const auto * name : {"summary.json", "pending.jsonl", "issues.jsonl"}) {
    const auto from = source / "evidence" / name;
    if (!fs::exists(from)) {continue;}
    const auto relative = fs::path("reports") / name;
    fs::copy_file(from, work / relative);
    files[relative.generic_string()] = detail::preparationFileSha256(work / relative);
  }
  auto manifest = openOutput(work / "manifest.json");
  manifest << Json{{"artifact_kind", "spatial_memory"}, {"schema_version", "spatial_memory/1"},
    {"memory_version", version}, {"map_id", dataset.map_id}, {"frame_id", dataset.map_frame},
    {"geometry_revision", dataset.geometry_revision}, {"map_basis", dataset.map_basis},
    {"dataset_manifest_sha256", dataset.dataset_manifest_sha256},
    {"source_manifest_sha256", dataset.source_manifest_sha256}, {"counts", counts},
    {"file_sha256", std::move(files)}}.dump(2) << '\n';
  manifest.close();
  const Json result{{"status", "ok"}, {"memory_version", version},
    {"output_directory", output.string()}, {"counts", counts}};
  detail::publishDirectory(work, output);
  return result;
}
}  // namespace vln_memory
