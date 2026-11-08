#include <vln_memory/engine/observation_dataset.hpp>

#include <algorithm>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string_view>

#include <vln_memory/bag/builder/bag_types.hpp>
#include <vln_memory/bag/builder/preparation_compactor.hpp>
#include "../bag/builder/preparation_metadata.hpp"
#include "../bag/builder/preparation_storage.hpp"
#include "build_input_internal.hpp"
#include "observation_dataset_format.hpp"

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;

/** @brief 地图和原因字段与正式记录使用同样的非空文字约束，空数据集也要校验。 */
bool text(std::string_view value)
{
  return value.find_first_not_of(' ') != std::string_view::npos &&
    std::none_of(value.begin(), value.end(), [](unsigned char ch) {return ch < 32 || ch == 127;});
}

/** @brief 核实显式地图上下文，避免空选择绕过业务字段检查。 */
void validateContext(const PreparationReader & reader, const ObservationBuildContext & context)
{
  if (context.map_frame != reader.mapFrame() || !text(context.map_frame) ||
    context.map_frame.front() == '/' || context.map_frame.find(' ') != std::string::npos ||
    !text(context.map_id) || context.map_id.size() > 4096 || !text(context.geometry_revision) ||
    context.geometry_revision.size() > 4096 || !text(context.map_basis) || context.revision == 0 ||
    !std::all_of(context.additional_pending_reasons.begin(), context.additional_pending_reasons.end(), text)) {
    throw BagBuildError("invalid_context", "Invalid observation map context or frame differs from preparation");
  }
}

/** @brief 写入一个诊断条目，区分选择问题与已发布的合法 pending 记录。 */
void writeIssue(std::ofstream & issues, const BuildInputFrame & frame, std::string_view code,
  std::string_view message, const std::vector<std::string> & reasons)
{
  issues << Json{{"pair_id", frame.pair_id}, {"code", code}, {"message", message},
    {"reasons", reasons}}.dump() << '\n';
}

/** @brief 关闭输出文件并检查写入状态，失败保留 partial 目录。 */
void finish(std::ofstream & stream, std::string_view name)
{
  stream.close();
  if (!stream) {throw BagBuildError("io_error", "Cannot finish " + std::string(name));}
}

/** @brief 使用已校验的索引回读元数据、组装并保存观测，返回批次引用。 */
Json writeObservations(
  const PreparationReader & reader, const BuildInputResult & selection,
  const ObservationBuildContext & context, ObservationDatasetResult & result)
{
  auto work = result.work_directory;
  fs::create_directory(work / "reports");
  std::ofstream issues(work / "reports/issues.jsonl");
  std::ofstream observations(work / "observations.jsonl");
  if (!observations || !issues) {throw BagBuildError("io_error", "Cannot open observation dataset files");}
  auto batches = Json::array();
  std::set<std::string> observation_ids;
  for (const auto & frame : selection.frames) {
    if (!frame.eligible) {
      const auto * code = frame.read_code == PreparationReadCode::ready ?
        "selection_ineligible" : detail::preparationReadCodeName(frame.read_code);
      writeIssue(issues, frame, code, frame.read_message, frame.eligibility_reasons);
      ++result.issues;
      continue;
    }
    if (!frame.selected) {continue;}
    auto metadata = reader.loadPairMetadata(frame.pair_id);
    if (!metadata.ok()) {
      writeIssue(issues, frame, detail::preparationReadCodeName(metadata.code), metadata.message, {});
      throw BagBuildError(detail::preparationReadCodeName(metadata.code), frame.pair_id + ": " + metadata.message);
    }
    const auto & pair = *metadata.pair;
    if (pair.rgb_message_id != frame.rgb_message_id || pair.depth_message_id != frame.depth_message_id) {
      throw BagBuildError("source_changed", "Selected pair changed: " + frame.pair_id);
    }
    auto built = buildObservation(pair, context);
    if (!built.ok()) {
      const auto * code = observationCodeName(*built.observation_error);
      writeIssue(issues, frame, code, built.message, {});
      throw BagBuildError(code, frame.pair_id + ": " + built.message);
    }
    const auto & observation = *built.observation;
    const auto serialized = serializeObservation(observation);
    if (!serialized.ok()) {
      writeIssue(issues, frame, observationCodeName(serialized.code), serialized.message, {});
      throw BagBuildError(observationCodeName(serialized.code), frame.pair_id + ": " + serialized.message);
    }
    if (!observation_ids.insert(observation.id).second) {
      throw BagBuildError("duplicate_observation", "Duplicate observation for " + frame.pair_id);
    }
    if (*frame.batch_index == batches.size()) {
      batches.push_back({{"batch_index", *frame.batch_index}, {"observation_ids", Json::array()}});
    }
    batches.at(*frame.batch_index).at("observation_ids").push_back(observation.id);
    observations << serialized.json << '\n';
    ++result.observations;
    if (!observation.pending_reasons.empty()) {
      writeIssue(issues, frame, "observation_pending", "", observation.pending_reasons);
      ++result.pending;
      ++result.issues;
    }
  }
  finish(observations, "observations.jsonl");
  finish(issues, "reports/issues.jsonl");
  return batches;
}

/** @brief 写批次引用与来源绑定清单，引用路径均相对观测数据集根。 */
void writeDatasetIndex(
  const PreparationReader & reader, const BuildInputOptions & selection, const ObservationBuildContext & context,
  const Json & batches, const ObservationDatasetResult & result, const fs::path & preparation_hint,
  ObservationPreparationMode mode)
{
  const auto & work = result.work_directory;
  std::ofstream batch_file(work / "batches.jsonl");
  for (const auto & row : batches) {batch_file << row.dump() << '\n';}
  finish(batch_file, "batches.jsonl");

  auto files = Json::object();
  auto file_sha256 = Json::object();
  for (const auto & [name, relative] : detail::kObservationDatasetFiles) {
    files[name] = relative;
    file_sha256[name] = detail::preparationFileSha256(work / relative);
  }
  const Json manifest{
    {"schema_version", detail::kObservationDatasetVersion}, {"observation_schema_version", kObservationSchemaVersion},
    {"artifact_kind", "observation_dataset"}, {"status", result.issues == 0 ? "completed" : "completed_with_issues"},
    {"preparation_root", preparation_hint.generic_string()},
    {"preparation_mode", mode == ObservationPreparationMode::bundle ? "bundle" : "reference"},
    {"source_manifest_sha256", reader.manifestSha256()},
    {"map_id", context.map_id}, {"geometry_revision", context.geometry_revision}, {"map_frame", context.map_frame},
    {"map_basis", context.map_basis}, {"revision", std::to_string(context.revision)},
    {"additional_pending_reasons", context.additional_pending_reasons},
    {"selection_options", detail::buildInputOptionsJson(selection)}, {"candidates", result.candidates},
    {"observations", result.observations}, {"batches", result.batches}, {"pending", result.pending},
    {"issues", result.issues}, {"verified_depth_payloads", result.verified_depth_payloads},
    {"files", std::move(files)}, {"file_sha256", std::move(file_sha256)}};
  std::ofstream file(work / detail::kObservationDatasetManifest);
  file << manifest.dump(2) << '\n';
  finish(file, "observation_manifest.json");
}
}  // namespace

/** @brief 保留原有引用模式入口和链接符号。 */
ObservationDatasetResult buildObservationDataset(
  const fs::path & preparation, const fs::path & output,
  const BuildInputOptions & options, const ObservationBuildContext & context)
{
  return buildObservationDataset(preparation, output, options, context, ObservationPreparationMode::reference);
}

/** @brief 核验候选，从最终绑定的 preparation 组装观测并原子发布。 */
ObservationDatasetResult buildObservationDataset(
  const fs::path & preparation, const fs::path & output,
  const BuildInputOptions & options, const ObservationBuildContext & context, ObservationPreparationMode mode)
{
  ObservationDatasetResult result;
  try {
    auto opened = openPreparation(preparation);
    if (!opened.ok()) {throw BagBuildError(detail::preparationReadCodeName(opened.code), opened.message);}
    validateContext(*opened.reader, context);
    if (mode != ObservationPreparationMode::reference && mode != ObservationPreparationMode::bundle) {
      throw BagBuildError("invalid_options", "Unknown preparation delivery mode");
    }
    const auto final_path = detail::resolveOutputDirectory(output);
    result.work_directory = detail::createWorkDirectory(opened.reader->root(), final_path);
    auto preparation_hint = opened.reader->root();
    if (mode == ObservationPreparationMode::bundle) {
      preparation_hint = detail::kBundledPreparation;
      const auto bundled = compactPreparation(opened.reader->root(), result.work_directory / preparation_hint);
      if (!bundled.ok()) {
        throw BagBuildError(bundled.code, "Cannot bundle preparation: " + bundled.message);
      }
      opened = openPreparation(bundled.output_directory);
      if (!opened.ok()) {throw BagBuildError(detail::preparationReadCodeName(opened.code), opened.message);}
      validateContext(*opened.reader, context);
    }
    const auto & reader = *opened.reader;
    const auto scanned = detail::scanBuildInput(reader, options, true);
    result.candidates = scanned.selection.frames.size();
    result.verified_depth_payloads = scanned.verified_depth_payloads;
    const auto batches = writeObservations(reader, scanned.selection, context, result);
    result.batches = batches.size();
    if (result.observations != scanned.selection.selected || result.batches != scanned.selection.batches) {
      throw BagBuildError("selection_mismatch", "Observation batches do not match the selection");
    }
    detail::writeBuildInputSelection(
      result.work_directory / "build_input_manifest.json", preparation_hint, scanned.selection, options);
    writeDatasetIndex(reader, options, context, batches, result, preparation_hint, mode);
    const auto source = reader.verifyUnchanged(scanned.verified_files);
    if (!source.ok()) {throw BagBuildError(detail::preparationReadCodeName(source.code), source.message);}
    detail::publishDirectory(result.work_directory, final_path);
    result.output_directory = final_path;
    result.work_directory.clear();
    result.code = "ok";
    result.completed = true;
  } catch (const BagBuildError & error) {
    result.code = error.code();
    result.message = error.what();
  } catch (const std::invalid_argument & error) {
    result.code = "invalid_options_or_context";
    result.message = error.what();
  } catch (const std::exception & error) {
    result.code = "io_or_runtime_error";
    result.message = error.what();
  }
  return result;
}

}  // namespace vln_memory
