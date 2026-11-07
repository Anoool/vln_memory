#include <vln_memory/bag/builder/preparation_compactor.hpp>

#include <array>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include <nlohmann/json.hpp>
#include <vln_memory/bag/builder/bag_types.hpp>
#include <vln_memory/bag/builder/preparation_reader.hpp>

#include "preparation_metadata.hpp"
#include "preparation_storage.hpp"

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;

/** @brief 复制时记录大小和修改时间，发布前重新检查源文件的稳定性。 */
struct CopiedFile
{
  fs::path path;
  fs::path requested_path;
  std::uintmax_t bytes{0};
  fs::file_time_type modified;
};

/** @brief 拒绝越界或缺失输入文件，返回规范路径。 */
fs::path sourceMember(const fs::path & root, const std::string & name)
{
  const fs::path relative(name);
  if (relative.empty() || !relative.is_relative() || name.find('\0') != std::string::npos) {
    throw BagBuildError("invalid_reference", "Invalid dataset file path");
  }
  const auto path = fs::canonical(root / relative);
  if (!detail::containsPath(root, path) || !fs::is_regular_file(path))
  {
    throw BagBuildError("invalid_reference", "Invalid dataset file: " + name);
  }
  return path;
}

/** @brief 为已读取或复制的文件保留来源快照，额外空间与文件数成正比。 */
void watch(const fs::path & path, std::vector<CopiedFile> & files)
{
  const auto resolved = fs::canonical(path);
  files.push_back({resolved, path, fs::file_size(resolved), fs::last_write_time(resolved)});
}

/** @brief 新工作目录内创建输出流，拒绝重复写入和越界路径。 */
std::ofstream outputFile(const fs::path & root, const std::string & name)
{
  const fs::path relative(name);
  if (relative.empty() || !relative.is_relative() || name.find('\0') != std::string::npos) {
    throw BagBuildError("invalid_reference", "Invalid output file path");
  }
  const auto path = fs::weakly_canonical(root / relative);
  if (!detail::containsPath(root, path) || fs::exists(path)) {
    throw BagBuildError("invalid_output", "Invalid or duplicate output file: " + name);
  }
  fs::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  if (!file) {throw BagBuildError("io_error", "Cannot write " + path.string());}
  return file;
}

/** @brief 关闭并检查输出状态，失败时保持数据集未发布。 */
void closeOutput(std::ofstream & file, const std::string & name)
{
  file.close();
  if (!file) {throw BagBuildError("io_error", "Cannot finish " + name);}
}

/** @brief 逐行转换 JSONL，解析或写入失败携带文件路径。 */
template<typename Convert>
void rewriteLines(const fs::path & source, const fs::path & work, const std::string & name,
  std::vector<CopiedFile> & watched, Convert convert)
{
  const auto path = sourceMember(source, name);
  watch(source / name, watched);
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw BagBuildError("io_error", "Cannot read " + path.string());}
  auto output = outputFile(work, name);
  std::size_t line_number = 0;
  for (std::string line; detail::readPreparationLine(input, line);) {
    ++line_number;
    try {
      auto row = detail::parsePreparationJson(line);
      convert(row);
      output << row.dump() << '\n';
    } catch (const Json::exception & error) {
      throw BagBuildError("invalid_metadata",
        path.string() + ":" + std::to_string(line_number) + ": " + error.what());
    }
  }
  if (!input.eof() || input.bad()) {throw BagBuildError("io_error", "Cannot read " + path.string());}
  closeOutput(output, name);
}

/** @brief 复制普通文件的内容，源 symlink 解析后仍必须位于数据集内。 */
void copyFile(const fs::path & source, const fs::path & work, const std::string & name,
  std::vector<CopiedFile> & watched)
{
  const auto path = sourceMember(source, name);
  watch(source / name, watched);
  auto output = outputFile(work, name);
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw BagBuildError("io_error", "Cannot read " + path.string());}
  std::array<char, 65536> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    output.write(buffer.data(), input.gcount());
  }
  if (!input.eof() || input.bad()) {throw BagBuildError("io_error", "Cannot copy " + path.string());}
  closeOutput(output, name);
}

/** @brief 保持旧来源字段逐项一致，再统一清除嵌套的 CDR 路径。 */
void clearSource(Json & row, const std::map<std::string, Json> & messages)
{
  if (!row.contains("source")) {return;}
  auto & source = row.at("source");
  const auto id = source.at("message_id").get<std::string>();
  const auto match = messages.find(id);
  if (match == messages.end()) {throw BagBuildError("invalid_reference", "Unknown source: " + id);}
  for (auto field = source.begin(); field != source.end(); ++field) {
    if (!match->second.contains(field.key()) || match->second.at(field.key()) != field.value()) {
      throw BagBuildError("invalid_reference", "Source differs from message: " + id);
    }
  }
  source["raw_path"] = nullptr;
}

/** @brief 构建精简目录，消息来源保留为 O(N) 元数据，像素只在逐对验收时加载。 */
void runCompact(const fs::path & requested_source, const fs::path & requested_output,
  PreparationCompactResult & result)
{
  const auto source = fs::canonical(requested_source);
  auto source_reader = openPreparation(source);
  if (!source_reader.ok()) {throw BagBuildError("invalid_dataset", source_reader.message);}
  const auto source_pair_count = source_reader.reader->pairs().size();
  const std::string source_fingerprint(source_reader.reader->manifestSha256());
  source_reader.reader.reset();
  const auto final_path = detail::resolveOutputDirectory(requested_output);
  const auto work = detail::createWorkDirectory(source, final_path);
  result.work_directory = work;
  std::vector<CopiedFile> watched;
  const auto manifest_path = sourceMember(source, "bag_manifest.json");
  watch(source / "bag_manifest.json", watched);
  const auto manifest_bytes = detail::readPreparationDocument(manifest_path);
  if (detail::preparationSha256(manifest_bytes) != source_fingerprint) {
    throw BagBuildError("source_changed", "Preparation manifest changed during conversion");
  }
  auto manifest = detail::parsePreparationJson(manifest_bytes);
  if (manifest.contains("media_integrity")) {watch(source / detail::kPreparationMediaIndexFile, watched);}
  const auto source_media = detail::readPreparationMediaIndex(source, manifest);
  const auto original_version = manifest.at("builder_version");
  const bool saved = original_version != 7 || manifest.at("effective_options").at("save_raw_cdr").get<bool>();
  std::map<std::string, Json> messages;
  /** @brief 先核对所有消息保存策略，记录省略字节并保留原来源用于后续核验。 */
  const auto convert_message = [&](Json & row) {
      const auto id = row.at("message_id").get<std::string>();
      const auto & raw_path = row.at("raw_path");
      if (!row.at("serialized_bytes").is_number_unsigned()) {
        throw BagBuildError("invalid_metadata", "Invalid serialized_bytes: " + id);
      }
      if (saved) {
        const auto name = raw_path.get<std::string>();
        const auto path = sourceMember(source, name);
        watch(source / name, watched);
        const auto bytes = fs::file_size(path);
        if (bytes != row.at("serialized_bytes").get<std::uint64_t>() ||
          detail::preparationFileSha256(path) != row.at("payload_sha256").get<std::string>()) {
          throw BagBuildError("invalid_reference", "Raw evidence size or digest differs: " + id);
        }
        if (bytes > std::numeric_limits<std::uint64_t>::max() - result.raw_bytes_omitted) {
          throw BagBuildError("invalid_metadata", "Raw byte total overflow");
        }
        result.raw_bytes_omitted += bytes;
      } else if (!raw_path.is_null()) {
        throw BagBuildError("invalid_reference", "Raw path contradicts storage policy: " + id);
      }
      if (!messages.emplace(id, row).second) {throw BagBuildError("invalid_reference", "Duplicate message: " + id);}
      ++result.messages;
      row["raw_path"] = nullptr;
    };
  rewriteLines(source, work, "messages.jsonl", watched, convert_message);
  std::map<std::string, std::string> media;
  std::set<std::string> files{"messages.jsonl"};
  Json retained_files = Json::array({"messages.jsonl"});
  const std::set<std::string> required_files{"messages.jsonl", "frames.jsonl", "rgbd_pairs.jsonl",
    "camera_info.jsonl", "tf.jsonl", "reports/issues.jsonl", "reports/pending.jsonl", "reports/summary.json"};
  for (const auto & value : manifest.at("files")) {
    const auto name = value.get<std::string>();
    if (name == "messages.jsonl" || name == detail::kPreparationMediaIndexFile) {continue;}
    if (fs::path(name).extension() == ".cdr" || name.rfind("evidence/raw/", 0) == 0) {continue;}
    retained_files.push_back(name);
    if (!files.insert(name).second) {throw BagBuildError("invalid_metadata", "Duplicate file: " + name);}
    if (fs::path(name).extension() == ".jsonl") {
      /** @brief 复制所有记录并核对来源，收集每个实际图像媒体的相对路径。 */
      const auto convert = [&](Json & row) {
          if (row.at("schema_version") != "bag_preparation/1") {
            throw BagBuildError("invalid_metadata", "Unexpected JSONL schema: " + name);
          }
          clearSource(row, messages);
          if (name == "frames.jsonl") {
            const auto * sealed = detail::preparationFrameMedia(source_media, row);
            if (!row.at("media_path").is_null()) {
              const auto id = row.at("source").at("message_id").get<std::string>();
              const auto path = sourceMember(source, row.at("media_path").get<std::string>());
              const auto relative = path.lexically_relative(source).generic_string();
              if ((sealed && relative != sealed->path) || !media.emplace(id, relative).second) {
                throw BagBuildError("invalid_reference", "Invalid frame media binding: " + id);
              }
              row["media_path"] = relative;
            }
          }
        };
      rewriteLines(source, work, name, watched, convert);
    } else {copyFile(source, work, name, watched);}
  }
  for (const auto & name : required_files) {
    if (files.count(name) == 0) {throw BagBuildError("invalid_metadata", "Missing manifest file: " + name);}
  }
  if (source_media && source_media->size() != media.size()) {
    throw BagBuildError("invalid_reference", "Sealed media index does not match frame coverage");
  }
  auto media_file = outputFile(work, detail::kPreparationMediaIndexFile);
  std::map<std::string, std::string> copied_digests;
  for (const auto & [id, name] : media) {
    if (fs::path(name).extension() == ".cdr") {
      throw BagBuildError("invalid_reference", "Media cannot reference a CDR file: " + name);
    }
    if (files.insert(name).second) {copyFile(source, work, name, watched);}
    auto digest = copied_digests.find(name);
    if (digest == copied_digests.end()) {
      digest = copied_digests.emplace(name, detail::preparationFileSha256(work / name)).first;
    }
    if (source_media && source_media->at(id).sha256 != digest->second) {
      throw BagBuildError("invalid_media", "Copied media SHA-256 differs: " + name);
    }
    media_file << detail::preparationMediaRow(id, name, digest->second).dump() << '\n';
  }
  closeOutput(media_file, detail::kPreparationMediaIndexFile);
  const auto baseline = source_media ? manifest.at("media_integrity").at("baseline").get<std::string>() :
    std::string("legacy_conversion");
  manifest["media_integrity"] = {{"schema_version", detail::kPreparationMediaIndexSchema},
    {"baseline", baseline}, {"index_sha256", detail::preparationFileSha256(work / detail::kPreparationMediaIndexFile)}};
  retained_files.push_back(detail::kPreparationMediaIndexFile);
  manifest["files"] = std::move(retained_files);
  manifest["builder_version"] = 7;
  manifest["effective_options"]["save_raw_cdr"] = false;
  manifest["conversion"] = "omit_raw_cdr_v1";
  manifest["conversion_source_directory"] = source.string();
  manifest["conversion_source_builder_version"] = original_version;
  auto output = outputFile(work, "bag_manifest.json");
  output << manifest.dump(2) << '\n';
  closeOutput(output, "bag_manifest.json");
  messages.clear();
  auto target_reader = openPreparation(work);
  if (!target_reader.ok()) {throw BagBuildError("invalid_dataset", target_reader.message);}
  if (source_pair_count != target_reader.reader->pairs().size()) {
    throw BagBuildError("invalid_dataset", "Pair count changed during conversion");
  }
  for (const auto & info : target_reader.reader->pairs()) {
    const auto pair = target_reader.reader->loadPair(info.pair_id);
    if (!pair.ok()) {throw BagBuildError("invalid_dataset", info.pair_id + ": " + pair.message);}
    ++result.pairs_checked;
  }
  for (const auto & file : watched) {
    if (fs::canonical(file.requested_path) != file.path || fs::file_size(file.path) != file.bytes ||
      fs::last_write_time(file.path) != file.modified) {
      throw BagBuildError("source_changed", "Source changed during conversion: " + file.path.string());
    }
  }
  detail::publishDirectory(work, final_path);
  result.output_directory = final_path;
  result.work_directory.clear();
  result.completed = true;
  result.code = "ok";
}
}  // namespace

/** @brief 返回转换进度及结构化错误，失败工作目录保持可排查。 */
PreparationCompactResult compactPreparation(const fs::path & source, const fs::path & output)
{
  PreparationCompactResult result;
  try {
    runCompact(source, output, result);
  } catch (const BagBuildError & error) {
    result.code = error.code();
    result.message = error.what();
  } catch (const detail::PreparationMetadataError & error) {
    result.code = "invalid_dataset";
    result.message = error.what();
  } catch (const Json::exception & error) {
    result.code = "invalid_metadata";
    result.message = error.what();
  } catch (const std::exception & error) {
    result.code = "io_or_runtime_error";
    result.message = error.what();
  }
  return result;
}
}  // namespace vln_memory
