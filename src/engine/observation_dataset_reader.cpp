#include <vln_memory/engine/observation_dataset_reader.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>

#include "../bag/builder/preparation_metadata.hpp"
#include "observation_dataset_format.hpp"
#include "observation_source_internal.hpp"

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Code = ObservationDatasetReadCode;
using Json = nlohmann::json;

struct ReadError : std::runtime_error
{
  Code code;
  /** @brief 保留数据集错误类别，供公开结果映射。 */
  ReadError(Code value, const std::string & message) : std::runtime_error(message), code(value) {}
};

struct FileSnapshot
{
  fs::path path;
  std::uintmax_t bytes;
  fs::file_time_type modified;
};

struct ObservationPairReference
{
  std::string pair_id;
  std::string rgb_message_id;
  std::string depth_message_id;
  SensorStamp rgb_stamp;
  SensorStamp depth_stamp;
  std::int64_t delta_ns;
};

struct RowLocation
{
  std::streamoff offset;
  std::size_t bytes;
  std::string sha256;
  ObservationPairReference source_pair;
};

struct DatasetManifest
{
  ObservationDatasetInfo info;
  std::size_t batch_count{0};
  std::size_t batch_size{0};
  Json hashes;
  std::string sha256;
};

/** @brief 拒绝首次失配，调用方不会得到不完整索引或批次。 */
void require(bool condition, Code code, const std::string & message)
{
  if (!condition) {throw ReadError(code, message);}
}

/** @brief 按解析边界区分格式错误和底层读取错误。 */
template<typename Action>
auto parseChecked(Code invalid, Action action)
{
  try {
    return action();
  } catch (const Json::exception & error) {
    throw ReadError(invalid, error.what());
  } catch (const detail::PreparationMetadataError & error) {
    const auto code = error.code == PreparationReadCode::io_error ? Code::io_error : invalid;
    throw ReadError(code, error.what());
  }
}

/** @brief 将内部错误转为公开结果，其他标准异常视作读取失败。 */
template<typename Result, typename Action>
Result readResult(Action action)
{
  try {
    return action();
  } catch (const ReadError & error) {
    return {error.code, error.what(), {}};
  } catch (const std::exception & error) {
    return {Code::io_error, error.what(), {}};
  }
}

/** @brief 捕获固定根相对文件的状态，拒绝软链或越出根目录的解析。 */
FileSnapshot snapshot(const fs::path & path)
{
  require(fs::is_regular_file(fs::symlink_status(path)) && fs::canonical(path) == path,
    Code::invalid_manifest, "Missing or unsafe dataset file: " + path.string());
  return {path, fs::file_size(path), fs::last_write_time(path)};
}

/** @brief 发现打开后的一般文件改写、替换或删除。 */
void checkFiles(const std::vector<FileSnapshot> & files)
{
  for (const auto & file : files) {
    std::error_code error;
    const auto path = fs::canonical(file.path, error);
    require(!error && path == file.path &&
      fs::file_size(path, error) == file.bytes && !error &&
      fs::last_write_time(path, error) == file.modified && !error,
      Code::source_changed, "Dataset file changed: " + file.path.string());
  }
}

/** @brief 核对已打开的文件和清单原始字节，连同恢复修改时间的改写一起识别。 */
void checkDatasetSnapshot(const std::vector<FileSnapshot> & files, std::string_view manifest_sha256)
{
  checkFiles(files);
  try {
    const auto bytes = detail::readPreparationDocument(files.front().path);
    require(detail::preparationSha256(bytes) == manifest_sha256, Code::source_changed,
      "Observation dataset manifest bytes changed");
  } catch (const detail::PreparationMetadataError & error) {
    throw ReadError(Code::source_changed, error.what());
  }
}

/** @brief 读取有界整数计数，拒绝负数、浮点和溢出。 */
std::size_t count(const Json & value, Code code)
{
  require(value.is_number_unsigned() && value.get<std::uint64_t>() <= std::numeric_limits<std::size_t>::max(),
    code, "Expected an unsigned dataset count");
  return value.get<std::size_t>();
}

/** @brief 地图字段和来源提示须含文字且无控制字符。 */
bool validText(std::string_view value)
{
  return value.find_first_not_of(' ') != std::string_view::npos &&
    std::none_of(value.begin(), value.end(), [](unsigned char ch) {return ch < 32 || ch == 127;});
}

/** @brief 仅接受规范小写 SHA-256 十六进制。 */
bool validDigest(std::string_view value)
{
  return value.size() == 64 && value.find_first_not_of("0123456789abcdef") == std::string_view::npos;
}

/** @brief 解析已发布的观测目录清单，v1 不要求文件内容摘要。 */
DatasetManifest readManifest(const fs::path & path)
{
  const auto bytes = detail::readPreparationDocument(path);
  const auto row = detail::parsePreparationJson(bytes);
  const auto & version = row.at("schema_version");
  const bool direct = version == detail::kDirectObservationVersion;
  require((direct || version == detail::kObservationDatasetVersion || version == detail::kObservationDatasetLegacyVersion) &&
    row.at("artifact_kind") == "observation_dataset" && row.at("observation_schema_version") == kObservationSchemaVersion,
    Code::unsupported_dataset, "Unsupported observation dataset or record schema");
  DatasetManifest out;
  out.sha256 = detail::preparationSha256(bytes);
  auto & info = out.info;
  info.schema_version = version.get<std::string>();
  info.dataset_manifest_sha256 = out.sha256;
  info.direct_evidence = direct;
  const auto preparation = direct ? std::string{} : row.at("preparation_root").get<std::string>();
  info.preparation_root = preparation;
  if (!direct && row.contains("preparation_mode")) {
    const auto & mode = row.at("preparation_mode");
    require(mode == "reference" || mode == "bundle", Code::invalid_manifest, "Unknown preparation delivery mode");
    info.preparation_bundled = mode == "bundle";
    require(!info.preparation_bundled || preparation == detail::kBundledPreparation,
      Code::invalid_manifest, "Bundled preparation must use its fixed relative directory");
  }
  info.source_manifest_sha256 = row.at("source_manifest_sha256").get<std::string>();
  info.map_id = row.at("map_id").get<std::string>();
  info.geometry_revision = row.at("geometry_revision").get<std::string>();
  info.map_frame = row.at("map_frame").get<std::string>();
  info.map_basis = row.at("map_basis").get<std::string>();
  const auto revision = row.at("revision").get<std::string>();
  const auto revision_end = revision.data() + revision.size();
  const auto decoded = std::from_chars(revision.data(), revision_end, info.revision);
  require(decoded.ec == std::errc{} && decoded.ptr == revision_end && info.revision > 0 && revision.front() != '0' &&
    (direct || validText(preparation)) && validDigest(info.source_manifest_sha256) && validText(info.map_id) &&
    info.map_id.size() <= 4096 && validText(info.geometry_revision) && info.geometry_revision.size() <= 4096 &&
    validText(info.map_frame) && info.map_frame.front() != '/' && info.map_frame.find(' ') == std::string::npos &&
    validText(info.map_basis), Code::invalid_manifest, "Invalid dataset source or map context");
  info.observation_count = count(row.at("observations"), Code::invalid_manifest);
  info.pending_count = count(row.at("pending"), Code::invalid_manifest);
  if (direct) {
    out.batch_size = count(row.at("batch_size"), Code::invalid_manifest);
    out.hashes = row.at("file_sha256");
    require(out.batch_size > 0 && info.pending_count <= info.observation_count &&
      out.hashes.is_object() && out.hashes.size() == 2 &&
      validDigest(out.hashes.at("observations").get<std::string>()) &&
      validDigest(out.hashes.at("media").get<std::string>()),
      Code::invalid_manifest, "Invalid direct observation counts or file digests");
    return out;
  }
  const auto issue_count = count(row.at("issues"), Code::invalid_manifest);
  out.batch_count = count(row.at("batches"), Code::invalid_manifest);
  out.batch_size = count(row.at("selection_options").at("batch_size"), Code::invalid_manifest);
  require(out.batch_size > 0 && info.observation_count <= count(row.at("candidates"), Code::invalid_manifest) &&
    info.pending_count <= info.observation_count && info.pending_count <= issue_count &&
    out.batch_count <= info.observation_count &&
    row.at("status") == (issue_count ? "completed_with_issues" : "completed"),
    Code::invalid_manifest, "Dataset status or counts contradict");
  const auto & files = row.at("files");
  require(files.is_object() && files.size() == detail::kObservationDatasetFiles.size(), Code::invalid_manifest,
    "Unexpected observation dataset files");
  if (version == detail::kObservationDatasetVersion || row.contains("file_sha256")) {
    out.hashes = row.at("file_sha256");
    require(out.hashes.is_object() && out.hashes.size() == files.size(), Code::invalid_manifest,
      "Incomplete dataset file digests");
  }
  for (const auto & [name, relative] : detail::kObservationDatasetFiles) {
    require(files.at(name) == relative, Code::invalid_manifest, "Unexpected dataset path for " + std::string(name));
    if (!out.hashes.is_null()) {
      require(validDigest(out.hashes.at(name).get<std::string>()), Code::invalid_manifest, "Invalid dataset file digest");
    }
  }
  return out;
}

/** @brief 核对新产物的文件字节，旧产物继续走结构和引用校验。 */
void verifyDigest(const DatasetManifest & manifest, const char * name, const std::string & digest)
{
  require(manifest.hashes.is_null() || manifest.hashes.at(name) == digest, Code::integrity_mismatch,
    "Dataset file SHA-256 differs: " + std::string(name));
}

/** @brief 顺序读取批次，检查序号、大小、计数和版本摘要。 */
std::vector<ObservationDatasetBatch> readBatches(const fs::path & path, const DatasetManifest & manifest)
{
  std::vector<ObservationDatasetBatch> batches;
  std::size_t remaining = manifest.info.observation_count;
  /** @brief 追加非空的下一个批次，计数不超过清单。 */
  const auto visit = [&](std::streamoff, const std::string & line) {
      const auto row = detail::parsePreparationJson(line);
      const auto index = count(row.at("batch_index"), Code::invalid_batch);
      const auto & ids = row.at("observation_ids");
      require(row.size() == 2 && index == batches.size() && index < manifest.batch_count &&
        ids.is_array() && !ids.empty() && ids.size() <= manifest.batch_size && ids.size() <= remaining,
        Code::invalid_batch, "Invalid batch index, count or size");
      batches.push_back({index, ids.get<std::vector<std::string>>()});
      remaining -= ids.size();
    };
  verifyDigest(manifest, "batches", detail::scanPreparationJsonl(path, visit));
  require(batches.size() == manifest.batch_count && remaining == 0, Code::invalid_batch,
    "Batches do not account for the observation count");
  return batches;
}

/** @brief 在建立轻量引用时核对原始秒和纳秒范围。 */
SensorStamp sourceStamp(const Json & stamp)
{
  const auto & seconds = stamp.at("sec");
  const auto & nanoseconds = stamp.at("nanosec");
  require(seconds.is_number_integer() && stamp.size() == 2, Code::invalid_observation, "Invalid source stamp");
  // int32 的两个端点可以由 double 精确表示，先比较可避免无符号 JSON 大整数窄化。
  const auto seconds_value = seconds.get<double>();
  const auto nanoseconds_value = count(nanoseconds, Code::invalid_observation);
  require(seconds_value >= std::numeric_limits<std::int32_t>::min() &&
    seconds_value <= std::numeric_limits<std::int32_t>::max() && nanoseconds_value < 1000000000U,
    Code::invalid_observation, "Source stamp is out of range");
  return {static_cast<std::int32_t>(seconds_value), static_cast<std::uint32_t>(nanoseconds_value)};
}

/** @brief 提取后续绑定 preparation 需要的配对身份和时间字段。 */
ObservationPairReference sourcePairReference(const Json & row)
{
  ObservationPairReference reference{row.at("pair_id").get<std::string>(),
    row.at("rgb_message_id").get<std::string>(), row.at("depth_message_id").get<std::string>(),
    sourceStamp(row.at("rgb_stamp")), sourceStamp(row.at("depth_stamp")),
    detail::preparationSigned(row.at("delta_ns"))};
  require(validText(reference.pair_id) && validText(reference.rgb_message_id) &&
    validText(reference.depth_message_id) && reference.rgb_message_id != reference.depth_message_id,
    Code::invalid_observation, "Invalid observation pair identity");
  return reference;
}

/** @brief 索引 ID 与发布行，检查批次覆盖、地图上下文及 pending 计数。 */
std::map<std::string, RowLocation, std::less<>> indexObservations(
  const fs::path & path, const DatasetManifest & manifest, std::vector<ObservationDatasetBatch> & batches,
  const std::optional<detail::PreparationMediaIndex> & media)
{
  std::map<std::string, RowLocation, std::less<>> rows;
  const auto & info = manifest.info;
  const auto revision = std::to_string(info.revision);
  std::size_t batch = 0;
  std::size_t position = 0;
  std::size_t pending = 0;
  /** @brief 保留读过的原始行摘要，后续读取能检测恢复了时间戳的改写。 */
  const auto visit = [&](std::streamoff offset, const std::string & line) {
      const auto row = detail::parsePreparationJson(line);
      const auto & reasons = row.at("pending_reasons");
      require(row.at("schema_version") == kObservationSchemaVersion && row.at("map_id") == info.map_id &&
        row.at("geometry_revision") == info.geometry_revision && row.at("map_frame") == info.map_frame &&
        row.at("map_basis") == info.map_basis && row.at("revision") == revision &&
        row.at("source_manifest_sha256") == info.source_manifest_sha256 && reasons.is_array(),
        Code::invalid_observation, "Observation context or source differs from dataset manifest");
      const auto id = row.at("id").get<std::string>();
      require(!id.empty() && rows.size() < info.observation_count,
        Code::invalid_observation, "Unexpected observation row");
      RowLocation location{offset, line.size(), detail::preparationSha256(line), sourcePairReference(row)};
      if (media) {
        for (const auto * kind : {"rgb", "depth"}) {
          const auto & reference = row.at(std::string(kind) + "_media");
          const auto message = row.at(std::string(kind) + "_message_id").get<std::string>();
          const auto bound = media->find(message);
          require(reference.is_null() ? bound == media->end() :
            bound != media->end() && reference == bound->second.path,
            Code::invalid_observation, "Observation media differs from sealed index");
        }
        if (batches.empty() || batches.back().observation_ids.size() == manifest.batch_size) {
          batches.push_back({batches.size(), {}});
        }
        batches.back().observation_ids.push_back(id);
      } else {
        require(batch < batches.size() && id == batches[batch].observation_ids[position],
          Code::invalid_batch, "Batch reference differs from published observation order");
        if (++position == batches[batch].observation_ids.size()) {
          ++batch;
          position = 0;
        }
      }
      require(rows.emplace(id, std::move(location)).second, Code::invalid_observation, "Duplicate observation ID");
      if (!reasons.empty()) {++pending;}
    };
  verifyDigest(manifest, "observations", detail::scanPreparationJsonl(path, visit));
  require(rows.size() == info.observation_count && pending == info.pending_count,
    Code::invalid_observation, "Observation or pending count differs from dataset manifest");
  return rows;
}

/** @brief 读取已索引的一行并用正式记录解析器完整校验。 */
Observation loadIndexed(std::istream & stream, const RowLocation & location)
{
  stream.seekg(location.offset);
  std::string line(location.bytes, '\0');
  require(static_cast<bool>(stream.read(line.data(), static_cast<std::streamsize>(line.size()))),
    Code::source_changed, "Cannot reread indexed observation");
  require(detail::preparationSha256(line) == location.sha256, Code::source_changed, "Indexed observation bytes changed");
  auto parsed = deserializeObservation(line);
  require(parsed.ok(), Code::invalid_observation,
    std::string(observationCodeName(parsed.code)) + ": " + parsed.message);
  return std::move(*parsed.observation);
}

}  // namespace

/** @brief 索引只保存行位置、摘要和批次目录，不长期保存正式记录或像素。 */
struct ObservationDatasetReaderState
{
  fs::path root;
  ObservationDatasetInfo info;
  std::string manifest_sha256;
  std::vector<FileSnapshot> files;
  std::optional<detail::PreparationMediaIndex> media;
  std::vector<ObservationDatasetBatch> batches;
  std::map<std::string, RowLocation, std::less<>> rows;
};

namespace
{
/** @brief 验证正式观测的媒体角色与固定相对目录，解析后拒绝软链。 */
FileSnapshot directMediaFile(
  const ObservationDatasetReaderState & state, const std::string & message, const std::string & reference,
  const char * role)
{
  const auto bound = state.media->find(message);
  const fs::path relative(reference);
  require(bound != state.media->end() && bound->second.path == reference &&
    relative.parent_path() == fs::path("evidence") / role && relative.has_filename(),
    Code::invalid_observation, "Media reference does not match its sealed observation role");
  return snapshot(state.root / relative);
}

/** @brief 只用已建索引比对发布观测的配对身份及原时间，不读媒体。 */
std::optional<std::string> findSourcePairMismatch(
  const ObservationDatasetReaderState & state, const PreparationReader & source)
{
  if (state.rows.empty()) {return std::nullopt;}
  std::unordered_map<std::string_view, const PreparationPairInfo *> pairs;
  pairs.reserve(source.pairs().size());
  for (const auto & pair : source.pairs()) {pairs.emplace(pair.pair_id, &pair);}
  for (const auto & entry : state.rows) {
    const auto & ref = entry.second.source_pair;
    const auto found = pairs.find(ref.pair_id);
    if (found == pairs.end()) {return "Missing referenced preparation pair: " + ref.pair_id;}
    const auto & pair = *found->second;
    if (pair.rgb_message_id != ref.rgb_message_id || pair.depth_message_id != ref.depth_message_id ||
      !detail::sameStamp(pair.rgb_stamp, ref.rgb_stamp) || !detail::sameStamp(pair.depth_stamp, ref.depth_stamp) ||
      pair.delta_ns != ref.delta_ns) {
      return "Preparation pair identity or timing differs from the observation: " + ref.pair_id;
    }
  }
  return std::nullopt;
}
}  // namespace

/** @brief 接管已验证的索引。 */
ObservationDatasetReader::ObservationDatasetReader(std::unique_ptr<ObservationDatasetReaderState> state)
: state_(std::move(state)) {}

/** @brief 释放只读目录和索引。 */
ObservationDatasetReader::~ObservationDatasetReader() = default;

/** @brief 返回打开时读取并验证的清单信息。 */
const ObservationDatasetInfo & ObservationDatasetReader::info() const noexcept
{
  return state_->info;
}

/** @brief 返回批次及批次内的发布顺序。 */
const std::vector<ObservationDatasetBatch> & ObservationDatasetReader::batches() const noexcept
{
  return state_->batches;
}

/** @brief 一次定位索引和文件偏移，读取过程中数据集须保持稳定。 */
ObservationDatasetLoadResult ObservationDatasetReader::loadObservation(std::string_view observation_id) const
{
  /** @brief 回读并检查文件状态，任一失败即丢弃当前结果。 */
  return readResult<ObservationDatasetLoadResult>([&]() -> ObservationDatasetLoadResult {
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      const auto row = state_->rows.find(observation_id);
      require(row != state_->rows.end(), Code::observation_not_found, "Observation ID not found");
      std::ifstream stream(state_->root / detail::kObservationDatasetRows, std::ios::binary);
      require(stream.is_open(), Code::io_error, "Cannot open indexed observations");
      auto observation = loadIndexed(stream, row->second);
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      return {Code::ready, "", std::move(observation)};
    });
}

/** @brief 只匹配保存的配对，时间戳重复时让调用方提供确切 ID。 */
ObservationDatasetLoadResult ObservationDatasetReader::loadObservationByStamps(SensorStamp rgb, SensorStamp depth) const
{
  const std::string * matched = nullptr;
  for (const auto & [id, row] : state_->rows) {
    const auto & pair = row.source_pair;
    if (pair.rgb_stamp.sec != rgb.sec || pair.rgb_stamp.nanosec != rgb.nanosec ||
      pair.depth_stamp.sec != depth.sec || pair.depth_stamp.nanosec != depth.nanosec) {continue;}
    if (matched) {return {Code::ambiguous_observation, "Raw timestamps match multiple published observations", std::nullopt};}
    matched = &id;
  }
  if (!matched) {return {Code::observation_not_found, "No published pair for these raw timestamps", std::nullopt};}
  return loadObservation(*matched);
}

/** @brief 复用一个文件流读取整批，发生错误时不返回部分批次。 */
ObservationDatasetBatchResult ObservationDatasetReader::loadBatch(std::size_t batch_index) const
{
  /** @brief 在相同数据集状态下读取一个发布批次。 */
  return readResult<ObservationDatasetBatchResult>([&]() -> ObservationDatasetBatchResult {
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      require(batch_index < state_->batches.size(), Code::batch_not_found, "Batch index not found");
      std::ifstream stream(state_->root / detail::kObservationDatasetRows, std::ios::binary);
      require(stream.is_open(), Code::io_error, "Cannot open indexed observations");
      std::vector<Observation> observations;
      const auto & ids = state_->batches[batch_index].observation_ids;
      observations.reserve(ids.size());
      for (const auto & id : ids) {observations.push_back(loadIndexed(stream, state_->rows.at(id)));}
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      return {Code::ready, "", std::move(observations)};
    });
}

/** @brief 读回直出 PNG 的同一份字节，再校验内容摘要。 */
ObservationRgbResult ObservationDatasetReader::loadRgb(std::string_view id) const
{
  return readResult<ObservationRgbResult>([&]() -> ObservationRgbResult {
      require(state_->media.has_value(), Code::media_unavailable, "Use preparation for legacy media");
      auto loaded = loadObservation(id);
      require(loaded.ok(), loaded.code, loaded.message);
      const auto & obs = *loaded.observation;
      require(obs.rgb_media.has_value(), Code::media_unavailable, "Observation has no RGB media");
      const auto file = directMediaFile(*state_, obs.rgb_message_id, *obs.rgb_media, "rgb");
      constexpr auto max_rgb_bytes = 128U * 1024U * 1024U;
      require(file.bytes > 0 && file.bytes <= max_rgb_bytes, Code::invalid_observation, "RGB media exceeds read budget");
      std::vector<std::uint8_t> bytes(file.bytes);
      std::ifstream input(file.path, std::ios::binary);
      require(static_cast<bool>(input.read(reinterpret_cast<char *>(bytes.data()), bytes.size())) &&
        input.peek() == std::char_traits<char>::eof() && !input.bad(), Code::io_error, "Cannot read complete RGB media");
      const std::string_view view(reinterpret_cast<const char *>(bytes.data()), bytes.size());
      require(detail::preparationSha256(view) == state_->media->at(obs.rgb_message_id).sha256,
        Code::integrity_mismatch, "RGB media SHA-256 differs");
      checkFiles({file});
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      return {Code::ready, "", std::move(bytes)};
    });
}

/** @brief 直接从深度证据恢复像素，并校验读回前后的文件状态。 */
ObservationDepthResult ObservationDatasetReader::loadDepth(std::string_view id) const
{
  return readResult<ObservationDepthResult>([&]() -> ObservationDepthResult {
      require(state_->media.has_value(), Code::media_unavailable, "Use preparation for legacy media");
      auto loaded = loadObservation(id);
      require(loaded.ok(), loaded.code, loaded.message);
      const auto & obs = *loaded.observation;
      require(obs.depth_media && obs.depth_storage, Code::media_unavailable, "Observation has no depth media");
      const auto file = directMediaFile(*state_, obs.depth_message_id, *obs.depth_media, "depth");
      const auto & source = *obs.depth_storage;
      const PreparedDepthStorage storage{source.format, source.dtype, source.unit, source.scale_to_m,
        source.invalid_rule, source.source_encoding, source.source_unit, source.source_scale_to_m,
        source.min_depth_m, source.max_depth_m, source.valid_pixels, source.invalid_pixels, false};
      require(file.bytes <= DepthLimits{}.max_input_bytes, Code::invalid_observation, "Depth media exceeds read budget");
      require(detail::preparationFileSha256(file.path) == state_->media->at(obs.depth_message_id).sha256,
        Code::integrity_mismatch, "Depth media SHA-256 differs");
      auto depth = parseChecked(Code::invalid_observation,
          [&] {return detail::readStoredDepth(file.path, obs.depth_image, storage);});
      depth.alignment = obs.depth_alignment;
      depth.alignment_basis = obs.alignment_basis;
      if (depth.alignment == DepthAlignment::aligned_to_rgb) {
        depth.rgb_grid = {obs.rgb_image.width, obs.rgb_image.height, obs.rgb_image.frame_id};
      }
      checkFiles({file});
      require(detail::preparationFileSha256(file.path) == state_->media->at(obs.depth_message_id).sha256,
        Code::source_changed, "Depth media changed while reading");
      checkDatasetSnapshot(state_->files, state_->manifest_sha256);
      return {Code::ready, "", std::move(depth)};
    });
}

/** @brief 核对来源身份后才把 PreparationReader 交给旧格式调用方。 */
PreparationOpenResult ObservationDatasetReader::openSourcePreparation(const fs::path & override_root) const
{
  try {
    checkDatasetSnapshot(state_->files, state_->manifest_sha256);
    if (state_->info.direct_evidence) {
      return {PreparationReadCode::unsupported_dataset, "Direct observations contain their own evidence", nullptr};
    }
    const auto & hint = state_->info.preparation_root;
    const auto root = override_root.empty() ? state_->root / hint : override_root;
    auto opened = openPreparation(root);
    if (!opened.ok()) {return opened;}
    const auto & source = *opened.reader;
    if (source.manifestSha256() != state_->info.source_manifest_sha256 ||
      source.mapFrame() != state_->info.map_frame ||
      (state_->info.preparation_bundled && override_root.empty() && source.root() != root)) {
      return {PreparationReadCode::invalid_reference, "Preparation does not match dataset source and map frame", nullptr};
    }
    if (const auto mismatch = findSourcePairMismatch(*state_, source)) {
      return {PreparationReadCode::invalid_reference, *mismatch, nullptr};
    }
    const auto stable = source.verifyUnchanged();
    if (!stable.ok()) {return {stable.code, stable.message, nullptr};}
    checkDatasetSnapshot(state_->files, state_->manifest_sha256);
    return opened;
  } catch (const ReadError & error) {
    return {PreparationReadCode::source_changed, error.what(), nullptr};
  } catch (const std::exception & error) {
    return {PreparationReadCode::io_error, error.what(), nullptr};
  }
}

/** @brief 校验清单和索引，再发布可供调用方复用的只读 reader。 */
ObservationDatasetOpenResult openObservationDataset(const fs::path & root)
{
  /** @brief 只在所有文件和引用检查通过后交出索引。 */
  return readResult<ObservationDatasetOpenResult>([&]() -> ObservationDatasetOpenResult {
      auto state = std::make_unique<ObservationDatasetReaderState>();
      state->root = fs::canonical(root);
      const auto direct_path = state->root / detail::kDirectObservationManifest;
      const bool direct = fs::exists(fs::symlink_status(direct_path));
      const auto manifest_path = direct ? direct_path : state->root / detail::kObservationDatasetManifest;
      state->files.push_back(snapshot(manifest_path));
      const auto manifest = parseChecked(Code::invalid_manifest, [&] {return readManifest(manifest_path);});
      require(direct == manifest.info.direct_evidence, Code::invalid_manifest, "Manifest is at the wrong path");
      if (direct) {
        const auto source = state->root / detail::kDirectObservationSource;
        const auto media = state->root / detail::kDirectObservationMediaIndex;
        state->files.push_back(snapshot(state->root / detail::kObservationDatasetRows));
        state->files.push_back(snapshot(source));
        state->files.push_back(snapshot(media));
        require(detail::preparationFileSha256(source) == manifest.info.source_manifest_sha256,
          Code::integrity_mismatch, "Direct observation source digest differs");
        state->media = parseChecked(Code::integrity_mismatch, [&] {
            return detail::readSealedMediaIndex(media, manifest.hashes.at("media").get<std::string>());
          });
      } else {
        for (const auto & [name, relative] : detail::kObservationDatasetFiles) {
          state->files.push_back(snapshot(state->root / relative));
          if (!manifest.hashes.is_null() && (std::string_view(name) == "selection" || std::string_view(name) == "issues")) {
            verifyDigest(manifest, name, detail::preparationFileSha256(state->root / relative));
          }
        }
        state->batches = parseChecked(Code::invalid_batch, [&] {
            return readBatches(state->root / detail::kObservationDatasetBatches, manifest);
          });
      }
      state->rows = parseChecked(Code::invalid_observation, [&] {
          return indexObservations(
            state->root / detail::kObservationDatasetRows, manifest, state->batches, state->media);
        });
      state->info = manifest.info;
      state->info.file_integrity_verified = !manifest.hashes.is_null();
      state->manifest_sha256 = manifest.sha256;
      checkDatasetSnapshot(state->files, state->manifest_sha256);
      return {Code::ready, "", std::unique_ptr<ObservationDatasetReader>(new ObservationDatasetReader(std::move(state)))};
    });
}

}  // namespace vln_memory
