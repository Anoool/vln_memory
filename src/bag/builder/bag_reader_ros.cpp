#include <vln_memory/bag/builder/bag_reader_ros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <utility>
#include <openssl/evp.h>
#include <rclcpp/serialization.hpp>
#include <rcutils/error_handling.h>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <vln_memory/bag/builder/jsonl_writer.hpp>
#include <vln_memory/bag/builder/image_decoder_ros.hpp>
#include <vln_memory/bag/camera_info/camera_info_ros.hpp>
#include <vln_memory/bag/tf/tf_ros.hpp>

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Reader = rosbag2_cpp::readers::SequentialReader;
using RawMessage = rosbag2_storage::SerializedBagMessage;
using Digest = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;

/** @brief 建立 OpenSSL SHA-256 上下文，库错误不被当作坏消息跳过。 */
Digest newDigest()
{
  Digest digest(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    throw BagBuildError("hash_error", "Cannot initialize SHA-256");
  }
  return digest;
}

/** @brief 完成 SHA-256 为稳定十六进制文本，不使用 std::hash 的非持久语义。 */
std::string finishDigest(Digest & digest)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(digest.get(), bytes.data(), &length) != 1) {
    throw BagBuildError("hash_error", "Cannot finish SHA-256");
  }
  std::ostringstream result;
  for (unsigned int index = 0; index < length; ++index) {
    result << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(bytes[index]);
  }
  return result.str();
}

/** @brief 同步哈希借用字节；输入生命周期只需覆盖本次调用。 */
std::string bytesDigest(const std::uint8_t * data, std::size_t bytes)
{
  auto digest = newDigest();
  if (bytes > 0 && EVP_DigestUpdate(digest.get(), data, bytes) != 1) {
    throw BagBuildError("hash_error", "Cannot hash message bytes");
  }
  return finishDigest(digest);
}

/** @brief 分块哈希只读输入文件，不把整个分卷加载进内存。 */
std::string fileDigest(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {throw BagBuildError("input_error", "Cannot read " + path.string());}
  auto digest = newDigest();
  std::array<char, 65536> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (input.gcount() > 0 && EVP_DigestUpdate(
        digest.get(), buffer.data(),
        static_cast<std::size_t>(input.gcount())) != 1)
    {
      throw BagBuildError("hash_error", "Cannot hash input file");
    }
  }
  if (!input.eof()) {throw BagBuildError("input_error", "Input file read failed");}
  return finishDigest(digest);
}

/** @brief 拒绝活动 SQLite 日志，指纹不能忽略尚未落到主库的记录。 */
void rejectActiveSqlite(const fs::path & path)
{
  for (const auto * suffix : {"-wal", "-journal"}) {
    const auto sidecar = fs::path(path.string() + suffix);
    if (fs::exists(sidecar) && fs::file_size(sidecar) > 0) {
      throw BagBuildError("active_input", "Close the recording/checkpoint its SQLite journal first");
    }
  }
}

/**
 * @brief Humble 原生 metadata 时间解码使用 uint64，无法读回 Writer 写出的负起点。
 * @note 仅在内存副本临时替换这些字段供官方解析，再恢复完整有符号起点；从不改输入文件。
 */
class SignedMetadataIo : public rosbag2_storage::MetadataIo
{
public:
  /** @brief 复用官方结构解析，唯一兼容调整是根/分卷的 signed starting_time。 */
  rosbag2_storage::BagMetadata read_metadata(const std::string & uri) override
  {
    auto info = YAML::LoadFile((fs::path(uri) / "metadata.yaml").string())["rosbag2_bagfile_information"];
    const auto start_ns = info["starting_time"]["nanoseconds_since_epoch"].as<std::int64_t>();
    if (start_ns < 0) {info["starting_time"]["nanoseconds_since_epoch"] = 0;}
    std::vector<std::int64_t> file_starts;
    if (info["version"].as<int>() >= 5) {
      for (auto entry : info["files"]) {
        const auto file_ns = entry["starting_time"]["nanoseconds_since_epoch"].as<std::int64_t>();
        file_starts.push_back(file_ns);
        if (file_ns < 0) {entry["starting_time"]["nanoseconds_since_epoch"] = 0;}
      }
    }
    auto metadata = rosbag2_storage::MetadataIo::deserialize_metadata(YAML::Dump(info));
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    metadata.starting_time = TimePoint(std::chrono::nanoseconds(start_ns));
    if (file_starts.size() != metadata.files.size()) {
      throw BagBuildError("invalid_metadata", "File time metadata count mismatch");
    }
    for (std::size_t index = 0; index < file_starts.size(); ++index) {
      metadata.files[index].starting_time = TimePoint(std::chrono::nanoseconds(file_starts[index]));
    }
    return metadata;
  }
};

/** @brief 使用官方可注入 MetadataIo 的构造入口；存储和序列化仍是标准实现。 */
Reader makeReader()
{
  return Reader(
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::make_shared<rosbag2_cpp::SerializationFormatConverterFactory>(), std::make_unique<SignedMetadataIo>());
}

/** @brief 官方只读打开，显式最小记录起点保留负 bag time_stamp。 */
void openReader(const BagBuildOptions & options, Reader & reader)
{
  rosbag2_storage::StorageOptions storage;
  storage.uri = fs::canonical(options.bag_uri).string();
  storage.storage_id = options.storage_id;
  try {
    reader.open(storage, {"", ""});
    reader.seek(std::numeric_limits<rcutils_time_point_value_t>::min());
  } catch (const std::bad_alloc &) {
    throw;
  } catch (const std::exception & error) {
    throw BagBuildError("bag_open_failed", error.what());
  }
  rosbag2_storage::StorageFilter filter;
  filter.topics = {options.rgb_camera.image_topic, options.depth_camera.image_topic,
    options.rgb_camera.camera_info_topic, options.depth_camera.camera_info_topic,
    options.tf_topic, options.tf_static_topic};
  std::sort(filter.topics.begin(), filter.topics.end());
  filter.topics.erase(std::unique(filter.topics.begin(), filter.topics.end()), filter.topics.end());
  reader.set_filter(filter);
}

/** @brief 验证目录 bag 完整元数据或独立 .db3 范围，不根据后缀猜测任意格式。 */
void checkInput(const BagBuildOptions & options)
{
  const fs::path input(options.bag_uri);
  if (fs::is_directory(input)) {
    if (!fs::is_regular_file(input / "metadata.yaml")) {
      throw BagBuildError("missing_metadata", "Directory input must contain metadata.yaml");
    }
  } else if (!fs::is_regular_file(input) || input.extension() != ".db3") {
    throw BagBuildError("unsupported_input", "Expected SQLite bag directory or single .db3");
  } else {rejectActiveSqlite(input);}
}

/** @brief 按 metadata 原有顺序解析分卷并生成内容身份；缺卷或压缩提前拒绝。 */
void inspectSources(const BagBuildOptions & options, const Reader & reader, BagScan & scan)
{
  const auto & metadata = reader.get_metadata();
  if (metadata.storage_identifier != "sqlite3" || !metadata.compression_format.empty() ||
    (!metadata.compression_mode.empty() && metadata.compression_mode != "none"))
  {
    throw BagBuildError("unsupported_input", "Only uncompressed sqlite3 input is supported");
  }
  const auto input = fs::canonical(options.bag_uri);
  auto base = input.parent_path();
  scan.input_scope = "single_storage_file";
  if (fs::is_directory(input)) {
    scan.input_scope = "metadata_directory";
    base = input;
    if (metadata.version < 4) {base = input.parent_path();}
    scan.metadata_sha256 = fileDigest(input / "metadata.yaml");
  }
  std::string identity;
  std::set<std::string> unique_paths;
  for (const auto & file : metadata.relative_file_paths) {
    fs::path path(file);
    if (!path.is_absolute()) {path = base / path;}
    if (!fs::is_regular_file(path) || path.extension() != ".db3") {
      throw BagBuildError("missing_storage_file", "Missing/unsupported storage file " + path.string());
    }
    path = fs::canonical(path);
    if (!unique_paths.insert(path.string()).second) {throw BagBuildError("invalid_metadata", "Repeated storage file");}
    rejectActiveSqlite(path);
    BagSourceFile source{path.string(), fs::file_size(path), fileDigest(path)};
    identity += std::to_string(source.bytes) + ":" + source.sha256 + "\n";
    scan.source_files.push_back(std::move(source));
  }
  if (scan.source_files.empty()) {throw BagBuildError("empty_bag", "No storage files listed");}
  scan.bag_id = bytesDigest(reinterpret_cast<const std::uint8_t *>(identity.data()), identity.size());
}

/** @brief 确认类型一致、必需图像/标定存在；TF 可缺，但会在历史链检查中报告。 */
void inspectTopics(const BagBuildOptions & options, const Reader & reader, BagScan & scan)
{
  std::map<std::string, std::string> expected{{options.rgb_camera.image_topic, "sensor_msgs/msg/Image"},
    {options.depth_camera.image_topic, "sensor_msgs/msg/Image"},
    {options.rgb_camera.camera_info_topic, "sensor_msgs/msg/CameraInfo"},
    {options.depth_camera.camera_info_topic, "sensor_msgs/msg/CameraInfo"},
    {options.tf_topic, "tf2_msgs/msg/TFMessage"}, {options.tf_static_topic, "tf2_msgs/msg/TFMessage"}};
  std::set<std::string> present;
  std::uint64_t selected_count = 0;
  for (const auto & topic : reader.get_metadata().topics_with_message_count) {
    const auto & value = topic.topic_metadata;
    if (!present.insert(value.name).second) {throw BagBuildError("invalid_metadata", "Duplicate topic name");}
    const auto selected = expected.find(value.name);
    if (selected != expected.end()) {
      const bool image_topic = value.name == options.rgb_camera.image_topic ||
        value.name == options.depth_camera.image_topic;
      bool correct_type = selected->second == value.type;
      if (image_topic && value.type == "sensor_msgs/msg/CompressedImage") {correct_type = true;}
      if (!correct_type || value.serialization_format != "cdr") {
        throw BagBuildError("topic_type_mismatch", "Type/serialization mismatch for " + value.name);
      }
      if (topic.message_count > options.max_messages - selected_count) {
        throw BagBuildError("limit_exceeded", "Declared selected message count exceeds limit");
      }
      selected_count += topic.message_count;
    }
    scan.topics.push_back(
      {value.name, value.type, value.serialization_format, value.offered_qos_profiles,
        topic.message_count, 0, selected != expected.end()});
  }
  for (const auto & required : {options.rgb_camera.image_topic, options.depth_camera.image_topic,
      options.rgb_camera.camera_info_topic, options.depth_camera.camera_info_topic})
  {
    if (present.count(required) == 0) {throw BagBuildError("missing_topic", "Missing required topic " + required);}
  }
  std::sort(
    scan.topics.begin(), scan.topics.end(), [](const auto & left, const auto & right) {
      return left.name < right.name;
    });
}

/** @brief 同步反序列化，只有此步骤的消息格式错误可按单条报告；分配失败直接上抛。 */
template<typename Message>
std::optional<Message> deserialize(const RawMessage & raw, std::string & error)
{
  try {
    rclcpp::SerializedMessage serialized(*raw.serialized_data);
    Message message;
    rclcpp::Serialization<Message> decoder;
    // 一次调用独立消费底层错误状态，成功的类型支持回退也可能留下诊断。
    if (rcutils_error_is_set()) {rcutils_reset_error();}
    decoder.deserialize_message(&serialized, &message);
    if (rcutils_error_is_set()) {rcutils_reset_error();}
    return message;
  } catch (const std::bad_alloc &) {
    throw;
  } catch (const std::exception & exception) {
    error = exception.what();
    if (error.empty()) {error = "Message deserialization failed";}
    rcutils_reset_error();
  }
  return std::nullopt;
}

/** @brief 源映射状态只在一次扫描内拥有；跨遍稳定性依赖不变的文件/类型/读取约定。 */
struct SourceCursor
{
  std::map<std::string, std::size_t> files;
  std::map<std::string, std::size_t> topics;
  std::map<std::pair<std::size_t, std::string>, std::uint64_t> ordinals;
  std::uint64_t order{0};
  std::string current_file;
  std::size_t current_file_index{0};
};

/** @brief 一次建立查找表，不为每条消息线性扫描分卷和话题表。 */
SourceCursor makeCursor(const BagScan & scan)
{
  SourceCursor cursor;
  for (std::size_t index = 0; index < scan.source_files.size(); ++index) {
    cursor.files.emplace(scan.source_files[index].path, index);
  }
  for (std::size_t index = 0; index < scan.topics.size(); ++index) {
    cursor.topics.emplace(scan.topics[index].name, index);
  }
  return cursor;
}

/** @brief 构造稳定来源并检查消息预算；get_current_file 在 has_next 分卷切换后读取。 */
BagMessageRef sourceReference(
  const Reader & reader, const RawMessage & raw, const BagScan & scan, SourceCursor & cursor,
  const BagBuildOptions & options)
{
  if (!raw.serialized_data || raw.serialized_data->buffer_length > options.max_serialized_bytes ||
    (raw.serialized_data->buffer_length > 0 && raw.serialized_data->buffer == nullptr) ||
    raw.serialized_data->buffer_length > raw.serialized_data->buffer_capacity)
  {
    throw BagBuildError("limit_exceeded", "Invalid/oversized serialized message buffer");
  }
  if (cursor.order >= options.max_messages) {throw BagBuildError("limit_exceeded", "Message count limit exceeded");}
  const auto current_file = reader.get_current_file();
  if (current_file != cursor.current_file) {
    const auto file = cursor.files.find(fs::canonical(current_file).string());
    if (file == cursor.files.end()) {throw BagBuildError("source_changed", "Unexpected storage file");}
    cursor.current_file = current_file;
    cursor.current_file_index = file->second;
  }
  const auto topic = cursor.topics.find(raw.topic_name);
  if (topic == cursor.topics.end() || !scan.topics[topic->second].selected) {
    throw BagBuildError("source_changed", "Unexpected source file or topic during scan");
  }
  BagMessageRef source;
  source.topic = raw.topic_name;
  source.raw_cdr_saved = options.save_raw_cdr;
  source.file_index = cursor.current_file_index;
  source.topic_ordinal = cursor.ordinals[{source.file_index, source.topic}]++;
  source.message_order = cursor.order++;
  source.bag_record_time_ns = raw.time_stamp;
  source.serialized_bytes = raw.serialized_data->buffer_length;
  source.payload_sha256 = bytesDigest(raw.serialized_data->buffer, source.serialized_bytes);
  source.message_id = "m_" + scan.bag_id + "_f" + std::to_string(source.file_index) + "_t" +
    std::to_string(topic->second) + "_" + std::to_string(source.topic_ordinal);
  return source;
}

/** @brief 添加源关联问题，保留原始字节；问题最终统一写 reports/。 */
void addIssue(
  BagScan & scan, const BagMessageRef & source, const std::string & stage,
  const std::string & code, const std::string & message)
{
  scan.issues.push_back({stage, code, message, source.message_id, {}});
}

/** @brief 只处理图像时间重排预算，不能从回退推断并跨新的 clock epoch 继续。 */
void checkImageOrder(
  const BagImageRecord & image, std::map<std::string, std::int64_t> & high_water,
  const BagBuildOptions & options)
{
  if (image.stamp.nanosec >= 1000000000U) {return;}
  const auto time_ns = static_cast<std::int64_t>(image.stamp.sec) * 1000000000LL + image.stamp.nanosec;
  const auto found = high_water.find(image.source.topic);
  if (found != high_water.end() && time_ns < found->second && found->second - time_ns > options.max_header_reorder_ns) {
    throw BagBuildError("ambiguous_time_segment", "Image header rollback exceeds declared reordering allowance");
  }
  if (found == high_water.end() || time_ns > found->second) {high_water[image.source.topic] = time_ns;}
}

/** @brief 复制真实消息 header；即使压缩负载损坏，仍可保留有效采样时刻。 */
void copyImageHeader(const std_msgs::msg::Header & header, BagImageRecord & record)
{
  record.stamp = {header.stamp.sec, header.stamp.nanosec};
  record.frame_id = header.frame_id;
}

/** @brief 原始 Image 只保存布局，反序列化缓冲在本函数退出时释放。 */
void indexRawImage(const RawMessage & raw, BagImageRecord & record)
{
  const auto image = deserialize<sensor_msgs::msg::Image>(raw, record.deserialization_error);
  if (!image) {return;}
  copyImageHeader(image->header, record);
  record.width = image->width;
  record.height = image->height;
  record.encoding = image->encoding;
  record.step = image->step;
  record.is_bigendian = image->is_bigendian;
  record.data_bytes = image->data.size();
}

/** @brief 首遍仅预检压缩头尺寸/预算，不解压；负载错误与 CDR 错误分开保存。 */
void indexCompressedImage(const RawMessage & raw, BagImageRecord & record, const BagBuildOptions & options)
{
  const auto image = deserialize<sensor_msgs::msg::CompressedImage>(raw, record.deserialization_error);
  if (!image) {return;}
  copyImageHeader(image->header, record);
  record.compressed_format = image->format;
  record.data_bytes = image->data.size();
  const bool depth_stream = record.source.topic == options.depth_camera.image_topic;
  const auto role = depth_stream ? BagImageRole::depth : BagImageRole::rgb;
  const auto max_pixels = depth_stream ? options.depth_limits.max_pixels : options.rgb_limits.max_pixels;
  const auto max_bytes = depth_stream ? options.depth_limits.max_input_bytes : options.rgb_limits.max_input_bytes;
  const auto info = inspectCompressedImage(*image, role, max_pixels, max_bytes);
  record.width = info.width;
  record.height = info.height;
  record.encoding = info.encoding;
  if (!info.ok()) {
    record.payload_error_code = info.code;
    record.payload_error = info.message;
  }
}

/** @brief 两种消息共享原始 header 和来源索引；坏 CDR 不冒充零时刻图像。 */
void indexImage(
  const RawMessage & raw, const BagMessageRef & source, const std::string & message_type, BagScan & scan,
  const BagBuildOptions & options, std::map<std::string, std::int64_t> & high_water)
{
  if (scan.images.size() >= options.max_frames) {throw BagBuildError("limit_exceeded", "Image index limit exceeded");}
  BagImageRecord record;
  record.source = source;
  record.message_type = message_type;
  if (message_type == "sensor_msgs/msg/CompressedImage") {indexCompressedImage(raw, record, options);} else {
    indexRawImage(raw, record);
  }
  if (!record.deserialization_error.empty()) {
    addIssue(scan, source, "deserialize", "deserialize_failed", record.deserialization_error);
  } else {checkImageOrder(record, high_water, options);}
  scan.images.push_back(std::move(record));
}

/** @brief 保存所有标定（包含坏候选）及来源，CDR 副本按保存策略生成。 */
void indexCameraInfo(
  const RawMessage & raw, const BagMessageRef & source, BagScan & scan,
  const BagBuildOptions & options, OutputFiles & output, bool write_preparation)
{
  if (scan.calibrations.size() >= options.max_calibrations) {
    throw BagBuildError("limit_exceeded", "Calibration limit exceeded");
  }
  std::string error;
  const auto message = deserialize<sensor_msgs::msg::CameraInfo>(raw, error);
  if (!message) {
    addIssue(scan, source, "deserialize", "deserialize_failed", error);
    throw BagBuildError(
            "unreadable_calibration",
            "Cannot establish calibration version boundary: " + source.message_id);
  }
  BagCameraInfoRecord record;
  record.source = source;
  record.data = copyRosCameraInfo(*message, source.topic);
  const auto original = decodeCameraInfo(record.data);
  record.validation_code = original.code;
  record.validation_message = original.message;
  const auto * binding = &options.depth_camera;
  if (source.topic == options.rgb_camera.camera_info_topic) {binding = &options.rgb_camera;}
  record.compatibility = binding->compatibility;
  auto normalized = normalizeCameraInfo(record.data, record.compatibility);
  record.normalized_code = normalized.code;
  record.normalization_applied = normalized.normalization_applied;
  record.normalized = std::move(normalized.info);
  if (!record.normalized) {
    addIssue(scan, source, "camera_info", cameraInfoErrorCodeName(normalized.code), normalized.message);
  }
  if (write_preparation) {writeJsonLine(output, "camera_info.jsonl", cameraInfoJson(record));}
  scan.calibrations.push_back(std::move(record));
}

/** @brief 原始 TF 条目导出，NaN/Inf 由 JSON 库写 null 并标记；原字节需源消息对照。 */
nlohmann::json transformJson(
  const BagMessageRef & source, const geometry_msgs::msg::TransformStamped & value,
  std::size_t index, TfInsertCode code, bool is_static)
{
  const auto & translation = value.transform.translation;
  const auto & rotation = value.transform.rotation;
  const std::array<double, 3> xyz{translation.x, translation.y, translation.z};
  const std::array<double, 4> xyzw{rotation.x, rotation.y, rotation.z, rotation.w};
  bool nonfinite = false;
  for (const auto number : xyz) {
    nonfinite = nonfinite || !std::isfinite(number);
  }
  for (const auto number : xyzw) {
    nonfinite = nonfinite || !std::isfinite(number);
  }
  return {{"source", sourceJson(source)}, {"transform_index", index}, {"static", is_static},
    {"stamp", stampJson({value.header.stamp.sec, value.header.stamp.nanosec})},
    {"parent_frame", value.header.frame_id}, {"child_frame", value.child_frame_id},
    {"translation_m", xyz}, {"rotation_xyzw", xyzw}, {"insert_code", tfInsertCodeName(code)},
    {"source_nonfinite", nonfinite}};
}

/** @brief 复用已有 TF 适配和冲突规则，逐条输出状态；资源失败不能按坏帧忽略。 */
void indexTf(
  const RawMessage & raw, const BagMessageRef & source, BagScan & scan,
  const BagBuildOptions & options, OutputFiles & output, bool write_preparation)
{
  std::string error;
  const auto message = deserialize<tf2_msgs::msg::TFMessage>(raw, error);
  if (!message) {
    throw BagBuildError("unreadable_tf", "Cannot safely establish TF history: " + source.message_id + ": " + error);
  }
  if (message->transforms.size() > options.max_tf_entries - scan.tf_entries) {
    throw BagBuildError("limit_exceeded", "Raw TF entry limit exceeded");
  }
  scan.tf_entries += message->transforms.size();
  const auto report = ingestRosTf(
    scan.tf_index, *message, source.topic, source.message_id,
    {options.tf_topic, options.tf_static_topic});
  scan.tf_inserted += report.inserted;
  scan.tf_duplicates += report.duplicates;
  for (const auto & issue : report.issues) {
    if (issue.code == TfInsertCode::limit_exceeded) {throw BagBuildError("limit_exceeded", issue.message);}
    addIssue(scan, source, "tf", tfInsertCodeName(issue.code), issue.message);
  }
  if (report.entry_codes.size() != message->transforms.size()) {
    throw BagBuildError("tf_adapter_error", "TF adapter did not process the complete message");
  }
  if (!write_preparation) {return;}
  for (std::size_t index = 0; index < message->transforms.size(); ++index) {
    writeJsonLine(
      output, "tf.jsonl", transformJson(
        source, message->transforms[index], index,
        report.entry_codes[index], source.topic == options.tf_static_topic));
  }
}

/** @brief 顺序扫描调度，反序列化的各类消息在辅助函数结束时释放。 */
void scanMessages(
  const BagBuildOptions & options, Reader & reader, BagScan & scan, OutputFiles & output,
  bool write_preparation)
{
  auto cursor = makeCursor(scan);
  std::map<std::string, std::int64_t> high_water;
  while (reader.has_next()) {
    const auto raw = reader.read_next();
    if (!raw) {throw BagBuildError("bag_read_failed", "Reader returned a null message");}
    auto source = sourceReference(reader, *raw, scan, cursor, options);
    if (options.save_raw_cdr) {
      output.write("evidence/raw/" + source.message_id + ".cdr",
        raw->serialized_data->buffer, source.serialized_bytes);
    }
    if (write_preparation) {writeJsonLine(output, "messages.jsonl", sourceJson(source));}
    ++scan.topics[cursor.topics.at(source.topic)].scanned_count;
    scan.messages.push_back(source);
    if (source.topic == options.rgb_camera.image_topic || source.topic == options.depth_camera.image_topic) {
      indexImage(*raw, source, scan.topics[cursor.topics.at(source.topic)].type, scan, options, high_water);
    } else if (source.topic == options.tf_topic || source.topic == options.tf_static_topic) {
      indexTf(*raw, source, scan, options, output, write_preparation);
    } else {indexCameraInfo(*raw, source, scan, options, output, write_preparation);}
  }
  for (const auto & topic : scan.topics) {
    if (topic.selected && topic.declared_count != topic.scanned_count) {
      throw BagBuildError("message_count_mismatch", "Declared/read count mismatch: " + topic.name);
    }
  }
}

/** @brief 同一位置必须有完全相同的来源/字节，不能用近似时间替代。 */
void checkSameSource(const BagMessageRef & expected, const BagMessageRef & actual)
{
  if (expected.message_id != actual.message_id || expected.topic != actual.topic ||
    expected.bag_record_time_ns != actual.bag_record_time_ns || expected.serialized_bytes != actual.serialized_bytes ||
    expected.payload_sha256 != actual.payload_sha256)
  {
    throw BagBuildError("source_changed", "Source sequence/content changed between passes");
  }
}

/** @brief 第二遍逐帧解压；借用指针仅在 consumer 内有效，失败保留 header/来源。 */
void consumeCompressedImage(
  const RawMessage & raw, BagImageRecord record, const BagBuildOptions & options, const BagImageConsumer & consumer)
{
  std::string error;
  const auto message = deserialize<sensor_msgs::msg::CompressedImage>(raw, error);
  if (!message) {throw BagBuildError("source_changed", "Previously readable CompressedImage failed: " + error);}
  const bool depth_stream = record.source.topic == options.depth_camera.image_topic;
  const auto role = depth_stream ? BagImageRole::depth : BagImageRole::rgb;
  const auto max_pixels = depth_stream ? options.depth_limits.max_pixels : options.rgb_limits.max_pixels;
  const auto max_bytes = depth_stream ? options.depth_limits.max_input_bytes : options.rgb_limits.max_input_bytes;
  const auto decoded = decodeCompressedImage(*message, role, max_pixels, max_bytes);
  if (!decoded.ok()) {
    record.payload_error_code = decoded.code;
    record.payload_error = decoded.message;
  }
  consumer(record, decoded.image.get());
}

/** @brief 跨阶段快速核对大小/日志/metadata；最终再用完整 SHA 校验所有字节。 */
void checkSourceStats(const BagBuildOptions & options, const BagScan & scan)
{
  for (const auto & file : scan.source_files) {
    rejectActiveSqlite(file.path);
    if (!fs::is_regular_file(file.path) || fs::file_size(file.path) != file.bytes) {
      throw BagBuildError("source_changed", "Input storage file size changed");
    }
  }
  if (!scan.metadata_sha256.empty() &&
    fileDigest(fs::path(options.bag_uri) / "metadata.yaml") != scan.metadata_sha256)
  {
    throw BagBuildError("source_changed", "Input metadata changed");
  }
}

}  // namespace

/** @brief 输入不变检查包含整个主库和 metadata，活动 journal 不属于可消费输入。 */
void verifyBagSources(const BagBuildOptions & options, const BagScan & scan)
{
  checkSourceStats(options, scan);
  for (const auto & file : scan.source_files) {
    if (fileDigest(file.path) != file.sha256) {throw BagBuildError("source_changed", "Input storage file changed");}
  }
}

/** @brief 完整扫描后返回稳定索引，失败保留工作目录而不发布 manifest。 */
BagScan scanBag(const BagBuildOptions & options, OutputFiles & output, bool write_preparation)
{
  const auto errors = validateBagBuildOptions(options);
  if (!errors.empty()) {throw BagBuildError("invalid_config", errors.front());}
  checkInput(options);
  std::string metadata_before_open;
  if (fs::is_directory(options.bag_uri)) {
    metadata_before_open = fileDigest(fs::path(options.bag_uri) / "metadata.yaml");
  }
  auto reader = makeReader();
  openReader(options, reader);
  BagScan scan;
  scan.tf_index = TfIndex({options.clock_id, options.segment_id}, options.tf_limits);
  inspectSources(options, reader, scan);
  if (metadata_before_open != scan.metadata_sha256) {
    throw BagBuildError("source_changed", "Metadata changed while opening");
  }
  inspectTopics(options, reader, scan);
  scanMessages(options, reader, scan, output, write_preparation);
  reader.close();
  verifyBagSources(options, scan);
  return scan;
}

/** @brief 第二遍只临时解码当前图像；坏 CDR/负载用 nullptr 回调，不静默改配。 */
void readBagImages(const BagBuildOptions & options, const BagScan & scan, const BagImageConsumer & consumer)
{
  if (!consumer) {throw BagBuildError("invalid_consumer", "Image consumer is required");}
  checkSourceStats(options, scan);
  auto reader = makeReader();
  openReader(options, reader);
  auto cursor = makeCursor(scan);
  std::size_t message_index = 0;
  std::size_t image_index = 0;
  while (reader.has_next()) {
    const auto raw = reader.read_next();
    if (!raw) {throw BagBuildError("bag_read_failed", "Reader returned a null message");}
    const auto source = sourceReference(reader, *raw, scan, cursor, options);
    if (message_index >= scan.messages.size()) {throw BagBuildError("source_changed", "Extra message in second pass");}
    checkSameSource(scan.messages[message_index++], source);
    if (source.topic != options.rgb_camera.image_topic && source.topic != options.depth_camera.image_topic) {continue;}
    if (image_index >= scan.images.size() || scan.images[image_index].source.message_id != source.message_id) {
      throw BagBuildError("source_changed", "Image index does not match source sequence");
    }
    const auto & record = scan.images[image_index++];
    if (!record.deserialization_error.empty() || !record.payload_error_code.empty()) {
      consumer(record, nullptr);
      continue;
    }
    if (record.message_type == "sensor_msgs/msg/CompressedImage") {
      consumeCompressedImage(*raw, record, options, consumer);
      continue;
    }
    std::string error;
    const auto image = deserialize<sensor_msgs::msg::Image>(*raw, error);
    if (!image) {throw BagBuildError("source_changed", "Previously readable Image failed: " + error);}
    consumer(record, &*image);
  }
  if (message_index != scan.messages.size() || image_index != scan.images.size()) {
    throw BagBuildError("source_changed", "Second pass ended before the first pass index");
  }
  reader.close();
  verifyBagSources(options, scan);
}

}  // namespace vln_memory
