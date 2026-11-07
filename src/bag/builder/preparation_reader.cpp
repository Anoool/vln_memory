#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>
#include "preparation_metadata.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <iterator>
#include <tuple>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

namespace vln_memory
{
namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;
using Code = PreparationReadCode;
using Records = std::map<std::string, Json>;

using ReadError = detail::PreparationMetadataError;

/** @brief 用具体的契约错误中止当前配对读取。 */
void require(bool condition, Code code, std::string_view message)
{
  if (!condition) {throw ReadError(code, std::string(message));}
}

/** @brief 解析非负整数或时间戳字段，拒绝浮点、符号和范围错误。 */
template<typename T>
T integer(const Json & value)
{
  const auto lower = static_cast<long double>(std::numeric_limits<T>::lowest());
  const auto upper = static_cast<long double>(std::numeric_limits<T>::max());
  require(value.is_number_integer() && value.get<long double>() >= lower &&
    value.get<long double>() <= upper, Code::invalid_metadata, "Integer field is out of range");
  return value.get<T>();
}

/** @brief 保留原始秒和纳秒并核对纳秒范围。 */
SensorStamp readStamp(const Json & value)
{
  const auto nanos = integer<std::uint32_t>(value.at("nanosec"));
  require(nanos < 1000000000U, Code::invalid_metadata, "Invalid timestamp nanoseconds");
  return {integer<std::int32_t>(value.at("sec")), nanos};
}

/** @brief 验证已保存的时间戳字段，区分有效时间与导出缺项。 */
bool sameStamp(const Json & first, const Json & second)
{
  const auto a = readStamp(first);
  const auto b = readStamp(second);
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

/** @brief 按整数纳秒排序 TF 端点，不转换时钟基准。 */
std::int64_t stampNs(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

/** @brief 打开普通文件，失败时将路径纳入错误消息。 */
std::ifstream input(const fs::path & path)
{
  std::ifstream file(path, std::ios::binary);
  require(file.is_open(), Code::io_error, "Cannot open " + path.string());
  return file;
}

/** @brief 解出相对路径并校验规范化目标在同一数据集目录。 */
fs::path member(const fs::path & root, const std::string & relative)
{
  const fs::path path(relative);
  require(!path.empty() && path.is_relative() && relative.find('\0') == std::string::npos, Code::invalid_reference, "Expected a relative evidence path");
  const auto resolved = fs::weakly_canonical(root / path);
  const auto rel = resolved.lexically_relative(root);
  require(!rel.empty() && *rel.begin() != ".." && fs::is_regular_file(resolved),
    Code::invalid_reference, "Missing or out-of-dataset file: " + relative);
  return resolved;
}

/** @brief 扫描 JSONL 一次，仅保留被当前配对引用的行并拒绝重复键。 */
template<typename Key>
Records selectRows(const fs::path & path, const std::set<std::string> & keys, Key key)
{
  Records selected;
  if (keys.empty()) {return selected;}
  auto file = input(path);
  for (std::string line; detail::readPreparationLine(file, line);) {
    auto row = detail::parsePreparationJson(line);
    const auto id = key(row);
    if (keys.count(id) != 0) {
      require(row.at("schema_version") == "bag_preparation/1", Code::invalid_metadata,
        "Unexpected schema in " + path.filename().string());
      require(selected.emplace(id, std::move(row)).second, Code::invalid_reference,
        "Duplicate reference: " + id);
    }
  }
  require(file.eof() && !file.bad(), Code::io_error, "Cannot read " + path.string());
  return selected;
}

/** @brief 取得已索引的记录或返回断引用。 */
const Json & referenced(const Records & records, const std::string & id)
{
  const auto match = records.find(id);
  require(match != records.end(), Code::invalid_reference, "Missing reference: " + id);
  return match->second;
}

/** @brief 确认记录身份在 manifest 的 bag_id 命名空间内。 */
void checkSource(const std::string & id, const std::string & bag_id)
{
  require(id.rfind("m_" + bag_id + "_", 0) == 0, Code::invalid_reference, "Foreign source ID: " + id);
}

/** @brief 把导出的 TF 端点引用归一为同一 JSONL 查找键。 */
std::string tfKey(const std::string & id, const Json & index)
{
  return id + ":" + std::to_string(integer<std::size_t>(index));
}

/** @brief 保存元数据文件的位置与常规变更依据。 */
struct SourceFile
{
  fs::path path;
  std::uintmax_t bytes{0};
  fs::file_time_type modified;
};

/** @brief 一个 JSONL 行的二进制位置，长度不含换行符。 */
struct RowLocation
{
  std::streamoff offset{0};
  std::size_t bytes{0};
  std::size_t line{0};
};

/** @brief 单个文件的只读 ID 索引，完整 JSON 由逐对读取阶段解析。 */
struct JsonlIndex
{
  SourceFile source;
  std::map<std::string, RowLocation> rows;
};

/** @brief 取得根目录内文件的规范路径、大小和修改时间。 */
SourceFile sourceFile(const fs::path & root, const std::string & name)
{
  const auto path = member(root, name);
  return {path, fs::file_size(path), fs::last_write_time(path)};
}

/** @brief 检查路径、大小和修改时间，文件消失或变化时报告 source_changed。 */
void checkSourceFile(const fs::path & root, const std::string & name, const SourceFile & source)
{
  const auto message = "Source file changed: " + (root / name).string();
  std::error_code error;
  const auto path = fs::canonical(root / name, error);
  require(!error && path == source.path, Code::source_changed, message);
  const auto bytes = fs::file_size(path, error);
  require(!error && bytes == source.bytes, Code::source_changed, message);
  const auto modified = fs::last_write_time(path, error);
  require(!error && modified == source.modified, Code::source_changed, message);
}

/** @brief 保存已由 member 解析的媒体或 CDR 状态。 */
PreparedFileSnapshot snapshotFile(const fs::path & root, const fs::path & path)
{
  return {path.lexically_relative(root), fs::file_size(path), fs::last_write_time(path)};
}

/** @brief 核对规范根相对路径和文件状态，用于读取与发布边界。 */
void checkFileSnapshots(const fs::path & root, const std::vector<PreparedFileSnapshot> & files)
{
  for (const auto & file : files) {
    const auto & relative = file.relative_path;
    const auto name = relative.generic_string();
    require(!relative.empty() && relative.is_relative() && relative == relative.lexically_normal() &&
      name.find('\0') == std::string::npos && *relative.begin() != "..",
      Code::invalid_reference, "Invalid source snapshot path");
    checkSourceFile(root, name, {root / relative, file.bytes, file.modified});
  }
}

/** @brief 逐行建立 ID 到偏移的索引，校验所有行的 schema 与键唯一性。 */
template<typename Key>
JsonlIndex indexRows(const fs::path & root, const std::string & name, Key key)
{
  JsonlIndex index;
  index.source = sourceFile(root, name);
  require(index.source.bytes <= static_cast<std::uintmax_t>(
    std::numeric_limits<std::streamoff>::max()), Code::invalid_metadata, "Metadata file is too large");
  auto file = input(index.source.path);
  std::size_t line_number = 0;
  for (std::string line;;) {
    const auto offset = file.tellg();
    if (!detail::readPreparationLine(file, line)) {break;}
    ++line_number;
    const auto context = index.source.path.string() + ":" + std::to_string(line_number) + ": ";
    try {
      require(offset >= std::streampos(0) &&
        line.size() <= static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()),
        Code::invalid_metadata, "JSONL offset or length is out of range");
      auto row = detail::parsePreparationJson(line);
      require(row.at("schema_version") == "bag_preparation/1", Code::invalid_metadata,
        "Unexpected schema");
      const auto id = key(row);
      require(!id.empty(), Code::invalid_reference, "Empty record ID");
      const RowLocation location{static_cast<std::streamoff>(offset), line.size(), line_number};
      require(index.rows.emplace(id, location).second, Code::invalid_reference,
        "Duplicate reference: " + id);
    } catch (const ReadError & error) {
      throw ReadError(error.code, context + error.what());
    } catch (const Json::exception & error) {
      throw ReadError(Code::invalid_metadata, context + error.what());
    } catch (const std::logic_error & error) {
      throw ReadError(Code::invalid_metadata, context + error.what());
    }
    if (file.eof()) {break;}
  }
  require(file.eof() && !file.bad(), Code::io_error, "Cannot read " + index.source.path.string());
  checkSourceFile(root, name, index.source);
  return index;
}

/** @brief 按文件偏移读取所需行，缺失引用由配对组装阶段报告。 */
template<typename Key>
Records indexedRows(const JsonlIndex & index, const std::set<std::string> & keys, Key key)
{
  Records selected;
  if (keys.empty()) {return selected;}
  auto file = input(index.source.path);
  for (const auto & id : keys) {
    const auto found = index.rows.find(id);
    if (found == index.rows.end()) {continue;}
    const auto & location = found->second;
    const auto context = index.source.path.string() + ":" + std::to_string(location.line);
    file.seekg(location.offset);
    std::string line(location.bytes, '\0');
    require(static_cast<bool>(file.read(line.data(), static_cast<std::streamsize>(line.size()))),
      Code::io_error, "Cannot read indexed row: " + context);
    auto row = detail::parsePreparationJson(line);
    require(key(row) == id && row.at("schema_version") == "bag_preparation/1",
      Code::invalid_reference, "Indexed row identity differs: " + context);
    selected.emplace(id, std::move(row));
  }
  return selected;
}

/** @brief 读取现有 v5/v6 已发布 manifest，保持单对入口的校验契约。 */
Json readManifest(const fs::path & root, std::string & fingerprint)
{
  const auto bytes = detail::readPreparationDocument(member(root, "bag_manifest.json"));
  auto manifest = detail::parsePreparationJson(bytes);
  fingerprint = detail::preparationSha256(bytes);
  const auto version = manifest.at("builder_version");
  require(manifest.at("artifact_kind") == "bag_preparation" &&
    manifest.at("schema_version") == "bag_preparation/1" && (version == 5 || version == 6 || version == 7) &&
    (manifest.at("status") == "completed" || manifest.at("status") == "completed_with_issues"),
    Code::unsupported_dataset, "Expected published BagBuilder v5, v6 or v7 preparation");
  if (version == 7) {
    require(manifest.at("effective_options").at("save_raw_cdr").is_boolean(),
      Code::invalid_metadata, "v7 requires boolean save_raw_cdr");
  }
  return manifest;
}

/** @brief 历史版本保存 CDR，v7 明确声明实际保存策略。 */
bool savesRawCdr(const Json & manifest)
{
  if (manifest.at("builder_version") == 7) {
    return manifest.at("effective_options").at("save_raw_cdr").get<bool>();
  }
  return true;
}

/** @brief 将已有文件、格式和引用错误映射到公开结果，保持失败时空值。 */
template<typename Result, typename Operation>
Result readResult(Operation operation)
{
  try {
    return operation();
  } catch (const ReadError & error) {
    return {error.code, error.what(), {}};
  } catch (const fs::filesystem_error & error) {
    return {Code::io_error, error.what(), {}};
  } catch (const cv::Exception & error) {
    return {Code::invalid_media, error.what(), {}};
  } catch (const Json::exception & error) {
    return {Code::invalid_metadata, error.what(), {}};
  } catch (const std::logic_error & error) {
    return {Code::invalid_metadata, error.what(), {}};
  }
}

/** @brief 获取所需源消息 ID，不为缺失位姿创建替代值。 */
void collectPoseRefs(const Json & frame, const char * key, std::set<std::string> & ids)
{
  if (!frame.contains(key)) {return;}
  for (const auto & edge : frame.at(key).at("edges")) {
    for (const char * endpoint : {"before", "after"}) {
      const auto & ref = edge.at(endpoint);
      if (!ref.is_null()) {
        ids.insert(tfKey(ref.at("message_id").get<std::string>(), ref.at("transform_index")));
      }
    }
  }
}

/** @brief 从当前数据集解析并核对一条 TF 源端点。 */
TfTransform readEndpoint(const Json & ref, const Records & tf, const std::string & bag_id)
{
  const auto id = ref.at("message_id").get<std::string>();
  checkSource(id, bag_id);
  const auto & row = referenced(tf, tfKey(id, ref.at("transform_index")));
  const auto & source = row.at("source");
  require(source.at("topic") == ref.at("topic") && sameStamp(row.at("stamp"), ref.at("stamp")),
    Code::invalid_reference, "TF endpoint topic or stamp mismatch");
  TfTransform out;
  if (row.at("static").get<bool>()) {out.kind = TfKind::static_transform;}
  out.stamp = readStamp(row.at("stamp"));
  out.parent_frame = row.at("parent_frame").get<std::string>();
  out.child_frame = row.at("child_frame").get<std::string>();
  out.translation_m = row.at("translation_m").get<std::array<double, 3>>();
  out.rotation_xyzw = row.at("rotation_xyzw").get<std::array<double, 4>>();
  out.source_topic = source.at("topic").get<std::string>();
  out.source_message_id = id;
  out.source_transform_index = integer<std::size_t>(ref.at("transform_index"));
  return out;
}

/** @brief 核对所引用的边确实覆盖该帧时间，保留原始端点供证据回查。 */
void checkEdge(const TfSelectedEdge & edge, const std::string & from, const std::string & to,
  const SensorStamp & stamp)
{
  const auto & a = edge.before;
  const auto & child = edge.inverted ? to : from;
  const auto & parent = edge.inverted ? from : to;
  require(a.child_frame == child && a.parent_frame == parent, Code::invalid_reference,
    "TF endpoint does not join the saved pose chain");
  const auto t = stampNs(stamp);
  if (edge.after) {
    const auto & b = *edge.after;
    require(a.kind == TfKind::dynamic && b.kind == TfKind::dynamic && b.child_frame == child &&
      b.parent_frame == parent && stampNs(a.stamp) < t && t < stampNs(b.stamp),
      Code::invalid_reference, "Invalid TF interpolation endpoints");
  } else {
    require(a.kind == TfKind::static_transform || stampNs(a.stamp) == t,
      Code::invalid_reference, "Dynamic TF sample does not match pose query time");
  }
}

/** @brief 读取已求解的位姿，逐条解析其证据。缺失时保留空值。 */
std::optional<TfPoseResult> readPose(
  const Json & frame, const char * key, const std::string & source_frame, const PreparedPair & pair,
  const Records & tf)
{
  if (!frame.contains(key)) {return std::nullopt;}
  const auto & record = frame.at(key);
  const auto & numeric = record.at("pose");
  const bool ready = record.at("pose_code") == "ready";
  require(ready == !numeric.is_null(), Code::invalid_metadata, "Pose result contradicts its status");
  if (!ready) {return std::nullopt;}
  require(record.at("code") == "covered" && record.at("chain_code") == "connected" &&
    sameStamp(record.at("stamp"), frame.at("stamp")) && numeric.at("source_frame") == source_frame &&
    numeric.at("target_frame") == pair.map_frame, Code::invalid_reference, "Pose time or frame mismatch");
  TfPoseResult out;
  out.code = TfPoseCode::ready;
  out.time.code = TfTimeCode::covered;
  out.time.chain_code = TfChainCode::connected;
  out.time.stamp = readStamp(record.at("stamp"));
  out.time.frames = record.at("frames").get<std::vector<std::string>>();
  const auto & edges = record.at("edges");
  require(out.time.frames.size() == edges.size() + 1 && out.time.frames.front() == source_frame &&
    out.time.frames.back() == pair.map_frame, Code::invalid_reference, "Invalid saved pose chain");
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const auto & row = edges.at(i);
    TfSelectedEdge edge;
    edge.inverted = row.at("inverted").get<bool>();
    edge.before = readEndpoint(row.at("before"), tf, pair.bag_id);
    if (!row.at("after").is_null()) {edge.after = readEndpoint(row.at("after"), tf, pair.bag_id);}
    checkEdge(edge, out.time.frames[i], out.time.frames[i + 1], out.time.stamp);
    out.time.edges.push_back(std::move(edge));
  }
  out.pose = TfPose{numeric.at("translation_m").get<std::array<double, 3>>(),
    numeric.at("rotation_xyzw").get<std::array<double, 4>>()};
  require(transformTfPoint(*out.pose, {0, 0, 0}).has_value(), Code::invalid_metadata, "Invalid pose values");
  return out;
}

/** @brief 复用导出的有效模型；校验实际网格并绑定本帧原标定。 */
std::optional<CameraModel> readModel(
  const Json & frame, const CameraImageReference & image, const Json * calibration)
{
  if (!frame.contains("camera_model")) {return std::nullopt;}
  const auto & j = frame.at("camera_model");
  if (j.at("code") != "ready") {return std::nullopt;}
  require(calibration && calibration->at("frame_id") == image.frame_id &&
    j.at("frame_id") == image.frame_id && integer<std::uint32_t>(j.at("width")) == image.width &&
    integer<std::uint32_t>(j.at("height")) == image.height, Code::invalid_reference, "Model grid or calibration mismatch");
  CameraModel model;
  model.width = image.width;
  model.height = image.height;
  model.frame_id = image.frame_id;
  const auto domain = j.at("image_domain");
  require(domain == "raw" || domain == "rectified", Code::invalid_metadata, "Unsupported camera model domain");
  model.image_domain = domain == "raw" ? CameraImageDomain::raw : CameraImageDomain::rectified;
  model.distortion_model = j.at("distortion_model").get<std::string>();
  require(j.at("d").is_array() && j.at("k").is_array() && j.at("k").size() == 9 &&
    j.at("r").is_array() && j.at("r").size() == 9 && j.at("p").is_array() && j.at("p").size() == 12,
    Code::invalid_metadata, "Invalid effective camera matrix shape");
  model.d = j.at("d").get<std::vector<double>>();
  model.k = j.at("k").get<std::array<double, 9>>();
  model.r = j.at("r").get<std::array<double, 9>>();
  model.p = j.at("p").get<std::array<double, 12>>();
  model.normalization_applied = j.at("normalization_applied").get<bool>();
  return model;
}

/** @brief 读取导出帧的网格、原始时间及来源话题。 */
CameraImageReference readImage(const Json & frame)
{
  return {frame.at("source").at("topic").get<std::string>(),
    integer<std::uint32_t>(frame.at("width")), integer<std::uint32_t>(frame.at("height")),
    frame.at("frame_id").get<std::string>(), readStamp(frame.at("stamp"))};
}

/** @brief 校验 NPY 1.0 写端契约，并读取二维小端 float32 米制深度。 */
void checkNpyHeader(std::istream & file, const CameraImageReference & image)
{
  std::array<unsigned char, 10> prefix{};
  const std::array<unsigned char, 8> magic{0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0};
  require(static_cast<bool>(file.read(reinterpret_cast<char *>(prefix.data()), prefix.size())) &&
    std::equal(magic.begin(), magic.end(), prefix.begin()), Code::invalid_media, "Expected NPY 1.0");
  const auto header_bytes = static_cast<std::size_t>(prefix[8]) + 256U * prefix[9];
  require(header_bytes > 0 && header_bytes <= 256, Code::invalid_media, "Invalid NPY header size");
  std::string header(header_bytes, '\0');
  require(static_cast<bool>(file.read(header.data(), static_cast<std::streamsize>(header.size()))), Code::invalid_media, "Truncated NPY header");
  const std::string shape = "{'descr': '<f4', 'fortran_order': False, 'shape': (" +
    std::to_string(image.height) + ", " + std::to_string(image.width) + "), }";
  require(header.back() == '\n' && header.substr(0, header.find_last_not_of(" \n") + 1) == shape,
    Code::invalid_media, "NPY shape, order or dtype differs from writer contract");
}

/** @brief 读取经过尺寸检查的 NPY float32 像素，保留无效值。 */
std::vector<float> readNpy(const fs::path & path, const CameraImageReference & image)
{
  auto file = input(path);
  checkNpyHeader(file, image);
  const auto count = static_cast<std::size_t>(image.width) * image.height;
  std::vector<unsigned char> bytes(count * 4);
  require(static_cast<bool>(file.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) &&
    file.peek() == std::char_traits<char>::eof() && !file.bad(), Code::invalid_media, "Invalid NPY payload size");
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE float32 required");
  std::vector<float> depth(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto offset = i * 4;
    std::uint32_t bits = 0;
    for (unsigned byte = 0; byte < 4; ++byte) {
      bits |= static_cast<std::uint32_t>(bytes[offset + byte]) << (8U * byte);
    }
    std::memcpy(&depth[i], &bits, sizeof(bits));
  }
  return depth;
}

/** @brief 检查 PNG 的实际网格、样本位数和 IHDR 颜色类型。 */
void checkPngHeader(const fs::path & path, const CameraImageReference & image,
  unsigned char bit_depth, unsigned char color_type)
{
  auto file = input(path);
  std::array<unsigned char, 26> bytes{};
  const std::array<unsigned char, 16> prefix{137, 'P', 'N', 'G', 13, 10, 26, 10, 0, 0, 0, 13, 'I', 'H', 'D', 'R'};
  require(static_cast<bool>(file.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) &&
    std::equal(prefix.begin(), prefix.end(), bytes.begin()), Code::invalid_media, "Invalid PNG header");
  /** @brief 将 IHDR 的四字节大端尺寸解为整数。 */
  const auto dimension = [&bytes](std::size_t offset) {
    std::uint32_t result = 0;
    for (std::size_t i = 0; i < 4; ++i) {result = (result << 8U) | bytes[offset + i];}
    return result;
  };
  require(dimension(16) == image.width && dimension(20) == image.height && bytes[24] == bit_depth &&
    bytes[25] == color_type, Code::invalid_media, "PNG dimensions or format differ from frame metadata");
}

/** @brief 完整读取校验 RGB 压缩像素，元数据入口仅校验 PNG 头和预算。 */
void checkRgbMedia(const fs::path & path, const CameraImageReference & image, bool validate_pixels)
{
  const auto pixels = static_cast<std::uint64_t>(image.width) * image.height;
  const RgbLimits limits;
  require(pixels > 0 && pixels <= limits.max_pixels && fs::file_size(path) <= limits.max_input_bytes,
    Code::invalid_media, "Invalid RGB grid or resource budget");
  checkPngHeader(path, image, 8, 2);
  if (!validate_pixels) {return;}
  const auto decoded = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  require(!decoded.empty() && decoded.type() == CV_8UC3 && decoded.cols == static_cast<int>(image.width) &&
    decoded.rows == static_cast<int>(image.height), Code::invalid_media, "Cannot decode RGB PNG");
}

/** @brief 按逐帧存储约定恢复 PNG，复用现有深度单位与无效值转换。 */
DepthFrame readPng(const fs::path & path, const CameraImageReference & image, const PreparedDepthStorage & storage)
{
  DepthOptions options;
  options.input_unit = storage.unit;
  options.convention = DepthConvention::optical_z;
  options.min_depth_m = storage.min_depth_m;
  if (storage.max_depth_m) {options.max_depth_m = *storage.max_depth_m;}
  checkPngHeader(path, image, 16, 0);
  const auto raw = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
  require(raw.type() == CV_16UC1 && raw.isContinuous() && raw.cols == static_cast<int>(image.width) &&
    raw.rows == static_cast<int>(image.height), Code::invalid_media, "Cannot decode source PNG16");
  const std::uint16_t endian = 0x0100;
  const auto big_endian = reinterpret_cast<const std::uint8_t *>(&endian)[0];
  const DepthImageView view{image.stamp, image.frame_id, image.width, image.height, "16UC1",
    image.width * 2U, big_endian, raw.data, raw.total() * raw.elemSize()};
  auto result = decodeDepth(view, options);
  require(result.ok(), Code::invalid_media, result.message);
  return std::move(*result.frame);
}

/** @brief 验证深度网格和资源上限，返回有界像素数。 */
std::uint64_t checkDepthBudget(const fs::path & path, const CameraImageReference & image)
{
  const auto pixels = static_cast<std::uint64_t>(image.width) * image.height;
  const DepthLimits limits;
  require(pixels > 0 && pixels <= limits.max_pixels && fs::file_size(path) <= limits.max_input_bytes,
    Code::invalid_media, "Invalid depth grid or resource budget");
  return pixels;
}

/** @brief 只核对深度文件头与可计算的载荷长度，不读取像素。 */
void checkDepthHeader(const fs::path & path, const CameraImageReference & image, const PreparedDepthStorage & storage)
{
  const auto pixels = checkDepthBudget(path, image);
  if (storage.format == "png") {
    checkPngHeader(path, image, 16, 0);
    return;
  }
  auto file = input(path);
  checkNpyHeader(file, image);
  const auto offset = static_cast<std::streamoff>(file.tellg());
  require(offset >= 0 && fs::file_size(path) == static_cast<std::uint64_t>(offset) + pixels * 4,
    Code::invalid_media, "Invalid NPY payload size");
}

/** @brief 读取已解析存储契约的媒体，确保质量计数与实际米制像素一致。 */
DepthFrame readDepth(const fs::path & path, const CameraImageReference & image, const Json & details,
  const PreparedDepthStorage & storage)
{
  const auto pixels = checkDepthBudget(path, image);
  DepthFrame out;
  if (storage.format == "png") {
    out = readPng(path, image, storage);
  } else {
    out.min_depth_m = storage.min_depth_m;
    if (storage.max_depth_m) {out.max_depth_m = *storage.max_depth_m;}
    out.depth_m = readNpy(path, image);
    out.stamp = image.stamp;
    out.frame_id = image.frame_id;
    out.width = image.width;
    out.height = image.height;
    out.step = image.width * sizeof(float);
  }
  std::size_t valid = 0;
  for (const auto value : out.depth_m) {
    if (std::isnan(value)) {continue;}
    require(std::isfinite(value) && value > 0 &&
      value >= static_cast<float>(out.min_depth_m) && value <= static_cast<float>(out.max_depth_m),
      Code::invalid_media, "Invalid stored depth sample");
    ++valid;
  }
  require(valid == storage.valid_pixels && pixels - valid == storage.invalid_pixels,
    Code::invalid_media, "Stored depth quality counts differ from media");
  out.valid_pixels = valid;
  out.invalid_pixels = static_cast<std::size_t>(pixels - valid);
  out.valid_ratio = static_cast<double>(valid) / static_cast<double>(pixels);
  out.zero_pixels = integer<std::size_t>(details.at("zero_pixels"));
  out.negative_pixels = integer<std::size_t>(details.at("negative_pixels"));
  out.nonfinite_pixels = integer<std::size_t>(details.at("nonfinite_pixels"));
  out.out_of_range_pixels = integer<std::size_t>(details.at("out_of_range_pixels"));
  out.unrepresentable_pixels = integer<std::size_t>(details.at("unrepresentable_pixels"));
  out.source_encoding = storage.source_encoding;
  if (out.source_encoding == "16UC1") {out.source_numeric_type = DepthNumericType::uint16;}
  if (out.source_encoding == "32FC1") {out.source_numeric_type = DepthNumericType::float32;}
  out.source_unit = storage.source_unit;
  out.source_scale_to_m = storage.source_scale_to_m;
  return out;
}

/** @brief 返回已保存的注册检查事实；没有检查记录时保持未知，不猜测成功。 */
void readAlignment(PreparedPair & pair, const Json & frame)
{
  const auto & details = frame.at("details");
  if (!details.contains("alignment_code")) {return;}
  const auto name = details.at("alignment_code").get<std::string>();
  for (const auto code : {DepthAlignmentStatus::unknown, DepthAlignmentStatus::not_aligned, DepthAlignmentStatus::aligned,
    DepthAlignmentStatus::invalid_reference, DepthAlignmentStatus::inconsistent_declaration,
    DepthAlignmentStatus::size_mismatch, DepthAlignmentStatus::frame_mismatch}) {
    if (name == depthAlignmentStatusName(code)) {pair.depth_alignment_check = code; break;}
  }
  require(pair.depth_alignment_check.has_value(), Code::invalid_metadata, "Unknown depth alignment check code");
  pair.alignment_basis = details.at("alignment_basis").get<std::string>();
  if (*pair.depth_alignment_check == DepthAlignmentStatus::not_aligned) {pair.depth_alignment = DepthAlignment::not_aligned;}
  if (*pair.depth_alignment_check == DepthAlignmentStatus::aligned) {
    require(!pair.alignment_basis.empty() && pair.rgb_image.width > 0 && pair.rgb_image.height > 0 &&
      pair.rgb_image.width == pair.depth_image.width &&
      pair.rgb_image.height == pair.depth_image.height && pair.rgb_image.frame_id == pair.depth_image.frame_id,
      Code::invalid_reference, "Aligned depth grid differs from paired RGB");
    pair.depth_alignment = DepthAlignment::aligned_to_rgb;
  }
}

/** @brief 读取被配对指定的帧，核对角色、时间与标定 ID。 */
const Json & pairFrame(const Json & pair, const Records & frames, const std::string & stream,
  const std::string & id, const std::string & calibration_id)
{
  const auto & frame = referenced(frames, id);
  require(frame.at("kind") == stream && sameStamp(frame.at("stamp"), pair.at(stream + "_stamp")),
    Code::invalid_reference, "Pair does not reference the saved " + stream + " frame");
  const auto calibration = frame.value("calibration", Json::object());
  require(calibration.value("calibration_id", std::string{}) == calibration_id,
    Code::invalid_reference, "Pair and frame calibration references disagree");
  return frame;
}

/** @brief 保留空媒体状态，绑定存在的媒体路径与期望摘要。 */
std::optional<fs::path> readMediaPath(
  const fs::path & root, const Json & frame, const std::optional<detail::PreparationMediaIndex> & index,
  std::vector<PreparedFileSnapshot> & files)
{
  const auto * sealed = detail::preparationFrameMedia(index, frame);
  if (frame.at("media_path").is_null()) {return std::nullopt;}
  const auto path = member(root, frame.at("media_path").get<std::string>());
  auto snapshot = snapshotFile(root, path);
  if (sealed) {
    require(snapshot.relative_path.generic_string() == sealed->path, Code::invalid_reference,
      "Sealed media path resolves to a different file: " + sealed->path);
    snapshot.sha256 = sealed->sha256;
  }
  files.push_back(std::move(snapshot));
  return path;
}

/** @brief 解析配对来源与状态，保留已经保存的配对 ID。 */
void readPairFields(PreparedPair & result, const std::string & bag_id, const Json & row)
{
  result.pair_id = row.at("pair_id").get<std::string>();
  result.rgb_message_id = row.at("rgb_message_id").get<std::string>();
  result.depth_message_id = row.at("depth_message_id").get<std::string>();
  result.rgb_calibration_id = row.at("rgb_calibration_id").get<std::string>();
  result.depth_calibration_id = row.at("depth_calibration_id").get<std::string>();
  checkSource(result.rgb_message_id, bag_id);
  checkSource(result.depth_message_id, bag_id);
  result.delta_ns = detail::preparationSigned(row.at("delta_ns"));
  result.sync_tolerance_ns = detail::preparationSigned(row.at("sync_tolerance_ns"));
  result.capture_sync_basis = row.value("capture_sync_basis", std::string{});
  result.timestamp_match_policy = row.at("policy").get<std::string>();
  result.capture_sync_status = row.at("capture_sync_status").get<std::string>();
  result.status = row.at("status").get<std::string>();
  result.pending_reasons = row.at("reasons").get<std::vector<std::string>>();
}

/** @brief 跨字段语义错误只影响当前配对，不扩大全局索引打开的错误范围。 */
void checkPairTiming(const PreparedPair & pair, const PreparedPair & context, const Json & row)
{
  require(pair.sync_tolerance_ns == context.sync_tolerance_ns &&
    pair.timestamp_match_policy == context.timestamp_match_policy, Code::invalid_reference,
    "Pair tolerance/policy differs from manifest");
  const auto timing = checkSensorTimes(readStamp(row.at("rgb_stamp")), readStamp(row.at("depth_stamp")),
    {SensorClockRelation::same_clock, pair.sync_tolerance_ns});
  require(timing.matched() && *timing.delta_ns == pair.delta_ns &&
    row.at("exact").get<bool>() == (pair.delta_ns == 0), Code::invalid_reference, "Saved pair time/delta/tolerance contradict");
}

/** @brief 对照消息清单核实选中帧、标定和 TF 的源证据文件引用。 */
template<typename Select>
void checkEvidence(PreparedPair & pair, const Json & rgb, const Json & depth,
  const Records & calibrations, const Records & tf, bool save_raw_cdr, bool validate_payloads, Select select)
{
  std::vector<const Json *> sources{&rgb.at("source"), &depth.at("source")};
  for (const auto & [id, record] : calibrations) {sources.push_back(&record.at("source"));}
  for (const auto & [id, record] : tf) {sources.push_back(&record.at("source"));}
  std::set<std::string> ids;
  for (const auto * source : sources) {
    ids.insert(source->at("message_id").get<std::string>());
  }
  const Records messages = select("messages.jsonl", ids,
      [](const Json & j) {return j.at("message_id").get<std::string>();});
  std::set<std::string> checked;
  for (const auto * source : sources) {
    const auto id = source->at("message_id").get<std::string>();
    checkSource(id, pair.bag_id);
    const auto & row = referenced(messages, id);
    for (auto field = source->begin(); field != source->end(); ++field) {
      require(row.contains(field.key()) && row.at(field.key()) == field.value(),
        Code::invalid_reference, "Message source metadata differs: " + id);
    }
    if (!checked.insert(id).second) {continue;}
    auto summary = detail::readPreparationMessage(row);
    const auto serialized_bytes = summary.serialized_bytes;
    const auto & raw_path = row.at("raw_path");
    require(save_raw_cdr == raw_path.is_string() && (save_raw_cdr || raw_path.is_null()),
      Code::invalid_reference, "Raw path contradicts save_raw_cdr: " + id);
    if (save_raw_cdr) {
      const auto raw = member(pair.preparation_root, raw_path.get<std::string>());
      auto snapshot = snapshotFile(pair.preparation_root, raw);
      require(snapshot.bytes == serialized_bytes, Code::invalid_reference, "Raw evidence size differs: " + id);
      if (validate_payloads) {
        require(detail::preparationFileSha256(raw) == summary.payload_sha256,
          Code::invalid_reference, "Raw evidence digest differs: " + id);
      }
      summary.raw_path = snapshot.relative_path;
      pair.file_snapshots.push_back(std::move(snapshot));
      if (id == pair.rgb_message_id) {pair.rgb_raw_evidence = raw;}
      if (id == pair.depth_message_id) {pair.depth_raw_evidence = raw;}
    }
    pair.message_summaries.push_back(std::move(summary));
  }
  /** @brief 按原始消息顺序稳定列出闭包，ID 仅用于确定同序号的顺序。 */
  const auto source_order = [](const PreparedMessageSummary & a, const PreparedMessageSummary & b) {
      return std::tie(a.message_order, a.message_id) < std::tie(b.message_order, b.message_id);
    };
  std::sort(pair.message_summaries.begin(), pair.message_summaries.end(), source_order);
}

/** @brief 通过指定记录读取方式组装一对，复用来源、几何和媒体校验。 */
template<typename Select>
PreparedPair readPair(
  const fs::path & root, const Json & manifest, const PreparedPair & context,
  const std::optional<detail::PreparationMediaIndex> & media_index, std::string_view pair_id,
  bool validate_pixels, Select select)
{
  const std::string id(pair_id);
  const Records pairs = select("rgbd_pairs.jsonl", {id},
      [](const Json & row) {return row.at("pair_id").get<std::string>();});
  require(pairs.count(id) != 0, Code::pair_not_found, "Pair not found: " + id);
  const auto & row = pairs.at(id);
  auto result = context;
  readPairFields(result, context.bag_id, row);
  checkPairTiming(result, context, row);
  result.preparation_root = root;
  const Records frames = select("frames.jsonl", {result.rgb_message_id, result.depth_message_id},
      [](const Json & j) {return j.at("source").at("message_id").get<std::string>();});
  const auto & rgb = pairFrame(row, frames, "rgb", result.rgb_message_id, result.rgb_calibration_id);
  const auto & depth = pairFrame(row, frames, "depth", result.depth_message_id, result.depth_calibration_id);
  result.rgb_image = readImage(rgb);
  result.depth_image = readImage(depth);
  result.rgb_media = readMediaPath(root, rgb, media_index, result.file_snapshots);
  if (result.rgb_media) {
    result.rgb_media_relative = result.file_snapshots.back().relative_path;
    checkRgbMedia(*result.rgb_media, result.rgb_image, validate_pixels);
  }
  result.depth_media = readMediaPath(root, depth, media_index, result.file_snapshots);
  if (result.depth_media) {result.depth_media_relative = result.file_snapshots.back().relative_path;}

  std::set<std::string> calibration_ids;
  for (const auto & key : {result.rgb_calibration_id, result.depth_calibration_id}) {
    if (key.empty()) {continue;}
    checkSource(key, result.bag_id);
    calibration_ids.insert(key);
  }
  const Records calibrations = select("camera_info.jsonl", calibration_ids,
      [](const Json & j) {return j.at("calibration_id").get<std::string>();});
  /** @brief 解析可空的标定引用并核对源消息身份。 */
  const auto cal = [&calibrations](const std::string & key) -> const Json * {
    if (key.empty()) {return nullptr;}
    const auto & record = referenced(calibrations, key);
    require(record.at("source").at("message_id") == key, Code::invalid_reference, "Calibration source ID mismatch");
    return &record;
  };
  const auto * rgb_calibration = cal(result.rgb_calibration_id);
  const auto * depth_calibration = cal(result.depth_calibration_id);
  result.rgb_calibration = detail::readPreparationCalibration(rgb, rgb_calibration, context.rgb_calibration.selection);
  result.depth_calibration = detail::readPreparationCalibration(depth, depth_calibration, context.depth_calibration.selection);
  result.rgb_model = readModel(rgb, result.rgb_image, rgb_calibration);
  result.depth_model = readModel(depth, result.depth_image, depth_calibration);
  detail::checkPreparationModel(result.rgb_calibration, result.rgb_model, result.rgb_image);
  detail::checkPreparationModel(result.depth_calibration, result.depth_model, result.depth_image);

  std::set<std::string> tf_ids;
  collectPoseRefs(rgb, "map_camera_tf", tf_ids);
  collectPoseRefs(depth, "map_camera_tf", tf_ids);
  collectPoseRefs(rgb, "map_body_tf", tf_ids);
  const Records tf = select("tf.jsonl", tf_ids, [](const Json & j) {
      return tfKey(j.at("source").at("message_id").get<std::string>(), j.at("transform_index"));
    });
  result.rgb_pose = readPose(rgb, "map_camera_tf", result.rgb_image.frame_id, result, tf);
  result.depth_pose = readPose(depth, "map_camera_tf", result.depth_image.frame_id, result, tf);
  result.body_pose = readPose(rgb, "map_body_tf",
    result.body_frame, result, tf);
  checkEvidence(result, rgb, depth, calibrations, tf, savesRawCdr(manifest), validate_pixels, select);
  result.depth_storage = detail::readPreparationDepthStorage(depth, manifest.at("effective_options").at("depth"));
  readAlignment(result, depth);
  if (result.depth_media) {
    require(result.depth_storage.has_value(), Code::invalid_media, "Depth file lacks storage interpretation");
    if (!validate_pixels) {
      checkDepthHeader(*result.depth_media, result.depth_image, *result.depth_storage);
      checkFileSnapshots(root, result.file_snapshots);
      return result;
    }
    result.depth = readDepth(*result.depth_media, result.depth_image, depth.at("details"), *result.depth_storage);
    result.depth->alignment = result.depth_alignment;
    result.depth->alignment_basis = result.alignment_basis;
    if (result.depth_alignment == DepthAlignment::aligned_to_rgb) {
      result.depth->rgb_grid = {result.rgb_image.width, result.rgb_image.height, result.rgb_image.frame_id};
    }
  }
  if (validate_pixels) {
    for (const auto & file : result.file_snapshots) {
      if (file.sha256.empty()) {continue;}
      require(detail::preparationFileSha256(root / file.relative_path) == file.sha256,
        Code::invalid_media, "Media SHA-256 differs: " + file.relative_path.string());
    }
  }
  checkFileSnapshots(root, result.file_snapshots);
  return result;
}

}  // namespace

/** @brief reader 私有状态，仅拥有元数据和行位置，逐对读取使用局部文件流。 */
struct PreparationReaderState
{
  fs::path root;
  SourceFile manifest_source;
  std::optional<SourceFile> media_index_source;
  std::optional<detail::PreparationMediaIndex> media_index;
  Json manifest;
  PreparedPair context;
  std::map<std::string, JsonlIndex> indexes;
  std::vector<PreparationPairInfo> pairs;
};

namespace
{

/** @brief 在操作边界核对所有元数据文件，避免在变化的偏移索引上继续读取。 */
void checkSources(const PreparationReaderState & state)
{
  checkSourceFile(state.root, "bag_manifest.json", state.manifest_source);
  if (state.media_index_source) {
    checkSourceFile(state.root, detail::kPreparationMediaIndexFile, *state.media_index_source);
  }
  for (const auto & [name, index] : state.indexes) {
    checkSourceFile(state.root, name, index.source);
  }
}

}  // namespace

/** @brief 接管已建立的只读状态，reader 对象由打开结果独占。 */
PreparationReader::PreparationReader(std::unique_ptr<PreparationReaderState> state)
: state_(std::move(state))
{
}

/** @brief 释放元数据索引及目录，已返回配对拥有自己的存储。 */
PreparationReader::~PreparationReader() = default;

/** @brief 返回打开时建立的目录，顺序和原始时间均保持不变。 */
const std::vector<PreparationPairInfo> & PreparationReader::pairs() const noexcept
{
  return state_->pairs;
}

/** @brief 保留已有完整读取入口和米制深度输出。 */
PreparationReadResult PreparationReader::loadPair(std::string_view pair_id) const
{
  return loadPairImpl(pair_id, true);
}

/** @brief 保留组装所需信息和媒体头校验，跳过深度像素解码。 */
PreparationReadResult PreparationReader::loadPairMetadata(std::string_view pair_id) const
{
  return loadPairImpl(pair_id, false);
}

/** @brief 返回索引绑定的规范根路径。 */
const fs::path & PreparationReader::root() const noexcept
{
  return state_->root;
}

/** @brief 返回打开时计算的 manifest 指纹。 */
std::string_view PreparationReader::manifestSha256() const noexcept
{
  return state_->context.source_manifest_sha256;
}

/** @brief 返回索引源的地图 frame。 */
std::string_view PreparationReader::mapFrame() const noexcept
{
  return state_->context.map_frame;
}

/** @brief 核对原索引与已验证的媒体或 CDR，保持统一读取错误语义。 */
PreparationSourceCheck PreparationReader::verifyUnchanged(const std::vector<PreparedFileSnapshot> & files) const
{
  try {
    checkSources(*state_);
    checkFileSnapshots(state_->root, files);
    require(detail::preparationFileSha256(state_->manifest_source.path) == manifestSha256(),
      Code::source_changed, "Preparation manifest content changed");
    if (state_->media_index_source) {
      require(detail::preparationFileSha256(state_->media_index_source->path) ==
        state_->manifest.at("media_integrity").at("index_sha256").get<std::string>(),
        Code::source_changed, "Preparation media index content changed");
    }
    std::map<fs::path, std::string_view> checked;
    for (const auto & file : files) {
      if (file.sha256.empty()) {continue;}
      const auto found = checked.emplace(file.relative_path, file.sha256);
      if (!found.second) {
        require(found.first->second == file.sha256, Code::source_changed, "Conflicting source snapshots");
        continue;
      }
      require(detail::preparationFileSha256(state_->root / file.relative_path) == file.sha256,
        Code::source_changed, "Source content changed: " + file.relative_path.string());
    }
    checkFileSnapshots(state_->root, files);
    checkSources(*state_);
    return {Code::ready, ""};
  } catch (const ReadError & error) {
    return {error.code, error.what()};
  }
}

/** @brief 只定位当前配对引用的行，成功结果独立持有数据，变化时丢弃本次结果。 */
PreparationReadResult PreparationReader::loadPairImpl(std::string_view pair_id, bool validate_pixels) const
{
  return readResult<PreparationReadResult>([&]() -> PreparationReadResult {
    checkSources(*state_);
    /** @brief 用只读偏移索引替代整文件扫描，沿用配对组装所需的键规则。 */
    const auto select = [this](const char * name, const std::set<std::string> & keys, auto key) {
        return indexedRows(state_->indexes.at(name), keys, key);
      };
    try {
      auto pair = readPair(state_->root, state_->manifest, state_->context,
        state_->media_index, pair_id, validate_pixels, select);
      checkSources(*state_);
      return {Code::ready, "", std::move(pair)};
    } catch (...) {
      checkSources(*state_);
      throw;
    }
  });
}

/** @brief 原子构建目录和五类索引，任意打开失败都不会发布部分 reader。 */
PreparationOpenResult openPreparation(const fs::path & preparation_root)
{
  return readResult<PreparationOpenResult>([&]() -> PreparationOpenResult {
    auto state = std::make_unique<PreparationReaderState>();
    state->root = fs::canonical(preparation_root);
    state->manifest_source = sourceFile(state->root, "bag_manifest.json");
    std::string fingerprint;
    state->manifest = readManifest(state->root, fingerprint);
    if (state->manifest.contains("media_integrity")) {
      state->media_index_source = sourceFile(state->root, detail::kPreparationMediaIndexFile);
    }
    state->media_index = detail::readPreparationMediaIndex(state->root, state->manifest);
    state->context = detail::readPreparationContext(state->manifest);
    state->context.source_manifest_sha256 = std::move(fingerprint);
    /** @brief 建立源顺序目录，配对字段用现有解析器校验。 */
    const auto pair_key = [&state](const Json & row) {
        PreparedPair pair;
        readPairFields(pair, state->context.bag_id, row);
        state->pairs.push_back({pair.pair_id, pair.rgb_message_id, pair.depth_message_id,
          readStamp(row.at("rgb_stamp")), readStamp(row.at("depth_stamp")), pair.delta_ns,
          pair.status, pair.pending_reasons});
        return pair.pair_id;
      };
    state->indexes.emplace("rgbd_pairs.jsonl", indexRows(state->root, "rgbd_pairs.jsonl", pair_key));
    state->indexes.emplace("frames.jsonl", indexRows(state->root, "frames.jsonl",
      [](const Json & row) {return row.at("source").at("message_id").get<std::string>();}));
    state->indexes.emplace("camera_info.jsonl", indexRows(state->root, "camera_info.jsonl",
      [](const Json & row) {return row.at("calibration_id").get<std::string>();}));
    state->indexes.emplace("tf.jsonl", indexRows(state->root, "tf.jsonl", [](const Json & row) {
        return tfKey(row.at("source").at("message_id").get<std::string>(), row.at("transform_index"));
      }));
    state->indexes.emplace("messages.jsonl", indexRows(state->root, "messages.jsonl",
      [](const Json & row) {return row.at("message_id").get<std::string>();}));
    checkSources(*state);
    return {Code::ready, "", std::unique_ptr<PreparationReader>(
      new PreparationReader(std::move(state)))};
  });
}

/** @brief 保留单对入口的选择性扫描语义，复用完整配对组装与错误映射。 */
PreparationReadResult loadPreparedPair(const fs::path & preparation_root, std::string_view pair_id)
{
  return readResult<PreparationReadResult>([&]() -> PreparationReadResult {
    const auto root = fs::canonical(preparation_root);
    std::string fingerprint;
    const auto manifest = readManifest(root, fingerprint);
    const auto media_index = detail::readPreparationMediaIndex(root, manifest);
    auto context = detail::readPreparationContext(manifest);
    context.source_manifest_sha256 = std::move(fingerprint);
    /** @brief 旧入口只保留当前 ID 的记录，校验范围沿用历史契约。 */
    const auto select = [&root](const char * name, const std::set<std::string> & keys, auto key) {
        return selectRows(member(root, name), keys, key);
      };
    return {Code::ready, "", readPair(root, manifest, context, media_index, pair_id, true, select)};
  });
}

}  // namespace vln_memory
