#include <vln_memory/bag/builder/preparation_reader.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
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

struct ReadError : std::runtime_error
{
  Code code;

  /** @brief 携带稳定错误码并向公开入口报告读取失败。 */
  ReadError(Code reason, const std::string & message) : std::runtime_error(message), code(reason) {}
};

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
  require(!path.empty() && path.is_relative(), Code::invalid_reference, "Expected a relative evidence path");
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
  for (std::string line; std::getline(file, line);) {
    auto row = Json::parse(line);
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
  const auto message = "Metadata changed: " + (root / name).string();
  std::error_code error;
  const auto path = fs::canonical(root / name, error);
  require(!error && path == source.path, Code::source_changed, message);
  const auto bytes = fs::file_size(path, error);
  require(!error && bytes == source.bytes, Code::source_changed, message);
  const auto modified = fs::last_write_time(path, error);
  require(!error && modified == source.modified, Code::source_changed, message);
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
    if (!std::getline(file, line)) {break;}
    ++line_number;
    const auto context = index.source.path.string() + ":" + std::to_string(line_number) + ": ";
    try {
      require(offset >= std::streampos(0) &&
        line.size() <= static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()),
        Code::invalid_metadata, "JSONL offset or length is out of range");
      auto row = Json::parse(line);
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
    auto row = Json::parse(line);
    require(key(row) == id && row.at("schema_version") == "bag_preparation/1",
      Code::invalid_reference, "Indexed row identity differs: " + context);
    selected.emplace(id, std::move(row));
  }
  return selected;
}

/** @brief 读取现有 v5/v6 已发布 manifest，保持单对入口的校验契约。 */
Json readManifest(const fs::path & root)
{
  auto file = input(member(root, "bag_manifest.json"));
  Json manifest;
  file >> manifest;
  const auto version = manifest.at("builder_version");
  require(manifest.at("artifact_kind") == "bag_preparation" &&
    manifest.at("schema_version") == "bag_preparation/1" && (version == 5 || version == 6) &&
    (manifest.at("status") == "completed" || manifest.at("status") == "completed_with_issues"),
    Code::unsupported_dataset, "Expected published BagBuilder v5 or v6 preparation");
  return manifest;
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
std::vector<float> readNpy(const fs::path & path, const CameraImageReference & image)
{
  auto file = input(path);
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

/** @brief 按逐帧存储约定恢复 PNG，复用现有深度单位与无效值转换。 */
DepthFrame readPng(const fs::path & path, const CameraImageReference & image, const Json & storage)
{
  const auto unit = storage.at("unit");
  DepthOptions options;
  if (unit == "mm") {options.input_unit = DepthUnit::millimeters;}
  if (unit == "m") {options.input_unit = DepthUnit::meters;}
  const auto scale = storage.at("scale_to_m").get<double>();
  require(storage.at("dtype") == "uint16" && storage.at("invalid_rule") == "zero_or_out_of_range_to_nan" &&
    ((unit == "mm" && scale == 0.001) || (unit == "m" && scale == 1.0)),
    Code::invalid_media, "Invalid PNG depth unit, dtype or scale");
  options.convention = DepthConvention::optical_z;
  options.min_depth_m = storage.at("min_depth_m").get<double>();
  if (!storage.at("max_depth_m").is_null()) {options.max_depth_m = storage.at("max_depth_m").get<double>();}
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

/** @brief 读取约定的深度媒体，确保范围和质量计数与文件匹配。 */
DepthFrame readDepth(const fs::path & path, const CameraImageReference & image, const Json & details,
  const Json & depth_options, const CameraImageReference & rgb)
{
  const auto pixels = static_cast<std::uint64_t>(image.width) * image.height;
  const DepthLimits limits;
  require(pixels > 0 && pixels <= limits.max_pixels && fs::file_size(path) <= limits.max_input_bytes &&
    details.at("unit") == "meters" && details.at("convention") == "optical_z",
    Code::invalid_media, "Invalid depth grid, budget or geometry");
  DepthFrame out;
  const bool has_storage = details.contains("storage");
  if (has_storage && details.at("storage").at("format") == "png") {
    out = readPng(path, image, details.at("storage"));
  } else {
    if (has_storage) {
      const auto & s = details.at("storage");
      require(s.at("format") == "npy" && s.at("dtype") == "float32" && s.at("unit") == "m" &&
        s.at("scale_to_m") == 1.0 && s.at("invalid_rule") == "nan",
        Code::invalid_media, "Invalid NPY depth storage contract");
    }
    const auto & range = has_storage ? details.at("storage") : depth_options;
    out.min_depth_m = range.at("min_depth_m").get<double>();
    if (!range.at("max_depth_m").is_null()) {out.max_depth_m = range.at("max_depth_m").get<double>();}
    require(std::isfinite(out.min_depth_m) && out.min_depth_m >= 0 && out.max_depth_m >= out.min_depth_m,
      Code::invalid_media, "Invalid metric depth range");
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
  out.valid_pixels = integer<std::size_t>(details.at("valid_pixels"));
  out.invalid_pixels = integer<std::size_t>(details.at("invalid_pixels"));
  require(valid == out.valid_pixels && pixels - valid == out.invalid_pixels,
    Code::invalid_media, "Stored depth quality counts differ from media");
  out.valid_ratio = static_cast<double>(valid) / static_cast<double>(pixels);
  out.zero_pixels = integer<std::size_t>(details.at("zero_pixels"));
  out.negative_pixels = integer<std::size_t>(details.at("negative_pixels"));
  out.nonfinite_pixels = integer<std::size_t>(details.at("nonfinite_pixels"));
  out.out_of_range_pixels = integer<std::size_t>(details.at("out_of_range_pixels"));
  out.unrepresentable_pixels = integer<std::size_t>(details.at("unrepresentable_pixels"));
  out.source_encoding = details.at("source_encoding").get<std::string>();
  if (out.source_encoding == "16UC1") {out.source_numeric_type = DepthNumericType::uint16;}
  if (out.source_encoding == "32FC1") {out.source_numeric_type = DepthNumericType::float32;}
  const auto source_unit = details.at("source_unit");
  if (source_unit == "mm") {out.source_unit = DepthUnit::millimeters;}
  if (source_unit == "m") {out.source_unit = DepthUnit::meters;}
  out.source_scale_to_m = details.at("source_scale_to_m").get<double>();
  out.alignment_basis = details.at("alignment_basis").get<std::string>();
  const auto alignment = details.at("alignment_code");
  if (alignment == "not_aligned") {out.alignment = DepthAlignment::not_aligned;}
  if (alignment == "aligned") {
    require(!out.alignment_basis.empty() && rgb.width == out.width && rgb.height == out.height &&
      rgb.frame_id == out.frame_id, Code::invalid_reference, "Aligned depth grid differs from paired RGB");
    out.alignment = DepthAlignment::aligned_to_rgb;
    out.rgb_grid = {rgb.width, rgb.height, rgb.frame_id};
  }
  return out;
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

/** @brief 检查存在的媒体路径，保持明确导出的 null 媒体状态。 */
std::optional<fs::path> readMediaPath(const fs::path & root, const Json & frame)
{
  if (frame.at("media_path").is_null()) {return std::nullopt;}
  return member(root, frame.at("media_path").get<std::string>());
}

/** @brief 校验数据集公共地图与时钟上下文，供目录和逐对读取复用。 */
PreparedPair datasetContext(const Json & manifest)
{
  PreparedPair result;
  const auto & options = manifest.at("effective_options");
  result.bag_id = manifest.at("bag_id").get<std::string>();
  result.map_frame = options.at("map_frame").get<std::string>();
  result.time_scope = {options.at("clock_id").get<std::string>(), options.at("segment_id").get<std::string>()};
  result.clock_basis = options.at("clock_basis").get<std::string>();
  result.segment_basis = options.at("segment_basis").get<std::string>();
  require(!result.bag_id.empty() && !result.map_frame.empty() && !result.time_scope.clock_id.empty() &&
    !result.time_scope.segment_id.empty() && !result.clock_basis.empty() &&
    !result.segment_basis.empty() && options.at("clock_relation") == "same_clock",
    Code::invalid_metadata, "Missing dataset or time scope");
  return result;
}

/** @brief 解析配对来源与状态，保留已经保存的配对 ID。 */
PreparedPair pairContext(const Json & manifest, const Json & row)
{
  auto result = datasetContext(manifest);
  result.pair_id = row.at("pair_id").get<std::string>();
  result.rgb_message_id = row.at("rgb_message_id").get<std::string>();
  result.depth_message_id = row.at("depth_message_id").get<std::string>();
  result.rgb_calibration_id = row.at("rgb_calibration_id").get<std::string>();
  result.depth_calibration_id = row.at("depth_calibration_id").get<std::string>();
  checkSource(result.rgb_message_id, result.bag_id);
  checkSource(result.depth_message_id, result.bag_id);
  const auto delta = row.at("delta_ns").get<std::string>();
  std::size_t used = 0;
  result.delta_ns = std::stoll(delta, &used);
  require(used == delta.size(), Code::invalid_metadata, "Invalid saved delta_ns");
  result.timestamp_match_policy = row.at("policy").get<std::string>();
  result.capture_sync_status = row.at("capture_sync_status").get<std::string>();
  result.status = row.at("status").get<std::string>();
  result.pending_reasons = row.at("reasons").get<std::vector<std::string>>();
  return result;
}

/** @brief 对照消息清单核实选中帧、标定和 TF 的源证据文件引用。 */
template<typename Select>
void checkEvidence(PreparedPair & pair, const Json & rgb, const Json & depth,
  const Records & calibrations, const Records & tf, Select select)
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
  for (const auto * source : sources) {
    const auto id = source->at("message_id").get<std::string>();
    checkSource(id, pair.bag_id);
    const auto & row = referenced(messages, id);
    for (auto field = source->begin(); field != source->end(); ++field) {
      require(row.contains(field.key()) && row.at(field.key()) == field.value(),
        Code::invalid_reference, "Message source metadata differs: " + id);
    }
    const auto raw = member(pair.preparation_root, row.at("raw_path").get<std::string>());
    require(fs::file_size(raw) == integer<std::uintmax_t>(row.at("serialized_bytes")),
      Code::invalid_reference, "Raw evidence size differs: " + id);
    if (id == pair.rgb_message_id) {pair.rgb_raw_evidence = raw;}
    if (id == pair.depth_message_id) {pair.depth_raw_evidence = raw;}
  }
}

/** @brief 通过指定记录读取方式组装一对，复用来源、几何和媒体校验。 */
template<typename Select>
PreparedPair readPair(
  const fs::path & root, const Json & manifest, std::string_view pair_id, Select select)
{
  const std::string id(pair_id);
  const Records pairs = select("rgbd_pairs.jsonl", {id},
      [](const Json & row) {return row.at("pair_id").get<std::string>();});
  require(pairs.count(id) != 0, Code::pair_not_found, "Pair not found: " + id);
  const auto & row = pairs.at(id);
  auto result = pairContext(manifest, row);
  result.preparation_root = root;
  const Records frames = select("frames.jsonl", {result.rgb_message_id, result.depth_message_id},
      [](const Json & j) {return j.at("source").at("message_id").get<std::string>();});
  const auto & rgb = pairFrame(row, frames, "rgb", result.rgb_message_id, result.rgb_calibration_id);
  const auto & depth = pairFrame(row, frames, "depth", result.depth_message_id, result.depth_calibration_id);
  result.rgb_image = readImage(rgb);
  result.depth_image = readImage(depth);
  result.rgb_media = readMediaPath(root, rgb);
  if (result.rgb_media) {checkPngHeader(*result.rgb_media, result.rgb_image, 8, 2);}
  result.depth_media = readMediaPath(root, depth);

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
  result.rgb_model = readModel(rgb, result.rgb_image, cal(result.rgb_calibration_id));
  result.depth_model = readModel(depth, result.depth_image, cal(result.depth_calibration_id));

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
    manifest.at("effective_options").at("body_frame").get<std::string>(), result, tf);
  checkEvidence(result, rgb, depth, calibrations, tf, select);
  if (result.depth_media) {
    result.depth = readDepth(*result.depth_media, result.depth_image, depth.at("details"),
      manifest.at("effective_options").at("depth"), result.rgb_image);
  }
  return result;
}

}  // namespace

/** @brief reader 私有状态，仅拥有元数据和行位置，逐对读取使用局部文件流。 */
struct PreparationReaderState
{
  fs::path root;
  SourceFile manifest_source;
  Json manifest;
  std::map<std::string, JsonlIndex> indexes;
  std::vector<PreparationPairInfo> pairs;
};

namespace
{

/** @brief 在操作边界核对所有元数据文件，避免在变化的偏移索引上继续读取。 */
void checkSources(const PreparationReaderState & state)
{
  checkSourceFile(state.root, "bag_manifest.json", state.manifest_source);
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

/** @brief 只定位当前配对引用的行，成功结果独立持有数据，变化时丢弃本次结果。 */
PreparationReadResult PreparationReader::loadPair(std::string_view pair_id) const
{
  return readResult<PreparationReadResult>([&]() -> PreparationReadResult {
    checkSources(*state_);
    /** @brief 用只读偏移索引替代整文件扫描，沿用配对组装所需的键规则。 */
    const auto select = [this](const char * name, const std::set<std::string> & keys, auto key) {
        return indexedRows(state_->indexes.at(name), keys, key);
      };
    try {
      auto pair = readPair(state_->root, state_->manifest, pair_id, select);
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
    state->manifest = readManifest(state->root);
    datasetContext(state->manifest);
    /** @brief 建立源顺序目录，配对字段用现有解析器校验。 */
    const auto pair_key = [&state](const Json & row) {
        const auto pair = pairContext(state->manifest, row);
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
    const auto manifest = readManifest(root);
    /** @brief 旧入口只保留当前 ID 的记录，校验范围沿用历史契约。 */
    const auto select = [&root](const char * name, const std::set<std::string> & keys, auto key) {
        return selectRows(member(root, name), keys, key);
      };
    return {Code::ready, "", readPair(root, manifest, pair_id, select)};
  });
}

}  // namespace vln_memory
