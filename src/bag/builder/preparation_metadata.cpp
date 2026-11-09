#include "preparation_metadata.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <memory>
#include <openssl/evp.h>

#include <vln_memory/bag/common/sensor_names.hpp>

namespace vln_memory::detail
{
namespace
{
using Json = nlohmann::json;
using Code = PreparationReadCode;

/** @brief 只中止当前读取，合法传感器失败状态另行保留。 */
void require(bool value, Code code, const std::string & message)
{
  if (!value) {throw PreparationMetadataError(code, message);}
}

/** @brief 规范十进制字符串不经过 double，保留全部 64 位范围。 */
template<typename T>
T decimal(const Json & value)
{
  require(value.is_string(), Code::invalid_metadata, "Expected decimal string");
  const auto & text = value.get_ref<const std::string &>();
  T number{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), number);
  require(parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size() &&
    std::to_string(number) == text, Code::invalid_metadata, "Invalid/out-of-range decimal string");
  return number;
}

/** @brief 无符号 JSON 整数拒绝负数和浮点；不把 uint64 大值转成 int64。 */
std::uint64_t unsignedInteger(const Json & value)
{
  require(value.is_number_integer() && (value.is_number_unsigned() || value.get<std::int64_t>() >= 0),
    Code::invalid_metadata, "Expected unsigned integer");
  return value.get<std::uint64_t>();
}

/** @brief 有界 uint32 字段允许语义坏值原样保留，范围外属于格式损坏。 */
std::uint32_t u32(const Json & value)
{
  const auto number = unsignedInteger(value);
  require(number <= std::numeric_limits<std::uint32_t>::max(), Code::invalid_metadata, "uint32 overflow");
  return static_cast<std::uint32_t>(number);
}

/** @brief 有界原始秒数，不允许大无符号整数发生环绕。 */
std::int32_t i32(const Json & value)
{
  require(value.is_number_integer(), Code::invalid_metadata, "Expected integer seconds");
  if (value.is_number_unsigned()) {
    require(value.get<std::uint64_t>() <= static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()),
      Code::invalid_metadata, "Seconds overflow");
  } else {
    require(value.get<std::int64_t>() >= std::numeric_limits<std::int32_t>::min() &&
      value.get<std::int64_t>() <= std::numeric_limits<std::int32_t>::max(), Code::invalid_metadata, "Seconds overflow");
  }
  return value.get<std::int32_t>();
}

/** @brief 格式必须是有限数；源非有限参数的 null 由原始标定分支显式处理。 */
double finiteNumber(const Json & value)
{
  require(value.is_number(), Code::invalid_metadata, "Expected number");
  const auto number = value.get<double>();
  require(std::isfinite(number), Code::invalid_metadata, "Nonfinite metadata number");
  return number;
}

/** @brief 已保存的可选阈值必须非负有限，不赋予隐含上限。 */
std::optional<double> limit(const Json & value)
{
  if (value.is_null()) {return std::nullopt;}
  const auto number = finiteNumber(value);
  require(number >= 0, Code::invalid_metadata, "Negative metadata limit");
  return number;
}

/** @brief 按稳定名称恢复原始标定错误码，未知词汇属于不支持的元数据。 */
CameraInfoErrorCode cameraCode(const Json & value)
{
  const auto name = value.get<std::string>();
  for (const auto code : {CameraInfoErrorCode::none, CameraInfoErrorCode::invalid_topic,
    CameraInfoErrorCode::invalid_frame, CameraInfoErrorCode::invalid_stamp, CameraInfoErrorCode::invalid_dimensions,
    CameraInfoErrorCode::uncalibrated, CameraInfoErrorCode::nonfinite_parameters,
    CameraInfoErrorCode::unsupported_distortion_model, CameraInfoErrorCode::invalid_distortion,
    CameraInfoErrorCode::invalid_intrinsics, CameraInfoErrorCode::invalid_rectification,
    CameraInfoErrorCode::invalid_projection, CameraInfoErrorCode::invalid_roi,
    CameraInfoErrorCode::unsupported_binning, CameraInfoErrorCode::invalid_options}) {
    if (name == cameraInfoErrorCodeName(code)) {return code;}
  }
  throw PreparationMetadataError(Code::invalid_metadata, "Unknown camera validation code: " + name);
}

/** @brief 兼容策略沿用写端既有名称，不把原始 D8 解释为 D5。 */
CameraInfoCompatibility compatibility(const Json & value)
{
  const auto name = value.get<std::string>();
  for (const auto policy : {CameraInfoCompatibility::strict, CameraInfoCompatibility::plumb_bob_zero_tail_d8}) {
    if (name == cameraInfoCompatibilityName(policy)) {return policy;}
  }
  throw PreparationMetadataError(Code::invalid_metadata, "Unknown camera compatibility policy");
}

/** @brief 从显式 manifest 配置恢复绑定；同一时钟关系已由上下文声明核对。 */
CameraInfoMatchOptions selectionOptions(const Json & value)
{
  CameraInfoMatchOptions out;
  out.camera_info_topic = value.at("camera_info_topic").get<std::string>();
  out.image_topic = value.at("image_topic").get<std::string>();
  out.optical_frame = value.at("optical_frame").get<std::string>();
  out.binding_basis = value.at("binding_basis").get<std::string>();
  const auto domain = value.at("image_domain").get<std::string>();
  require(domain == "raw" || domain == "rectified", Code::invalid_metadata, "Unknown image domain");
  out.image_domain = domain == "raw" ? CameraImageDomain::raw : CameraImageDomain::rectified;
  const auto policy = value.at("time_policy").get<std::string>();
  require(policy == "synchronized" || policy == "static_calibration", Code::invalid_metadata, "Unknown calibration time policy");
  out.time_policy = policy == "synchronized" ? CameraInfoTimePolicy::synchronized : CameraInfoTimePolicy::static_calibration;
  out.max_delta_ns = decimal<std::int64_t>(value.at("max_delta_ns"));
  out.compatibility = compatibility(value.at("compatibility"));
  out.clock_relation = SensorClockRelation::same_clock;
  require(out.max_delta_ns >= 0 && isResolvedTopicName(out.camera_info_topic) &&
    isResolvedTopicName(out.image_topic) && !out.optical_frame.empty() && !out.binding_basis.empty(),
    Code::invalid_metadata, "Invalid calibration binding configuration");
  return out;
}

/** @brief 同步选择的实际检查结果保留其代码与可空差值。 */
SensorTimeCheck timing(const Json & value)
{
  SensorTimeCheck out;
  const auto name = value.at("code").get<std::string>();
  bool found = false;
  for (const auto code : {SensorTimeCode::matched, SensorTimeCode::outside_tolerance, SensorTimeCode::invalid_stamp,
    SensorTimeCode::invalid_tolerance, SensorTimeCode::clock_unknown, SensorTimeCode::clock_mismatch}) {
    if (name == sensorTimeCodeName(code)) {out.code = code; found = true; break;}
  }
  require(found, Code::invalid_metadata, "Unknown calibration timing code");
  if (!value.at("delta_ns").is_null()) {out.delta_ns = decimal<std::int64_t>(value.at("delta_ns"));}
  return out;
}

/** @brief 原始浮点数组的 null 只可能表示写端明确声明的非有限源值。 */
bool missingParameters(const Json & value)
{
  bool missing = false;
  for (const char * name : {"d", "k", "r", "p"}) {
    const auto & array = value.at(name);
    require(array.is_array(), Code::invalid_metadata, "Expected calibration array");
    for (const auto & number : array) {
      if (number.is_null()) {missing = true;} else {(void)finiteNumber(number);}
    }
  }
  require(value.at("k").size() == 9 && value.at("r").size() == 9 && value.at("p").size() == 12,
    Code::invalid_metadata, "Invalid calibration matrix length");
  return missing;
}

/** @brief 全部原值可表达时恢复拥有型 CameraInfo；不运行规范化或替换坏数值。 */
std::optional<CameraInfoData> originalInfo(const Json & value, bool unavailable)
{
  CameraInfoData out;
  const auto & stamp = value.at("stamp");
  out.stamp = {i32(stamp.at("sec")), u32(stamp.at("nanosec"))};
  out.topic = value.at("source").at("topic").get<std::string>();
  out.frame_id = value.at("frame_id").get<std::string>();
  out.width = u32(value.at("width"));
  out.height = u32(value.at("height"));
  out.distortion_model = value.at("distortion_model").get<std::string>();
  out.binning_x = u32(value.at("binning_x"));
  out.binning_y = u32(value.at("binning_y"));
  const auto & roi = value.at("roi");
  out.roi = {u32(roi.at("x_offset")), u32(roi.at("y_offset")), u32(roi.at("width")),
    u32(roi.at("height")), roi.at("do_rectify").get<bool>()};
  if (unavailable) {return std::nullopt;}
  out.d = value.at("d").get<std::vector<double>>();
  out.k = value.at("k").get<std::array<double, 9>>();
  out.r = value.at("r").get<std::array<double, 9>>();
  out.p = value.at("p").get<std::array<double, 12>>();
  return out;
}

/** @brief 重建值只用于一致性比较，不覆盖返回的原始有效模型。 */
template<typename Values>
bool equalParameters(const Values & a, const Values & b)
{
  if (a.size() != b.size()) {return false;}
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
      std::abs(a[i] - b[i]) > 1e-10 * std::max({1.0, std::abs(a[i]), std::abs(b[i])})) {return false;}
  }
  return true;
}

/** @brief 与现有深度单位解释保持一致，未知值不能默认为米。 */
DepthUnit unit(const Json & value)
{
  if (value == "m") {return DepthUnit::meters;}
  if (value == "mm") {return DepthUnit::millimeters;}
  throw PreparationMetadataError(Code::invalid_media, "Unknown depth storage/source unit");
}

/** @brief 单位与比例必须显式一致。 */
bool unitScale(DepthUnit value, double scale)
{
  return (value == DepthUnit::meters && scale == 1.0) ||
         (value == DepthUnit::millimeters && scale == 0.001);
}
}  // namespace

/** @brief 从流中读取一条大小有界的 JSONL，行尾换行不计入预算。 */
bool readPreparationLine(std::istream & input, std::string & line)
{
  line.clear();
  std::array<char, 4096> chunk{};
  for (;;) {
    input.getline(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    const auto count = static_cast<std::size_t>(input.gcount());
    require(!input.bad(), Code::io_error, "Cannot read preparation JSONL");
    const bool complete = !input.fail() && !input.eof();
    const auto data_count = complete ? count - 1 : count;
    require(data_count <= kMaxPreparationJsonBytes - line.size(), Code::invalid_metadata,
      "Preparation JSONL row exceeds size limit");
    line.append(chunk.data(), data_count);
    if (complete) {return true;}
    if (input.eof()) {return !line.empty();}
    require(count == chunk.size() - 1, Code::io_error, "Cannot read preparation JSONL");
    input.clear(input.rdstate() & ~std::ios::failbit);
  }
}

/** @brief 有界读取 manifest 原始字节，保留其原始指纹。 */
std::string readPreparationDocument(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  require(input.is_open(), Code::io_error, "Cannot open preparation manifest: " + path.string());
  const auto end = static_cast<std::streamoff>(input.tellg());
  require(end > 0 && static_cast<std::uintmax_t>(end) <= kMaxPreparationJsonBytes,
    Code::invalid_metadata, "Empty or oversized preparation manifest");
  std::string bytes(static_cast<std::size_t>(end), '\0');
  input.seekg(0);
  require(static_cast<bool>(input.read(bytes.data(), end)) &&
    input.peek() == std::char_traits<char>::eof() && !input.bad(),
    Code::io_error, "Cannot read complete preparation manifest");
  return bytes;
}

/** @brief 拒绝重复键和深层嵌套，避免 JSON 库静默覆盖字段。 */
nlohmann::json parsePreparationJson(std::string_view bytes)
{
  require(!bytes.empty() && bytes.size() <= kMaxPreparationJsonBytes,
    Code::invalid_metadata, "Empty or oversized preparation JSON");
  std::vector<std::set<std::string>> keys;
  /** @brief 每层对象单独记录键；过深结构在构建完整 DOM 前停止。 */
  const auto callback = [&keys](int depth, Json::parse_event_t event, Json & value) {
      require(depth <= 64, Code::invalid_metadata, "Preparation JSON nesting exceeds 64");
      if (event == Json::parse_event_t::object_start) {keys.emplace_back();}
      if (event == Json::parse_event_t::key) {
        require(!keys.empty() && keys.back().insert(value.get<std::string>()).second,
          Code::invalid_metadata, "Duplicate preparation JSON key");
      }
      if (event == Json::parse_event_t::object_end) {keys.pop_back();}
      return true;
    };
  return Json::parse(bytes.begin(), bytes.end(), callback);
}

namespace
{
using HashContext = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;

/** @brief 初始化 SHA-256，统一字节缓冲和文件的失败语义。 */
HashContext beginSha256()
{
  HashContext context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  require(context && EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) == 1,
    Code::io_error, "Cannot initialize preparation hash");
  return context;
}

/** @brief 以规范小写十六进制输出 SHA-256。 */
std::string finishSha256(EVP_MD_CTX * context)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0;
  require(EVP_DigestFinal_ex(context, digest.data(), &size) == 1 && size == 32,
    Code::io_error, "Cannot finish preparation hash");
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(64);
  for (unsigned int i = 0; i < size; ++i) {
    result.push_back(hex[digest[i] >> 4]);
    result.push_back(hex[digest[i] & 15]);
  }
  return result;
}
}  // namespace

/** @brief 从已经读取的 manifest 字节计算原始指纹。 */
std::string preparationSha256(std::string_view bytes)
{
  auto context = beginSha256();
  require(EVP_DigestUpdate(context.get(), bytes.data(), bytes.size()) == 1,
    Code::io_error, "Cannot hash preparation manifest");
  return finishSha256(context.get());
}

/** @brief 以固定大小缓冲校验 CDR 字节，不持有整个原始消息。 */
std::string preparationFileSha256(const std::filesystem::path & path)
{
  std::ifstream file(path, std::ios::binary);
  require(file.is_open(), Code::io_error, "Cannot open evidence for digest: " + path.string());
  auto context = beginSha256();
  std::array<char, 64U * 1024U> buffer;
  do {
    file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    require(EVP_DigestUpdate(context.get(), buffer.data(), static_cast<std::size_t>(file.gcount())) == 1,
      Code::io_error, "Cannot hash evidence: " + path.string());
  } while (file);
  require(file.eof() && !file.bad(), Code::io_error, "Cannot read evidence: " + path.string());
  return finishSha256(context.get());
}

/** @brief 按字节扫描 JSONL；包含实际存在的行尾，访客只借用当前行。 */
std::string scanPreparationJsonl(
  const std::filesystem::path & path,
  const std::function<void(std::streamoff, const std::string &)> & visit)
{
  std::ifstream file(path, std::ios::binary);
  require(file.is_open(), Code::io_error, "Cannot open JSONL: " + path.string());
  auto hash = beginSha256();
  for (std::string line;;) {
    const auto offset = file.tellg();
    if (!readPreparationLine(file, line)) {break;}
    require(offset >= std::streampos(0) && EVP_DigestUpdate(hash.get(), line.data(), line.size()) == 1 &&
      (file.eof() || EVP_DigestUpdate(hash.get(), "\n", 1) == 1),
      Code::io_error, "Cannot hash JSONL: " + path.string());
    visit(static_cast<std::streamoff>(offset), line);
    if (file.eof()) {break;}
  }
  return finishSha256(hash.get());
}

/** @brief 校验规范小写 SHA-256，避免同一摘要有多种编码。 */
bool validSha256(std::string_view value)
{
  return value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

/** @brief 检查索引路径由根目录内的规范相对分量组成。 */
bool validMediaPath(const std::string & name)
{
  const std::filesystem::path path(name);
  if (name.empty() || name.find('\0') != std::string::npos || !path.is_relative() ||
    path != path.lexically_normal() || path.filename().empty()) {return false;}
  for (const auto & part : path) {
    if (part == "." || part == "..") {return false;}
  }
  return true;
}

/** @brief 严格解析媒体索引并同时计算所读字节的摘要。 */
std::optional<PreparationMediaIndex> readPreparationMediaIndex(
  const std::filesystem::path & root, const Json & manifest)
{
  const auto path = root / kPreparationMediaIndexFile;
  if (!manifest.contains("media_integrity")) {
    require(!std::filesystem::exists(path) && !std::filesystem::is_symlink(path),
      Code::invalid_metadata, "Media index is present without manifest binding");
    return std::nullopt;
  }
  const auto & seal = manifest.at("media_integrity");
  const auto digest = seal.at("index_sha256").get<std::string>();
  require(seal.at("schema_version") == kPreparationMediaIndexSchema && validSha256(digest) &&
    (seal.at("baseline") == "exported" || seal.at("baseline") == "legacy_conversion"),
    Code::invalid_metadata, "Invalid preparation media integrity declaration");
  return readSealedMediaIndex(path, digest);
}

/** @brief 验证发布时的索引内容，保留消息 ID、路径和摘要的一对一绑定。 */
PreparationMediaIndex readSealedMediaIndex(const std::filesystem::path & path, std::string_view digest)
{
  require(!std::filesystem::is_symlink(path) && std::filesystem::is_regular_file(path),
    Code::invalid_metadata, "Missing or invalid preparation media index");
  PreparationMediaIndex result;
  /** @brief 校验并索引单条媒体绑定。 */
  const auto index_row = [&result](std::streamoff, const std::string & line) {
      const auto row = parsePreparationJson(line);
      const auto id = row.at("message_id").get<std::string>();
      PreparationMediaDigest media{row.at("media_path").get<std::string>(), row.at("sha256").get<std::string>()};
      require(row.size() == 4 && row.at("schema_version") == kPreparationMediaIndexSchema &&
        !id.empty() && validMediaPath(media.path) && validSha256(media.sha256) &&
        result.emplace(id, std::move(media)).second,
        Code::invalid_metadata, "Invalid or duplicate preparation media entry");
    };
  require(scanPreparationJsonl(path, index_row) == digest, Code::invalid_metadata,
    "Preparation media index digest differs from manifest");
  return result;
}

/** @brief 绑定帧消息和其媒体路径，拒绝把已验证内容误配给另一帧。 */
const PreparationMediaDigest * preparationFrameMedia(
  const std::optional<PreparationMediaIndex> & index, const Json & frame)
{
  if (!index) {return nullptr;}
  const auto id = frame.at("source").at("message_id").get<std::string>();
  const auto found = index->find(id);
  const auto & path = frame.at("media_path");
  if (path.is_null()) {
    require(found == index->end(), Code::invalid_reference, "Frame omits sealed media: " + id);
    return nullptr;
  }
  require(found != index->end() && path == found->second.path,
    Code::invalid_reference, "Frame media reference differs from sealed index: " + id);
  return &found->second;
}

/** @brief 编码一条媒体记录，始终保留固定 schema。 */
Json preparationMediaRow(std::string_view message_id, std::string_view path, std::string_view sha256)
{
  return {{"schema_version", kPreparationMediaIndexSchema}, {"message_id", message_id},
    {"media_path", path}, {"sha256", sha256}};
}

/** @brief 对外共享的有符号门限/差值解析。 */
std::int64_t preparationSigned(const Json & value)
{
  return decimal<std::int64_t>(value);
}

/** @brief 数据集级配置只含来源事实；新索引入口在打开时缓存一次。 */
PreparedPair readPreparationContext(const Json & manifest)
{
  PreparedPair out;
  const auto & options = manifest.at("effective_options");
  out.builder_version = static_cast<int>(unsignedInteger(manifest.at("builder_version")));
  out.bag_id = manifest.at("bag_id").get<std::string>();
  out.map_frame = options.at("map_frame").get<std::string>();
  out.body_frame = options.at("body_frame").get<std::string>();
  out.time_scope = {options.at("clock_id").get<std::string>(), options.at("segment_id").get<std::string>()};
  out.clock_basis = options.at("clock_basis").get<std::string>();
  out.segment_basis = options.at("segment_basis").get<std::string>();
  out.timestamp_match_policy = manifest.at("timestamp_match_policy").get<std::string>();
  require(!out.timestamp_match_policy.empty(), Code::invalid_metadata, "Missing dataset pairing policy");
  out.sync_tolerance_ns = decimal<std::int64_t>(options.at("sync_tolerance_ns"));
  out.tf_max_gap_ns = decimal<std::int64_t>(options.at("tf_max_gap_ns"));
  out.tf_jump_limits = {limit(options.at("tf_max_translation_step_m")), limit(options.at("tf_max_rotation_step_rad"))};
  require(!out.bag_id.empty() && !out.map_frame.empty() && !out.body_frame.empty() &&
    !out.time_scope.clock_id.empty() && !out.time_scope.segment_id.empty() &&
    !out.clock_basis.empty() && !out.segment_basis.empty() && options.at("clock_relation") == "same_clock" &&
    out.sync_tolerance_ns >= 0 && out.tf_max_gap_ns >= 0, Code::invalid_metadata, "Invalid dataset/time/TF context");
  out.rgb_calibration.selection = selectionOptions(options.at("rgb_camera"));
  out.depth_calibration.selection = selectionOptions(options.at("depth_camera"));
  return out;
}

/** @brief 返回本帧实际选择状态，候选原始非有限值不可从 JSON null 猜回。 */
PreparedCalibration readPreparationCalibration(
  const Json & frame, const Json * record, const CameraInfoMatchOptions & selection)
{
  PreparedCalibration out;
  out.selection = selection;
  if (frame.contains("calibration")) {
    const auto & saved = frame.at("calibration");
    out.calibration_id = saved.at("calibration_id").get<std::string>();
    out.selection_code = saved.at("code").get<std::string>();
    out.selection_message = saved.at("message").get<std::string>();
    out.calibration_code = cameraCode(saved.at("calibration_code"));
    out.used_static_calibration = saved.at("used_static_calibration").get<bool>();
    if (saved.contains("timing")) {out.timing = timing(saved.at("timing"));}
  }
  if (!record) {
    require(out.calibration_id.empty(), Code::invalid_reference, "Selected calibration is missing");
    return out;
  }
  require(record->at("calibration_id") == out.calibration_id, Code::invalid_reference, "Calibration identity differs");
  out.source_message_id = record->at("source").at("message_id").get<std::string>();
  out.source_validation_code = cameraCode(record->at("validation_code"));
  out.source_validation_message = record->at("validation_message").get<std::string>();
  out.normalized_validation_code = cameraCode(record->at("normalized_code"));
  out.source_compatibility = compatibility(record->at("compatibility"));
  out.normalization_applied = record->at("normalization_applied").get<bool>();
  out.source_nonfinite = record->at("source_nonfinite").get<bool>();
  const bool missing = missingParameters(*record);
  require(missing == out.source_nonfinite, Code::invalid_metadata, "Calibration nonfinite declaration contradicts raw parameters");
  out.original = originalInfo(*record, missing);
  if (missing) {out.original_unavailable_reason = "nonfinite_source_parameters_not_representable";}
  return out;
}

/** @brief 只对声明 matched/ready 的记录重建，坏候选的来源与错误继续可读。 */
void checkPreparationModel(
  const PreparedCalibration & calibration, const std::optional<CameraModel> & model,
  const CameraImageReference & image)
{
  const auto & c = calibration;
  if (c.selection_code == "matched") {
    require(c.original.has_value(), Code::invalid_reference, "Matched calibration has no representable original");
    const auto match = checkCameraInfoImage(*c.original, image, c.selection);
    require(match.matched() && c.calibration_code == match.calibration_code &&
      c.used_static_calibration == match.used_static_calibration && c.timing.has_value() == match.timing.has_value(),
      Code::invalid_reference, "Saved calibration match differs from original/options");
    if (c.timing) {
      require(c.timing->code == match.timing->code && c.timing->delta_ns == match.timing->delta_ns,
        Code::invalid_reference, "Saved calibration timing differs");
    }
  }
  if (!model) {return;}
  require(c.original && c.selection_code == "matched" && c.normalized_validation_code == CameraInfoErrorCode::none,
    Code::invalid_reference, "Ready model lacks valid matched/normalized calibration");
  const auto rebuilt = buildCameraModel(*c.original, image, c.selection);
  require(rebuilt.ok(), Code::invalid_reference, "Cannot rebuild saved camera model: " + rebuilt.message);
  const auto & expected = *rebuilt.model;
  require(model->width == expected.width && model->height == expected.height && model->frame_id == expected.frame_id &&
    model->image_domain == expected.image_domain && model->distortion_model == expected.distortion_model &&
    model->normalization_applied == expected.normalization_applied &&
    c.normalization_applied == expected.normalization_applied &&
    equalParameters(model->k, expected.k) && equalParameters(model->d, expected.d) &&
    equalParameters(model->r, expected.r) && equalParameters(model->p, expected.p),
    Code::invalid_reference, "Saved model differs from original calibration/options");
}

/** @brief 解释单帧实际存储；没有媒体/存储说明的失败帧不伪造存储类型。 */
std::optional<PreparedDepthStorage> readPreparationDepthStorage(const Json & frame, const Json & depth_options)
{
  const auto & details = frame.at("details");
  if (!details.contains("storage") && frame.at("media_path").is_null()) {return std::nullopt;}
  PreparedDepthStorage out;
  const Json * storage = &depth_options;
  out.legacy_metric_npy = !details.contains("storage");
  if (!out.legacy_metric_npy) {storage = &details.at("storage");}
  out.format = "npy";
  out.dtype = "float32";
  out.unit = DepthUnit::meters;
  out.scale_to_m = 1;
  out.invalid_rule = "nan";
  if (!out.legacy_metric_npy) {
    out.format = storage->at("format").get<std::string>();
    out.dtype = storage->at("dtype").get<std::string>();
    out.unit = unit(storage->at("unit"));
    out.scale_to_m = finiteNumber(storage->at("scale_to_m"));
    out.invalid_rule = storage->at("invalid_rule").get<std::string>();
  }
  out.min_depth_m = finiteNumber(storage->at("min_depth_m"));
  out.max_depth_m = limit(storage->at("max_depth_m"));
  out.source_encoding = details.at("source_encoding").get<std::string>();
  out.source_unit = unit(details.at("source_unit"));
  out.source_scale_to_m = finiteNumber(details.at("source_scale_to_m"));
  out.valid_pixels = unsignedInteger(details.at("valid_pixels"));
  out.invalid_pixels = unsignedInteger(details.at("invalid_pixels"));
  const bool npy = out.format == "npy" && out.dtype == "float32" && out.unit == DepthUnit::meters &&
    out.scale_to_m == 1 && out.invalid_rule == "nan";
  const bool png = out.format == "png" && out.dtype == "uint16" &&
    out.invalid_rule == "zero_or_out_of_range_to_nan" && out.source_encoding == "16UC1" &&
    out.unit == out.source_unit && out.scale_to_m == out.source_scale_to_m;
  require((npy || png) && unitScale(out.unit, out.scale_to_m) && unitScale(out.source_unit, out.source_scale_to_m) &&
    (out.source_encoding == "16UC1" || out.source_encoding == "32FC1") && out.min_depth_m >= 0 &&
    (!out.max_depth_m || *out.max_depth_m >= out.min_depth_m) &&
    details.at("unit") == "meters" && details.at("convention") == "optical_z",
    Code::invalid_media, "Invalid depth storage/source interpretation");
  const auto pixels = static_cast<std::uint64_t>(u32(frame.at("width"))) * u32(frame.at("height"));
  require(out.valid_pixels <= pixels && out.invalid_pixels == pixels - out.valid_pixels,
    Code::invalid_media, "Depth metadata quality counts differ from grid");
  return out;
}

/** @brief 完整返回当前引用消息的精确摘要；文件检查由 reader 的路径边界执行。 */
PreparedMessageSummary readPreparationMessage(const Json & row)
{
  PreparedMessageSummary out;
  out.message_id = row.at("message_id").get<std::string>();
  out.topic = row.at("topic").get<std::string>();
  out.file_index = unsignedInteger(row.at("file_index"));
  out.topic_ordinal = decimal<std::uint64_t>(row.at("topic_ordinal"));
  out.message_order = decimal<std::uint64_t>(row.at("message_order"));
  out.bag_record_time_ns = decimal<std::int64_t>(row.at("bag_record_time_ns"));
  out.serialized_bytes = unsignedInteger(row.at("serialized_bytes"));
  out.payload_sha256 = row.at("payload_sha256").get<std::string>();
  require(!out.message_id.empty() && isResolvedTopicName(out.topic) && out.payload_sha256.size() == 64,
    Code::invalid_metadata, "Invalid message summary");
  for (const char c : out.payload_sha256) {
    require((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'), Code::invalid_metadata, "Invalid message digest");
  }
  if (!row.at("raw_path").is_null()) {out.raw_path = row.at("raw_path").get<std::string>();}
  return out;
}

}  // namespace vln_memory::detail
