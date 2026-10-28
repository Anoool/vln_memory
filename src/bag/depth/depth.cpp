#include "vln_memory/bag/depth/depth.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace vln_memory
{
namespace
{
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
  "Depth decoding requires IEEE-754 float32");

DepthResult failure(DepthErrorCode code, const char * message)
{
  return {code, message, std::nullopt};
}

bool checkedMultiply(std::size_t left, std::size_t right, std::size_t & result)
{
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {return false;}
  result = left * right;
  return true;
}

/** 显式按消息字节序拼装位模式，不要求像素指针或每行起点满足数值类型对齐。 */
std::uint32_t readBits(const std::uint8_t * data, std::size_t bytes, bool big_endian)
{
  std::uint32_t result = 0;
  for (std::size_t index = 0; index < bytes; ++index) {
    const auto offset = big_endian ? index : bytes - 1 - index;
    result = (result << 8U) | static_cast<std::uint32_t>(data[offset]);
  }
  return result;
}

double readSample(const std::uint8_t * data, DepthNumericType type, bool big_endian)
{
  if (type == DepthNumericType::uint16) {
    return static_cast<double>(readBits(data, 2, big_endian));
  }
  const auto bits = readBits(data, 4, big_endian);
  float value = 0.0F;
  // memcpy 传递位模式，避免 reinterpret_cast<float*> 的未对齐访问和别名问题。
  std::memcpy(&value, &bits, sizeof(value));
  return static_cast<double>(value);
}

/** 解码和配对检查共用同一套声明约束；不从相同尺寸推断几何注册。 */
bool validAlignment(
  std::uint32_t width, std::uint32_t height, std::string_view frame_id,
  const DepthAlignmentInfo & alignment)
{
  switch (alignment.state) {
    case DepthAlignment::unknown:
    case DepthAlignment::not_aligned:
      return alignment.rgb_grid.width == 0 && alignment.rgb_grid.height == 0 &&
             alignment.rgb_grid.frame_id.empty();
    case DepthAlignment::aligned_to_rgb:
      return !alignment.basis.empty() && !frame_id.empty() &&
             alignment.rgb_grid.frame_id == frame_id && alignment.rgb_grid.width == width &&
             alignment.rgb_grid.height == height && width > 0 && height > 0;
  }
  return false;
}

}  // namespace

const char * depthErrorCodeName(DepthErrorCode code) noexcept
{
  switch (code) {
    case DepthErrorCode::none: return "none";
    case DepthErrorCode::invalid_limits: return "invalid_limits";
    case DepthErrorCode::invalid_dimensions: return "invalid_dimensions";
    case DepthErrorCode::invalid_stamp: return "invalid_stamp";
    case DepthErrorCode::unsupported_encoding: return "unsupported_encoding";
    case DepthErrorCode::invalid_endianness: return "invalid_endianness";
    case DepthErrorCode::invalid_step: return "invalid_step";
    case DepthErrorCode::size_overflow: return "size_overflow";
    case DepthErrorCode::size_limit_exceeded: return "size_limit_exceeded";
    case DepthErrorCode::invalid_data: return "invalid_data";
    case DepthErrorCode::missing_unit: return "missing_unit";
    case DepthErrorCode::unsupported_unit: return "unsupported_unit";
    case DepthErrorCode::missing_convention: return "missing_convention";
    case DepthErrorCode::unsupported_convention: return "unsupported_convention";
    case DepthErrorCode::invalid_range: return "invalid_range";
    case DepthErrorCode::invalid_alignment: return "invalid_alignment";
  }
  return "unknown";
}

const char * depthUnitName(DepthUnit unit) noexcept
{
  switch (unit) {
    case DepthUnit::unspecified: return "unspecified";
    case DepthUnit::meters: return "m";
    case DepthUnit::millimeters: return "mm";
  }
  return "unknown";
}

const char * depthNumericTypeName(DepthNumericType type) noexcept
{
  switch (type) {
    case DepthNumericType::unknown: return "unknown";
    case DepthNumericType::uint16: return "uint16";
    case DepthNumericType::float32: return "float32";
  }
  return "unknown";
}

const char * depthAlignmentStatusName(DepthAlignmentStatus status) noexcept
{
  switch (status) {
    case DepthAlignmentStatus::unknown: return "unknown";
    case DepthAlignmentStatus::not_aligned: return "not_aligned";
    case DepthAlignmentStatus::aligned: return "aligned";
    case DepthAlignmentStatus::invalid_reference: return "invalid_reference";
    case DepthAlignmentStatus::inconsistent_declaration: return "inconsistent_declaration";
    case DepthAlignmentStatus::size_mismatch: return "size_mismatch";
    case DepthAlignmentStatus::frame_mismatch: return "frame_mismatch";
  }
  return "unknown";
}

DepthResult decodeDepth(
  const DepthImageView & input, const DepthOptions & options, const DepthLimits & limits)
{
  // 第一段：检查消息布局；编码仅决定数值类型，不决定单位或 RGB 对齐状态。
  if (limits.max_pixels == 0 || limits.max_input_bytes == 0) {
    return failure(DepthErrorCode::invalid_limits, "Depth limits must be positive");
  }
  if (input.width == 0 || input.height == 0) {
    return failure(DepthErrorCode::invalid_dimensions, "Image dimensions must be positive");
  }
  if (input.stamp.nanosec >= 1000000000U) {
    return failure(DepthErrorCode::invalid_stamp, "Stamp nanosec must be less than 1000000000");
  }
  DepthNumericType type = DepthNumericType::unknown;
  std::size_t bytes_per_pixel = 0;
  if (input.encoding == "16UC1") {
    type = DepthNumericType::uint16;
    bytes_per_pixel = 2;
  } else if (input.encoding == "32FC1") {
    type = DepthNumericType::float32;
    bytes_per_pixel = 4;
  } else {
    return failure(DepthErrorCode::unsupported_encoding, "Supported depth encodings: 16UC1, 32FC1");
  }
  if (input.is_bigendian > 1) {
    return failure(DepthErrorCode::invalid_endianness, "is_bigendian must be 0 or 1");
  }

  // 第二段：单位/深度含义必须显式指定；避免已米制的 32FC1 再被默认乘 0.001。
  double scale_to_m = 0.0;
  switch (options.input_unit) {
    case DepthUnit::meters: scale_to_m = 1.0; break;
    case DepthUnit::millimeters: scale_to_m = 0.001; break;
    case DepthUnit::unspecified:
      return failure(DepthErrorCode::missing_unit, "Depth input unit must be explicit");
    default:
      return failure(DepthErrorCode::unsupported_unit, "Supported depth units: meters, millimeters");
  }
  if (options.convention == DepthConvention::unspecified) {
    return failure(DepthErrorCode::missing_convention, "Depth convention must be explicit");
  }
  if (options.convention != DepthConvention::optical_z) {
    return failure(DepthErrorCode::unsupported_convention, "Only optical_z depth is supported");
  }
  if (!std::isfinite(options.min_depth_m) || options.min_depth_m < 0.0 ||
    std::isnan(options.max_depth_m) || options.max_depth_m <= options.min_depth_m)
  {
    return failure(DepthErrorCode::invalid_range, "Depth range must have finite min>=0 and max>min");
  }
  if (!validAlignment(input.width, input.height, input.frame_id, options.alignment)) {
    return failure(DepthErrorCode::invalid_alignment,
      "RGB alignment declaration requires consistent grid/frame and a non-empty basis");
  }

  // 第三段：所有尺寸运算与上限都在像素访问之前完成，源行填充也计入输入预算。
  std::size_t row_bytes = 0;
  std::size_t input_bytes = 0;
  std::size_t pixel_count = 0;
  std::size_t output_bytes = 0;
  std::size_t output_step = 0;
  if (!checkedMultiply(input.width, bytes_per_pixel, row_bytes) ||
    !checkedMultiply(input.step, input.height, input_bytes) ||
    !checkedMultiply(input.width, input.height, pixel_count) ||
    !checkedMultiply(pixel_count, sizeof(float), output_bytes) ||
    !checkedMultiply(input.width, sizeof(float), output_step) ||
    output_step > std::numeric_limits<std::uint32_t>::max())
  {
    return failure(DepthErrorCode::size_overflow, "Depth size cannot be represented safely");
  }
  if (input.step < row_bytes) {
    return failure(DepthErrorCode::invalid_step, "step is smaller than the encoded depth row");
  }
  if (pixel_count > limits.max_pixels || input_bytes > limits.max_input_bytes) {
    return failure(DepthErrorCode::size_limit_exceeded, "Image exceeds configured depth limits");
  }
  if (input.data == nullptr || input.data_size != input_bytes) {
    return failure(DepthErrorCode::invalid_data, "data must be non-null and exactly step*height bytes");
  }

  DepthFrame output;
  output.stamp = input.stamp;
  output.frame_id = std::string(input.frame_id);
  output.width = input.width;
  output.height = input.height;
  output.step = static_cast<std::uint32_t>(output_step);
  output.source_encoding = std::string(input.encoding);
  output.source_numeric_type = type;
  output.source_unit = options.input_unit;
  output.source_scale_to_m = scale_to_m;
  output.source_step = input.step;
  output.source_is_bigendian = input.is_bigendian != 0;
  output.alignment = options.alignment;
  output.min_depth_m = options.min_depth_m;
  output.max_depth_m = options.max_depth_m;
  output.depth_m.assign(pixel_count, std::numeric_limits<float>::quiet_NaN());

  // 第四段：逐像素转换，只将可信的正有限米制值写入预填 NaN 的输出。
  auto & statistics = output.statistics;
  for (std::size_t row = 0; row < input.height; ++row) {
    const auto * source_row = input.data + row * input.step;
    for (std::size_t column = 0; column < input.width; ++column) {
      const auto raw = readSample(
        source_row + column * bytes_per_pixel, type, input.is_bigendian != 0);
      if (!std::isfinite(raw)) {
        ++statistics.nonfinite_pixels;
      } else if (raw == 0.0) {
        ++statistics.zero_pixels;
      } else if (raw < 0.0) {
        ++statistics.negative_pixels;
      } else {
        const auto meters = raw * scale_to_m;
        if (meters < options.min_depth_m || meters > options.max_depth_m) {
          ++statistics.out_of_range_pixels;
        } else if (!std::isfinite(meters) || meters > std::numeric_limits<float>::max()) {
          ++statistics.unrepresentable_pixels;
        } else {
          const auto value = static_cast<float>(meters);
          // 极小正值在 float32 中可能下溢为零，不能作为有效深度保留。
          if (!std::isfinite(value) || value <= 0.0F) {
            ++statistics.unrepresentable_pixels;
          } else {
            output.depth_m[row * input.width + column] = value;
            ++statistics.valid_pixels;
          }
        }
      }
    }
  }
  statistics.invalid_pixels = pixel_count - statistics.valid_pixels;
  statistics.valid_ratio = static_cast<double>(statistics.valid_pixels) /
    static_cast<double>(pixel_count);
  return {DepthErrorCode::none, {}, std::move(output)};
}

DepthAlignmentResult checkDepthRgbAlignment(const DepthFrame & depth, const ImageGeometry & rgb)
{
  if (rgb.width == 0 || rgb.height == 0 || rgb.frame_id.empty()) {
    return {DepthAlignmentStatus::invalid_reference, "Actual RGB grid is incomplete"};
  }
  // 相同尺寸也不能把 unknown/not_aligned 升级为 aligned。
  switch (depth.alignment.state) {
    case DepthAlignment::unknown:
      return {DepthAlignmentStatus::unknown, "RGB alignment has not been declared"};
    case DepthAlignment::not_aligned:
      return {DepthAlignmentStatus::not_aligned, "Depth is explicitly not aligned to RGB"};
    case DepthAlignment::aligned_to_rgb: break;
    default:
      return {DepthAlignmentStatus::inconsistent_declaration, "Invalid RGB alignment state"};
  }
  if (!validAlignment(depth.width, depth.height, depth.frame_id, depth.alignment)) {
    return {DepthAlignmentStatus::inconsistent_declaration, "Stored alignment declaration is inconsistent"};
  }
  if (depth.width != rgb.width || depth.height != rgb.height) {
    return {DepthAlignmentStatus::size_mismatch, "Actual RGB dimensions do not match the registered depth"};
  }
  if (depth.frame_id != rgb.frame_id) {
    return {DepthAlignmentStatus::frame_mismatch, "Actual RGB optical frame does not match the declaration"};
  }
  return {DepthAlignmentStatus::aligned,
    "Declared registration matches RGB metadata; calibration and time sync remain external checks"};
}

}  // namespace vln_memory
