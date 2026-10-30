#include "vln_memory/bag/depth/depth.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

/**
 * @file
 * 实现路径：消息布局 -> 显式数值语义 -> 安全容量 -> 逐像素转换与分类统计。
 * 输出 NaN 表示无效像素；全无效仍保留成功帧，交给组装层决定是否可用。
 */
namespace vln_memory
{
namespace
{
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
  "Depth decoding requires IEEE-754 float32");

/** @brief 用给定错误码和原因创建失败结果；不返回部分帧，原因字符串由结果拥有。 */
DepthResult failure(DepthErrorCode code, const char * message)
{
  return {code, message, std::nullopt};
}

/** @brief 检查两个容量因子的乘积；溢出返回 false，成功才写入 result。 */
bool checkedMultiply(std::size_t left, std::size_t right, std::size_t & result)
{
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

/** @brief 从已校验的 2/4 个借用字节拼装位模式；按指定字节序解释，无地址对齐要求。 */
std::uint32_t readBits(const std::uint8_t * data, std::size_t byte_count, bool is_big_endian)
{
  std::uint32_t result = 0;
  for (std::size_t index = 0; index < byte_count; ++index) {
    const auto offset = is_big_endian ? index : byte_count - 1 - index;
    result = (result << 8U) | static_cast<std::uint32_t>(data[offset]);
  }
  return result;
}

/** @brief 从已校验的借用字节读取一个源数值；按指定类型/字节序解释，不做单位换算。 */
double readSample(const std::uint8_t * data, DepthNumericType numeric_type, bool is_big_endian)
{
  if (numeric_type == DepthNumericType::uint16) {
    return static_cast<double>(readBits(data, 2, is_big_endian));
  }
  const auto bits = readBits(data, 4, is_big_endian);
  float value = 0.0F;
  // memcpy 传递位模式，避免 reinterpret_cast<float*> 的未对齐访问和别名问题。
  std::memcpy(&value, &bits, sizeof(value));
  return static_cast<double>(value);
}

/**
 * @brief 核对深度布局与注册状态、目标网格、依据是否一致，不比较像素或时间。
 * @return 未知枚举或不完整/矛盾声明返回 false；不保存借用的帧名和依据。
 */
bool validAlignment(
  std::uint32_t width, std::uint32_t height, std::string_view frame_id,
  DepthAlignment alignment, const ImageGeometry & rgb_grid, std::string_view alignment_basis)
{
  switch (alignment) {
    case DepthAlignment::unknown:
    case DepthAlignment::not_aligned:
      return rgb_grid.width == 0 && rgb_grid.height == 0 &&
             rgb_grid.frame_id.empty();
    case DepthAlignment::aligned_to_rgb:
      return !alignment_basis.empty() && !frame_id.empty() &&
             rgb_grid.frame_id == frame_id && rgb_grid.width == width &&
             rgb_grid.height == height && width > 0 && height > 0;
  }
  return false;
}

/**
 * @brief 将一个源值转成米制 float32，并只增加一个质量计数。
 * @param raw_value 按编码读出的原始数值，按 output.source_scale_to_m 换算为米。
 * @param output 已初始化的范围、比例与累计计数；不修改其像素缓冲区。
 * @return 有效正深度，或 quiet NaN；先分类源值，再检查米制范围与可表示性。
 */
float convertDepthSample(double raw_value, DepthFrame & output)
{
  const auto invalid_depth = std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(raw_value)) {
    ++output.nonfinite_pixels;
    return invalid_depth;
  }
  if (raw_value == 0.0) {
    ++output.zero_pixels;
    return invalid_depth;
  }
  if (raw_value < 0.0) {
    ++output.negative_pixels;
    return invalid_depth;
  }

  const auto depth_m = raw_value * output.source_scale_to_m;
  if (depth_m < output.min_depth_m || depth_m > output.max_depth_m) {
    ++output.out_of_range_pixels;
    return invalid_depth;
  }
  if (!std::isfinite(depth_m) || depth_m > std::numeric_limits<float>::max()) {
    ++output.unrepresentable_pixels;
    return invalid_depth;
  }

  const auto depth_value = static_cast<float>(depth_m);
  // 极小正值可能在 float32 中下溢为零，仍须归为不可表示深度。
  if (!std::isfinite(depth_value) || depth_value <= 0.0F) {
    ++output.unrepresentable_pixels;
    return invalid_depth;
  }
  ++output.valid_pixels;
  return depth_value;
}

/**
 * @brief 遍历已校验图像，填满预分配的独立输出并汇总质量。
 * @note 输入只在本调用期间借用；输出元数据已初始化、计数为零。
 *       每个像素读取一次，时间/额外空间为 O(width*height)/O(1)。
 */
void convertDepthPixels(const DepthImageView & input, DepthFrame & output)
{
  std::size_t bytes_per_pixel = 4;
  if (output.source_numeric_type == DepthNumericType::uint16) {
    bytes_per_pixel = 2;
  }
  const bool is_big_endian = input.is_bigendian != 0;
  for (std::size_t row = 0; row < input.height; ++row) {
    const auto * source_row = input.data + row * input.step;
    for (std::size_t column = 0; column < input.width; ++column) {
      const auto raw_value = readSample(
        source_row + column * bytes_per_pixel, output.source_numeric_type, is_big_endian);
      output.depth_m[row * input.width + column] = convertDepthSample(raw_value, output);
    }
  }
  output.invalid_pixels = output.depth_m.size() - output.valid_pixels;
  output.valid_ratio = static_cast<double>(output.valid_pixels) /
    static_cast<double>(output.depth_m.size());
}

}  // namespace

/** @brief 返回错误码的稳定静态名称；未知值为 unknown，无分配。 */
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

/** @brief 返回源单位的稳定静态名称；未知值为 unknown，无分配。 */
const char * depthUnitName(DepthUnit unit) noexcept
{
  switch (unit) {
    case DepthUnit::unspecified: return "unspecified";
    case DepthUnit::meters: return "m";
    case DepthUnit::millimeters: return "mm";
  }
  return "unknown";
}

/** @brief 返回源数值类型的稳定静态名称；未知值为 unknown，无分配。 */
const char * depthNumericTypeName(DepthNumericType numeric_type) noexcept
{
  switch (numeric_type) {
    case DepthNumericType::unknown: return "unknown";
    case DepthNumericType::uint16: return "uint16";
    case DepthNumericType::float32: return "float32";
  }
  return "unknown";
}

/** @brief 返回注册核对状态的稳定静态名称；未知值为 unknown，无分配。 */
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

/**
 * @brief 按布局、语义、容量的顺序校验，再初始化独立输出并转换像素。
 * @note 输入/options 只在调用期间借用；失败保留原有错误优先级，无部分帧。
 *       输出拥有全部数据；系统分配异常继续上抛，时间/输出空间均为 O(像素数)。
 */
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
  DepthNumericType numeric_type = DepthNumericType::unknown;
  std::size_t bytes_per_pixel = 0;
  if (input.encoding == "16UC1") {
    numeric_type = DepthNumericType::uint16;
    bytes_per_pixel = 2;
  } else if (input.encoding == "32FC1") {
    numeric_type = DepthNumericType::float32;
    bytes_per_pixel = 4;
  } else {
    return failure(DepthErrorCode::unsupported_encoding, "Supported depth encodings: 16UC1, 32FC1");
  }
  if (input.is_bigendian > 1) {
    return failure(DepthErrorCode::invalid_endianness, "is_bigendian must be 0 or 1");
  }

  // 第二段：单位/深度含义必须显式指定；避免已米制的 32FC1 再被默认乘 0.001。
  // 编码只决定如何读原始数值。比例只能来自显式单位，防止米/毫米重复换算。
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
  if (!validAlignment(input.width, input.height, input.frame_id,
      options.alignment, options.rgb_grid, options.alignment_basis))
  {
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
  output.source_numeric_type = numeric_type;
  output.source_unit = options.input_unit;
  output.source_scale_to_m = scale_to_m;
  output.source_step = input.step;
  output.source_is_bigendian = input.is_bigendian != 0;
  output.alignment = options.alignment;
  output.rgb_grid = options.rgb_grid;
  output.alignment_basis = options.alignment_basis;
  output.min_depth_m = options.min_depth_m;
  output.max_depth_m = options.max_depth_m;
  output.depth_m.assign(pixel_count, std::numeric_limits<float>::quiet_NaN());

  // 第四段：转换与原因分类独立阅读；全无效仍交给组装层做质量门控。
  convertDepthPixels(input, output);
  return {DepthErrorCode::none, {}, std::move(output)};
}

/**
 * @brief 用实际 RGB 网格核对深度的注册声明；只读参数，不计算标定或时间同步。
 * @return 无效参考、未知/未注册、矛盾声明或网格差异分别返回既有状态。
 */
DepthAlignmentResult checkDepthRgbAlignment(const DepthFrame & depth, const ImageGeometry & rgb)
{
  // 本函数仅检查注册声明/网格；不得从这里的 aligned 推断像素、标定或时间已经可信。
  if (rgb.width == 0 || rgb.height == 0 || rgb.frame_id.empty()) {
    return {DepthAlignmentStatus::invalid_reference, "Actual RGB grid is incomplete"};
  }
  // 相同尺寸也不能把 unknown/not_aligned 升级为 aligned。
  switch (depth.alignment) {
    case DepthAlignment::unknown:
      return {DepthAlignmentStatus::unknown, "RGB alignment has not been declared"};
    case DepthAlignment::not_aligned:
      return {DepthAlignmentStatus::not_aligned, "Depth is explicitly not aligned to RGB"};
    case DepthAlignment::aligned_to_rgb: break;
    default:
      return {DepthAlignmentStatus::inconsistent_declaration, "Invalid RGB alignment state"};
  }
  if (!validAlignment(depth.width, depth.height, depth.frame_id,
      depth.alignment, depth.rgb_grid, depth.alignment_basis))
  {
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
