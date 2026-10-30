#include <vln_memory/bag/rgb/rgb.hpp>

#include <algorithm>
#include <limits>

/**
 * @file
 * 审阅路径：decodeRgb 检查元数据 -> decodeRgbBuffer 检查布局 -> makeRgbFrame 复制像素。
 * 输入始终借用，输出独立持有；所有可预期错误在输出分配和像素读取前拒绝。
 */
namespace vln_memory
{
namespace
{

/** 根据错误码和固定原因构造失败结果，不携带半成品图像。 */
RgbResult failure(RgbErrorCode code, const char * message)
{
  return {code, message, std::nullopt};
}

/** 安全计算尺寸乘积；成功写入 result，溢出返回 false 且不修改 result。 */
bool checkedMultiply(std::size_t left, std::size_t right, std::size_t & result)
{
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

/** 将一行 width 个 BGR 像素写成 RGB；两段有效像素区须不重叠，仅读写 width*3 字节。 */
void convertBgrRow(
  const std::uint8_t * source_row, std::uint8_t * rgb_row, std::size_t width)
{
  for (std::size_t column = 0; column < width; ++column) {
    const auto pixel_offset = column * 3;
    rgb_row[pixel_offset] = source_row[pixel_offset + 2];
    rgb_row[pixel_offset + 1] = source_row[pixel_offset + 1];
    rgb_row[pixel_offset + 2] = source_row[pixel_offset];
  }
}

/** 将一行 width 个灰度值展开成 RGB；调用者提供有效、不重叠的输入和三通道输出。 */
void expandMonoRow(
  const std::uint8_t * source_row, std::uint8_t * rgb_row, std::size_t width)
{
  for (std::size_t column = 0; column < width; ++column) {
    const auto intensity = source_row[column];
    const auto pixel_offset = column * 3;
    rgb_row[pixel_offset] = intensity;
    rgb_row[pixel_offset + 1] = intensity;
    rgb_row[pixel_offset + 2] = intensity;
  }
}

/**
 * 从已校验的输入构造拥有型 RGB 帧；output_step/output_bytes 是紧密输出的字节数。
 * 仅分配一次像素缓冲区，逐行跳过源填充；返回值不借用输入，系统分配异常直接传播。
 */
RgbFrame makeRgbFrame(
  const RgbImageView & input, std::size_t output_step, std::size_t output_bytes)
{
  RgbFrame output;
  output.stamp = input.stamp;
  output.frame_id = std::string(input.frame_id);
  output.width = input.width;
  output.height = input.height;
  output.step = static_cast<std::uint32_t>(output_step);

  output.source_encoding = std::string(input.encoding);
  output.source_step = input.step;
  output.source_is_bigendian = input.is_bigendian != 0;
  output.pixels.resize(output_bytes);

  for (std::size_t row = 0; row < input.height; ++row) {
    const auto * source_row = input.data + row * input.step;
    auto * rgb_row = output.pixels.data() + row * output_step;
    if (input.encoding == "rgb8") {
      std::copy_n(source_row, output_step, rgb_row);
    } else if (input.encoding == "bgr8") {
      convertBgrRow(source_row, rgb_row, input.width);
    } else {
      // 元数据已验证，此分支只处理 mono8；不从缓冲区长度猜编码。
      expandMonoRow(source_row, rgb_row, input.width);
    }
  }
  return output;
}

/**
 * 元数据合法后，按原顺序检查尺寸溢出、行跨度、资源上限和缓冲区，再构造输出。
 * source_channels 只能是已确认的 1 或 3；错误返回无图像的结果，不访问源像素。
 */
RgbResult decodeRgbBuffer(
  const RgbImageView & input, const RgbLimits & limits, std::size_t source_channels)
{
  // 有效行字节数不含填充，输入总字节数包含填充；所有乘法先防溢出。
  std::size_t source_row_bytes = 0;
  std::size_t input_bytes = 0;
  std::size_t pixel_count = 0;
  std::size_t output_bytes = 0;
  std::size_t output_step = 0;
  if (!checkedMultiply(input.width, source_channels, source_row_bytes) ||
    !checkedMultiply(input.step, input.height, input_bytes) ||
    !checkedMultiply(input.width, input.height, pixel_count) ||
    !checkedMultiply(pixel_count, 3, output_bytes) ||
    !checkedMultiply(input.width, 3, output_step) ||
    output_step > std::numeric_limits<std::uint32_t>::max())
  {
    return failure(RgbErrorCode::size_overflow, "Image size cannot be represented safely");
  }
  if (input.step < source_row_bytes) {
    return failure(RgbErrorCode::invalid_step, "step is smaller than the encoded pixel row");
  }
  if (pixel_count > limits.max_pixels || input_bytes > limits.max_input_bytes) {
    return failure(RgbErrorCode::size_limit_exceeded, "Image exceeds configured RGB limits");
  }
  if (input.data == nullptr || input.data_size != input_bytes) {
    return failure(RgbErrorCode::invalid_data, "data must be non-null and exactly step*height bytes");
  }

  return {RgbErrorCode::none, {}, makeRgbFrame(input, output_step, output_bytes)};
}

}  // namespace

/** 返回供报告使用的稳定错误名；未知枚举返回 unknown，不分配内存。 */
const char * rgbErrorCodeName(RgbErrorCode code) noexcept
{
  switch (code) {
    case RgbErrorCode::none: return "none";
    case RgbErrorCode::invalid_limits: return "invalid_limits";
    case RgbErrorCode::invalid_dimensions: return "invalid_dimensions";
    case RgbErrorCode::invalid_stamp: return "invalid_stamp";
    case RgbErrorCode::unsupported_encoding: return "unsupported_encoding";
    case RgbErrorCode::invalid_endianness: return "invalid_endianness";
    case RgbErrorCode::invalid_step: return "invalid_step";
    case RgbErrorCode::size_overflow: return "size_overflow";
    case RgbErrorCode::size_limit_exceeded: return "size_limit_exceeded";
    case RgbErrorCode::invalid_data: return "invalid_data";
  }
  return "unknown";
}

/** 检查单帧限制、尺寸、采样时间和编码；合法输入交给缓冲区检查，不保存借用引用。 */
RgbResult decodeRgb(const RgbImageView & input, const RgbLimits & limits)
{
  if (limits.max_pixels == 0 || limits.max_input_bytes == 0) {
    return failure(RgbErrorCode::invalid_limits, "RGB limits must be positive");
  }
  if (input.width == 0 || input.height == 0) {
    return failure(RgbErrorCode::invalid_dimensions, "Image dimensions must be positive");
  }
  // sec 可以为负，零时刻也合法；这里只检查纳秒分量。
  if (input.stamp.nanosec >= 1000000000U) {
    return failure(RgbErrorCode::invalid_stamp, "Stamp nanosec must be less than 1000000000");
  }

  std::size_t source_channels = 0;
  if (input.encoding == "rgb8" || input.encoding == "bgr8") {
    source_channels = 3;
  } else if (input.encoding == "mono8") {
    source_channels = 1;
  } else {
    return failure(RgbErrorCode::unsupported_encoding, "Supported encodings: rgb8, bgr8, mono8");
  }
  // 支持的编码均为 8 位通道，无需交换字节序；标志仍须为合法的 0/1。
  if (input.is_bigendian > 1) {
    return failure(RgbErrorCode::invalid_endianness, "is_bigendian must be 0 or 1");
  }

  return decodeRgbBuffer(input, limits, source_channels);
}

}  // namespace vln_memory
