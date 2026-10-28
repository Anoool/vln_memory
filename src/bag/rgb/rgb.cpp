#include "vln_memory/bag/rgb/rgb.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace vln_memory
{
namespace
{

// 所有可预期输入错误都通过此入口返回，确保失败结果不携带半成品图像。
RgbResult failure(RgbErrorCode code, const char * message)
{
  return {code, message, std::nullopt};
}

/** 在检查容量之前也必须防止乘法溢出，不能先乘后检查。 */
bool checkedMultiply(std::size_t left, std::size_t right, std::size_t & result)
{
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

}  // namespace

const char * rgbErrorCodeName(RgbErrorCode code) noexcept
{
  // 名称供后续 JSONL 报告使用；增加错误类型时需同步维护这里的映射。
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

RgbResult decodeRgb(const RgbImageView & input, const RgbLimits & limits)
{
  // 第一段：元数据检查。不借助后续深度、标定或 TF 来猜测图像格式。
  if (limits.max_pixels == 0 || limits.max_input_bytes == 0) {
    return failure(RgbErrorCode::invalid_limits, "RGB limits must be positive");
  }
  if (input.width == 0 || input.height == 0) {
    return failure(RgbErrorCode::invalid_dimensions, "Image dimensions must be positive");
  }
  // sec 可以为负、采样时刻也可以为零；这里只检查 ROS 纳秒字段的合法范围。
  if (input.stamp.nanosec >= 1000000000U) {
    return failure(RgbErrorCode::invalid_stamp, "Stamp nanosec must be less than 1000000000");
  }

  // 显式按编码解释通道，不能根据数据长度猜测；mono8 允许扩展为三通道灰度。
  std::size_t channels = 0;
  if (input.encoding == "rgb8" || input.encoding == "bgr8") {
    channels = 3;
  } else if (input.encoding == "mono8") {
    channels = 1;
  } else {
    return failure(RgbErrorCode::unsupported_encoding, "Supported encodings: rgb8, bgr8, mono8");
  }
  // 支持的编码均为单字节通道，不需要字节交换，但标志本身仍应符合消息约定。
  if (input.is_bigendian > 1) {
    return failure(RgbErrorCode::invalid_endianness, "is_bigendian must be 0 or 1");
  }

  // 第二段：先检查所有尺寸运算和上限，再访问像素或申请输出内存。
  // source_row_bytes 只计算有效像素；input_bytes 还包含每行填充。
  // output_step 保持 uint32_t 表示，便于后续与图像消息和保存接口对接。
  std::size_t source_row_bytes = 0;
  std::size_t input_bytes = 0;
  std::size_t pixel_count = 0;
  std::size_t output_bytes = 0;
  std::size_t output_step = 0;
  if (!checkedMultiply(input.width, channels, source_row_bytes) ||
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
  // 同时限制像素数与源字节数：少量像素加上异常巨大的行填充也可能超限。
  if (pixel_count > limits.max_pixels || input_bytes > limits.max_input_bytes) {
    return failure(RgbErrorCode::size_limit_exceeded, "Image exceeds configured RGB limits");
  }
  // 按 Image 的 step*height 严格检查长度；截断和多余字节都交给上层报告。
  if (input.data == nullptr || input.data_size != input_bytes) {
    return failure(RgbErrorCode::invalid_data, "data must be non-null and exactly step*height bytes");
  }

  // 校验全部通过后才分配输出。字符串也转为拥有型，避免源消息释放后悬空。
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

  // 第三段：逐行处理，跳过源行尾填充；输出始终为 RGB，而非 OpenCV 常用的 BGR。
  for (std::size_t row = 0; row < input.height; ++row) {
    // 输入与输出使用各自的行跨度，填充字节不会进入标准 RGB 像素。
    const auto * source = input.data + row * input.step;
    auto * destination = output.pixels.data() + row * output_step;
    if (input.encoding == "rgb8") {
      std::copy_n(source, source_row_bytes, destination);
    } else if (input.encoding == "bgr8") {
      // 仅交换红蓝通道，绿色和像素位置不变。
      for (std::size_t column = 0; column < input.width; ++column) {
        const auto offset = column * 3;
        destination[offset] = source[offset + 2];
        destination[offset + 1] = source[offset + 1];
        destination[offset + 2] = source[offset];
      }
    } else {
      // 前面的编码检查保证这里只可能是 mono8；三个输出通道保留相同灰度。
      for (std::size_t column = 0; column < input.width; ++column) {
        const auto value = source[column];
        const auto offset = column * 3;
        destination[offset] = value;
        destination[offset + 1] = value;
        destination[offset + 2] = value;
      }
    }
  }
  return {RgbErrorCode::none, {}, std::move(output)};
}

}  // namespace vln_memory
