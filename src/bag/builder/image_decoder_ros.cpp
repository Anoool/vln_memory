#include <vln_memory/bag/builder/image_decoder_ros.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <new>
#include <utility>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>

namespace vln_memory
{
namespace
{
using Bytes = std::vector<std::uint8_t>;

/** @brief 构造无布局的失败；返回值拥有文本，不引用输入消息。 */
CompressedImageInfo invalidInfo(const char * code, const std::string & message)
{
  CompressedImageInfo result;
  result.code = code;
  result.message = message;
  return result;
}

/** @brief 读取已由调用者检查边界的 JPEG 网络序 uint16，不作未对齐指针转换。 */
std::uint32_t readBig16(const Bytes & data, std::size_t offset)
{
  return static_cast<std::uint32_t>(data[offset]) * 256U + data[offset + 1];
}

/** @brief 读取已检查边界的 PNG 网络序 uint32，先提升类型避免有符号移位。 */
std::uint32_t readBig32(const Bytes & data, std::size_t offset)
{
  return (static_cast<std::uint32_t>(data[offset]) << 24U) |
         (static_cast<std::uint32_t>(data[offset + 1]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 2]) << 8U) | data[offset + 3];
}

/** @brief 识别包含尺寸的 JPEG SOF；排除 DHT/JPG/DAC 等非帧头标记。 */
bool isStartOfFrame(std::uint8_t marker)
{
  return marker >= 0xc0 && marker <= 0xcf && marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
}

/** @brief 有界扫描 JPEG 帧头，不解压像素；仅接受 8 位三通道，损坏流留给解码器复查。 */
CompressedImageInfo inspectJpeg(const Bytes & data)
{
  if (data.size() < 4 || data[0] != 0xff || data[1] != 0xd8) {
    return invalidInfo("invalid_compressed_payload", "Missing JPEG SOI marker");
  }
  std::size_t offset = 2;
  while (offset < data.size()) {
    if (data[offset++] != 0xff) {break;}
    while (offset < data.size() && data[offset] == 0xff) {++offset;}
    if (offset == data.size()) {break;}
    const auto marker = data[offset++];
    if (marker == 0xd9 || marker == 0xda) {break;}
    if (marker == 0x01 || (marker >= 0xd0 && marker <= 0xd7)) {continue;}
    if (data.size() - offset < 2) {break;}
    const auto length = readBig16(data, offset);
    if (length < 2 || length > data.size() - offset) {break;}
    if (isStartOfFrame(marker)) {
      if (length != 17 || data[offset + 2] != 8 || data[offset + 7] != 3) {
        return invalidInfo("unsupported_compressed_format", "JPEG must be 8-bit, three-channel RGB");
      }
      CompressedImageInfo result;
      result.code = "none";
      result.width = readBig16(data, offset + 5);
      result.height = readBig16(data, offset + 3);
      result.encoding = "bgr8";  // OpenCV 解码缓冲采用 BGR，随后复用 RGB 核心做通道转换。
      return result;
    }
    offset += length;
  }
  return invalidInfo("invalid_compressed_payload", "Truncated JPEG or missing frame dimensions");
}

/** @brief 验证官方配置头后的 PNG IHDR；这里只接受无损 16 位单通道深度。 */
CompressedImageInfo inspectDepthPng(const Bytes & data)
{
  const std::size_t offset = sizeof(compressed_depth_image_transport::ConfigHeader);
  const std::array<std::uint8_t, 8> signature{137, 80, 78, 71, 13, 10, 26, 10};
  if (offset > data.size() || data.size() - offset < 33 ||
    !std::equal(signature.begin(), signature.end(), data.begin() + static_cast<Bytes::difference_type>(offset)))
  {
    return invalidInfo("invalid_compressed_payload", "Missing compressedDepth configuration header/PNG");
  }
  const std::array<std::uint8_t, 4> ihdr{'I', 'H', 'D', 'R'};
  const auto type = data.begin() + static_cast<Bytes::difference_type>(offset + 12);
  if (readBig32(data, offset + 8) != 13 || !std::equal(ihdr.begin(), ihdr.end(), type) ||
    data[offset + 24] != 16 || data[offset + 25] != 0)
  {
    return invalidInfo("unsupported_compressed_format", "compressedDepth PNG must be uint16 single-channel");
  }
  CompressedImageInfo result;
  result.code = "none";
  result.width = readBig32(data, offset + 16);
  result.height = readBig32(data, offset + 20);
  result.encoding = "16UC1";
  result.payload_offset = offset;
  // ConfigHeader 的 depthParam 不作为 16UC1 的单位/scale；源格式由调用者另行声明。
  return result;
}

/** @brief 用除法检查像素/行宽/解码字节预算，避免乘法溢出或解压后才发现超限。 */
CompressedImageInfo checkLayoutBudget(
  CompressedImageInfo info, std::size_t max_pixels, std::size_t max_bytes)
{
  if (!info.ok()) {return info;}
  if (info.width == 0 || info.height == 0) {
    return invalidInfo("invalid_compressed_payload", "Image dimensions must be positive");
  }
  std::size_t pixel_bytes = 3;
  if (info.encoding == "16UC1") {pixel_bytes = 2;}
  if (info.width > max_pixels / info.height || info.width > max_bytes / info.height / pixel_bytes ||
    info.width > std::numeric_limits<std::uint32_t>::max() / pixel_bytes)
  {
    return invalidInfo("limit_exceeded", "Decoded image layout exceeds pixel/byte/row budget");
  }
  return info;
}

/** @brief 核对库输出与已预检布局及原 header 一致，不静默修正错误时间或编码。 */
bool hasExpectedImage(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CompressedImage & source,
  const CompressedImageInfo & info, std::size_t max_bytes)
{
  return image.width == info.width && image.height == info.height && image.encoding == info.encoding &&
         image.header.stamp == source.header.stamp && image.header.frame_id == source.header.frame_id &&
         image.data.size() <= max_bytes;
}

}  // namespace

/** @brief 格式白名单和编码尺寸先行；所有访问由长度检查约束，未修改输入。 */
CompressedImageInfo inspectCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes)
{
  if (max_pixels == 0 || max_input_bytes == 0 || message.data.size() > max_input_bytes) {
    return invalidInfo("limit_exceeded", "Compressed input exceeds configured budget");
  }
  CompressedImageInfo info;
  if (role == BagImageRole::rgb &&
    (message.format == "rgb8; jpeg compressed bgr8" || message.format == "bgr8; jpeg compressed bgr8"))
  {
    info = inspectJpeg(message.data);
  } else if (role == BagImageRole::depth &&
    (message.format == "16UC1; compressedDepth" || message.format == "16UC1; compressedDepth png"))
  {
    info = inspectDepthPng(message.data);
  } else {
    return invalidInfo("unsupported_compressed_format", "Unsupported role/format: " + message.format);
  }
  return checkLayoutBudget(std::move(info), max_pixels, max_input_bytes);
}

/** @brief 一次只拥有当前图像；库错误作为坏帧报告，真正内存耗尽继续向外传播。 */
CompressedImageResult decodeCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes)
{
  const auto info = inspectCompressedImage(message, role, max_pixels, max_input_bytes);
  if (!info.ok()) {return {nullptr, info.code, info.message};}
  sensor_msgs::msg::Image::SharedPtr image;
  try {
    if (role == BagImageRole::depth) {
      image = compressed_depth_image_transport::decodeCompressedDepthImage(message);
    } else {
      // 相机网格必须保持原方向，不能由 JPEG EXIF 自动旋转并改变像素/标定对应关系。
      const auto pixels = cv::imdecode(message.data, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
      if (!pixels.empty() && pixels.type() == CV_8UC3) {
        image = cv_bridge::CvImage(message.header, info.encoding, pixels).toImageMsg();
      }
    }
  } catch (const std::bad_alloc &) {
    throw;
  } catch (const cv::Exception & error) {
    if (error.code == cv::Error::StsNoMem) {throw std::bad_alloc();}
    return {nullptr, "invalid_compressed_payload", error.what()};
  }
  if (!image || !hasExpectedImage(*image, message, info, max_input_bytes)) {
    return {nullptr, "invalid_compressed_payload", "Decoder failed or returned unexpected layout/header"};
  }
  return {std::move(image), "none", ""};
}

}  // namespace vln_memory
