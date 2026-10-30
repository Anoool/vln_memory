#include "vln_memory/bag/builder/image_decoder_ros.hpp"
#include "vln_memory/bag/rgb/rgb_ros.hpp"

#include <array>
#include <iostream>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>

namespace
{
using namespace vln_memory;

/** @brief Release 仍执行检查，错误文本明确指出损坏的接口契约。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 由标准编码器生成小型 JPEG，header 保留非零纳秒用于时间一致性检查。 */
sensor_msgs::msg::CompressedImage jpeg()
{
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera_optical";
  message.header.stamp.sec = 5;
  message.header.stamp.nanosec = 123;
  message.format = "rgb8; jpeg compressed bgr8";
  const cv::Mat pixels(24, 32, CV_8UC3, cv::Scalar(13, 71, 209));
  expect(cv::imencode(".jpg", pixels, message.data), "JPEG fixture encoding failed");
  return message;
}

/** @brief 由官方 compressedDepth 编码器生成 PNG，原 uint16 包括零和跨字节值。 */
sensor_msgs::msg::CompressedImage depthPng()
{
  std_msgs::msg::Header header;
  header.frame_id = "camera_optical";
  header.stamp.sec = -1;
  header.stamp.nanosec = 999999999;
  cv::Mat pixels(2, 2, CV_16UC1);
  pixels.at<std::uint16_t>(0, 0) = 0;
  pixels.at<std::uint16_t>(0, 1) = 1;
  pixels.at<std::uint16_t>(1, 0) = 256;
  pixels.at<std::uint16_t>(1, 1) = 1000;
  const auto image = cv_bridge::CvImage(header, "16UC1", pixels).toImageMsg();
  const auto encoded = compressed_depth_image_transport::encodeCompressedDepthImage(*image, 10, 100, 1);
  expect(encoded != nullptr, "Depth fixture encoding failed");
  return *encoded;
}

/** @brief 校验规范 BGR/RGB 通道、深度原值、原始 header 及输出拥有型生命周期。 */
void validImages()
{
  const auto rgb_message = jpeg();
  const auto rgb = decodeCompressedImage(rgb_message, BagImageRole::rgb, 2048, 65536);
  expect(rgb.ok() && rgb.image->header == rgb_message.header, "JPEG/header conversion failed");
  const auto expected = cv::imdecode(rgb_message.data, cv::IMREAD_COLOR);
  expect(rgb.image->data[0] == expected.at<cv::Vec3b>(0, 0)[0], "JPEG BGR byte changed");
  const auto normalized = decodeRosRgb(*rgb.image);
  expect(normalized.ok(), "Existing RGB adapter rejected decoded JPEG");
  expect(normalized.frame->pixels[0] == expected.at<cv::Vec3b>(0, 0)[2], "RGB red/blue swapped");
  const auto depth_message = depthPng();
  const auto depth = decodeCompressedImage(depth_message, BagImageRole::depth, 2048, 65536);
  expect(depth.ok() && depth.image->header == depth_message.header, "Depth/header conversion failed");
  const auto view = cv_bridge::toCvCopy(depth.image, "16UC1");
  expect(view->image.at<std::uint16_t>(0, 0) == 0, "Invalid depth zero changed");
  expect(view->image.at<std::uint16_t>(1, 0) == 256, "Depth endian changed");
  expect(view->image.at<std::uint16_t>(1, 1) == 1000, "Depth raw count scaled");
  auto modern = depth_message;
  modern.format = "16UC1; compressedDepth png";
  expect(decodeCompressedImage(modern, BagImageRole::depth, 2048, 65536).ok(), "PNG suffix not accepted");
}

/** @brief EXIF 旋转只能是照片显示提示，不能旋转相机网格或改变原始标定像素坐标。 */
void preservesCameraOrientation()
{
  auto message = jpeg();
  const std::array<std::uint8_t, 36> orientation{
    0xff, 0xe1, 0, 34, 'E', 'x', 'i', 'f', 0, 0,
    'I', 'I', 42, 0, 8, 0, 0, 0, 1, 0,
    0x12, 1, 3, 0, 1, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0};
  message.data.insert(message.data.begin() + 2, orientation.begin(), orientation.end());
  const auto result = decodeCompressedImage(message, BagImageRole::rgb, 2048, 65536);
  expect(result.ok() && result.image->width == 32 && result.image->height == 24,
    "EXIF rotated the camera grid");
}

/** @brief 压缩字节、展开尺寸、未知格式和畸形帧头在像素分配前受到约束。 */
void invalidImages()
{
  auto message = jpeg();
  expect(!inspectCompressedImage(message, BagImageRole::rgb, 1, 65536).ok(), "Pixel budget ignored");
  expect(!inspectCompressedImage(message, BagImageRole::rgb, 2048, 1).ok(), "Input byte budget ignored");
  expect(!inspectCompressedImage(message, BagImageRole::rgb, 2048, message.data.size()).ok(),
    "Decoded byte budget ignored");
  message.data.resize(3);
  expect(!decodeCompressedImage(message, BagImageRole::rgb, 2048, 65536).ok(), "Truncated JPEG accepted");
  message = depthPng();
  message.format = "32FC1; compressedDepth";
  expect(!decodeCompressedImage(message, BagImageRole::depth, 2048, 65536).ok(), "Unknown depth codec accepted");
  message.format = "16UC1; compressedDepth rvl";
  expect(!decodeCompressedImage(message, BagImageRole::depth, 2048, 65536).ok(), "RVL decoded as PNG");
  message = depthPng();
  const auto offset = sizeof(compressed_depth_image_transport::ConfigHeader);
  for (std::size_t index = offset + 16; index < offset + 24; ++index) {message.data[index] = 255;}
  expect(!inspectCompressedImage(message, BagImageRole::depth, 2048, 65536).ok(), "Oversized PNG accepted");
  message = depthPng();
  message.data.resize(offset + 33);
  expect(inspectCompressedImage(message, BagImageRole::depth, 2048, 65536).ok(), "Valid PNG header rejected");
  expect(!decodeCompressedImage(message, BagImageRole::depth, 2048, 65536).ok(), "Broken PNG body accepted");
}
}

/** @brief 独立运行压缩适配检查，依赖库/断言失败均返回非零。 */
int main()
{
  try {validImages(); preservesCameraOrientation(); invalidImages(); return 0;}
  catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
