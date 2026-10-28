#pragma once

#include <sensor_msgs/msg/image.hpp>

#include "vln_memory/bag/rgb/rgb.hpp"

namespace vln_memory
{

/**
 * @brief ROS Image 到标准 RGB 帧的薄适配；不订阅话题，也不读取 bag。
 * @note image 在调用期间不得修改；借用仅限调用内部，成功输出不依赖 image。
 *       所有格式、尺寸、行跨度与时间戳校验均复用 decodeRgb。
 */
RgbResult decodeRosRgb(
  const sensor_msgs::msg::Image & image, const RgbLimits & limits = {});

}  // namespace vln_memory
