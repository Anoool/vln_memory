#pragma once

#include <sensor_msgs/msg/image.hpp>

#include "vln_memory/bag/depth/depth.hpp"

/**
 * @file
 * ROS Depth 边界：消息负责像素布局，DepthOptions 负责单位/含义/注册依据。
 * 适配不从 topic 或 encoding 猜语义，转换和错误处理全部委托核心。
 */
namespace vln_memory
{

/**
 * @brief 将真实、未压缩 ROS Image 的布局映射给 Depth 核心。
 * @note 消息类型由函数签名限定；encoding 仍需核心校验，单位与注册信息由
 *       调用方显式给出。当前没有 CompressedImage/DisparityImage 重载。
 */
DepthResult decodeRosDepth(
  const sensor_msgs::msg::Image & image, const DepthOptions & options,
  const DepthLimits & limits = {});

}  // namespace vln_memory
