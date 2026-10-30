#pragma once

#include <string_view>
#include <sensor_msgs/msg/camera_info.hpp>

#include "vln_memory/bag/camera_info/camera_info.hpp"

/**
 * @file
 * ROS CameraInfo 边界：复制完整标定和操作参数，额外接收真实来源 topic。
 * 返回对象独立拥有数据，所有校验规则与非 ROS 调用保持一致。
 */
namespace vln_memory
{

/**
 * @brief 将真实 ROS CameraInfo 字段与来源 topic 映射到核心；不推测 topic 或流绑定。
 * @note 所有输出独立拥有数据，不依赖源消息/topic 生命周期；复用核心的全部校验。
 */
CameraInfoResult decodeRosCameraInfo(
  const sensor_msgs::msg::CameraInfo & message, std::string_view topic);

}  // namespace vln_memory
