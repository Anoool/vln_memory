#include <vln_memory/bag/camera_info/camera_info_ros.hpp>

#include <utility>

/**
 * @file
 * 复制 ROS CameraInfo 全部几何与操作字段，并补充消息自身不携带的来源话题。
 * 不修改 stamp/K/P/ROI，不在适配层补“默认标定”，成功与失败由核心统一解释。
 */
namespace vln_memory
{

/** @brief 只复制原始字段，不做隐式修复；返回值独立持有，坏标定也可保存为证据。 */
CameraInfoData copyRosCameraInfo(
  const sensor_msgs::msg::CameraInfo & message, std::string_view topic)
{
  CameraInfoData input;
  // 消息只提供 header，真实 topic 必须由 bag reader/订阅者补充。
  input.topic = topic;
  input.stamp = {message.header.stamp.sec, message.header.stamp.nanosec};
  input.frame_id = message.header.frame_id;
  input.width = message.width;
  input.height = message.height;
  input.distortion_model = message.distortion_model;
  // D 长度由模型决定；K/R/P 已由 ROS 类型固定大小，仍需核心验证有限性和结构。
  input.d = message.d;
  input.k = message.k;
  input.r = message.r;
  input.p = message.p;
  // 保留操作参数原值，由核心解释零 binning 和默认 ROI。
  input.binning_x = message.binning_x;
  input.binning_y = message.binning_y;
  input.roi.x_offset = message.roi.x_offset;
  input.roi.y_offset = message.roi.y_offset;
  input.roi.width = message.roi.width;
  input.roi.height = message.roi.height;
  input.roi.do_rectify = message.roi.do_rectify;
  return input;
}

/** @brief 复制后调用统一兼容入口；默认严格，显式零尾 D8 的结果与核心一致。 */
CameraInfoResult decodeRosCameraInfo(
  const sensor_msgs::msg::CameraInfo & message, std::string_view topic,
  CameraInfoCompatibility compatibility)
{
  return normalizeCameraInfo(copyRosCameraInfo(message, topic), compatibility);
}

}  // namespace vln_memory
