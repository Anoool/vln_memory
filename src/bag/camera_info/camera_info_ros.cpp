#include "vln_memory/bag/camera_info/camera_info_ros.hpp"

#include <utility>

namespace vln_memory
{

CameraInfoResult decodeRosCameraInfo(
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
  input.d = message.d;
  input.k = message.k;
  input.r = message.r;
  input.p = message.p;
  // 保留操作参数原值，由核心解释零 binning 和默认 ROI。
  input.binning_x = message.binning_x;
  input.binning_y = message.binning_y;
  input.roi = {message.roi.x_offset, message.roi.y_offset, message.roi.width,
    message.roi.height, message.roi.do_rectify};
  return decodeCameraInfo(std::move(input));
}

}  // namespace vln_memory
