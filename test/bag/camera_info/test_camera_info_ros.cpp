#include "vln_memory/bag/camera_info/camera_info_ros.hpp"

#include <iostream>
#include <string>

/** @brief 用真实 ROS 消息核对全部字段、独立所有权及未标定错误；失败返回非零。 */
int main()
{
  sensor_msgs::msg::CameraInfo message;
  message.header.stamp.sec = 8;
  message.header.stamp.nanosec = 1234;
  message.header.frame_id = "camera_optical";
  message.width = 640;
  message.height = 480;
  message.distortion_model = "plumb_bob";
  message.d = {0.1, 0.2, 0.3, 0.4, 0.5};
  message.k = {500, 0, 320, 0, 510, 240, 0, 0, 1};
  message.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  message.p = {490, 0, 315, -49, 0, 500, 239, 1, 0, 0, 1, 0};
  message.binning_x = 2;
  message.binning_y = 3;
  message.roi.x_offset = 20;
  message.roi.y_offset = 30;
  message.roi.width = 320;
  message.roi.height = 240;
  message.roi.do_rectify = true;
  std::string topic = "/camera/camera_info";
  const auto result = vln_memory::decodeRosCameraInfo(message, topic);
  if (!result.ok()) {std::cerr << result.message << '\n'; return 1;}
  const auto & info = result.info.value();
  if (info.topic != topic || info.frame_id != message.header.frame_id ||
    info.stamp.sec != 8 || info.stamp.nanosec != 1234 || info.width != 640 || info.height != 480 ||
    info.k != message.k || info.r != message.r || info.p != message.p || info.d != message.d ||
    info.distortion_model != message.distortion_model || info.binning_x != 2 || info.binning_y != 3 ||
    info.roi.x_offset != 20 || info.roi.y_offset != 30 || info.roi.width != 320 ||
    info.roi.height != 240 || !info.roi.do_rectify)
  {
    std::cerr << "ROS CameraInfo field mapping changed\n";
    return 1;
  }
  message.d[0] = 9;
  message.header.frame_id = "changed";
  topic = "/changed";
  if (info.d[0] != 0.1 || info.frame_id != "camera_optical" || info.topic != "/camera/camera_info") {
    std::cerr << "ROS CameraInfo retained borrowed data\n";
    return 1;
  }
  message.k[0] = 0;
  const auto invalid = vln_memory::decodeRosCameraInfo(message, topic);
  if (invalid.ok() || invalid.code != vln_memory::CameraInfoErrorCode::uncalibrated) {return 1;}
  std::cout << "ROS CameraInfo mapping and ownership checks: passed\n";
  return 0;
}
