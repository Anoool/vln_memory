#include "vln_memory/bag/depth/depth_ros.hpp"

namespace vln_memory
{

DepthResult decodeRosDepth(
  const sensor_msgs::msg::Image & image, const DepthOptions & options, const DepthLimits & limits)
{
  DepthImageView input;
  // 只复制身份和布局；消息时间不替换为 bag 接收时间，也不从话题猜测单位。
  input.stamp = {image.header.stamp.sec, image.header.stamp.nanosec};
  input.frame_id = image.header.frame_id;
  input.width = image.width;
  input.height = image.height;
  input.encoding = image.encoding;
  input.step = image.step;
  input.is_bigendian = image.is_bigendian;
  // 借用只持续到同步解码结束；成功输出的像素、字符串与注册声明均独立持有。
  input.data = image.data.data();
  input.data_size = image.data.size();
  return decodeDepth(input, options, limits);
}

}  // namespace vln_memory
