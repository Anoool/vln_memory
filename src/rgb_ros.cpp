#include "vln_memory/rgb_ros.hpp"

namespace vln_memory
{

RgbResult decodeRosRgb(const sensor_msgs::msg::Image & image, const RgbLimits & limits)
{
  // bag 模块完成反序列化后可直接调用；此处不再复制完整输入像素。
  RgbImageView input;
  // 保留原始采样身份；bag 接收时间和 clock_id 由后续 bag 模块单独记录。
  input.stamp = {image.header.stamp.sec, image.header.stamp.nanosec};
  input.frame_id = image.header.frame_id;
  // 原样传递行跨度和编码，让核心按同一规则处理填充与颜色通道。
  input.width = image.width;
  input.height = image.height;
  input.encoding = image.encoding;
  input.step = image.step;
  input.is_bigendian = image.is_bigendian;
  // 仅在同步调用 decodeRgb 期间借用；空数据也交由核心统一返回错误。
  input.data = image.data.data();
  input.data_size = image.data.size();
  return decodeRgb(input, limits);
}

}  // namespace vln_memory
