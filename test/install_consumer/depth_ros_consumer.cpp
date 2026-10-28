#include <vln_memory/bag/depth/depth_ros.hpp>

#include <iostream>

int main()
{
  // 用安装目标传递的 sensor_msgs 依赖，不手动添加 ROS 源码 include 路径。
  sensor_msgs::msg::Image image;
  image.width = 1;
  image.height = 1;
  image.encoding = "32FC1";
  image.step = 4;
  image.data = {0x00, 0x00, 0x80, 0x3F};
  vln_memory::DepthOptions options;
  options.input_unit = vln_memory::DepthUnit::meters;
  options.convention = vln_memory::DepthConvention::optical_z;
  const auto result = vln_memory::decodeRosDepth(image, options);
  if (!result.ok() || result.frame->depth_m[0] != 1.0F) {return 1;}
  std::cout << "Installed ROS Depth API: passed\n";
  return 0;
}
