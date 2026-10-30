#include "vln_memory/bag/depth/depth_ros.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

/** @brief 用真实 Image 验证核心映射、扁平注册字段及输出所有权；失败返回非零。 */
int main()
{
  try {
    sensor_msgs::msg::Image image;
    image.header.stamp.sec = 42;
    image.header.stamp.nanosec = 123;
    image.header.frame_id = "rgb_optical";
    image.width = 2;
    image.height = 1;
    image.encoding = "16UC1";
    image.step = 5;
    image.is_bigendian = 1;
    image.data = {0x03, 0xE8, 0x00, 0x00, 0xAA};
    vln_memory::DepthOptions options;
    options.input_unit = vln_memory::DepthUnit::millimeters;
    options.convention = vln_memory::DepthConvention::optical_z;
    options.alignment = vln_memory::DepthAlignment::aligned_to_rgb;
    options.rgb_grid = {2, 1, "rgb_optical"};
    options.alignment_basis = "registered_stream_config";
    const auto result = vln_memory::decodeRosDepth(image, options);
    if (!result.ok()) {throw std::runtime_error(result.message);}
    const auto & frame = result.frame.value();
    if (frame.depth_m[0] != 1.0F || !std::isnan(frame.depth_m[1]) || frame.stamp.sec != 42 ||
      frame.stamp.nanosec != 123 || frame.frame_id != "rgb_optical" ||
      frame.source_step != 5 || !frame.source_is_bigendian || frame.zero_pixels != 1)
    {
      throw std::runtime_error("ROS depth metadata or numeric mapping failed");
    }
    if (!vln_memory::checkDepthRgbAlignment(frame, {2, 1, "rgb_optical"}).alignedToRgb()) {
      throw std::runtime_error("ROS registration declaration lost");
    }
    image.data.assign(5, 0);
    image.header.frame_id = "changed";
    options.alignment_basis = "changed";
    if (frame.depth_m[0] != 1.0F || frame.frame_id != "rgb_optical" ||
      frame.alignment_basis != "registered_stream_config")
    {
      throw std::runtime_error("ROS output retained borrowed input");
    }
    image.encoding = "mono16";
    const auto rejected = vln_memory::decodeRosDepth(image, options);
    if (rejected.ok() || rejected.code != vln_memory::DepthErrorCode::unsupported_encoding) {
      throw std::runtime_error("ROS adapter bypassed numeric type validation");
    }
    std::cout << "ROS depth adapter: passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "ROS depth adapter failed: " << error.what() << '\n';
    return 1;
  }
}
