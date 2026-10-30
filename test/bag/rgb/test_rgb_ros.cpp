#include <vln_memory/bag/rgb/rgb_ros.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>

int main()
{
  try {
    // 使用真实消息类型，设置两字节行填充，同时检查布局和元数据的映射。
    sensor_msgs::msg::Image image;
    image.header.stamp.sec = 42;
    image.header.stamp.nanosec = 123;
    image.header.frame_id = "camera_optical";
    image.width = 2;
    image.height = 1;
    image.encoding = "bgr8";
    image.step = 8;
    image.is_bigendian = 1;
    image.data = {0, 0, 255, 0, 255, 0, 71, 73};
    const auto result = vln_memory::decodeRosRgb(image);
    if (!result.ok()) {throw std::runtime_error(result.message);}
    const auto & frame = result.frame.value();
    const std::vector<std::uint8_t> expected{255, 0, 0, 0, 255, 0};
    if (frame.pixels != expected || frame.stamp.sec != 42 || frame.stamp.nanosec != 123 ||
      frame.frame_id != "camera_optical" || !frame.source_is_bigendian || frame.source_step != 8)
    {
      throw std::runtime_error("ROS fields were not mapped correctly");
    }
    // 适配后的输出应独立拥有数据，不受源消息复用影响。
    image.data.assign(8, 0);
    image.header.frame_id = "changed";
    if (frame.pixels != expected || frame.frame_id != "camera_optical") {
      throw std::runtime_error("ROS output borrows mutable source data");
    }
    // 未支持编码必须透传核心错误，不能在适配层绕过检查。
    image.encoding = "16UC1";
    const auto rejected = vln_memory::decodeRosRgb(image);
    if (rejected.ok() || rejected.code != vln_memory::RgbErrorCode::unsupported_encoding) {
      throw std::runtime_error("ROS adapter bypassed core validation");
    }
    std::cout << "ROS RGB adapter: passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "ROS RGB adapter failed: " << error.what() << '\n';
    return 1;
  }
}
