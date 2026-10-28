#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>

#include <iostream>
#include <vector>

int main()
{
  // 只使用安装后的两个公开模块，在 bag 组装的位置构造共享网格契约。
  const std::vector<std::uint8_t> rgb_bytes{255, 0, 0, 0, 255, 0};
  vln_memory::RgbImageView rgb_input;
  rgb_input.frame_id = "rgb_optical";
  rgb_input.width = 2;
  rgb_input.height = 1;
  rgb_input.encoding = "rgb8";
  rgb_input.step = 6;
  rgb_input.data = rgb_bytes.data();
  rgb_input.data_size = rgb_bytes.size();
  const auto rgb = vln_memory::decodeRgb(rgb_input);
  if (!rgb.ok()) {return 1;}
  const vln_memory::ImageGeometry grid{
    rgb.frame->width, rgb.frame->height, rgb.frame->frame_id};

  const std::vector<std::uint8_t> depth_bytes{0xE8, 0x03, 0xD0, 0x07};
  vln_memory::DepthImageView depth_input;
  depth_input.frame_id = "rgb_optical";
  depth_input.width = 2;
  depth_input.height = 1;
  depth_input.encoding = "16UC1";
  depth_input.step = 4;
  depth_input.data = depth_bytes.data();
  depth_input.data_size = depth_bytes.size();
  vln_memory::DepthOptions options;
  options.input_unit = vln_memory::DepthUnit::millimeters;
  options.convention = vln_memory::DepthConvention::optical_z;
  options.alignment = {vln_memory::DepthAlignment::aligned_to_rgb, grid, "registered_stream_config"};
  const auto depth = vln_memory::decodeDepth(depth_input, options);
  if (!depth.ok() || depth.frame->depth_m != std::vector<float>{1.0F, 2.0F} ||
    !vln_memory::checkDepthRgbAlignment(depth.frame.value(), grid).alignedToRgb())
  {
    std::cerr << "Installed Depth/RGB APIs failed\n";
    return 1;
  }
  std::cout << "Installed Depth/RGB APIs: passed\n";
  return 0;
}
