#include <vln_memory/bag/camera_info/camera_info.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>

#include <iostream>
#include <vector>

/** @brief 仅使用安装后的公开目标验证跨模块协作；失败返回非零。 */
int main()
{
  using namespace vln_memory;
  // 模拟 builder 的一次组合：topic 来自 bag，像素和 stamp 来自公开帧输出。
  const std::vector<std::uint8_t> rgb_bytes{255, 0, 0, 0, 255, 0};
  RgbImageView rgb_input;
  rgb_input.stamp = {10, 0};
  rgb_input.frame_id = "rgb_optical";
  rgb_input.width = 2;
  rgb_input.height = 1;
  rgb_input.encoding = "rgb8";
  rgb_input.step = 6;
  rgb_input.data = rgb_bytes.data();
  rgb_input.data_size = rgb_bytes.size();
  const auto rgb = decodeRgb(rgb_input);
  if (!rgb.ok()) {return 1;}
  const ImageGeometry grid{rgb.frame->width, rgb.frame->height, rgb.frame->frame_id};
  const std::vector<std::uint8_t> depth_bytes{0xE8, 0x03, 0xD0, 0x07};
  DepthImageView depth_input;
  depth_input.stamp = {10, 5000000};
  depth_input.frame_id = "rgb_optical";
  depth_input.width = 2;
  depth_input.height = 1;
  depth_input.encoding = "16UC1";
  depth_input.step = 4;
  depth_input.data = depth_bytes.data();
  depth_input.data_size = depth_bytes.size();
  DepthOptions depth_options;
  depth_options.input_unit = DepthUnit::millimeters;
  depth_options.convention = DepthConvention::optical_z;
  depth_options.alignment = DepthAlignment::aligned_to_rgb;
  depth_options.rgb_grid = grid;
  depth_options.alignment_basis = "registered_profile_v1";
  const auto depth = decodeDepth(depth_input, depth_options);
  if (!depth.ok() || !checkDepthRgbAlignment(*depth.frame, grid).alignedToRgb()) {return 1;}
  const SensorTimeOptions sync{SensorClockRelation::same_clock, 10000000};
  const auto paired = checkSensorTimes(rgb.frame->stamp, depth.frame->stamp, sync);
  if (!paired.matched() || paired.delta_ns != 5000000) {return 1;}

  CameraInfoData source;
  source.topic = "/rgb/camera_info";
  source.frame_id = "rgb_optical";
  source.stamp = {10, 0};
  source.width = 4;
  source.height = 2;
  source.binning_x = 2;
  source.binning_y = 2;
  source.distortion_model = "plumb_bob";
  source.d = {0, 0, 0, 0, 0};
  source.k = {4, 0, 2, 0, 4, 1, 0, 0, 1};
  source.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  source.p = {4, 0, 2, 0, 0, 4, 1, 0, 0, 0, 1, 0};
  const auto camera = decodeCameraInfo(source);
  if (!camera.ok()) {return 1;}
  CameraInfoMatchOptions options;
  options.camera_info_topic = source.topic;
  options.image_topic = "/rgb/image_raw";
  options.optical_frame = "rgb_optical";
  options.binding_basis = "device_profile_v1";
  options.image_domain = CameraImageDomain::raw;
  options.clock_relation = sync.clock_relation;
  options.max_delta_ns = sync.max_delta_ns;
  CameraImageReference rgb_reference;
  rgb_reference.topic = "/rgb/image_raw";
  rgb_reference.width = rgb.frame->width;
  rgb_reference.height = rgb.frame->height;
  rgb_reference.frame_id = rgb.frame->frame_id;
  rgb_reference.stamp = rgb.frame->stamp;
  if (!checkCameraInfoImage(*camera.info, rgb_reference, options).matched()) {return 1;}
  // 同一标定也可以验证已注册深度的目标网格，但需单独、显式的流绑定。
  options.image_topic = "/depth/aligned_to_rgb";
  CameraImageReference depth_reference;
  depth_reference.topic = options.image_topic;
  depth_reference.width = depth.frame->width;
  depth_reference.height = depth.frame->height;
  depth_reference.frame_id = depth.frame->frame_id;
  depth_reference.stamp = depth.frame->stamp;
  if (!checkCameraInfoImage(*camera.info, depth_reference, options).matched()) {return 1;}
  std::cout << "Installed RGB/Depth/CameraInfo interfaces: passed, depth-rgb delta_ns=" <<
    *paired.delta_ns << '\n';
  return 0;
}
