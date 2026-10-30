#include <vln_memory/bag/builder/bag_builder.hpp>
#include <vln_memory/bag/builder/image_decoder_ros.hpp>

#include <iostream>

/** @brief 包外只通过安装目标调用；可带 YAML 路径运行，也可检查默认配置失败契约。 */
int main(int argc, char ** argv)
{
  if (argc > 1) {
    auto options = vln_memory::loadBagBuildOptions(argv[1]);
    if (argc > 2) {options.output_directory = argv[2];}
    const auto result = vln_memory::buildBag(options);
    std::cout << vln_memory::bagBuildStatusName(result.status) << ": " << result.code << '\n';
    if (!result.ok()) {return 1;}
    return 0;
  }
  sensor_msgs::msg::CompressedImage unsupported;
  const auto decoded = vln_memory::decodeCompressedImage(
    unsupported, vln_memory::BagImageRole::rgb, 16, 128);
  if (decoded.ok() || decoded.code != "unsupported_compressed_format") {return 1;}
  const auto result = vln_memory::buildBag({});
  if (result.ok() || result.code != "invalid_config") {return 1;}
  return 0;
}
