#include "vln_memory/bag/builder/bag_builder.hpp"

#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

namespace
{
/** @brief 命令只加载配置和路径覆盖，不在 CLI 中复制算法。 */
void usage()
{
  std::cout << "Usage: ros2 run vln_memory bag_builder --config FILE [--bag INPUT] [--output DIRECTORY]\n";
}
}

/** @brief 薄离线入口；0=完整发布（可带 issues），1=构建失败，2=参数/配置错误。 */
int main(int argc, char ** argv)
{
  try {
    std::string config;
    std::string input;
    std::string output;
    for (int index = 1; index < argc; ++index) {
      const std::string flag(argv[index]);
      if (flag == "--help" || flag == "-h") {usage(); return 0;}
      if ((flag != "--config" && flag != "--bag" && flag != "--output") || index + 1 >= argc) {
        usage(); return 2;
      }
      const std::string value(argv[++index]);
      if (flag == "--config") {config = value;}
      else if (flag == "--bag") {input = value;}
      else {output = value;}
    }
    if (config.empty()) {usage(); return 2;}
    auto options = vln_memory::loadBagBuildOptions(config);
    if (!input.empty()) {options.bag_uri = input;}
    if (!output.empty()) {options.output_directory = output;}
    const auto result = vln_memory::buildBag(options);
    const nlohmann::json report{{"status", vln_memory::bagBuildStatusName(result.status)},
      {"code", result.code}, {"message", result.message}, {"output_directory", result.output_directory},
      {"work_directory", result.work_directory}, {"rgb_frames", result.rgb_frames},
      {"depth_frames", result.depth_frames}, {"decoded_frames", result.decoded_frames},
      {"pairs", result.pairs}, {"prepared_pairs", result.prepared_pairs}, {"issue_count", result.issue_count}};
    std::cout << report.dump(2) << '\n';
    if (!result.ok()) {return 1;}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
