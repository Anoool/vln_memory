#include <iostream>
#include <nlohmann/json.hpp>
#include "build_input_cli.hpp"

namespace
{
namespace cli = vln_memory::cli;

/** @brief 显示现有输入筛选 CLI 的参数。 */
void usage(std::ostream & out)
{
  out << "Usage: memory_build_input " << cli::selectionArguments << '\n';
}
}

/** @brief 解析选项并生成输入目录；0 成功、1 业务失败、2 参数错误。 */
int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage(std::cout);
    return 0;
  }
  try {
    const auto args = cli::parseArguments(argc, argv);
    const auto result = vln_memory::buildInputManifest(
      cli::required(args, "--input"), cli::required(args, "--output"), cli::selectionOptions(args));
    const nlohmann::json report{{"completed", result.completed}, {"code", result.code},
      {"message", result.message}, {"candidates", result.frames.size()}, {"selected", result.selected},
      {"batches", result.batches}, {"output_directory", result.output_directory.string()},
      {"work_directory", result.work_directory.string()}};
    std::cout << report.dump(2) << '\n';
    return result.ok() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    usage(std::cerr);
    return 2;
  }
}
