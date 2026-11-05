#include <vln_memory/bag/builder/preparation_compactor.hpp>

#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

/** @brief 转换到新目录，成功返回 0，转换失败返回 1，参数错误返回 2。 */
int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: preparation_compact --input DATASET --output NEW_DIRECTORY\n";
    return 0;
  }
  if (argc != 5 || std::string(argv[1]) != "--input" || std::string(argv[3]) != "--output") {
    std::cerr << "Usage: preparation_compact --input DATASET --output NEW_DIRECTORY\n";
    return 2;
  }
  const auto result = vln_memory::compactPreparation(argv[2], argv[4]);
  const nlohmann::json report{{"completed", result.completed}, {"code", result.code},
    {"message", result.message}, {"output_directory", result.output_directory.string()},
    {"work_directory", result.work_directory.string()}, {"messages", result.messages},
    {"pairs_checked", result.pairs_checked}, {"raw_bytes_omitted", result.raw_bytes_omitted}};
  std::cout << report.dump(2) << '\n';
  return result.ok() ? 0 : 1;
}
