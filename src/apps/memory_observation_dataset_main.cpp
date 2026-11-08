#include <vln_memory/engine/observation_dataset.hpp>

#include <iostream>
#include <nlohmann/json.hpp>
#include "build_input_cli.hpp"

namespace
{
namespace cli = vln_memory::cli;

/** @brief 显示公开 CLI 参数；地图上下文与采样策略均由调用方声明。 */
void usage(std::ostream & out)
{
  out << "Usage: memory_observation_dataset " << cli::selectionArguments
      << " --map-id ID --geometry-revision REV --map-frame FRAME --map-basis TEXT"
      << " [--preparation-mode reference|bundle]\n";
}
}

/** @brief 从 preparation 直接发布观测数据集，返回可机读的完成信息。 */
int main(int argc, char ** argv)
{
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    usage(std::cout);
    return 0;
  }
  try {
    const auto args = cli::parseArguments(argc, argv,
      {"--map-id", "--geometry-revision", "--map-frame", "--map-basis", "--preparation-mode"});
    auto mode = vln_memory::ObservationPreparationMode::reference;
    if (const auto option = args.find("--preparation-mode"); option != args.end()) {
      if (option->second == "bundle") {mode = vln_memory::ObservationPreparationMode::bundle;}
      else if (option->second != "reference") {throw std::invalid_argument("Unknown preparation delivery mode");}
    }
    const vln_memory::ObservationBuildContext context{
      cli::required(args, "--map-id"), cli::required(args, "--geometry-revision"),
      cli::required(args, "--map-frame"), cli::required(args, "--map-basis"), 1, {}};
    const auto result = vln_memory::buildObservationDataset(
      cli::required(args, "--input"), cli::required(args, "--output"), cli::selectionOptions(args), context, mode);
    const nlohmann::json report{{"completed", result.completed}, {"code", result.code}, {"message", result.message},
      {"output_directory", result.output_directory.string()}, {"work_directory", result.work_directory.string()},
      {"candidates", result.candidates}, {"observations", result.observations}, {"batches", result.batches},
      {"pending", result.pending}, {"issues", result.issues}, {"verified_depth_payloads", result.verified_depth_payloads}};
    std::cout << report.dump(2) << '\n';
    return result.ok() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    usage(std::cerr);
    return 2;
  }
}
