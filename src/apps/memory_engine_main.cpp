#include "../engine/model_batches.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
using Json = nlohmann::json;

/** @brief 从标准输入读取一行函数请求，将结果写回 JSONL。 */
int runTools(vln_memory::MemoryTools & tools)
{
  bool failed = false;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) {continue;}
    Json output;
    try {
      output = tools.call(Json::parse(line));
    } catch (const Json::exception & error) {
      output = {{"status", "invalid_request"}, {"message", error.what()}};
    }
    failed = failed || output.at("status") != "ok";
    std::cout << output.dump(-1, ' ', false, Json::error_handler_t::replace) << '\n';
  }
  return failed || std::cin.bad() || !std::cout ? 1 : 0;
}

/** @brief 读取批次和模型参数，密钥文件以主配置的目录为基准。 */
vln_memory::ModelBuildOptions modelOptions(
  const YAML::Node & config, const std::filesystem::path & config_path)
{
  vln_memory::ModelBuildOptions options;
  const auto model = config["model"];
  options.base_url = model["base_url"].as<std::string>();
  options.model = model["model"].as<std::string>();
  const auto key_file = model["api_key_file"].as<std::string>("");
  if (!key_file.empty()) {options.api_key_file = config_path.parent_path() / key_file;}
  options.api_key_env = model["api_key_env"].as<std::string>("");
  options.timeout_s = model["timeout_s"].as<long>(options.timeout_s);
  if (const auto build = config["build"]) {
    options.observations_per_batch = build["observations_per_batch"].as<std::size_t>(options.observations_per_batch);
    const auto rounds = build["max_tool_rounds"].as<long>(static_cast<long>(options.max_tool_rounds));
    if (rounds <= 0) {throw std::invalid_argument("build.max_tool_rounds must be positive");}
    options.max_tool_rounds = static_cast<std::size_t>(rounds);
    options.resume = build["resume"].as<bool>(options.resume);
  }
  if ((options.base_url.rfind("http://", 0) != 0 && options.base_url.rfind("https://", 0) != 0) ||
    options.model.empty() || options.timeout_s <= 0 ||
    options.observations_per_batch < 1 || options.observations_per_batch > 4 || options.max_tool_rounds == 0)
  {
    throw std::invalid_argument("Expected an HTTP(S) model URL, a model, 1-4 observations and positive time/round limits");
  }
  return options;
}
}  // namespace

/** @brief 从统一 YAML 打开 BagBuilder 产物，按所选模式执行单条工具或离线模型批次。 */
int main(int argc, char ** argv)
{
  try {
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
      std::cout << "Usage: memory_engine --config config/vln_memory.yaml [--build]\n";
      return 0;
    }
    const bool build = argc == 4 && std::string_view(argv[3]) == "--build";
    if ((argc != 3 && !build) || std::string_view(argv[1]) != "--config") {
      throw std::invalid_argument("Expected --config FILE [--build]");
    }
    const auto config = YAML::LoadFile(argv[2]);
    const auto root = config["dataset"]["memory_dir"].as<std::string>();
    const auto opened = vln_memory::openObservationDataset(root);
    if (!opened.ok()) {throw std::runtime_error(opened.message);}
    if (!opened.reader->info().direct_evidence) {
      throw std::runtime_error("Expected a direct BagBuilder dataset");
    }
    vln_memory::NearOptions near;
    if (const auto geometry = config["geometry"]) {
      near.dimension = geometry["near_dimension"].as<std::string>(near.dimension);
      near.threshold_m = geometry["near_threshold_m"].as<double>(near.threshold_m);
    }
    if ((near.dimension != "xy" && near.dimension != "xyz") || !std::isfinite(near.threshold_m) ||
      near.threshold_m <= 0)
    {
      throw std::invalid_argument("geometry needs near_dimension xy or xyz and positive near_threshold_m");
    }
    vln_memory::MemoryTools tools(*opened.reader, root, near);
    if (!build) {return runTools(tools);}
    std::cout << vln_memory::runModelBatches(*opened.reader, tools, modelOptions(config, argv[2])).dump() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
