#include "../engine/memory_publication.hpp"
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
  if (failed || std::cin.bad() || !std::cout) {
    return 1;
  }
  return 0;
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
  options.rgb_jpeg_quality = model["rgb_jpeg_quality"].as<int>(options.rgb_jpeg_quality);
  if (options.rgb_jpeg_quality < 0 || options.rgb_jpeg_quality > 100) {
    throw std::invalid_argument("model.rgb_jpeg_quality must be 0 (PNG) or between 1 and 100");
  }
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

/** @brief 从统一 YAML 打开观测，按模式调用工具、构建或发布版本。 */
int main(int argc, char ** argv)
{
  try {
    constexpr char usage[] = "Usage: memory_engine --config FILE [--build | --publish OUTPUT_DIR]";
    if (argc == 2 && std::string_view(argv[1]) == "--help") {
      std::cout << usage << '\n';
      return 0;
    }
    const bool build = argc == 4 && std::string_view(argv[3]) == "--build";
    const bool publish = argc == 5 && std::string_view(argv[3]) == "--publish";
    if ((argc != 3 && !build && !publish) || std::string_view(argv[1]) != "--config") {
      throw std::invalid_argument(usage);
    }
    const auto config = YAML::LoadFile(argv[2]);
    const auto root = config["dataset"]["memory_dir"].as<std::string>();
    const auto opened = vln_memory::openObservationDataset(root);
    if (!opened.ok()) {throw std::runtime_error(opened.message);}
    if (!opened.reader->info().direct_evidence) {
      throw std::runtime_error("Expected a direct BagBuilder dataset");
    }
    const auto published = vln_memory::publishedMemoryVersion(opened.reader->info(), root);
    if (published && (publish || build)) {
      throw std::invalid_argument("Use the working dataset for --build or --publish");
    }
    if (publish) {
      std::cout << vln_memory::publishMemoryVersion(opened.reader->info(), root, argv[4]).dump() << '\n';
      return 0;
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
    vln_memory::MemoryTools tools(*opened.reader, root, near, published);
    if (!build) {return runTools(tools);}
    std::cout << vln_memory::runModelBatches(*opened.reader, tools, modelOptions(config, argv[2])).dump() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
