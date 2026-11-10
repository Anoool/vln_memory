#pragma once

#include "memory_tools.hpp"

#include <filesystem>

namespace vln_memory
{
struct ModelBuildOptions
{
  std::string base_url;
  std::string model;
  std::filesystem::path api_key_file;
  std::string api_key_env;
  long timeout_s{60};
  std::size_t observations_per_batch{4};
  std::size_t max_tool_rounds{12};
  bool resume{true};
};

/** @brief 从已提交的 SQLite 恢复点继续运行一组模型批次。 */
nlohmann::json runModelBatches(
  const ObservationDatasetReader & dataset, MemoryTools & tools, const ModelBuildOptions & options);
}  // namespace vln_memory
