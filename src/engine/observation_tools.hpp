#pragma once

#include <filesystem>
#include <string_view>

#include <nlohmann/json_fwd.hpp>
#include <vln_memory/engine/observation_dataset_reader.hpp>

namespace vln_memory
{
/** @brief 执行两个观测工具，返回可直接作为函数调用输出的 JSON。 */
nlohmann::json runObservationTool(
  const ObservationDatasetReader & dataset, const std::filesystem::path & root,
  std::string_view name, const nlohmann::json & arguments);
}  // namespace vln_memory
