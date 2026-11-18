#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include <nlohmann/json_fwd.hpp>
#include <vln_memory/engine/observation_dataset_reader.hpp>

namespace vln_memory
{
/** @brief 返回已发布版本的身份，且核对它与已打开的观测目录相符。 */
std::optional<std::string> publishedMemoryVersion(
  const ObservationDatasetInfo & dataset, const std::filesystem::path & root);

/** @brief 从工作库和正式观测写成可独立回读的版本，成功后才公布目标目录。 */
nlohmann::json publishMemoryVersion(
  const ObservationDatasetInfo & dataset, const std::filesystem::path & source,
  const std::filesystem::path & output);
}  // namespace vln_memory
