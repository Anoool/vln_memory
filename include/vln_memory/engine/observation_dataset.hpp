#pragma once

#include <vln_memory/engine/build_input.hpp>
#include <vln_memory/engine/observation_builder.hpp>

namespace vln_memory
{

/** @brief 观测数据集的发布结果；失败时保留工作目录以便诊断。 */
struct ObservationDatasetResult
{
  bool completed{false};
  std::string code;
  std::string message;
  std::filesystem::path output_directory;
  std::filesystem::path work_directory;
  std::size_t candidates{0};
  std::size_t observations{0};
  std::size_t batches{0};
  std::size_t pending{0};
  std::size_t issues{0};
  std::size_t verified_depth_payloads{0};

  /** @brief 成功表示目录完整发布，允许合法空数据集及 pending 观测。 */
  bool ok() const noexcept {return completed;}
};

/**
 * @brief 从一个 preparation 核验、采样并原子发布正式观测数据集。
 * @note 入选前沿用 build_input 的完整深度读取。组装只加载元数据，
 *       发布前复查已读媒体和索引。坏候选记录为问题，入选项组装失败则中止发布。
 */
ObservationDatasetResult buildObservationDataset(
  const std::filesystem::path & preparation, const std::filesystem::path & output,
  const BuildInputOptions & selection, const ObservationBuildContext & context);

}  // namespace vln_memory
