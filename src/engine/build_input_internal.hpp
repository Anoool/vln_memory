#pragma once

#include <vln_memory/engine/build_input.hpp>
#include <nlohmann/json.hpp>

namespace vln_memory::detail
{

/** @brief 已按用途核验和排序的选择结果，以及可选的源文件状态。 */
struct BuildInputScan
{
  BuildInputResult selection;
  std::vector<PreparedFileSnapshot> verified_files;
  std::size_t verified_depth_payloads{0};
};

/** @brief 扫描一次完整媒体、运行共享采样；source_changed 抛出全局错误。 */
BuildInputScan scanBuildInput(
  const PreparationReader & reader, const BuildInputOptions & options, bool retain_file_snapshots);

/** @brief 使用同一组选项字段记录实际阈值和角度单位。 */
nlohmann::json buildInputOptionsJson(const BuildInputOptions & options);

/** @brief 写出完整候选、决策和原配对批次，用于追溯正式观测。 */
void writeBuildInputSelection(
  const std::filesystem::path & path, const std::filesystem::path & root,
  const BuildInputResult & selection, const BuildInputOptions & options);

/** @brief 返回稳定的 preparation 读取错误名称。 */
const char * preparationReadCodeName(PreparationReadCode code);

}  // namespace vln_memory::detail
