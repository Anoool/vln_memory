#pragma once

#include <filesystem>

namespace vln_memory::detail
{
/** @brief 按路径组件检查 candidate 是否位于 parent 内。 */
bool containsPath(const std::filesystem::path & parent, const std::filesystem::path & candidate);
/** @brief 拒绝路径重叠或既有目标，创建同文件系统上的唯一 partial 目录。 */
std::filesystem::path createWorkDirectory(
  const std::filesystem::path & input, const std::filesystem::path & output);
/** @brief 使用 Linux renameat2 原子发布，拒绝覆盖并发出现的目录。 */
void publishDirectory(const std::filesystem::path & work, const std::filesystem::path & output);
}  // namespace vln_memory::detail
