#pragma once

#include <filesystem>

namespace vln_memory::detail
{
/** @brief 按路径组件检查 candidate 是否位于 parent 内。 */
bool containsPath(const std::filesystem::path & parent, const std::filesystem::path & candidate);
/** @brief 固定绝对发布目标，拒绝空路径、嵌入 NUL 和输出符号链接。 */
std::filesystem::path resolveOutputDirectory(const std::filesystem::path & output);
/** @brief 用已固定的目标路径，检查重叠和覆盖后在同一父目录创建唯一 partial。 */
std::filesystem::path createWorkDirectory(
  const std::filesystem::path & input, const std::filesystem::path & output);
/** @brief 使用 Linux renameat2 原子发布，拒绝覆盖并发出现的目录。 */
void publishDirectory(const std::filesystem::path & work, const std::filesystem::path & output);
}  // namespace vln_memory::detail
