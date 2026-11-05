#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace vln_memory
{
/** @brief 精简操作的实际进度与发布结果，失败时可能保留工作目录。 */
struct PreparationCompactResult
{
  bool completed{false};
  std::string code;
  std::string message;
  std::filesystem::path output_directory;
  std::filesystem::path work_directory;
  std::uint64_t messages{0};
  std::uint64_t pairs_checked{0};
  std::uint64_t raw_bytes_omitted{0};

  /** @brief 仅完整校验并原子发布后返回成功。 */
  bool ok() const noexcept {return completed;}
};

/**
 * @brief 只读转换 v5/v6/v7 preparation 到省略 CDR 的 v7 新目录。
 * @note 保留 ID、时间、标定、位姿、配对和媒体。源/目标互不包含，已有目标不覆盖。
 *       所有配对通过完整读取后发布，失败工作目录可排查，源目录始终只读。
 */
PreparationCompactResult compactPreparation(
  const std::filesystem::path & source, const std::filesystem::path & output);
}  // namespace vln_memory
