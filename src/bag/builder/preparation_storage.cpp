#include "preparation_storage.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#ifdef __linux__
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif
#include <vln_memory/bag/builder/bag_types.hpp>

namespace vln_memory::detail
{
namespace fs = std::filesystem;

/** @brief 检查路径包含关系，按组件比较，不能用字符串前缀误判 sibling。 */
bool containsPath(const fs::path & parent, const fs::path & candidate)
{
  auto next = candidate.begin();
  for (const auto & part : parent) {
    if (next == candidate.end() || part != *next) {return false;}
    ++next;
  }
  return true;
}

/** @brief 输出与输入不重叠、旧结果不覆盖；创建相同文件系统上的唯一工作目录。 */
fs::path createWorkDirectory(const fs::path & input_path, const fs::path & output_path)
{
  const auto input = fs::weakly_canonical(input_path);
  const auto output = fs::weakly_canonical(output_path);
  if (containsPath(input, output) || containsPath(output, input)) {
    throw BagBuildError("invalid_output", "Input and output paths must not contain each other");
  }
  if (fs::exists(output) || fs::is_symlink(output)) {throw BagBuildError("output_exists", "Output already exists");}
  fs::create_directories(output.parent_path());
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto work = fs::path(output.string() + ".partial-" + std::to_string(tick) + "-" + std::to_string(attempt));
    if (fs::create_directory(work)) {return work;}
  }
  throw BagBuildError("invalid_output", "Cannot allocate unique work directory");
}

/** @brief Ubuntu/WSL 原生不覆盖原子改名；并发出现空目录也不能被替换。 */
void publishDirectory(const fs::path & work, const fs::path & final_path)
{
#ifdef __linux__
  if (::syscall(SYS_renameat2, AT_FDCWD, work.c_str(), AT_FDCWD, final_path.c_str(), RENAME_NOREPLACE) != 0) {
    const auto error = errno;
    if (error == EEXIST || error == ENOTEMPTY) {throw BagBuildError("output_exists", "Output appeared during build");}
    throw BagBuildError("io_error", "Atomic publication failed: " + std::string(std::strerror(error)));
  }
#else
  (void)work;
  (void)final_path;
  throw BagBuildError("unsupported_platform", "First-version publication requires Linux renameat2");
#endif
}

}  // namespace vln_memory::detail
