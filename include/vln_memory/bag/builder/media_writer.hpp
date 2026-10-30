#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <vln_memory/bag/builder/bag_types.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>

namespace vln_memory
{

/** @brief 同一工作目录的文件 writer，共用输出字节预算；不允许覆盖已有文件。 */
class OutputFiles
{
public:
  /** @brief root 必须是 builder 刚创建的工作目录；最大字节数是所有输出合计。 */
  OutputFiles(std::filesystem::path root, std::uint64_t max_bytes);
  /** @brief 同步写拥有/借用字节，检查写入和关闭；返回后不保留 data。 */
  void write(const std::string & relative_path, const std::uint8_t * data, std::size_t bytes);
  /** @brief 追加完整 UTF-8 文本行并计入预算；流复用，避免每条记录重开文件。 */
  void appendLine(const std::string & relative_path, const std::string & line);
  /** @brief 创建可为空的 JSONL 文件；已打开同一文件时不重复创建。 */
  void ensureLines(const std::string & relative_path);
  /** @brief 明确 flush/close 并检查错误；完成后才能发布数据集。 */
  void close();
  /** @brief 返回实际已写字节数，不含文件系统元数据。 */
  std::uint64_t bytesWritten() const noexcept {return written_bytes_;}

private:
  /** @brief 校验相对路径及预算，不允许绝对路径或 .. 越界。 */
  std::filesystem::path targetPath(const std::string & relative_path) const;
  /** @brief 先检查再累计，避免 unsigned 加法溢出；失败抛 limit_exceeded。 */
  void reserveBytes(std::size_t bytes);
  std::filesystem::path root_;
  std::uint64_t max_bytes_;
  std::uint64_t written_bytes_{0};
  std::map<std::string, std::ofstream> line_streams_;
};

/** @brief 同步编码紧密 RGB8 为 PNG；不修改原帧，失败抛结构化 I/O 错误。 */
std::string writeRgb(OutputFiles & files, const std::string & message_id, const RgbFrame & frame);
/** @brief 保存二维小端 float32 米制 NPY，NaN 保留；不做毫米量化或归一化。 */
std::string writeDepth(OutputFiles & files, const std::string & message_id, const DepthFrame & frame);

}  // namespace vln_memory
