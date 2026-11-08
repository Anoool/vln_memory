#pragma once

#include <functional>
#include <istream>
#include <map>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <vln_memory/bag/builder/preparation_reader.hpp>

namespace vln_memory::detail
{
struct PreparationMetadataError : std::runtime_error
{
  PreparationReadCode code;
  /** @brief 私有格式/引用错误，公开读取边界转成稳定错误码。 */
  PreparationMetadataError(PreparationReadCode value, const std::string & message)
  : std::runtime_error(message), code(value) {}
};

inline constexpr std::size_t kMaxPreparationJsonBytes = 16U * 1024U * 1024U;

/** @brief 有界读取一条 JSONL，遇到 EOF 返回 false。 */
bool readPreparationLine(std::istream & input, std::string & line);
/** @brief 逐条访问有界 JSONL 原始行及偏移，返回同一遍所读文件的字节摘要。 */
std::string scanPreparationJsonl(
  const std::filesystem::path & path,
  const std::function<void(std::streamoff, const std::string &)> & visit);
/** @brief 有界读取 manifest 原始字节，供解析和指纹核对。 */
std::string readPreparationDocument(const std::filesystem::path & path);
/** @brief 按重复键和嵌套限制解析一条 preparation JSON。 */
nlohmann::json parsePreparationJson(std::string_view bytes);

/** @brief 用已读取的 manifest 原始字节计算 SHA-256，不再次读取文件。 */
std::string preparationSha256(std::string_view bytes);
/** @brief 流式计算已保存原始 CDR 的 SHA-256。 */
std::string preparationFileSha256(const std::filesystem::path & path);
inline constexpr char kPreparationMediaIndexFile[] = "media_sha256.jsonl";
inline constexpr char kPreparationMediaIndexSchema[] = "preparation_media/1";

struct PreparationMediaDigest
{
  std::string path;
  std::string sha256;
};

using PreparationMediaIndex = std::map<std::string, PreparationMediaDigest>;

/** @brief 读取并验证 manifest 绑定的媒体索引；旧产物返回空 optional。 */
std::optional<PreparationMediaIndex> readPreparationMediaIndex(
  const std::filesystem::path & root, const nlohmann::json & manifest);
/** @brief 核对帧的消息身份和媒体路径，返回绑定的内容摘要。 */
const PreparationMediaDigest * preparationFrameMedia(
  const std::optional<PreparationMediaIndex> & index, const nlohmann::json & frame);
/** @brief 生成规范的索引行；路径、消息身份和媒体字节摘要一起绑定。 */
nlohmann::json preparationMediaRow(std::string_view message_id, std::string_view path, std::string_view sha256);

/** @brief 严格读取规范十进制 int64，不接受浮点或溢出。 */
std::int64_t preparationSigned(const nlohmann::json & value);
/** @brief 解析数据集事实和两路相机选择配置，不决定正式地图身份。 */
PreparedPair readPreparationContext(const nlohmann::json & manifest);
/** @brief 保存实际选择结果及可解释原始标定；非有限原值明确不可恢复。 */
PreparedCalibration readPreparationCalibration(
  const nlohmann::json & frame, const nlohmann::json * calibration,
  const CameraInfoMatchOptions & selection);
/** @brief 用原始参数重建已保存模型，核对实际匹配结果与配置。 */
void checkPreparationModel(
  const PreparedCalibration & calibration, const std::optional<CameraModel> & model,
  const CameraImageReference & image);
/** @brief 解出当前帧存储说明，历史记录回退必须由随后 NPY 读取核验。 */
std::optional<PreparedDepthStorage> readPreparationDepthStorage(
  const nlohmann::json & frame, const nlohmann::json & depth_options);
/** @brief 解析完整消息摘要，保留精确整数和可选 CDR 路径。 */
PreparedMessageSummary readPreparationMessage(const nlohmann::json & row);
}  // namespace vln_memory::detail
