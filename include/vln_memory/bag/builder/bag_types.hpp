#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <vln_memory/bag/common/sensor_types.hpp>

namespace vln_memory
{

/** @brief 可预期的配置/来源/存储失败；顶层转成结构化结果，底层不吞错误。 */
class BagBuildError : public std::runtime_error
{
public:
  /** @brief 保存稳定错误码和拥有型说明；不会引用调用方字符串。 */
  BagBuildError(std::string code, const std::string & message)
  : std::runtime_error(message), code_(std::move(code)) {}
  /** @brief 返回拥有的错误码引用，生命周期与本异常一致。 */
  const std::string & code() const noexcept {return code_;}

private:
  std::string code_;
};

/** @brief 完整来源定位；ordinal 是分卷内同 topic 顺序，order 是选定流全局扫描顺序。 */
struct BagMessageRef
{
  std::string message_id;
  std::string topic;
  std::size_t file_index{0};
  std::uint64_t topic_ordinal{0};
  std::uint64_t message_order{0};
  std::int64_t bag_record_time_ns{0};
  std::size_t serialized_bytes{0};
  std::string payload_sha256;
};

/** @brief 图像轻量索引，拥有字符串；不保存序列化或像素缓冲区。 */
struct BagImageRecord
{
  BagMessageRef source;
  SensorStamp stamp;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::uint32_t step{0};
  std::uint8_t is_bigendian{0};
  std::size_t data_bytes{0};
  std::string deserialization_error;
  // 压缩消息没有原始 step；布局由编码头检查得到，原 format/CDR 单独保留。
  std::string message_type{"sensor_msgs/msg/Image"};
  std::string compressed_format;
  std::string payload_error_code;
  std::string payload_error;
};

/** @brief 一个实际存储分卷及其内容指纹，文件顺序取自 bag 元数据。 */
struct BagSourceFile
{
  std::string path;
  std::uint64_t bytes{0};
  std::string sha256;
};

/** @brief 声明计数和实际选定读取计数分别记录；未选话题实际读取为零。 */
struct BagTopicRecord
{
  std::string name;
  std::string type;
  std::string serialization_format;
  std::string offered_qos_profiles;
  std::uint64_t declared_count{0};
  std::uint64_t scanned_count{0};
  bool selected{false};
};

/** @brief 机器可读问题；message_id/pair_id 可空，不能靠解析 message 决定行为。 */
struct BagIssue
{
  std::string stage;
  std::string code;
  std::string message;
  std::string message_id;
  std::string pair_id;
};

enum class BagBuildStatus {failed, completed, completed_with_issues};

/** @brief 对外仅返回状态、计数和路径；失败可保留 work_directory 供排查。 */
struct BagBuildResult
{
  BagBuildStatus status{BagBuildStatus::failed};
  std::string code;
  std::string message;
  std::string output_directory;
  std::string work_directory;
  // 提取成功与 header 匹配分开；本字段不代表设备曝光同步。
  std::string timestamp_match_status{"not_checked"};
  std::size_t rgb_frames{0};
  std::size_t depth_frames{0};
  std::size_t decoded_frames{0};
  std::size_t pairs{0};
  std::size_t prepared_pairs{0};
  std::size_t issue_count{0};
  /** @brief 完整发布（可带问题）返回 true，不表示所有帧或几何均可用。 */
  bool ok() const noexcept {return status != BagBuildStatus::failed;}
};

/** @brief 稳定状态名称，无所有权转移。 */
inline const char * bagBuildStatusName(BagBuildStatus status) noexcept
{
  switch (status) {
    case BagBuildStatus::failed: return "failed";
    case BagBuildStatus::completed: return "completed";
    case BagBuildStatus::completed_with_issues: return "completed_with_issues";
  }
  return "unknown";
}

}  // namespace vln_memory
