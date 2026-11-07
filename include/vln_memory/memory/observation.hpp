#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/tf/tf.hpp>

namespace vln_memory
{

inline constexpr char kObservationSchemaVersion[] = "observation/1";

/** @brief 原始消息定位和摘要；raw_path 可为空，所有路径相对于来源数据集根。 */
struct ObservationEvidence
{
  std::string message_id;
  std::string topic;
  std::uint64_t file_index{0};
  std::uint64_t topic_ordinal{0};
  std::uint64_t message_order{0};
  std::int64_t bag_record_time_ns{0};
  std::uint64_t serialized_bytes{0};
  std::string payload_sha256;
  std::optional<std::string> raw_path;
};

/** @brief 原始 CameraInfo 和实际选择参数；有效模型在 Observation 中分别保存。 */
struct ObservationCalibration
{
  std::string calibration_id;
  std::string source_message_id;
  CameraInfoData original;
  CameraInfoMatchOptions selection;
};

/** @brief 深度文件实际存储语义；运行时恢复为米制 optical-z，不保存像素。 */
struct ObservationDepthStorage
{
  std::string format;
  std::string dtype;
  DepthUnit unit{DepthUnit::unspecified};
  double scale_to_m{0};
  std::string invalid_rule;
  std::string source_encoding;
  DepthUnit source_unit{DepthUnit::unspecified};
  double source_scale_to_m{0};
  double min_depth_m{0};
  std::optional<double> max_depth_m;
  std::uint64_t valid_pixels{0};
  std::uint64_t invalid_pixels{0};
};

/** @brief 正式拥有型观测；可缺媒体/模型/位姿，但缺项和未知质量必须保留 pending。 */
struct Observation
{
  std::string id;
  std::uint64_t revision{1};
  std::string bag_id;
  std::string pair_id;
  std::string rgb_message_id;
  std::string depth_message_id;
  std::string source_manifest_sha256;
  std::string clock_id;
  std::string segment_id;
  std::string clock_basis;
  std::string segment_basis;
  std::string map_id;
  std::string geometry_revision;
  std::string map_frame;
  std::string body_frame;
  std::string map_basis;
  SensorStamp stamp;
  CameraImageReference rgb_image;
  CameraImageReference depth_image;
  std::int64_t delta_ns{0};
  std::int64_t sync_tolerance_ns{0};
  std::string time_match;
  std::string pairing_policy;
  std::string source_pair_status;
  std::vector<std::string> source_pending_reasons;
  std::string capture_sync_status{"unknown"};
  std::string capture_sync_basis;
  DepthAlignment depth_alignment{DepthAlignment::unknown};
  std::string alignment_basis;
  std::optional<std::string> rgb_media;
  std::optional<std::string> depth_media;
  std::optional<ObservationDepthStorage> depth_storage;
  std::optional<ObservationCalibration> rgb_calibration;
  std::optional<ObservationCalibration> depth_calibration;
  std::optional<CameraModel> rgb_model;
  std::optional<CameraModel> depth_model;
  std::int64_t tf_max_gap_ns{0};
  TfJumpLimits tf_jump_limits;
  std::optional<TfPoseResult> camera_pose_map;
  std::optional<TfPoseResult> depth_camera_pose_map;
  std::optional<TfPoseResult> body_pose_map;
  std::vector<ObservationEvidence> evidence;
  std::vector<std::string> pending_reasons;
};

enum class ObservationCode
{
  ready, invalid_identity, invalid_context, invalid_time, invalid_media, invalid_calibration,
  invalid_pose, invalid_reference, invalid_pending, invalid_json, unsupported_schema
};

struct ObservationCheck
{
  ObservationCode code{ObservationCode::invalid_context};
  std::string message;
  /** @brief 只表示记录结构和内部一致性合法；不证明物理精度或外部文件真实性。 */
  bool ok() const noexcept {return code == ObservationCode::ready;}
};

struct ObservationJsonResult
{
  ObservationCode code{ObservationCode::invalid_json};
  std::string message;
  std::string json;
  /** @brief 成功返回单条完整 JSON，失败字符串为空。 */
  bool ok() const noexcept {return code == ObservationCode::ready && !json.empty();}
};

struct ObservationParseResult
{
  ObservationCode code{ObservationCode::invalid_json};
  std::string message;
  std::optional<Observation> observation;
  /** @brief 成功可包含合法 pending 观测，失败不返回部分对象。 */
  bool ok() const noexcept {return code == ObservationCode::ready && observation.has_value();}
};

/** @brief 返回稳定错误名，未知枚举返回 unknown，无所有权转移。 */
const char * observationCodeName(ObservationCode code) noexcept;

/**
 * @brief 用来源 bag/pair/RGB/Depth/clock/segment 的长度前缀元组生成确定键。
 * @throws std::invalid_argument 任一身份分量为空、含控制字符或超过 4096 字节。
 * @note 不含 map/geometry revision；键不用于路径或可读名称。
 */
std::string makeObservationId(const Observation & observation);

/** @brief 保留来源及已有原因，再补齐缺项/未知声明的规范原因，不修改输入。 */
std::vector<std::string> collectObservationPendingReasons(const Observation & observation);

/** @brief 校验时间、模型、TF 重算、存储及证据引用；不解码媒体、不读取源 bag。 */
ObservationCheck validateObservation(const Observation & observation);

/** @brief 校验成功后编码 observation/1 单行 JSON；失败返回错误和空字符串。 */
ObservationJsonResult serializeObservation(const Observation & observation);

/**
 * @brief 严格解析并校验 observation/1；拒绝重复键、未知字段、非整数类型与部分对象。
 * @param max_json_bytes 输入上限，默认 16 MiB；限制解析阶段，不限制调用前的字符串分配。
 * @note JSON 最大嵌套深度为 64；媒体实际内容仍由消费/审计边界核验。
 */
ObservationParseResult deserializeObservation(
  std::string_view json, std::size_t max_json_bytes = 16U * 1024U * 1024U);

}  // namespace vln_memory
