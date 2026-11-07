#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/tf/tf.hpp>

namespace vln_memory
{

enum class PreparationReadCode
{
  ready, unsupported_dataset, pair_not_found, invalid_metadata, invalid_reference, invalid_media, io_error,
  source_changed
};

/** @brief 标定选择事实；即使无候选也保留配置，原始参数与有效模型分开。 */
struct PreparedCalibration
{
  std::string calibration_id;
  std::string source_message_id;
  CameraInfoMatchOptions selection;
  std::string selection_code{"not_evaluated"};
  std::string selection_message;
  std::optional<CameraInfoErrorCode> calibration_code;
  bool used_static_calibration{false};
  std::optional<SensorTimeCheck> timing;
  std::optional<CameraInfoData> original;
  std::optional<CameraInfoErrorCode> source_validation_code;
  std::string source_validation_message;
  std::optional<CameraInfoErrorCode> normalized_validation_code;
  CameraInfoCompatibility source_compatibility{CameraInfoCompatibility::strict};
  bool normalization_applied{false};
  bool source_nonfinite{false};
  std::string original_unavailable_reason;
};

/** @brief 当前配对实际引用的消息摘要；路径相对于 preparation 根，省略 CDR 时为空。 */
struct PreparedMessageSummary
{
  std::string message_id;
  std::string topic;
  std::uint64_t file_index{0};
  std::uint64_t topic_ordinal{0};
  std::uint64_t message_order{0};
  std::int64_t bag_record_time_ns{0};
  std::uint64_t serialized_bytes{0};
  std::string payload_sha256;
  std::optional<std::filesystem::path> raw_path;
};

/** @brief 实际深度媒体解释，不含像素；与恢复出的 DepthFrame 使用同一份解析结果。 */
struct PreparedDepthStorage
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
  // 旧记录无逐帧 storage 时使用米制 NPY 契约；可能来自 v5 的 v7 转换副本。
  bool legacy_metric_npy{false};
};

/** @brief 已解析至根相对路径的媒体或 CDR；用于发现常规文件替换。 */
struct PreparedFileSnapshot
{
  std::filesystem::path relative_path;
  std::uintmax_t bytes{0};
  std::filesystem::file_time_type modified;
  std::string sha256{};
};

/** @brief 同一 preparation 目录的已保存配对，所有值和路径由结果持有。 */
struct PreparedPair
{
  std::string bag_id;
  std::string pair_id;
  std::string map_frame;
  TfScope time_scope;
  std::string clock_basis;
  std::string segment_basis;
  std::int64_t delta_ns{0};
  std::string timestamp_match_policy;
  std::string capture_sync_status;
  std::string status;
  std::vector<std::string> pending_reasons;
  std::string rgb_message_id;
  std::string depth_message_id;
  std::string rgb_calibration_id;
  std::string depth_calibration_id;
  CameraImageReference rgb_image;
  CameraImageReference depth_image;
  std::filesystem::path preparation_root;
  // 省略 CDR 的 v7 返回空值，历史格式和显式保留模式返回实际文件路径。
  std::optional<std::filesystem::path> rgb_raw_evidence;
  std::optional<std::filesystem::path> depth_raw_evidence;
  std::optional<std::filesystem::path> rgb_media;
  std::optional<std::filesystem::path> depth_media;
  std::optional<DepthFrame> depth;
  std::optional<CameraModel> rgb_model;
  std::optional<CameraModel> depth_model;
  std::optional<TfPoseResult> rgb_pose;
  std::optional<TfPoseResult> depth_pose;
  std::optional<TfPoseResult> body_pose;

  // 组装需要的来源事实；不包含 map_id、geometry_revision 等业务决策。
  int builder_version{0};
  std::string source_manifest_sha256;
  std::string body_frame;
  std::int64_t sync_tolerance_ns{0};
  std::int64_t tf_max_gap_ns{0};
  TfJumpLimits tf_jump_limits;
  std::string capture_sync_basis;
  PreparedCalibration rgb_calibration;
  PreparedCalibration depth_calibration;
  std::vector<PreparedMessageSummary> message_summaries;
  std::optional<PreparedDepthStorage> depth_storage;
  std::optional<std::filesystem::path> rgb_media_relative;
  std::optional<std::filesystem::path> depth_media_relative;
  DepthAlignment depth_alignment{DepthAlignment::unknown};
  std::optional<DepthAlignmentStatus> depth_alignment_check;
  std::string alignment_basis;
  std::vector<PreparedFileSnapshot> file_snapshots;
};

struct PreparationSourceCheck
{
  PreparationReadCode code{PreparationReadCode::io_error};
  std::string message;
  /** @brief 已核对索引来源和传入文件的路径、大小、修改时间。 */
  bool ok() const noexcept {return code == PreparationReadCode::ready;}
};

struct PreparationReadResult
{
  PreparationReadCode code{PreparationReadCode::io_error};
  std::string message;
  std::optional<PreparedPair> pair;

  /** @brief 读取成功可包含 pending 配对，所需的可选字段由消费操作检查。 */
  bool ok() const noexcept {return code == PreparationReadCode::ready && pair.has_value();}
};

/** @brief 已保存配对的轻量目录项，时间为原始 sec/nanosec，顺序与 JSONL 一致。 */
struct PreparationPairInfo
{
  std::string pair_id;
  std::string rgb_message_id;
  std::string depth_message_id;
  SensorStamp rgb_stamp;
  SensorStamp depth_stamp;
  std::int64_t delta_ns{0};
  std::string status;
  std::vector<std::string> pending_reasons;
};

struct PreparationReaderState;
struct PreparationOpenResult;

/** @brief 独占一个已发布数据集的只读索引，通过 openPreparation 创建。 */
class PreparationReader
{
public:
  /** @brief 释放索引，已返回的 PreparedPair 继续独立持有其数据。 */
  ~PreparationReader();

  /** @brief 返回源文件顺序的目录，引用有效期与 reader 一致。 */
  const std::vector<PreparationPairInfo> & pairs() const noexcept;

  /** @brief 按索引读取一对并恢复深度，失败时返回空 pair，可继续读取其他 ID。 */
  PreparationReadResult loadPair(std::string_view pair_id) const;

  /** @brief 校验配对、证据、媒体头和存储契约；不解码深度，PreparedPair::depth 为空。 */
  PreparationReadResult loadPairMetadata(std::string_view pair_id) const;

  /** @brief 返回打开时绑定的规范数据集根；引用随 reader 生命周期有效。 */
  const std::filesystem::path & root() const noexcept;

  /** @brief 返回 manifest 原始字节的 SHA-256；视图随 reader 生命周期有效。 */
  std::string_view manifestSha256() const noexcept;

  /** @brief 返回 preparation 中声明的 map frame；视图随 reader 生命周期有效。 */
  std::string_view mapFrame() const noexcept;

  /** @brief 复核索引、文件状态和已绑定的内容摘要，变化时报告 source_changed。 */
  PreparationSourceCheck verifyUnchanged(const std::vector<PreparedFileSnapshot> & files = {}) const;

private:
  /** @brief 完整读取校验 RGB 像素并恢复深度；两种模式共享索引和元数据检查。 */
  PreparationReadResult loadPairImpl(std::string_view pair_id, bool validate_pixels) const;
  /** @brief 接管已经完整建立的索引，调用方通过工厂取得唯一所有权。 */
  explicit PreparationReader(std::unique_ptr<PreparationReaderState> state);

  std::unique_ptr<PreparationReaderState> state_;
  friend PreparationOpenResult openPreparation(const std::filesystem::path & preparation_root);
};

struct PreparationOpenResult
{
  PreparationReadCode code{PreparationReadCode::io_error};
  std::string message;
  std::unique_ptr<PreparationReader> reader;

  /** @brief 打开成功时持有完整 reader，失败时 reader 为空。 */
  bool ok() const noexcept {return code == PreparationReadCode::ready && reader != nullptr;}
};

/**
 * @brief 扫描 v5/v6/v7 的五个 JSONL 建立 ID/偏移索引，并保留原始配对目录。
 * @note 打开时校验全局 schema 和键唯一性，关联与媒体在 loadPair 时校验。
 *       读取期间元数据需保持稳定；大小、修改时间或路径变化返回 source_changed。
 */
PreparationOpenResult openPreparation(const std::filesystem::path & preparation_root);

/**
 * @brief 只读加载 BagBuilder v5/v6/v7 已保存的一个配对，不重新匹配。
 * @note 返回绝对媒体路径和米制深度。文件或引用损坏时不返回部分结果。
 *       导出记录中的缺项保留为空及原始 pending 原因。
 */
PreparationReadResult loadPreparedPair(
  const std::filesystem::path & preparation_root, std::string_view pair_id);

}  // namespace vln_memory
