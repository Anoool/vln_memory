#pragma once

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

private:
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
