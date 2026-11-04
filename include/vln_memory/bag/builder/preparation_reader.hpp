#pragma once

#include <filesystem>
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
  ready, unsupported_dataset, pair_not_found, invalid_metadata, invalid_reference, invalid_media, io_error
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
  std::filesystem::path rgb_raw_evidence;
  std::filesystem::path depth_raw_evidence;
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

/**
 * @brief 只读加载 BagBuilder v5/v6 已保存的一个配对，不重新匹配。
 * @note 返回绝对媒体路径和米制深度。文件或引用损坏时不返回部分结果。
 *       导出记录中的缺项保留为空及原始 pending 原因。
 */
PreparationReadResult loadPreparedPair(
  const std::filesystem::path & preparation_root, std::string_view pair_id);

}  // namespace vln_memory
