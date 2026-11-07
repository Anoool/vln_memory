#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <vln_memory/bag/builder/preparation_reader.hpp>

namespace vln_memory
{

enum class BuildInputPurpose {rgb, depth_map, prepared_rgbd};

/** @brief 离线输入要求；距离为米，角度为弧度，空阈值关闭对应规则。 */
struct BuildInputOptions
{
  BuildInputPurpose purpose{BuildInputPurpose::rgb};
  std::size_t batch_size{1};
  std::optional<double> distance_m;
  std::optional<double> yaw_rad;
  std::optional<double> max_step_m;
  std::optional<double> max_step_yaw_rad;
};

/** @brief 轻量候选及其选择结果；不持有像素，来源时间与 pending 原样保留。 */
struct BuildInputFrame
{
  std::size_t source_index{0};
  std::string pair_id;
  std::string bag_id;
  std::string map_frame;
  std::string clock_id;
  std::string segment_id;
  std::string rgb_message_id;
  std::string depth_message_id;
  SensorStamp rgb_stamp;
  SensorStamp depth_stamp;
  std::int64_t delta_ns{0};
  std::string pair_status;
  std::string capture_sync_status;
  std::vector<std::string> pending_reasons;
  PreparationReadCode read_code{PreparationReadCode::ready};
  std::string read_message;
  std::optional<TfPose> body_pose_map;
  bool eligible{false};
  std::vector<std::string> eligibility_reasons;
  bool selected{false};
  std::string selection_reason;
  std::optional<std::size_t> selection_index;
  std::optional<std::size_t> batch_index;
};

struct BuildInputResult
{
  bool completed{false};
  std::string code;
  std::string message;
  std::filesystem::path output_directory;
  std::filesystem::path work_directory;
  std::size_t selected{0};
  std::size_t batches{0};
  std::vector<BuildInputFrame> frames;

  /** @brief 成功表示清单完整发布；允许合法空清单或候选均不满足用途。 */
  bool ok() const noexcept {return completed;}
};

/** @brief 返回稳定用途名称；未知枚举返回 unknown。 */
const char * buildInputPurposeName(BuildInputPurpose purpose) noexcept;

/**
 * @brief 对已核验用途的轻量候选稳定按 RGB 时间排序，选择关键帧并分批。
 * @param frames 输入含 eligibility/来源/可选底盘位姿，输出覆盖全部选择字段。
 * @throws std::invalid_argument 配置、ID 或可用候选的地图/时钟上下文不一致。
 * @note 空阈值选择所有 eligible 候选；有位姿规则时缺姿态形成采样断点。
 *       显式步进上限触发后本段余下候选均拒绝，需独立核查轨迹。
 */
void selectBuildInputFrames(std::vector<BuildInputFrame> & frames, const BuildInputOptions & options);

/**
 * @brief 索引遍历 preparation、按用途筛选、选择与分批，原子发布到新目录。
 * @note 逐对完整回读但只保存轻量记录；不重新配对，不生成正式 Observation。
 *       逐对损坏记录在清单中，source_changed 或全局失败禁止发布。
 *       output 不得存在或与 preparation 重叠；失败工作目录保留供检查。
 */
BuildInputResult buildInputManifest(
  const std::filesystem::path & preparation, const std::filesystem::path & output,
  const BuildInputOptions & options);

}  // namespace vln_memory
