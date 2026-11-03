#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/tf/tf.hpp>

namespace vln_memory
{

enum class DepthProjectionCode
{
  ready, invalid_depth_frame, pixel_out_of_bounds, invalid_depth, model_mismatch,
  ray_unavailable, pose_unavailable, pose_mismatch, numerical_failure
};

struct DepthProjectionResult
{
  DepthProjectionCode code{DepthProjectionCode::invalid_depth_frame};
  std::optional<std::array<double, 3>> point_map_m;
  /** @brief 仅当成功返回独立持有的米制地图坐标时为 true。 */
  bool ok() const noexcept
  {
    return code == DepthProjectionCode::ready && point_map_m.has_value();
  }
};

/**
 * @brief 将深度原图整数列 u、行 v 的单点投到指定地图坐标系，输出单位米。
 * @note 输入只在调用期间借用。调用方提供该帧的有效模型和同钟同段的历史位姿，
 *       并确认深度为 depth.frame_id 原光学轴的 optical-z。失败时不返回坐标。
 */
DepthProjectionResult projectDepthPixelToMap(
  const DepthFrame & depth, const CameraModel & camera_model, const TfPoseResult & camera_pose,
  std::string_view map_frame, std::int64_t u, std::int64_t v);

}  // namespace vln_memory
