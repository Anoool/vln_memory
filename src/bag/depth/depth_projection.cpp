#include <vln_memory/bag/depth/depth_projection.hpp>

#include <cmath>

namespace vln_memory
{
namespace
{

/** @brief 校验米制光轴深度、采样时间和紧密布局，避免宽高相乘溢出。 */
bool validDepthFrame(const DepthFrame & depth)
{
  return depth.width > 0 && depth.height > 0 && !depth.frame_id.empty() &&
    depth.stamp.nanosec < 1000000000U && depth.unit == DepthUnit::meters &&
    depth.convention == DepthConvention::optical_z &&
    depth.step == static_cast<std::uint64_t>(depth.width) * sizeof(float) &&
    depth.depth_m.size() / depth.width == depth.height &&
    depth.depth_m.size() % depth.width == 0;
}

/** @brief 核对位姿的源帧、目标帧和原始查询时刻，不用近似时间代替。 */
bool matchesDepthPose(
  const DepthFrame & depth, const TfTimeCheck & time, std::string_view map_frame)
{
  return !map_frame.empty() && !time.frames.empty() &&
    time.frames.front() == depth.frame_id && time.frames.back() == map_frame &&
    time.stamp.sec == depth.stamp.sec && time.stamp.nanosec == depth.stamp.nanosec;
}

}  // namespace

/** @brief 只读取选中深度值，使用该帧相机射线和历史位姿生成一个地图点。 */
DepthProjectionResult projectDepthPixelToMap(
  const DepthFrame & depth, const CameraModel & camera_model, const TfPoseResult & camera_pose,
  std::string_view map_frame, std::int64_t u, std::int64_t v)
{
  using Code = DepthProjectionCode;
  if (!validDepthFrame(depth)) {return {Code::invalid_depth_frame, std::nullopt};}
  if (u < 0 || u >= depth.width || v < 0 || v >= depth.height) {
    return {Code::pixel_out_of_bounds, std::nullopt};
  }

  const auto offset = static_cast<std::size_t>(v) * depth.width + static_cast<std::size_t>(u);
  const double optical_z_m = depth.depth_m[offset];
  if (!std::isfinite(optical_z_m) || optical_z_m <= 0.0) {
    return {Code::invalid_depth, std::nullopt};
  }
  if (camera_model.frame_id != depth.frame_id || camera_model.width != depth.width ||
    camera_model.height != depth.height)
  {
    return {Code::model_mismatch, std::nullopt};
  }
  if (camera_pose.code != TfPoseCode::ready || !camera_pose.ok() || !camera_pose.time.covered()) {
    return {Code::pose_unavailable, std::nullopt};
  }
  if (!matchesDepthPose(depth, camera_pose.time, map_frame)) {
    return {Code::pose_mismatch, std::nullopt};
  }

  const auto ray = pixelToCameraRay(camera_model, static_cast<double>(u), static_cast<double>(v));
  if (!ray.ok()) {return {Code::ray_unavailable, std::nullopt};}
  auto point_camera_m = *ray.ray;
  for (auto & component : point_camera_m) {component *= optical_z_m;}
  const auto point_map_m = transformTfPoint(*camera_pose.pose, point_camera_m);
  if (!point_map_m) {return {Code::numerical_failure, std::nullopt};}
  return {Code::ready, point_map_m};
}

}  // namespace vln_memory
