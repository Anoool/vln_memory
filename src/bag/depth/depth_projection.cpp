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

/** @brief 判断位姿查询和求解是否均成功，且包含可用位姿。 */
bool availablePose(const TfPoseResult & pose)
{
  return pose.code == TfPoseCode::ready && pose.ok() && pose.time.covered();
}

/** @brief 核对位姿的源帧、目标帧和原始查询时刻。 */
bool matchesImagePose(
  std::string_view frame, const SensorStamp & stamp, const TfTimeCheck & time,
  std::string_view map_frame)
{
  return !frame.empty() && !map_frame.empty() && !time.frames.empty() &&
    stamp.nanosec < 1000000000U && time.frames.front() == frame &&
    time.frames.back() == map_frame && time.stamp.sec == stamp.sec &&
    time.stamp.nanosec == stamp.nanosec;
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
  if (!availablePose(camera_pose)) {
    return {Code::pose_unavailable, std::nullopt};
  }
  if (!matchesImagePose(depth.frame_id, depth.stamp, camera_pose.time, map_frame)) {
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

/** @brief 将深度地图点转入 RGB 光学帧并返回目标图上的候选像素。 */
DepthToRgbResult projectDepthPixelToRgb(
  const DepthFrame & depth, const CameraModel & depth_model, const TfPoseResult & depth_pose,
  const CameraImageReference & rgb, const CameraModel & rgb_model, const TfPoseResult & rgb_pose,
  std::string_view map_frame, std::int64_t u, std::int64_t v)
{
  using Code = DepthToRgbCode;
  const auto point_map = projectDepthPixelToMap(depth, depth_model, depth_pose, map_frame, u, v);
  if (!point_map.ok()) {return {Code::depth_projection_failed, std::nullopt, point_map.code};}
  if (rgb.width == 0 || rgb.height == 0 || rgb.frame_id.empty() ||
    rgb_model.frame_id != rgb.frame_id || rgb_model.width != rgb.width || rgb_model.height != rgb.height)
  {
    return {Code::rgb_model_mismatch, std::nullopt};
  }
  if (!availablePose(rgb_pose)) {return {Code::rgb_pose_unavailable, std::nullopt};}
  if (!matchesImagePose(rgb.frame_id, rgb.stamp, rgb_pose.time, map_frame)) {
    return {Code::rgb_pose_mismatch, std::nullopt};
  }

  const auto point_rgb_m = inverseTransformTfPoint(*rgb_pose.pose, *point_map.point_map_m);
  if (!point_rgb_m) {return {Code::numerical_failure, std::nullopt};}
  const auto pixel = cameraPointToPixel(rgb_model, *point_rgb_m);
  if (!pixel.ok()) {return {Code::rgb_projection_failed, std::nullopt, std::nullopt, pixel.code};}
  return {Code::ready, pixel.pixel_uv};
}

}  // namespace vln_memory
