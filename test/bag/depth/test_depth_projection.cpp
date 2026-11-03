#include <vln_memory/bag/depth/depth_projection.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace vln_memory;

/** @brief Release 构建同样检查结果，失败附用例说明。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 确认失败码明确且没有可消费的地图点。 */
void expectFailure(
  const DepthProjectionResult & result, DepthProjectionCode code, const char * message)
{
  expect(!result.ok() && !result.point_map_m && result.code == code, message);
}

/** @brief 离轴像素经光轴深度、非零旋转和位移投影，回归错误时刻与坏输入。 */
void projectionCases()
{
  using Code = DepthProjectionCode;
  DepthFrame depth;
  depth.stamp = {7, 123};
  depth.frame_id = "depth_optical";
  depth.width = 4;
  depth.height = 3;
  depth.step = 4 * sizeof(float);
  depth.depth_m.resize(12);
  depth.depth_m.back() = 2.0F;

  CameraModel camera;
  camera.width = depth.width;
  camera.height = depth.height;
  camera.frame_id = depth.frame_id;
  camera.image_domain = CameraImageDomain::raw;
  camera.distortion_model = "plumb_bob";
  camera.d = {0, 0, 0, 0, 0};
  camera.k = {2, 0, 1, 0, 2, 1, 0, 0, 1};
  camera.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  camera.p = {2, 0, 1, 0, 0, 2, 1, 0, 0, 0, 1, 0};

  TfIndex index({"clock", "segment"});
  TfTransform transform;
  transform.kind = TfKind::static_transform;
  transform.parent_frame = "map";
  transform.child_frame = depth.frame_id;
  transform.translation_m = {10, 20, 30};
  transform.rotation_xyzw = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  transform.source_topic = "/tf_static";
  transform.source_message_id = "test:tf:1";
  expect(index.insert(transform).accepted(), "Failed to insert fixture TF");
  const TfTimeOptions options{"clock", "segment", 0};
  const auto pose = index.lookupPose("map", depth.frame_id, depth.stamp, options);
  const auto result = projectDepthPixelToMap(depth, camera, pose, "map", 3, 2);
  expect(result.ok(), "Projection failed for valid inputs");
  const auto & point = *result.point_map_m;
  expect(std::abs(point[0] - 9.0) < 1e-9 &&
    std::abs(point[1] - 22.0) < 1e-9 &&
    std::abs(point[2] - 32.0) < 1e-9, "Expected map point (9, 22, 32) m");

  expectFailure(projectDepthPixelToMap(depth, camera, pose, "map", -1, 2),
    Code::pixel_out_of_bounds, "Negative pixel accepted");
  expectFailure(projectDepthPixelToMap(depth, camera, pose, "map", 3, 3),
    Code::pixel_out_of_bounds, "Pixel below image accepted");
  expectFailure(projectDepthPixelToMap(depth, camera, pose, "map", 2, 2),
    Code::invalid_depth, "Zero depth accepted");

  auto bad_depth = depth;
  bad_depth.depth_m.back() = std::numeric_limits<float>::quiet_NaN();
  expectFailure(projectDepthPixelToMap(bad_depth, camera, pose, "map", 3, 2),
    Code::invalid_depth, "NaN depth accepted");
  bad_depth.depth_m.pop_back();
  expectFailure(projectDepthPixelToMap(bad_depth, camera, pose, "map", 3, 2),
    Code::invalid_depth_frame, "Truncated depth buffer accepted");

  auto bad_camera = camera;
  bad_camera.frame_id = "rgb_optical";
  expectFailure(projectDepthPixelToMap(depth, bad_camera, pose, "map", 3, 2),
    Code::model_mismatch, "Different camera frame accepted");
  bad_camera = camera;
  --bad_camera.width;
  expectFailure(projectDepthPixelToMap(depth, bad_camera, pose, "map", 3, 2),
    Code::model_mismatch, "Different pixel grid accepted");
  bad_camera = camera;
  bad_camera.k[0] = 0;
  expectFailure(projectDepthPixelToMap(depth, bad_camera, pose, "map", 3, 2),
    Code::ray_unavailable, "Invalid camera intrinsics accepted");

  const auto wrong_time = index.lookupPose("map", depth.frame_id, {7, 124}, options);
  expect(wrong_time.ok(), "Fixture should have a pose at the wrong timestamp");
  expectFailure(projectDepthPixelToMap(depth, camera, wrong_time, "map", 3, 2),
    Code::pose_mismatch, "Pose queried at a different timestamp accepted");
  expectFailure(projectDepthPixelToMap(depth, camera, pose, "odom", 3, 2),
    Code::pose_mismatch, "Different target frame accepted");
  const auto reversed = index.lookupPose(depth.frame_id, "map", depth.stamp, options);
  expect(reversed.ok(), "Fixture should have a reverse pose");
  expectFailure(projectDepthPixelToMap(depth, camera, reversed, "map", 3, 2),
    Code::pose_mismatch, "Reverse transform accepted");
  expectFailure(projectDepthPixelToMap(depth, camera, {}, "map", 3, 2),
    Code::pose_unavailable, "Missing pose accepted");

  auto bad_pose = pose;
  bad_pose.pose->rotation_xyzw = {0, 0, 0, 0};
  expectFailure(projectDepthPixelToMap(depth, camera, bad_pose, "map", 3, 2),
    Code::numerical_failure, "Invalid rotation accepted");
}

}  // namespace

/** @brief 同一用例用于 CTest 和安装后的独立消费者验证。 */
int main()
{
  try {
    projectionCases();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "Depth pixel to map: (9, 22, 32) m and invalid inputs passed\n";
  return 0;
}
