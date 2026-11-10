#include <vln_memory/bag/depth/depth_region_projection.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace vln_memory;

struct Scene
{
  DepthFrame depth;
  CameraModel model;
  TfPoseResult pose;
};

/** @brief Release 构建也执行检查。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 生成已知米制深度、针孔模型和非平凡 map 变换的夹具。 */
Scene scene()
{
  Scene s;
  s.depth.stamp = {-1, 200};
  s.depth.frame_id = "depth_optical";
  s.depth.width = 6;
  s.depth.height = 5;
  s.depth.step = 6 * sizeof(float);
  s.depth.depth_m.assign(30, 2.0F);
  s.model.frame_id = s.depth.frame_id;
  s.model.width = s.depth.width;
  s.model.height = s.depth.height;
  s.model.image_domain = CameraImageDomain::rectified;
  s.model.distortion_model = "plumb_bob";
  s.model.d.assign(5, 0);
  s.model.k = {10, 0, 0, 0, 10, 0, 0, 0, 1};
  s.model.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  s.model.p = {10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 1, 0};
  TfIndex index({"clock", "segment"});
  TfTransform tf;
  tf.kind = TfKind::static_transform;
  tf.parent_frame = "map";
  tf.child_frame = s.depth.frame_id;
  tf.translation_m = {10, 20, 30};
  tf.rotation_xyzw = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  tf.source_topic = "/tf_static";
  tf.source_message_id = "fixture_tf";
  expect(index.insert(tf).accepted(), "Cannot insert fixture transform");
  s.pose = index.lookupPose("map", s.depth.frame_id, s.depth.stamp, {"clock", "segment", 0});
  expect(s.pose.ok(), "Cannot recover fixture pose");
  return s;
}

/** @brief 验证非平凡变换下坐标与深度点相符，代表点确属原图。 */
void checkPoint(const DepthRegionPoint & point)
{
  const auto & map = point.point_map_m;
  expect(std::abs(map[0] - (10 - 0.2 * point.pixel.v)) < 1e-9 &&
    std::abs(map[1] - (20 + 0.2 * point.pixel.u)) < 1e-9 &&
    std::abs(map[2] - 32) < 1e-9 && point.optical_z_m == 2.0F, "Incorrect observed map point");
}

/** @brief 核对半开 bbox、无效深度计数及输入错误。 */
void bboxCases()
{
  auto s = scene();
  s.depth.depth_m[0] = std::numeric_limits<float>::quiet_NaN();
  s.depth.depth_m[3] = 0;
  const DepthBoundingBox box{0, 0, 4, 3};
  DepthRegionOptions options;
  options.include_points = true;
  const auto result = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", box, options);
  expect(result.ok() && result.region_pixels == 12 && result.valid_points == 10 &&
    result.invalid_depth_pixels == 2 && result.projection_failures == 0 &&
    result.clusters.size() == 1 && result.clusters.front().points.size() == 10, "Incorrect bbox statistics");
  checkPoint(*result.representative);
  for (const auto & point : result.clusters.front().points) {
    expect(point.pixel.u < 4 && point.pixel.v < 3, "Half-open bbox included an end pixel");
    checkPoint(point);
  }
  auto wrong_pose = s.pose;
  ++wrong_pose.time.stamp.nanosec;
  const auto wrong = projectDepthRegionToMap(s.depth, s.model, wrong_pose, "map", box);
  expect(wrong.code == DepthRegionCode::invalid_input && wrong.point_code == DepthProjectionCode::pose_mismatch &&
    !wrong.representative, "Wrong historical time was accepted");
  const auto outside = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", DepthBoundingBox{-1, 0, 4, 3});
  expect(outside.code == DepthRegionCode::invalid_region, "Out-of-bounds bbox was accepted");
  options.max_region_pixels = 11;
  const auto limited = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", box, options);
  expect(limited.code == DepthRegionCode::region_too_large, "Pixel budget was ignored");
}

/** @brief 验证深度边界、歧义、孔洞和稀疏支持不会生成错误代表点。 */
void clusterCases()
{
  auto s = scene();
  const DepthBoundingBox box{0, 0, 6, 5};
  for (std::size_t row = 0; row < 5; ++row) {
    for (std::size_t column = 3; column < 6; ++column) {s.depth.depth_m[row * 6 + column] = 4.0F;}
  }
  const auto ambiguous = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", box);
  expect(ambiguous.code == DepthRegionCode::ambiguous_region && ambiguous.clusters.size() == 2 &&
    ambiguous.clusters[0].point_count == 15 && ambiguous.clusters[1].point_count == 15 &&
    ambiguous.clusters[0].max_optical_z_m == 2 && ambiguous.clusters[1].min_optical_z_m == 4 &&
    !ambiguous.representative, "Distinct equal-size surfaces were merged or selected");
  std::fill(s.depth.depth_m.begin(), s.depth.depth_m.end(), 0);
  const auto empty = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", box);
  expect(empty.code == DepthRegionCode::no_valid_depth && empty.invalid_depth_pixels == 30 &&
    !empty.representative, "Invalid pixels yielded a representative");
  const auto missing_pose = projectDepthRegionToMap(s.depth, s.model, {}, "map", box);
  expect(missing_pose.code == DepthRegionCode::invalid_input &&
    missing_pose.point_code == DepthProjectionCode::pose_unavailable,
    "An empty mask must not hide the missing historical pose");
  std::fill_n(s.depth.depth_m.begin(), 4, 2.0F);
  const auto sparse = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", box);
  expect(sparse.code == DepthRegionCode::insufficient_support && sparse.valid_points == 4 &&
    !sparse.representative, "Sparse depth was accepted");
}

/** @brief 凹形掩膜只包含正确的像素中心，两种绕序相同，自交和退化应失败。 */
void polygonCases()
{
  auto s = scene();
  s.depth.depth_m[4 * 6 + 4] = 10;
  DepthPolygon polygon{{{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 4}, {0, 4}}};
  auto result = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", polygon);
  expect(result.ok() && result.region_pixels == 16 && result.valid_points == 16 &&
    result.clusters.size() == 1 && result.clusters[0].max_optical_z_m == 2, "Concave polygon masked wrong pixels");
  std::reverse(polygon.vertices.begin(), polygon.vertices.end());
  auto reversed = projectDepthRegionToMap(s.depth, s.model, s.pose, "map", polygon);
  expect(reversed.ok() && reversed.region_pixels == result.region_pixels &&
    reversed.representative->point_map_m == result.representative->point_map_m, "Polygon winding changed result");
  for (const auto & bad : {DepthPolygon{{{0, 0}, {4, 4}, {0, 4}, {4, 0}}},
    DepthPolygon{{{0, 0}, {1, 1}, {2, 2}}}, DepthPolygon{{{0, 0}, {4, 0}, {5, 5}}},
    DepthPolygon{{{0, 0}, {4, 0}, {4, 4}, {0, 0}}}}) {
    expect(projectDepthRegionToMap(s.depth, s.model, s.pose, "map", bad).code == DepthRegionCode::invalid_region,
      "Invalid polygon was accepted");
  }
}
/** @brief RGB 位姿横移一个像素，仍应返回原始深度点的地图坐标。 */
void rgbRegionCase()
{
  const auto s = scene();
  auto rgb_pose = s.pose;
  rgb_pose.pose->translation_m[1] += 0.2;
  DepthRegionOptions options;
  options.include_points = true;
  const auto result = projectRgbRegionToMap(s.depth, s.model, s.pose, s.model, rgb_pose, "map",
    DepthBoundingBox{1, 1, 4, 3}, options);
  expect(result.ok() && result.valid_points == 6 && result.region_pixels == 6,
    "RGB selector was not matched to six original depth pixels");
  for (const auto & point : result.clusters.front().points) {
    expect(point.pixel.u >= 2 && point.pixel.u < 5 && point.pixel.v >= 1 && point.pixel.v < 3,
      "RGB selector reused same-coordinate depth pixels");
    checkPoint(point);
  }
}
}  // namespace

/** @brief 运行无 ROS 的已知坐标和分簇回归。 */
int main()
{
  try {
    bboxCases();
    clusterCases();
    polygonCases();
    rgbRegionCase();
    std::cout << "Depth region projection checks passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
