#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

#include <vln_memory/bag/depth/depth_projection.hpp>

namespace vln_memory
{

/** @brief 深度原图的整数像素坐标，u 为列，v 为行。 */
struct DepthPixel
{
  std::int64_t u{0};
  std::int64_t v{0};
};

/** @brief 深度原图上的半开框，包含 min，排除 end；越界请求返回错误。 */
struct DepthBoundingBox
{
  std::int64_t u_min{0};
  std::int64_t v_min{0};
  std::int64_t u_end{0};
  std::int64_t v_end{0};
};

/** @brief 深度原图上的简单多边形，3–64 个顶点，边界像素计入，隐式闭合。 */
struct DepthPolygon
{
  std::vector<DepthPixel> vertices;
};

using DepthRegion = std::variant<DepthBoundingBox, DepthPolygon>;

struct DepthRegionOptions
{
  // 原图中四邻接像素允许的最大 optical-z 差，单位米。
  double max_neighbor_depth_delta_m{0.05};
  std::size_t min_cluster_points{4};
  double min_valid_fraction{0.20};
  // 首簇占有效投影点的最低比例，取值 (0.5, 1]。
  double min_dominant_fraction{0.60};
  std::size_t max_region_pixels{1024U * 1024U};
  bool include_points{false};
};

struct DepthRegionPoint
{
  DepthPixel pixel;
  float optical_z_m{0};
  std::array<double, 3> point_map_m{};
};

/** @brief 达到最小点数的连通簇；范围及代表点均来自实际投影点。 */
struct DepthRegionCluster
{
  std::size_t point_count{0};
  float min_optical_z_m{0};
  float max_optical_z_m{0};
  std::array<double, 3> min_map_m{};
  std::array<double, 3> max_map_m{};
  DepthRegionPoint representative;
  std::vector<DepthRegionPoint> points;
};

enum class DepthRegionCode
{
  ready, invalid_options, invalid_region, region_too_large, invalid_input,
  no_valid_depth, projection_failed, insufficient_support, ambiguous_region
};

struct DepthRegionResult
{
  DepthRegionCode code{DepthRegionCode::invalid_input};
  std::optional<DepthProjectionCode> point_code;
  std::size_t region_pixels{0};
  std::size_t invalid_depth_pixels{0};
  std::size_t projection_failures{0};
  std::size_t valid_points{0};
  std::vector<DepthRegionCluster> clusters;
  std::optional<DepthRegionPoint> representative;

  /** @brief 代表点只在首簇满足点数、覆盖率和占比要求时存在。 */
  bool ok() const noexcept {return code == DepthRegionCode::ready && representative.has_value();}
};

/** @brief 返回稳定状态名，用于报告和后续工具层。 */
const char * depthRegionCodeName(DepthRegionCode code) noexcept;

/** @brief 用原图尺寸检查选区、阈值和预算，无需像素、标定或位姿。 */
DepthRegionCode checkDepthRegionSelection(
  std::uint32_t width, std::uint32_t height, const DepthRegion & region, const DepthRegionOptions & options = {});

/** @brief 在深度原图框内复用单点投影，结果单位米并独立持有。 */
DepthRegionResult projectDepthRegionToMap(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose,
  std::string_view map_frame, const DepthBoundingBox & region, const DepthRegionOptions & options = {});

/** @brief 对简单多边形掩膜内的原图像素执行与 bbox 相同的投影。 */
DepthRegionResult projectDepthRegionToMap(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose,
  std::string_view map_frame, const DepthPolygon & region, const DepthRegionOptions & options = {});

/** @brief 用两图的历史位姿将 RGB 选区关联到真实深度点，再在深度图上分簇。 */
DepthRegionResult projectRgbRegionToMap(
  const DepthFrame & depth, const CameraModel & depth_model, const TfPoseResult & depth_pose,
  const CameraModel & rgb_model, const TfPoseResult & rgb_pose, std::string_view map_frame,
  const DepthRegion & rgb_region, const DepthRegionOptions & options = {});

}  // namespace vln_memory
