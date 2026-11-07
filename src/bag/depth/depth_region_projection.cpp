#include <vln_memory/bag/depth/depth_region_projection.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include "depth_projection_internal.hpp"

namespace vln_memory
{
namespace
{
using Code = DepthRegionCode;
using PointCode = DepthProjectionCode;

struct ProjectedCell
{
  std::array<double, 3> point{};
  float depth_m{0};
  bool visited{false};
};

/** @brief 拒绝零、非有限或有歧义的阈值，限制临时网格大小。 */
bool validOptions(const DepthRegionOptions & options)
{
  return std::isfinite(options.max_neighbor_depth_delta_m) && options.max_neighbor_depth_delta_m > 0 &&
    options.min_cluster_points > 0 && std::isfinite(options.min_valid_fraction) &&
    options.min_valid_fraction > 0 && options.min_valid_fraction <= 1 &&
    std::isfinite(options.min_dominant_fraction) && options.min_dominant_fraction > 0.5 &&
    options.min_dominant_fraction <= 1 && options.max_region_pixels > 0 &&
    options.max_region_pixels <= DepthLimits{}.max_pixels;
}

/** @brief 校验半开框与像素预算，以除法避免窗口面积溢出。 */
Code checkBox(std::uint32_t width, std::uint32_t height, const DepthBoundingBox & box,
  const DepthRegionOptions & options)
{
  if (box.u_min < 0 || box.v_min < 0 || box.u_end <= box.u_min || box.v_end <= box.v_min ||
    box.u_end > width || box.v_end > height) {return Code::invalid_region;}
  const auto columns = static_cast<std::size_t>(box.u_end - box.u_min);
  const auto rows = static_cast<std::size_t>(box.v_end - box.v_min);
  return columns > options.max_region_pixels / rows ? Code::region_too_large : Code::ready;
}

/** @brief 将临时网格的行优先索引转换为可回查的原图像素。 */
DepthPixel sourcePixel(const DepthBoundingBox & box, std::size_t width, std::size_t index)
{
  return {box.u_min + static_cast<std::int64_t>(index % width),
    box.v_min + static_cast<std::int64_t>(index / width)};
}

/** @brief 依据已投影的真实像素组装可追溯点。 */
DepthRegionPoint pointAt(
  const std::vector<ProjectedCell> & cells, const DepthBoundingBox & box, std::size_t width, std::size_t index)
{
  return {sourcePixel(box, width, index), cells[index].depth_m, cells[index].point};
}

/** @brief 广度搜索四邻接且深度差在门限内的有效像素，空洞不会连接簇。 */
void collectCluster(
  std::vector<ProjectedCell> & cells, std::size_t start, std::size_t width, std::size_t height,
  double max_delta_m, std::vector<std::size_t> & queue)
{
  queue.clear();
  queue.push_back(start);
  cells[start].visited = true;
  for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
    const auto index = queue[cursor];
    const auto row = index / width;
    const auto column = index % width;
    const auto depth_m = static_cast<double>(cells[index].depth_m);
    /** @brief 只将可连接且尚未处理的相邻像素入队。 */
    const auto visit = [&](std::size_t neighbor) {
      auto & cell = cells[neighbor];
      if (cell.visited || cell.depth_m <= 0 || std::abs(depth_m - cell.depth_m) > max_delta_m) {return;}
      cell.visited = true;
      queue.push_back(neighbor);
    };
    if (column > 0) {visit(index - 1);}
    if (column + 1 < width) {visit(index + 1);}
    if (row > 0) {visit(index - width);}
    if (row + 1 < height) {visit(index + width);}
  }
}

/** @brief 统计一个簇，取距离质心最近的真实点，同距优先原图行序。 */
DepthRegionCluster describeCluster(
  const std::vector<ProjectedCell> & cells, const std::vector<std::size_t> & members,
  const DepthBoundingBox & box, std::size_t width, bool include_points)
{
  DepthRegionCluster out;
  out.point_count = members.size();
  const auto & first = cells[members.front()];
  out.min_optical_z_m = out.max_optical_z_m = first.depth_m;
  out.min_map_m = out.max_map_m = first.point;
  std::array<long double, 3> center{};
  const auto count = static_cast<long double>(members.size());
  if (include_points) {out.points.reserve(members.size());}
  for (const auto index : members) {
    const auto & cell = cells[index];
    out.min_optical_z_m = std::min(out.min_optical_z_m, cell.depth_m);
    out.max_optical_z_m = std::max(out.max_optical_z_m, cell.depth_m);
    for (std::size_t axis = 0; axis < center.size(); ++axis) {
      out.min_map_m[axis] = std::min(out.min_map_m[axis], cell.point[axis]);
      out.max_map_m[axis] = std::max(out.max_map_m[axis], cell.point[axis]);
      center[axis] += static_cast<long double>(cell.point[axis]) / count;
    }
    if (include_points) {out.points.push_back(pointAt(cells, box, width, index));}
  }
  auto best = members.front();
  auto best_distance = std::numeric_limits<long double>::infinity();
  for (const auto index : members) {
    const auto & point = cells[index].point;
    const auto distance = std::hypot(point[0] - center[0], point[1] - center[1], point[2] - center[2]);
    if (distance < best_distance || (distance == best_distance && index < best)) {
      best = index;
      best_distance = distance;
    }
  }
  out.representative = pointAt(cells, box, width, best);
  return out;
}

/** @brief 对掩膜内像素投影并分簇；bbox 和 polygon 共用所有质量判断。 */
template<typename Mask>
DepthRegionResult projectWindow(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose, std::string_view map_frame,
  const DepthBoundingBox & box, const DepthRegionOptions & options, Mask contains)
{
  DepthRegionResult out;
  const auto width = static_cast<std::size_t>(box.u_end - box.u_min);
  const auto height = static_cast<std::size_t>(box.v_end - box.v_min);

  const auto context = detail::checkDepthProjectionContext(depth, model, pose, map_frame);
  if (context != PointCode::ready) {
    out.code = Code::invalid_input;
    out.point_code = context;
    return out;
  }

  std::vector<ProjectedCell> cells(width * height);
  for (std::size_t index = 0; index < cells.size(); ++index) {
    const auto pixel = sourcePixel(box, width, index);
    if (!contains(pixel)) {continue;}
    ++out.region_pixels;
    const auto point = projectDepthPixelToMap(depth, model, pose, map_frame, pixel.u, pixel.v);
    if (point.ok()) {
      const auto offset = static_cast<std::size_t>(pixel.v) * depth.width + static_cast<std::size_t>(pixel.u);
      cells[index].depth_m = depth.depth_m[offset];
      cells[index].point = *point.point_map_m;
      ++out.valid_points;
    } else if (point.code == PointCode::invalid_depth) {
      ++out.invalid_depth_pixels;
    } else if (point.code == PointCode::ray_unavailable || point.code == PointCode::numerical_failure) {
      ++out.projection_failures;
      if (!out.point_code) {out.point_code = point.code;}
    } else {
      DepthRegionResult failed;
      failed.code = Code::invalid_input;
      failed.point_code = point.code;
      return failed;
    }
  }
  if (out.valid_points == 0) {
    out.code = out.projection_failures > 0 ? Code::projection_failed : Code::no_valid_depth;
    return out;
  }
  std::vector<std::size_t> queue;
  for (std::size_t index = 0; index < cells.size(); ++index) {
    if (cells[index].visited || cells[index].depth_m <= 0) {continue;}
    collectCluster(cells, index, width, height, options.max_neighbor_depth_delta_m, queue);
    if (queue.size() >= options.min_cluster_points) {
      out.clusters.push_back(describeCluster(cells, queue, box, width, options.include_points));
    }
  }
  /** @brief 点数相同时保持发现顺序，即各簇首像素的原图行序。 */
  std::stable_sort(out.clusters.begin(), out.clusters.end(), [](const auto & a, const auto & b) {
    return a.point_count > b.point_count;
  });
  if (out.clusters.empty() || static_cast<double>(out.valid_points) / out.region_pixels < options.min_valid_fraction) {
    out.code = Code::insufficient_support;
  } else if (static_cast<double>(out.clusters.front().point_count) / out.valid_points < options.min_dominant_fraction) {
    out.code = out.clusters.size() > 1 ? Code::ambiguous_region : Code::insufficient_support;
  } else {
    out.code = Code::ready;
    out.representative = out.clusters.front().representative;
  }
  return out;
}

/** @brief 获取有限简单多边形的深度原图包围窗口；坐标均是整数像素中心。 */
std::optional<DepthBoundingBox> polygonWindow(
  std::uint32_t width, std::uint32_t height, const DepthPolygon & polygon)
{
  constexpr std::size_t max_vertices = 64;
  if (polygon.vertices.size() < 3 || polygon.vertices.size() > max_vertices) {return std::nullopt;}
  const auto & first = polygon.vertices.front();
  DepthBoundingBox box{first.u, first.v, first.u, first.v};
  for (const auto & pixel : polygon.vertices) {
    if (pixel.u < 0 || pixel.u >= width || pixel.v < 0 || pixel.v >= height) {
      return std::nullopt;
    }
    box.u_min = std::min(box.u_min, pixel.u);
    box.v_min = std::min(box.v_min, pixel.v);
    box.u_end = std::max(box.u_end, pixel.u);
    box.v_end = std::max(box.v_end, pixel.v);
  }
  ++box.u_end;
  ++box.v_end;
  return box;
}

/** @brief 比较两像素，供闭合点及重复顶点检查。 */
bool samePixel(const DepthPixel & a, const DepthPixel & b)
{
  return a.u == b.u && a.v == b.v;
}

/** @brief 返回方向叉积；调用前已限制包围窗口面积，坐标差乘积有界。 */
std::int64_t orient(const DepthPixel & a, const DepthPixel & b, const DepthPixel & p)
{
  return (b.u - a.u) * (p.v - a.v) - (b.v - a.v) * (p.u - a.u);
}

/** @brief 判断已知共线的像素是否位于闭区间线段。 */
bool between(const DepthPixel & a, const DepthPixel & b, const DepthPixel & p)
{
  return p.u >= std::min(a.u, b.u) && p.u <= std::max(a.u, b.u) &&
    p.v >= std::min(a.v, b.v) && p.v <= std::max(a.v, b.v);
}

/** @brief 非相邻边的相交包含触碰和共线重叠。 */
bool intersects(const DepthPixel & a, const DepthPixel & b, const DepthPixel & c, const DepthPixel & d)
{
  const auto c_side = orient(a, b, c);
  const auto d_side = orient(a, b, d);
  const auto a_side = orient(c, d, a);
  const auto b_side = orient(c, d, b);
  if ((c_side == 0 && between(a, b, c)) || (d_side == 0 && between(a, b, d)) ||
    (a_side == 0 && between(c, d, a)) || (b_side == 0 && between(c, d, b))) {return true;}
  return c_side != 0 && d_side != 0 && a_side != 0 && b_side != 0 &&
    (c_side < 0) != (d_side < 0) && (a_side < 0) != (b_side < 0);
}

/** @brief 拒绝重复顶点、退化、回折和自交；允许凹多边形与两种绕序。 */
bool simplePolygon(const std::vector<DepthPixel> & vertices)
{
  const auto count = vertices.size();
  std::int64_t twice_area = 0;
  for (std::size_t i = 0; i < count; ++i) {
    const auto next = (i + 1) % count;
    const auto prev = (i + count - 1) % count;
    const auto & a = vertices[i];
    const auto & b = vertices[next];
    if (orient(vertices[prev], a, b) == 0 && !between(vertices[prev], b, a)) {return false;}
    twice_area += orient(vertices.front(), a, b);
    for (std::size_t j = i + 1; j < count; ++j) {
      if (samePixel(a, vertices[j])) {return false;}
      const auto end = (j + 1) % count;
      if (j != next && end != i && intersects(a, b, vertices[j], vertices[end])) {return false;}
    }
  }
  return twice_area != 0;
}

/** @brief 校验原图多边形并返回其窗口，面积受限后再计算整数叉积。 */
Code checkPolygon(
  std::uint32_t width, std::uint32_t height, const DepthPolygon & region,
  const DepthRegionOptions & options, DepthBoundingBox & window)
{
  const auto box = polygonWindow(width, height, region);
  if (!box) {return Code::invalid_region;}
  const auto code = checkBox(width, height, *box, options);
  if (code != Code::ready) {return code;}
  if (!simplePolygon(region.vertices)) {return Code::invalid_region;}
  window = *box;
  return Code::ready;
}

/** @brief 测试整数像素中心，包含多边形边界，水平射线按半开纵区间计数。 */
bool insidePolygon(const std::vector<DepthPixel> & vertices, const DepthPixel & p)
{
  bool inside = false;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[(i + 1) % vertices.size()];
    const auto side = orient(a, b, p);
    if (side == 0 && between(a, b, p)) {return true;}
    if ((a.v > p.v) != (b.v > p.v) && ((b.v > a.v && side > 0) || (b.v < a.v && side < 0))) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

/** @brief 返回稳定的区域投影状态名。 */
const char * depthRegionCodeName(DepthRegionCode code) noexcept
{
  switch (code) {
    case Code::ready: return "ready";
    case Code::invalid_options: return "invalid_options";
    case Code::invalid_region: return "invalid_region";
    case Code::region_too_large: return "region_too_large";
    case Code::invalid_input: return "invalid_input";
    case Code::no_valid_depth: return "no_valid_depth";
    case Code::projection_failed: return "projection_failed";
    case Code::insufficient_support: return "insufficient_support";
    case Code::ambiguous_region: return "ambiguous_region";
  }
  return "unknown";
}

/** @brief 在缺几何时也能按投影同一规则校验 bbox 和 polygon。 */
DepthRegionCode checkDepthRegionSelection(
  std::uint32_t width, std::uint32_t height, const DepthRegion & region, const DepthRegionOptions & options)
{
  if (!validOptions(options)) {return Code::invalid_options;}
  if (const auto * box = std::get_if<DepthBoundingBox>(&region)) {return checkBox(width, height, *box, options);}
  if (const auto * polygon = std::get_if<DepthPolygon>(&region)) {
    DepthBoundingBox window;
    return checkPolygon(width, height, *polygon, options, window);
  }
  return Code::invalid_region;
}

/** @brief 半开框内所有原图像素均参与投影。 */
DepthRegionResult projectDepthRegionToMap(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose,
  std::string_view map_frame, const DepthBoundingBox & region, const DepthRegionOptions & options)
{
  DepthRegionResult result;
  if (!validOptions(options)) {result.code = Code::invalid_options; return result;}
  result.code = checkBox(depth.width, depth.height, region, options);
  if (result.code != Code::ready) {return result;}
  /** @brief 框窗口已精确裁出请求范围。 */
  const auto all_pixels = [](const DepthPixel &) {return true;};
  return projectWindow(depth, model, pose, map_frame, region, options, all_pixels);
}

/** @brief 在简单多边形包围框中计算像素掩膜，交由同一投影与聚类流程。 */
DepthRegionResult projectDepthRegionToMap(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose,
  std::string_view map_frame, const DepthPolygon & region, const DepthRegionOptions & options)
{
  DepthRegionResult result;
  if (!validOptions(options)) {result.code = Code::invalid_options; return result;}
  DepthBoundingBox window;
  result.code = checkPolygon(depth.width, depth.height, region, options, window);
  if (result.code != Code::ready) {return result;}
  /** @brief 复用已校验的顶点序列计算原图像素中心的掩膜。 */
  const auto mask = [&region](const DepthPixel & pixel) {return insidePolygon(region.vertices, pixel);};
  return projectWindow(depth, model, pose, map_frame, window, options, mask);
}

}  // namespace vln_memory
