#include "entity_store_evidence.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace vln_memory::detail
{
namespace
{
using Json = nlohmann::json;
using Projection = DepthRegionCode;

/** @brief 拒绝整数回绕、浮点数和小数坐标。 */
std::int64_t integer(const Json & field)
{
  if (!field.is_number_integer() ||
    (field.is_number_unsigned() && field.get<std::uint64_t>() > std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument("Invalid evidence integer");
  }
  return field.get<std::int64_t>();
}

/** @brief 读取非负像素计数，保留 size_t 的完整范围。 */
std::size_t count(const Json & field)
{
  if (!field.is_number_unsigned() || field.get<std::uint64_t>() > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument("Invalid evidence count");
  }
  return field.get<std::size_t>();
}

/** @brief 检查固定长度数组，拒绝多余坐标。 */
void checkArray(const Json & value, std::size_t size)
{
  if (!value.is_array() || value.size() != size) {
    throw std::invalid_argument("Invalid evidence coordinates");
  }
}

/** @brief 编码深度原图的半开框或多边形顶点。 */
Json encodeSelection(const DepthRegion & selection)
{
  if (const auto * box = std::get_if<DepthBoundingBox>(&selection)) {
    return {{"bbox", {box->u_min, box->v_min, box->u_end, box->v_end}}};
  }
  auto vertices = Json::array();
  for (const auto & p : std::get<DepthPolygon>(selection).vertices) {vertices.push_back({p.u, p.v});}
  return {{"polygon", std::move(vertices)}};
}

/** @brief 从唯一形状字段恢复原图选区。 */
DepthRegion decodeSelection(const Json & selection)
{
  if (selection.contains("bbox") && selection.size() == 1) {
    const auto & b = selection.at("bbox");
    checkArray(b, 4);
    return DepthBoundingBox{integer(b.at(0)), integer(b.at(1)), integer(b.at(2)), integer(b.at(3))};
  }
  const auto & vertices = selection.at("polygon");
  if (selection.size() != 1 || !vertices.is_array() || vertices.size() < 3 || vertices.size() > 64) {
    throw std::invalid_argument("Invalid polygon evidence");
  }
  DepthPolygon polygon;
  for (const auto & p : vertices) {
    checkArray(p, 2);
    polygon.vertices.push_back({integer(p.at(0)), integer(p.at(1))});
  }
  return polygon;
}

/** @brief 只接受创建器可能保存的投影结果码。 */
Projection decodeProjection(const Json & value)
{
  for (const auto code : {Projection::ready, Projection::no_valid_depth,
      Projection::insufficient_support, Projection::ambiguous_region}) {
    if (value == depthRegionCodeName(code)) {return code;}
  }
  throw std::invalid_argument("Invalid projection status");
}

/** @brief 保存投影参数、质量计数和可空实测点。 */
Json encodeObject(const ObjectSelectionEvidence & object)
{
  const auto & o = object.options;
  Json point = nullptr;
  if (object.representative) {
    const auto & p = *object.representative;
    point = {{"u", p.pixel.u}, {"v", p.pixel.v}, {"optical_z_m", p.optical_z_m}, {"point_map_m", p.point_map_m}};
  }
  Json projection = nullptr;
  if (object.projection_code) {projection = depthRegionCodeName(*object.projection_code);}
  return {{"selection", encodeSelection(object.selection)}, {"projection_code", projection},
    {"max_neighbor_depth_delta_m", o.max_neighbor_depth_delta_m}, {"min_cluster_points", o.min_cluster_points},
    {"min_valid_fraction", o.min_valid_fraction}, {"min_dominant_fraction", o.min_dominant_fraction},
    {"max_region_pixels", o.max_region_pixels}, {"region_pixels", object.region_pixels},
    {"valid_points", object.valid_points}, {"leading_cluster_points", object.leading_cluster_points},
    {"representative", point}};
}

/** @brief 恢复物品证据，点集从不属于实体记录。 */
ObjectSelectionEvidence decodeObject(const Json & value)
{
  ObjectSelectionEvidence object;
  object.selection = decodeSelection(value.at("selection"));
  object.options = {value.at("max_neighbor_depth_delta_m").get<double>(), count(value.at("min_cluster_points")),
    value.at("min_valid_fraction").get<double>(), value.at("min_dominant_fraction").get<double>(),
    count(value.at("max_region_pixels")), false};
  const auto & projection = value.at("projection_code");
  if (!projection.is_null()) {object.projection_code = decodeProjection(projection);}
  object.region_pixels = count(value.at("region_pixels"));
  object.valid_points = count(value.at("valid_points"));
  object.leading_cluster_points = count(value.at("leading_cluster_points"));
  const auto & p = value.at("representative");
  if (!p.is_null()) {
    checkArray(p.at("point_map_m"), 3);
    object.representative = DepthRegionPoint{{integer(p.at("u")), integer(p.at("v"))},
      p.at("optical_z_m").get<float>(), p.at("point_map_m").get<std::array<double, 3>>()};
  }
  return object;
}

/** @brief 保留内部确认及可空底盘锚点，时刻仍为原始整数。 */
Json encodeRegion(const RegionInteriorEvidence & region)
{
  Json anchor = nullptr;
  if (region.body_anchor) {
    const auto & a = *region.body_anchor;
    anchor = {{"body_frame", a.body_frame}, {"sec", a.stamp.sec},
      {"nanosec", a.stamp.nanosec}, {"point_map_m", a.point_map_m}};
  }
  return {{"confirmation_ref", region.confirmation_ref}, {"body_anchor", anchor}};
}

/** @brief 恢复空间证据并检查原始时刻表示的范围。 */
RegionInteriorEvidence decodeRegion(const Json & value)
{
  RegionInteriorEvidence region{value.at("confirmation_ref").get<std::string>(), std::nullopt};
  const auto & anchor = value.at("body_anchor");
  if (!anchor.is_null()) {
    const auto sec = integer(anchor.at("sec"));
    const auto ns = count(anchor.at("nanosec"));
    if (sec < std::numeric_limits<std::int32_t>::min() || sec > std::numeric_limits<std::int32_t>::max() ||
      ns >= 1000000000U) {
      throw std::invalid_argument("Invalid body anchor time");
    }
    checkArray(anchor.at("point_map_m"), 3);
    region.body_anchor = RegionBodyAnchor{anchor.at("body_frame").get<std::string>(),
      {static_cast<std::int32_t>(sec), static_cast<std::uint32_t>(ns)},
      anchor.at("point_map_m").get<std::array<double, 3>>()};
  }
  return region;
}

/** @brief 点坐标必须可保存为 JSON 数值。 */
bool finitePoint(const std::array<double, 3> & point)
{
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}

/** @brief 验证选区和定位状态，地图点只能来自有效投影码。 */
bool validObject(
  const ObjectSelectionEvidence & object, bool located, std::uint32_t width, std::uint32_t height)
{
  if (object.options.include_points ||
    checkDepthRegionSelection(width, height, object.selection, object.options) != Projection::ready ||
    object.leading_cluster_points > object.valid_points || object.valid_points > object.region_pixels ||
    object.region_pixels > object.options.max_region_pixels ||
    located != object.representative.has_value()) {return false;}
  if (!object.projection_code) {return !located && object.region_pixels == 0;}
  switch (*object.projection_code) {
    case Projection::no_valid_depth: return !located && object.region_pixels > 0 && object.valid_points == 0;
    case Projection::insufficient_support:
    case Projection::ambiguous_region: return !located && object.region_pixels > 0;
    case Projection::ready: break;
    default: return false;
  }
  if (!located || object.leading_cluster_points < object.options.min_cluster_points) {return false;}
  const auto & p = *object.representative;
  return p.pixel.u >= 0 && p.pixel.v >= 0 && p.pixel.u < width && p.pixel.v < height &&
    std::isfinite(p.optical_z_m) && p.optical_z_m > 0 && finitePoint(p.point_map_m);
}

}  // namespace

/** @brief 编码前由工作库核对实体类型和证据，不写入点云或图像。 */
std::string encodeEntityEvidence(const EntityObservationRef & ref)
{
  if (ref.object_selection) {return encodeObject(*ref.object_selection).dump();}
  if (ref.region_interior) {return encodeRegion(*ref.region_interior).dump();}
  return "null";
}

/** @brief 从同一 schema 版本的证据恢复独立数据。 */
void decodeEntityEvidence(std::string_view payload, EntityKind kind, EntityObservationRef & ref)
{
  const auto value = Json::parse(payload);
  if (value.is_null()) {return;}
  if (kind == EntityKind::object) {ref.object_selection = decodeObject(value);}
  else {ref.region_interior = decodeRegion(value);}
}

/** @brief 校验两种实体的证据互斥，待定位记录不带推测坐标。 */
bool validEntityEvidence(const Entity & entity, std::uint32_t width, std::uint32_t height)
{
  const auto & ref = entity.observations.front();
  const bool located = entity.status == EntityStatus::located;
  if (entity.kind == EntityKind::object) {
    if (ref.region_interior) {return false;}
    if (!ref.object_selection) {return !located;}
    return validObject(*ref.object_selection, located, width, height);
  }
  if (ref.object_selection) {return false;}
  if (!ref.region_interior) {return !located;}
  const auto & region = *ref.region_interior;
  if (region.confirmation_ref.empty() || located != region.body_anchor.has_value()) {return false;}
  if (!region.body_anchor) {return true;}
  const auto & a = *region.body_anchor;
  return !a.body_frame.empty() && a.stamp.nanosec < 1000000000U && finitePoint(a.point_map_m);
}

}  // namespace vln_memory::detail
