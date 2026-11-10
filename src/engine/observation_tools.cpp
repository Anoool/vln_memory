#include "observation_tools.hpp"

#include <chrono>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <vln_memory/bag/depth/depth_region_projection.hpp>

namespace vln_memory
{
namespace
{
using Json = nlohmann::json;
namespace fs = std::filesystem;

/** @brief 拒绝小数和过大的整数，保持原图像素坐标精度。 */
std::int64_t coordinate(const Json & value)
{
  if (!value.is_number_integer() ||
    (value.is_number_unsigned() && value.get<std::uint64_t>() > std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("Pixel coordinates must be integers");
  }
  return value.get<std::int64_t>();
}

/** @brief 从两个整数恢复列、行顺序的像素。 */
DepthPixel pixel(const Json & value)
{
  if (!value.is_array() || value.size() != 2) {
    throw std::invalid_argument("Expected [u, v]");
  }
  return {coordinate(value[0]), coordinate(value[1])};
}

/** @brief 统一使用半开框和多边形，单点使用一像素半开框。 */
DepthRegion selection(const Json & selector, std::uint32_t width, std::uint32_t height)
{
  const auto type = selector.at("type").get<std::string>();
  if (type == "point") {
    const auto p = pixel(selector.at("pixel_xy"));
    if (p.u < 0 || p.v < 0 || p.u >= width || p.v >= height) {
      throw std::invalid_argument("Point is outside the image");
    }
    return DepthBoundingBox{p.u, p.v, p.u + 1, p.v + 1};
  }
  if (type == "bbox") {
    const auto & box = selector.at("xyxy_px");
    if (!box.is_array() || box.size() != 4) {throw std::invalid_argument("Expected [x1, y1, x2, y2]");}
    return DepthBoundingBox{coordinate(box[0]), coordinate(box[1]), coordinate(box[2]), coordinate(box[3])};
  }
  if (type == "polygon") {
    const auto & vertices = selector.at("vertices_xy_px");
    if (!vertices.is_array() || vertices.size() < 3 || vertices.size() > 64) {
      throw std::invalid_argument("Expected 3 to 64 polygon vertices");
    }
    DepthPolygon polygon;
    for (const auto & vertex : vertices) {polygon.vertices.push_back(pixel(vertex));}
    return polygon;
  }
  throw std::invalid_argument("Expected point, bbox or polygon");
}

/** @brief 生成定位产物的稳定文件名，调用参数中的文本不作为路径。 */
std::string digest(const std::string & input)
{
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(), hash);
  constexpr char hex[] = "0123456789abcdef";
  std::string value;
  value.reserve(sizeof(hash) * 2);
  for (const auto byte : hash) {
    value += hex[byte >> 4];
    value += hex[byte & 0xf];
  }
  return value;
}

/** @brief 将可信簇写成地图点 JSONL，写完后原子替换同一选区的结果。 */
std::string writePoints(
  const fs::path & root, const Observation & obs, const Json & selector, const DepthRegionCluster & cluster)
{
  const auto key = digest(obs.id + ":" + std::to_string(obs.revision) + ":" + selector.dump());
  const auto relative = fs::path("evidence/projected_points") / (key + ".jsonl");
  const auto destination = root / relative;
  fs::create_directories(destination.parent_path());
  auto temporary = destination;
  temporary += ".tmp." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  try {
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open(temporary);
    for (const auto & point : cluster.points) {
      file << Json{{"position_map", point.point_map_m}, {"depth_pixel_xy", {point.pixel.u, point.pixel.v}},
        {"optical_z_m", point.optical_z_m}}.dump() << '\n';
    }
    file.close();
    fs::rename(temporary, destination);
  } catch (...) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
    throw;
  }
  return relative.generic_string();
}

/** @brief 返回位姿和采样时刻，完整 TF 来源留在原观测。 */
Json poseSummary(const Json & pose)
{
  if (pose.is_null()) {return nullptr;}
  return {{"translation_m", pose.at("translation_m")}, {"rotation_xyzw", pose.at("rotation_xyzw")},
    {"stamp", pose.at("stamp")}};
}

/** @brief 获取现有观测格式中的证据引用和几何字段。 */
Json observationInfo(const Observation & observation)
{
  const auto serialized = serializeObservation(observation);
  if (!serialized.ok()) {throw std::runtime_error(serialized.message);}
  const auto row = Json::parse(serialized.json);
  return {{"status", "ok"}, {"revision", observation.revision},
    {"rgb_media", row.at("rgb_media")}, {"depth_media", row.at("depth_media")},
    {"rgb_image", row.at("rgb_image")}, {"depth_image", row.at("depth_image")},
    {"rgb_stamp", row.at("rgb_stamp")}, {"depth_stamp", row.at("depth_stamp")}, {"delta_ns", row.at("delta_ns")},
    {"camera_pose_map", poseSummary(row.at("camera_pose_map"))},
    {"depth_camera_pose_map", poseSummary(row.at("depth_camera_pose_map"))},
    {"body_pose_map", poseSummary(row.at("body_pose_map"))},
    {"depth_storage", row.at("depth_storage")}, {"depth_alignment", row.at("depth_alignment")},
    {"capture_sync_status", row.at("capture_sync_status")}, {"pending_reasons", observation.pending_reasons}};
}

/** @brief 用原始深度点生成结果，返回分簇摘要和可选证据引用。 */
Json project(
  const ObservationDatasetReader & dataset, const fs::path & root, const Observation & obs, const Json & args)
{
  const auto & selector = args.at("selector");
  const auto image = selector.value("image", std::string("rgb"));
  if (image != "rgb" && image != "depth") {throw std::invalid_argument("Expected rgb or depth");}
  const bool rgb = image == "rgb";
  if (!obs.depth_camera_pose_map || (rgb && !obs.camera_pose_map)) {
    return {{"status", "missing_map_pose"}, {"message", "Historical camera pose is missing"}};
  }
  if (!obs.depth_model || (rgb && !obs.rgb_model)) {
    return {{"status", "missing_camera_model"}, {"message", "Camera model is missing"}};
  }

  const auto & model = rgb ? *obs.rgb_model : *obs.depth_model;
  const auto region = selection(selector, model.width, model.height);
  const bool point = selector.at("type") == "point";
  DepthRegionOptions options;
  if (point) {options.min_cluster_points = 1;}
  options.include_points = args.value("include_points", false);

  const auto loaded = dataset.loadDepth(obs.id);
  if (!loaded.ok()) {return {{"status", "depth_read_failed"}, {"message", loaded.message}};}
  const auto & depth = *loaded.depth;
  DepthRegionResult result;
  if (rgb) {
    result = projectRgbRegionToMap(depth, *obs.depth_model, *obs.depth_camera_pose_map,
      *obs.rgb_model, *obs.camera_pose_map, obs.map_frame, region, options);
  } else {
    /** @brief 依据选择器类型调用同一套深度区域投影。 */
    const auto projectShape = [&](const auto & shape) {
        return projectDepthRegionToMap(depth, *obs.depth_model, *obs.depth_camera_pose_map,
          obs.map_frame, shape, options);
      };
    result = std::visit(projectShape, region);
  }

  const auto * status = depthRegionCodeName(result.code);
  if (result.ok()) {status = "ok";}
  if (result.code == DepthRegionCode::no_valid_depth) {status = "invalid_depth";}
  Json output{{"status", status}, {"selector_image", image},
    {"method", rgb ? "depth_reprojection" : "pixel_depth"},
    {"sample_count", result.valid_points}, {"region_pixels", result.region_pixels},
    {"cluster_count", result.clusters.size()}, {"capture_sync_status", obs.capture_sync_status},
    {"pending_reasons", obs.pending_reasons}};
  if (!result.ok()) {output["message"] = status;}
  auto candidates = Json::array();
  for (const auto & cluster : result.clusters) {
    if (candidates.size() == 4) {break;}
    candidates.push_back({{"sample_count", cluster.point_count}, {"position_map", cluster.representative.point_map_m},
      {"bounds_map", {cluster.min_map_m, cluster.max_map_m}}});
  }
  if (!candidates.empty()) {output["candidates"] = std::move(candidates);}
  if (result.ok()) {
    const auto & selected = *result.representative;
    const auto & cluster = result.clusters.front();
    output["position_map"] = selected.point_map_m;
    output["depth_pixel_xy"] = {selected.pixel.u, selected.pixel.v};
    output["optical_z_m"] = selected.optical_z_m;
    output["bounds_map"] = {cluster.min_map_m, cluster.max_map_m};
    if (options.include_points) {
      output["points_path"] = writePoints(root, obs, selector, cluster);
    }
  }
  return output;
}
}  // namespace

Json runObservationTool(
  const ObservationDatasetReader & dataset, const fs::path & root,
  std::string_view name, const Json & arguments)
{
  Json result;
  try {
    if (name != "get_observation" && name != "project_depth") {
      result = {{"status", "unknown_tool"}, {"message", "Unknown observation tool"}};
    } else {
      const auto id = arguments.at("observation_id").get<std::string>();
      const auto loaded = dataset.loadObservation(id);
      if (!loaded.ok()) {
        const auto * status = loaded.code == ObservationDatasetReadCode::observation_not_found ?
          "observation_not_found" : "observation_read_failed";
        result = {{"status", status}, {"message", loaded.message}};
      } else if (name == "get_observation") {
        result = observationInfo(*loaded.observation);
      } else {
        result = project(dataset, root, *loaded.observation, arguments);
      }
      result["observation_id"] = id;
    }
  } catch (const nlohmann::json::exception & error) {
    result = {{"status", "invalid_arguments"}, {"message", error.what()}};
  } catch (const std::invalid_argument & error) {
    result = {{"status", "invalid_arguments"}, {"message", error.what()}};
  } catch (const std::exception & error) {
    result = {{"status", "tool_error"}, {"message", error.what()}};
  }
  result["frame_id"] = dataset.info().map_frame;
  result["map_id"] = dataset.info().map_id;
  result["geometry_revision"] = dataset.info().geometry_revision;
  result["memory_version"] = nullptr;
  return result;
}
}  // namespace vln_memory
