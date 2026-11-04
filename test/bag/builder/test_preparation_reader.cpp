#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/bag/depth/depth_projection.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace
{
namespace fs = std::filesystem;
using Json = nlohmann::json;
using namespace vln_memory;

/** @brief 在 Release 构建中也报告可读的验收失败。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 加载小型夹具 JSON 或 JSONL 记录。 */
Json readJson(const fs::path & path)
{
  std::ifstream file(path);
  Json value;
  file >> value;
  return value;
}

/** @brief 将已知的小型 frames.jsonl 夹具转成可局部变异的记录数组。 */
std::vector<Json> readLines(const fs::path & path)
{
  std::ifstream file(path);
  std::vector<Json> rows;
  for (std::string line; std::getline(file, line);) {rows.push_back(Json::parse(line));}
  expect(file.eof(), "Cannot read fixture records");
  return rows;
}

/** @brief 仅写入独立夹具副本，原始 BagBuilder 产物由其他测试复用。 */
void writeLines(const fs::path & path, const std::vector<Json> & rows)
{
  std::ofstream file(path);
  for (const auto & row : rows) {file << row.dump() << '\n';}
  file.close();
  expect(static_cast<bool>(file), "Cannot update fixture records");
}

/** @brief 调用公开入口并要求拿到该 ID 关联的两帧与其各自位姿。 */
PreparedPair readPair(const fs::path & root, const std::string & id)
{
  auto result = loadPreparedPair(root, id);
  expect(result.ok(), result.message);
  auto pair = std::move(*result.pair);
  expect(pair.pair_id == id && pair.depth && pair.rgb_media && pair.depth_media &&
    pair.depth_model && pair.rgb_model && pair.rgb_pose && pair.depth_pose,
    "Complete fixture lacks required geometry inputs");
  expect(pair.rgb_pose->time.stamp.sec == pair.rgb_image.stamp.sec &&
    pair.rgb_pose->time.stamp.nanosec == pair.rgb_image.stamp.nanosec &&
    pair.depth_pose->time.stamp.sec == pair.depth_image.stamp.sec &&
    pair.depth_pose->time.stamp.nanosec == pair.depth_image.stamp.nanosec,
    "Image and pose stamps differ");
  expect(fs::is_regular_file(pair.rgb_raw_evidence) && fs::is_regular_file(pair.depth_raw_evidence),
    "Missing raw evidence paths");
  return pair;
}

/** @brief 核对两种媒体还原后的 float32 米制深度和无效掩码。 */
void compareMedia(const PreparedPair & npy, const PreparedPair & png)
{
  /** @brief 相同有效值或同一位置为无效 NaN。 */
  const auto equal = [](float a, float b) {return a == b || (std::isnan(a) && std::isnan(b));};
  expect(npy.bag_id == png.bag_id && npy.rgb_message_id == png.rgb_message_id &&
    npy.depth_message_id == png.depth_message_id && npy.delta_ns == png.delta_ns &&
    npy.depth->width == png.depth->width && npy.depth->height == png.depth->height &&
    std::equal(npy.depth->depth_m.begin(), npy.depth->depth_m.end(),
      png.depth->depth_m.begin(), png.depth->depth_m.end(), equal), "PNG and NPY media disagree");
}

/** @brief 验证结构化失败码且确认不会返回部分配对。 */
void expectFailure(const fs::path & root, const std::string & id, PreparationReadCode code)
{
  const auto result = loadPreparedPair(root, id);
  expect(!result.ok() && !result.pair && result.code == code, "Unexpected result: " + result.message);
}

/** @brief 从 ROS 构造 bag 导出物验证历史格式、坏引用、错时刻和坏媒体。 */
void fixtureCases(const fs::path & parent)
{
  std::ifstream latest(parent / "latest.txt");
  std::string path;
  expect(static_cast<bool>(std::getline(latest, path)), "Cannot locate BagBuilder fixture");
  const fs::path root(path);
  const auto normal = root / "normal_out";
  const auto id = readJson(normal / "rgbd_pairs.jsonl").at("pair_id").get<std::string>();
  const auto npy = readPair(normal, id);
  const auto png = readPair(root / "normal_png_out", id);
  compareMedia(npy, png);
  expect(npy.depth->depth_m.at(0) == 1.0F && std::isnan(npy.depth->depth_m.at(1)) &&
    npy.depth->alignment == DepthAlignment::aligned_to_rgb, "Fixture depth semantics differ");
  expectFailure(normal, "absent_pair", PreparationReadCode::pair_not_found);

  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto copy = root / ("reader_check_" + std::to_string(tick));
  expect(fs::create_directory(copy), "Expected a fresh fixture copy");
  fs::copy(normal, copy, fs::copy_options::recursive);
  auto manifest = readJson(copy / "bag_manifest.json");
  manifest["builder_version"] = 5;
  manifest["effective_options"].erase("depth_storage");
  {std::ofstream file(copy / "bag_manifest.json"); file << manifest.dump();}
  const auto frames_path = copy / "frames.jsonl";
  auto original = readLines(frames_path);
  for (auto & frame : original) {
    if (frame.at("kind") == "depth") {frame.at("details").erase("storage");}
  }
  writeLines(frames_path, original);
  compareMedia(readPair(copy, id), png);

  auto rows = original;
  /** @brief 按已配对的 Depth ID 定位要破坏的夹具记录。 */
  const auto is_depth = [&npy](const Json & row) {
      return row.at("source").at("message_id") == npy.depth_message_id;
    };
  auto selected = std::find_if(rows.begin(), rows.end(), is_depth);
  expect(selected != rows.end(), "Missing fixture frame");
  selected->at("map_camera_tf").at("stamp").at("sec") = -123;
  writeLines(frames_path, rows);
  expectFailure(copy, id, PreparationReadCode::invalid_reference);
  rows = original;
  rows.erase(std::find_if(rows.begin(), rows.end(), is_depth));
  writeLines(frames_path, rows);
  expectFailure(copy, id, PreparationReadCode::invalid_reference);
  rows = original;
  auto unavailable = std::find_if(rows.begin(), rows.end(), is_depth);
  auto & pose = unavailable->at("map_camera_tf");
  pose["pose"] = nullptr;
  pose["pose_code"] = "time_unavailable";
  pose["edges"] = Json::array();
  unavailable->at("pose_computed") = false;
  const auto pairs_path = copy / "rgbd_pairs.jsonl";
  auto pairs = readLines(pairs_path);
  const auto saved_pairs = pairs;
  pairs.front()["status"] = "pending";
  pairs.front()["pose_computed"] = false;
  pairs.front()["reasons"] = Json::array({"depth_tf_unavailable"});
  writeLines(frames_path, rows);
  writeLines(pairs_path, pairs);
  const auto pending = loadPreparedPair(copy, id);
  expect(pending.ok() && pending.pair->depth && !pending.pair->depth_pose &&
    pending.pair->pending_reasons == std::vector<std::string>{"depth_tf_unavailable"},
    "Declared missing pose was not retained as pending");
  writeLines(pairs_path, saved_pairs);
  writeLines(frames_path, original);

  const auto media = copy / npy.depth_media->lexically_relative(normal);
  {std::ofstream file(media, std::ios::binary); file << "broken npy";}
  expectFailure(copy, id, PreparationReadCode::invalid_media);
}

/** @brief 用真实短包的独立验收参考核对项目入口的逐点几何。 */
void realCase(const fs::path & npy_path, const fs::path & png_path, const fs::path & report_path)
{
  const auto report = readJson(report_path);
  const auto id = report.at("pair").at("pair_id").get<std::string>();
  const auto npy = readPair(npy_path, id);
  const auto png = readPair(png_path, id);
  compareMedia(npy, png);
  expect(npy.status == "pending" && png.pending_reasons == npy.pending_reasons &&
    png.pending_reasons == std::vector<std::string>{"depth_rgb_alignment_unverified"} &&
    png.depth->alignment == DepthAlignment::unknown, "Real-pair pending status was lost");
  for (const auto & sample : report.at("pixels")) {
    const auto u = sample.at("uv").at(0).get<int>();
    const auto v = sample.at("uv").at(1).get<int>();
    for (const auto * pair : {&npy, &png}) {
      const auto m = projectDepthPixelToMap(*pair->depth, *pair->depth_model, *pair->depth_pose,
        pair->map_frame, u, v);
      const auto p = projectDepthPixelToRgb(*pair->depth, *pair->depth_model, *pair->depth_pose,
        pair->rgb_image, *pair->rgb_model, *pair->rgb_pose, pair->map_frame, u, v);
      if (sample.at("xyz").is_null()) {
        expect(m.code == DepthProjectionCode::invalid_depth && !m.point_map_m && !p.pixel_rgb,
          "Invalid real depth was projected");
        continue;
      }
      expect(m.ok() && p.ok(), "Valid real depth could not be projected");
      const auto xyz = sample.at("reference_xyz_m").get<std::array<double, 3>>();
      const auto uv = sample.at("reference_rgb_uv").get<std::array<double, 2>>();
      expect(std::hypot((*m.point_map_m)[0] - xyz[0], (*m.point_map_m)[1] - xyz[1],
        (*m.point_map_m)[2] - xyz[2]) <= 1e-5 &&
        std::hypot((*p.pixel_rgb)[0] - uv[0], (*p.pixel_rgb)[1] - uv[1]) <= 0.05,
        "Projection differs from independent reference");
    }
  }
}

}  // namespace

/** @brief 无录包发布依赖的测试入口，支持构造夹具和本地真实验收产物。 */
int main(int argc, char ** argv)
{
  try {
    if (argc == 2) {
      fixtureCases(argv[1]);
    } else if (argc == 4) {
      realCase(argv[1], argv[2], argv[3]);
    } else {
      throw std::runtime_error("Usage: test_preparation_reader fixture_root | npy_root png_root reference.json");
    }
    std::cout << "Preparation reader checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
