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


/** @brief 比较可选模型，确保索引选中的标定与旧入口一致。 */
void compareModel(const std::optional<CameraModel> & first, const std::optional<CameraModel> & second)
{
  expect(first.has_value() == second.has_value(), "Indexed model presence differs");
  if (!first) {return;}
  expect(first->frame_id == second->frame_id && first->width == second->width &&
    first->height == second->height && first->image_domain == second->image_domain &&
    first->distortion_model == second->distortion_model && first->k == second->k &&
    first->d == second->d && first->r == second->r && first->p == second->p &&
    first->normalization_applied == second->normalization_applied, "Indexed model differs");
}

/** @brief 比较可选历史位姿的数值、查询时间及链身份。 */
void comparePose(const std::optional<TfPoseResult> & first, const std::optional<TfPoseResult> & second)
{
  expect(first.has_value() == second.has_value(), "Indexed pose presence differs");
  if (!first) {return;}
  expect(first->code == second->code && first->time.stamp.sec == second->time.stamp.sec &&
    first->time.stamp.nanosec == second->time.stamp.nanosec &&
    first->time.frames == second->time.frames && first->pose && second->pose &&
    first->pose->translation_m == second->pose->translation_m &&
    first->pose->rotation_xyzw == second->pose->rotation_xyzw, "Indexed pose differs");
}

/** @brief 对比新旧入口的配对上下文、路径、可选几何及所有深度像素。 */
void comparePair(const PreparedPair & indexed, const PreparedPair & legacy)
{
  expect(indexed.pair_id == legacy.pair_id && indexed.bag_id == legacy.bag_id &&
    indexed.map_frame == legacy.map_frame && indexed.time_scope.clock_id == legacy.time_scope.clock_id &&
    indexed.time_scope.segment_id == legacy.time_scope.segment_id &&
    indexed.clock_basis == legacy.clock_basis && indexed.segment_basis == legacy.segment_basis &&
    indexed.status == legacy.status && indexed.pending_reasons == legacy.pending_reasons &&
    indexed.timestamp_match_policy == legacy.timestamp_match_policy &&
    indexed.capture_sync_status == legacy.capture_sync_status && indexed.delta_ns == legacy.delta_ns &&
    indexed.rgb_message_id == legacy.rgb_message_id && indexed.depth_message_id == legacy.depth_message_id &&
    indexed.rgb_calibration_id == legacy.rgb_calibration_id &&
    indexed.depth_calibration_id == legacy.depth_calibration_id &&
    indexed.rgb_image.stamp.sec == legacy.rgb_image.stamp.sec &&
    indexed.rgb_image.stamp.nanosec == legacy.rgb_image.stamp.nanosec &&
    indexed.depth_image.stamp.sec == legacy.depth_image.stamp.sec &&
    indexed.depth_image.stamp.nanosec == legacy.depth_image.stamp.nanosec &&
    indexed.rgb_raw_evidence == legacy.rgb_raw_evidence &&
    indexed.depth_raw_evidence == legacy.depth_raw_evidence &&
    indexed.rgb_media == legacy.rgb_media && indexed.depth_media == legacy.depth_media &&
    indexed.depth.has_value() == legacy.depth.has_value(), "Indexed pair context differs");
  compareModel(indexed.rgb_model, legacy.rgb_model);
  compareModel(indexed.depth_model, legacy.depth_model);
  comparePose(indexed.rgb_pose, legacy.rgb_pose);
  comparePose(indexed.depth_pose, legacy.depth_pose);
  comparePose(indexed.body_pose, legacy.body_pose);
  if (legacy.depth) {compareMedia(indexed, legacy);}
}

/** @brief 打开一次后读取目录内所有配对，核对源顺序和新旧入口等价性。 */
void indexedDatasetCase(const fs::path & root)
{
  auto opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto rows = readLines(root / "rgbd_pairs.jsonl");
  const auto & catalog = opened.reader->pairs();
  expect(catalog.size() == rows.size(), "Indexed pair count differs");
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto & info = catalog[i];
    const auto & row = rows[i];
    expect(info.pair_id == row.at("pair_id") && info.rgb_message_id == row.at("rgb_message_id") &&
      info.depth_message_id == row.at("depth_message_id") &&
      info.rgb_stamp.sec == row.at("rgb_stamp").at("sec") &&
      info.rgb_stamp.nanosec == row.at("rgb_stamp").at("nanosec") &&
      info.depth_stamp.sec == row.at("depth_stamp").at("sec") &&
      info.depth_stamp.nanosec == row.at("depth_stamp").at("nanosec") &&
      info.delta_ns == std::stoll(row.at("delta_ns").get<std::string>()) &&
      info.status == row.at("status") && info.pending_reasons == row.at("reasons"),
      "Catalog changed source order or metadata");
    const auto actual = opened.reader->loadPair(info.pair_id);
    const auto expected = loadPreparedPair(root, info.pair_id);
    expect(actual.ok() && expected.ok(), actual.message + expected.message);
    comparePair(*actual.pair, *expected.pair);
  }
  const auto missing = opened.reader->loadPair("absent_pair");
  expect(!missing.ok() && !missing.pair && missing.code == PreparationReadCode::pair_not_found,
    "Indexed unknown ID returned a pair");
}

/** @brief 要求打开失败时提供指定稳定码及空 reader。 */
void expectOpenFailure(const fs::path & root, PreparationReadCode code)
{
  const auto opened = openPreparation(root);
  expect(!opened.ok() && !opened.reader && opened.code == code,
    "Unexpected open result: " + opened.message);
}

/** @brief 用独立夹具核查索引变更、错误隔离、空目录及未选中记录兼容。 */
void indexedFailureCases(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("indexed_reader_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  const auto pairs_path = root / "rgbd_pairs.jsonl";
  const auto frames_path = root / "frames.jsonl";
  const auto pairs = readLines(pairs_path);
  const auto frames = readLines(frames_path);
  expect(pairs.size() >= 2, "Isolation fixture needs two pairs");
  const auto first_id = pairs.front().at("pair_id").get<std::string>();
  const auto other_id = pairs.back().at("pair_id").get<std::string>();

  auto opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  fs::last_write_time(frames_path, fs::last_write_time(frames_path) + std::chrono::seconds(1));
  auto changed = opened.reader->loadPair(first_id);
  expect(!changed.pair && changed.code == PreparationReadCode::source_changed,
    "Metadata mtime change was ignored");
  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  {std::ofstream file(frames_path, std::ios::app); file << '\n';}
  changed = opened.reader->loadPair(first_id);
  expect(!changed.pair && changed.code == PreparationReadCode::source_changed,
    "Metadata size change was ignored");
  writeLines(frames_path, frames);
  indexedDatasetCase(root);
  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto saved_frames = root / "saved_frames.jsonl";
  fs::rename(frames_path, saved_frames);
  fs::create_symlink(saved_frames.filename(), frames_path);
  changed = opened.reader->loadPair(first_id);
  expect(!changed.pair && changed.code == PreparationReadCode::source_changed,
    "Metadata canonical path change was ignored");
  fs::remove(frames_path);
  fs::rename(saved_frames, frames_path);


  auto duplicate = pairs;
  duplicate.push_back(pairs.front());
  writeLines(pairs_path, duplicate);
  expectOpenFailure(root, PreparationReadCode::invalid_reference);
  writeLines(pairs_path, pairs);

  auto unrelated = frames.front();
  unrelated["source"]["message_id"] = "unused_record";
  auto extra = frames;
  extra.push_back(unrelated);
  extra.push_back(unrelated);
  writeLines(frames_path, extra);
  expectOpenFailure(root, PreparationReadCode::invalid_reference);
  expect(loadPreparedPair(root, first_id).ok(), "Unselected duplicate changed legacy behavior");
  extra.pop_back();
  extra.back()["schema_version"] = "unsupported";
  writeLines(frames_path, extra);
  expectOpenFailure(root, PreparationReadCode::invalid_metadata);
  expect(loadPreparedPair(root, first_id).ok(), "Unselected schema changed legacy behavior");
  writeLines(frames_path, frames);
  {std::ofstream file(frames_path, std::ios::app); file << "{broken json}\n";}
  expectOpenFailure(root, PreparationReadCode::invalid_metadata);
  writeLines(frames_path, frames);

  auto missing = frames;
  const auto depth_id = pairs.front().at("depth_message_id");
  /** @brief 只移除第一对引用的深度帧，保留另一对的全部记录。 */
  missing.erase(std::remove_if(missing.begin(), missing.end(), [&depth_id](const Json & row) {
      return row.at("source").at("message_id") == depth_id;
    }), missing.end());
  writeLines(frames_path, missing);
  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto broken = opened.reader->loadPair(first_id);
  expect(!broken.pair && broken.code == PreparationReadCode::invalid_reference,
    "Missing frame was not isolated");
  expect(opened.reader->loadPair(other_id).ok(), "Broken pair blocked a different pair");
  writeLines(frames_path, frames);

  writeLines(pairs_path, {});
  opened = openPreparation(root);
  expect(opened.ok() && opened.reader->pairs().empty(), "Empty catalog was rejected");
  writeLines(pairs_path, pairs);

  auto catalog_only = pairs;
  catalog_only.front()["rgb_stamp"] = {{"sec", 0}, {"nanosec", 0}};
  catalog_only.back()["rgb_stamp"] = {{"sec", -1}, {"nanosec", 999999999}};
  auto repeated = catalog_only.back();
  repeated["pair_id"] = "repeated_timestamp";
  catalog_only.push_back(repeated);
  writeLines(pairs_path, catalog_only);
  fs::resize_file(pairs_path, fs::file_size(pairs_path) - 1);
  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto & catalog = opened.reader->pairs();
  expect(catalog.size() == 3 && catalog.front().rgb_stamp.sec == 0 &&
    catalog[1].rgb_stamp.sec == -1 && catalog[2].rgb_stamp.sec == -1 &&
    catalog[1].pair_id == other_id, "Catalog sorted or dropped repeated/negative timestamps");
  writeLines(pairs_path, pairs);

  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto original = opened.reader->loadPair(first_id);
  expect(original.ok(), original.message);
  {std::ofstream file(*original.pair->depth_media, std::ios::binary); file << "broken npy";}
  const auto bad_media = opened.reader->loadPair(first_id);
  expect(!bad_media.pair && bad_media.code == PreparationReadCode::invalid_media,
    "Corrupt depth was accepted");
  auto survivor = opened.reader->loadPair(other_id);
  expect(survivor.ok(), survivor.message);
  opened.reader.reset();
  const auto legacy = loadPreparedPair(root, other_id);
  expect(legacy.ok(), legacy.message);
  comparePair(*survivor.pair, *legacy.pair);
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
  indexedDatasetCase(normal);
  indexedDatasetCase(root / "normal_png_out");
  indexedFailureCases(normal);
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
  indexedDatasetCase(copy);

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
  indexedDatasetCase(copy);
  unavailable->erase("camera_model");
  unavailable->at("media_path") = nullptr;
  writeLines(frames_path, rows);
  auto nullable = openPreparation(copy);
  expect(nullable.ok(), nullable.message);
  const auto nullable_pair = nullable.reader->loadPair(id);
  expect(nullable_pair.ok() && !nullable_pair.pair->depth && !nullable_pair.pair->depth_media &&
    !nullable_pair.pair->depth_model && !nullable_pair.pair->depth_pose &&
    nullable_pair.pair->pending_reasons == pending.pair->pending_reasons,
    "Optional missing fields were not retained");
  writeLines(pairs_path, saved_pairs);
  writeLines(frames_path, original);

  const auto media = copy / npy.depth_media->lexically_relative(normal);
  {std::ofstream file(media, std::ios::binary); file << "broken npy";}
  expectFailure(copy, id, PreparationReadCode::invalid_media);
}

/** @brief 用真实短包的独立验收参考核对项目入口的逐点几何。 */
void realCase(const fs::path & npy_path, const fs::path & png_path, const fs::path & report_path)
{
  indexedDatasetCase(npy_path);
  indexedDatasetCase(png_path);
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
