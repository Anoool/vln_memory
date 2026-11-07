#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/bag/builder/preparation_compactor.hpp>
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

/** @brief 将独立测试副本改为发布摘要前的历史契约。 */
void removeMediaSeal(const fs::path & root, Json & manifest)
{
  manifest.erase("media_integrity");
  auto & files = manifest.at("files");
  files.erase(std::remove(files.begin(), files.end(), Json("media_sha256.jsonl")), files.end());
  fs::remove(root / "media_sha256.jsonl");
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
  const auto manifest = readJson(root / "bag_manifest.json");
  const bool saved = manifest.at("builder_version") != 7 ||
    manifest.at("effective_options").at("save_raw_cdr").get<bool>();
  expect(pair.rgb_raw_evidence.has_value() == saved && pair.depth_raw_evidence.has_value() == saved,
    "Raw evidence presence contradicts storage policy");
  if (saved) {
    expect(fs::is_regular_file(*pair.rgb_raw_evidence) && fs::is_regular_file(*pair.depth_raw_evidence),
      "Missing raw evidence paths");
  }
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

/** @brief 比较两入口返回的原始数据，包含 ROI/binning/时刻，不能只比较有效 K。 */
void compareOriginal(const std::optional<CameraInfoData> & a, const std::optional<CameraInfoData> & b)
{
  expect(a.has_value() == b.has_value(), "Original calibration presence differs");
  if (!a) {return;}
  expect(a->stamp.sec == b->stamp.sec && a->stamp.nanosec == b->stamp.nanosec &&
    a->topic == b->topic && a->frame_id == b->frame_id && a->width == b->width && a->height == b->height &&
    a->distortion_model == b->distortion_model && a->k == b->k && a->d == b->d && a->r == b->r && a->p == b->p &&
    a->binning_x == b->binning_x && a->binning_y == b->binning_y &&
    a->roi.x_offset == b->roi.x_offset && a->roi.y_offset == b->roi.y_offset &&
    a->roi.width == b->roi.width && a->roi.height == b->roi.height && a->roi.do_rectify == b->roi.do_rectify,
    "Original calibration data differs");
}

/** @brief 新旧入口的选择配置与实际选择/规范化状态须完整一致。 */
void compareCalibration(const PreparedCalibration & a, const PreparedCalibration & b)
{
  const auto & x = a.selection;
  const auto & y = b.selection;
  expect(x.camera_info_topic == y.camera_info_topic && x.image_topic == y.image_topic &&
    x.optical_frame == y.optical_frame && x.binding_basis == y.binding_basis &&
    x.image_domain == y.image_domain && x.time_policy == y.time_policy && x.clock_relation == y.clock_relation &&
    x.max_delta_ns == y.max_delta_ns && x.compatibility == y.compatibility &&
    a.calibration_id == b.calibration_id && a.source_message_id == b.source_message_id &&
    a.selection_code == b.selection_code && a.selection_message == b.selection_message &&
    a.calibration_code == b.calibration_code && a.used_static_calibration == b.used_static_calibration &&
    a.timing.has_value() == b.timing.has_value() && a.source_validation_code == b.source_validation_code &&
    a.source_validation_message == b.source_validation_message &&
    a.normalized_validation_code == b.normalized_validation_code &&
    a.source_compatibility == b.source_compatibility && a.normalization_applied == b.normalization_applied &&
    a.source_nonfinite == b.source_nonfinite && a.original_unavailable_reason == b.original_unavailable_reason,
    "Calibration selection/state differs");
  if (a.timing) {expect(a.timing->code == b.timing->code && a.timing->delta_ns == b.timing->delta_ns, "Calibration timing differs");}
  compareOriginal(a.original, b.original);
}

/** @brief 比较新增存储、摘要及上下文，确保所有返回值独立拥有且两入口等价。 */
void compareAssemblyMetadata(const PreparedPair & a, const PreparedPair & b)
{
  expect(a.builder_version == b.builder_version && a.source_manifest_sha256 == b.source_manifest_sha256 &&
    a.body_frame == b.body_frame && a.sync_tolerance_ns == b.sync_tolerance_ns && a.tf_max_gap_ns == b.tf_max_gap_ns &&
    a.tf_jump_limits.translation_m == b.tf_jump_limits.translation_m &&
    a.tf_jump_limits.rotation_rad == b.tf_jump_limits.rotation_rad &&
    a.capture_sync_basis == b.capture_sync_basis && a.rgb_media_relative == b.rgb_media_relative &&
    a.depth_media_relative == b.depth_media_relative && a.depth_alignment == b.depth_alignment &&
    a.depth_alignment_check == b.depth_alignment_check && a.alignment_basis == b.alignment_basis &&
    a.depth_storage.has_value() == b.depth_storage.has_value(), "Assembly context differs");
  compareCalibration(a.rgb_calibration, b.rgb_calibration);
  compareCalibration(a.depth_calibration, b.depth_calibration);
  if (a.depth_storage) {
    const auto & x = *a.depth_storage;
    const auto & y = *b.depth_storage;
    expect(x.format == y.format && x.dtype == y.dtype && x.unit == y.unit && x.scale_to_m == y.scale_to_m &&
      x.invalid_rule == y.invalid_rule && x.source_encoding == y.source_encoding && x.source_unit == y.source_unit &&
      x.source_scale_to_m == y.source_scale_to_m && x.min_depth_m == y.min_depth_m && x.max_depth_m == y.max_depth_m &&
      x.valid_pixels == y.valid_pixels && x.invalid_pixels == y.invalid_pixels &&
      x.legacy_metric_npy == y.legacy_metric_npy, "Storage interpretation differs");
  }
  expect(a.message_summaries.size() == b.message_summaries.size(), "Message closure size differs");
  for (std::size_t i = 0; i < a.message_summaries.size(); ++i) {
    const auto & x = a.message_summaries[i];
    const auto & y = b.message_summaries[i];
    expect(x.message_id == y.message_id && x.topic == y.topic && x.file_index == y.file_index &&
      x.topic_ordinal == y.topic_ordinal && x.message_order == y.message_order &&
      x.bag_record_time_ns == y.bag_record_time_ns && x.serialized_bytes == y.serialized_bytes &&
      x.payload_sha256 == y.payload_sha256 && x.raw_path == y.raw_path, "Message summary differs");
  }
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
  compareAssemblyMetadata(indexed, legacy);
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
    const auto metadata = opened.reader->loadPairMetadata(info.pair_id);
    expect(metadata.ok() && !metadata.pair->depth &&
      metadata.pair->rgb_media == actual.pair->rgb_media && metadata.pair->depth_media == actual.pair->depth_media,
      "Metadata-only read changed paths or decoded depth");
    compareAssemblyMetadata(*metadata.pair, *actual.pair);
    comparePose(metadata.pair->depth_pose, actual.pair->depth_pose);
    expect(opened.reader->verifyUnchanged(actual.pair->file_snapshots).ok(), "Snapshot did not validate");
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

/** @brief 拒绝重复 JSON 字段、深嵌套和超限行。 */
void metadataBoundsCase(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("metadata_bounds_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  const auto path = root / "frames.jsonl";
  const auto original = readLines(path);
  expect(!original.empty(), "Missing fixture frame");
  const auto original_line = original.front().dump();
  {
    std::ofstream file(path);
    file << "{\"schema_version\":\"bag_preparation/1\"," << original_line.substr(1) << '\n';
  }
  const auto duplicate = openPreparation(root);
  expect(duplicate.code == PreparationReadCode::invalid_metadata &&
    duplicate.message.find("Duplicate") != std::string::npos, "Duplicate JSON key was accepted");
  {
    std::ofstream file(path);
    file << "{\"schema_version\":\"bag_preparation/1\",\"nested\":"
         << std::string(70, '[') << '0' << std::string(70, ']') << "}\n";
  }
  const auto deep = openPreparation(root);
  expect(deep.code == PreparationReadCode::invalid_metadata &&
    deep.message.find("nesting") != std::string::npos, "Deep JSON exceeded parser budget");
  {
    std::ofstream file(path);
    file << '{';
  }
  fs::resize_file(path, 16U * 1024U * 1024U + 1);
  const auto oversized = openPreparation(root);
  expect(oversized.code == PreparationReadCode::invalid_metadata &&
    oversized.message.find("size limit") != std::string::npos, "Oversized JSONL row was accepted");
}

/** @brief 完整读取拒绝长度不变但内容变化的 CDR，元数据读取仍只核对文件状态。 */
void rawDigestCase(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("raw_digest_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  auto opened = openPreparation(root);
  expect(opened.ok() && !opened.reader->pairs().empty(), "Cannot open raw digest fixture");
  const auto & id = opened.reader->pairs().front().pair_id;
  const auto before = opened.reader->loadPair(id);
  expect(before.ok() && before.pair->rgb_raw_evidence, "Missing saved CDR evidence");
  {
    std::fstream file(*before.pair->rgb_raw_evidence, std::ios::in | std::ios::out | std::ios::binary);
    char byte{};
    expect(static_cast<bool>(file.get(byte)), "Cannot read CDR fixture byte");
    file.seekp(0);
    file.put(static_cast<char>(static_cast<unsigned char>(byte) ^ 0x01U));
    expect(static_cast<bool>(file), "Cannot replace CDR fixture byte");
  }
  const auto metadata = opened.reader->loadPairMetadata(id);
  expect(metadata.ok(), "Metadata read should not hash CDR payloads");
  const auto full = opened.reader->loadPair(id);
  expect(full.code == PreparationReadCode::invalid_reference && full.message.find("digest") != std::string::npos,
    "Full read accepted equal-size corrupted CDR");
}

/** @brief 同尺寸、同 mtime 的合法 NPY 替换和帧媒体错绑必须被发现。 */
void mediaIntegrityCase(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("media_digest_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  auto opened = openPreparation(root);
  expect(opened.ok() && !opened.reader->pairs().empty(), "Cannot open sealed fixture");
  const auto id = opened.reader->pairs().front().pair_id;
  const auto full = opened.reader->loadPair(id);
  expect(full.ok() && full.pair->depth && full.pair->depth_media, "Missing metric depth fixture");
  const auto path = *full.pair->depth_media;
  const auto length = fs::file_size(path);
  const auto time = fs::last_write_time(path);
  {
    std::fstream file(path, std::ios::in | std::ios::out | std::ios::binary);
    const auto offset = static_cast<std::streamoff>(length - 4 * full.pair->depth->depth_m.size());
    file.seekg(offset);
    char byte{};
    expect(static_cast<bool>(file.get(byte)), "Cannot read NPY sample");
    file.seekp(offset);
    file.put(static_cast<char>(static_cast<unsigned char>(byte) ^ 0x01U));
    expect(static_cast<bool>(file), "Cannot replace NPY sample");
  }
  fs::last_write_time(path, time);
  expect(fs::file_size(path) == length && fs::last_write_time(path) == time, "Fixture changed size or mtime");
  const auto light = opened.reader->loadPairMetadata(id);
  expect(light.ok() && !light.pair->depth, "Metadata read should defer payload checking");
  const auto changed = opened.reader->verifyUnchanged(full.pair->file_snapshots);
  expect(changed.code == PreparationReadCode::source_changed, "Same-size same-mtime replacement was accepted");
  for (const auto & read : {opened.reader->loadPair(id), loadPreparedPair(root, id)}) {
    expect(read.code == PreparationReadCode::invalid_media && read.message.find("SHA-256") != std::string::npos,
      "Full read accepted replaced NPY");
  }
  const auto refused_path = root.parent_path() / ("invalid_compact_" + std::to_string(tick));
  const auto refused = compactPreparation(root, refused_path);
  expect(!refused.ok() && refused.code == "invalid_media" && !fs::exists(refused_path),
    "Conversion published replaced sealed media");

  // 同一改动在旧契约下仍是有效的 NPY，表明摘要是发现问题的依据。
  auto manifest = readJson(root / "bag_manifest.json");
  removeMediaSeal(root, manifest);
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  const auto legacy = loadPreparedPair(root, id);
  expect(legacy.ok(), "Modified NPY should remain structurally valid: " + legacy.message);

  const auto bindings = root.parent_path() / ("media_bindings_" + std::to_string(tick));
  fs::copy(source, bindings, fs::copy_options::recursive);
  auto frames = readLines(bindings / "frames.jsonl");
  const auto old_frames = frames;
  const auto original = std::find_if(frames.begin(), frames.end(), [&](const Json & row) {
      return row.at("source").at("message_id") == full.pair->rgb_message_id;
    });
  const auto other = std::find_if(frames.begin(), frames.end(), [&](const Json & row) {
      return row.at("kind") == "rgb" && row.at("source").at("message_id") != full.pair->rgb_message_id &&
        !row.at("media_path").is_null();
    });
  expect(original != frames.end() && other != frames.end(), "Fixture needs two RGB media files");
  original->at("media_path") = other->at("media_path");
  writeLines(bindings / "frames.jsonl", frames);
  const auto swapped = loadPreparedPair(bindings, id);
  expect(swapped.code == PreparationReadCode::invalid_reference, "Frame accepted another frame's valid media");
  writeLines(bindings / "frames.jsonl", old_frames);
  const auto index_path = bindings / "media_sha256.jsonl";
  auto index = readLines(index_path);
  index.front()["sha256"] = std::string(64, '0');
  writeLines(index_path, index);
  const auto index_check = openPreparation(bindings);
  expect(index_check.code == PreparationReadCode::invalid_metadata &&
    index_check.message.find("index digest") != std::string::npos, "Tampered media index was accepted");
}

/** @brief 文件替换破坏完整读取快照；PNG 头部与像素是两个校验层。 */
void metadataSnapshotCase(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("metadata_snapshot_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  auto opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  const auto & id = opened.reader->pairs().front().pair_id;
  const auto full = opened.reader->loadPair(id);
  expect(full.ok() && full.pair->depth_media && full.pair->depth, "Missing full PNG fixture");
  fs::resize_file(*full.pair->depth_media, 26);
  const auto changed = opened.reader->verifyUnchanged(full.pair->file_snapshots);
  expect(changed.code == PreparationReadCode::source_changed, "Media change was accepted");
  const auto header_only = opened.reader->loadPairMetadata(id);
  expect(header_only.ok() && !header_only.pair->depth, "Metadata-only mode decoded a valid PNG header");
  const auto corrupted = opened.reader->loadPair(id);
  expect(corrupted.code == PreparationReadCode::invalid_media && !corrupted.pair,
    "Full read accepted truncated PNG payload");
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


/** @brief 验证策略矛盾、旧格式缺文件和可选证据字段。 */
void cdrPolicyCases(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("cdr_policy_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  const auto id = readJson(root / "rgbd_pairs.jsonl").at("pair_id").get<std::string>();
  auto manifest = readJson(root / "bag_manifest.json");
  const auto original = manifest;
  manifest["effective_options"]["save_raw_cdr"] = false;
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  expectFailure(root, id, PreparationReadCode::invalid_reference);
  auto opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  expect(opened.reader->loadPair(id).code == PreparationReadCode::invalid_reference,
    "Indexed reader accepted contradictory raw paths");
  manifest["effective_options"]["save_raw_cdr"] = "false";
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  expectOpenFailure(root, PreparationReadCode::invalid_metadata);
  manifest["effective_options"].erase("save_raw_cdr");
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  expectOpenFailure(root, PreparationReadCode::invalid_metadata);
  manifest = original;
  manifest["builder_version"] = 6;
  manifest["effective_options"].erase("save_raw_cdr");
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  indexedDatasetCase(root);
  const auto pair = loadPreparedPair(root, id);
  expect(pair.ok() && pair.pair->rgb_raw_evidence, "Historical raw evidence not retained");
  fs::remove(*pair.pair->rgb_raw_evidence);
  expectFailure(root, id, PreparationReadCode::invalid_reference);
  opened = openPreparation(root);
  expect(opened.ok(), opened.message);
  expect(opened.reader->loadPair(id).code == PreparationReadCode::invalid_reference,
    "Indexed legacy missing CDR was accepted");
}

/** @brief 转换结果只改变存储引用，源目录和已发布目标不覆盖。 */
void compactionCases(const fs::path & source)
{
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto target = source.parent_path() / ("compact_out_" + std::to_string(tick));
  const auto converted = compactPreparation(source, target);
  expect(converted.ok(), converted.code + ": " + converted.message);
  expect(converted.pairs_checked == readLines(source / "rgbd_pairs.jsonl").size() &&
    converted.raw_bytes_omitted > 0 && !fs::exists(target / "evidence/raw"),
    "Compaction counters or raw omission wrong");
  const auto manifest = readJson(target / "bag_manifest.json");
  expect(manifest.at("builder_version") == 7 &&
    manifest.at("effective_options").at("save_raw_cdr") == false &&
    manifest.at("conversion_source_builder_version") == readJson(source / "bag_manifest.json").at("builder_version"),
    "Compaction provenance missing");
  indexedDatasetCase(target);
  for (const auto & row : readLines(target / "rgbd_pairs.jsonl")) {
    const auto id = row.at("pair_id").get<std::string>();
    const auto before = loadPreparedPair(source, id);
    const auto after = loadPreparedPair(target, id);
    expect(before.ok() && after.ok() && before.pair->rgb_raw_evidence &&
      !after.pair->rgb_raw_evidence && !after.pair->depth_raw_evidence,
      "Compaction evidence state wrong");
    compareMedia(*before.pair, *after.pair);
    compareModel(before.pair->depth_model, after.pair->depth_model);
    comparePose(before.pair->depth_pose, after.pair->depth_pose);
  }
  expect(compactPreparation(source, target).code == "output_exists", "Existing target was overwritten");
  expect(compactPreparation(source, source / "nested").code == "invalid_output", "Overlapping output accepted");
  const auto repeated = compactPreparation(target, target.string() + "_repeat");
  expect(repeated.ok() && repeated.raw_bytes_omitted == 0, "Already compact data could not be converted");

  const auto broken = source.parent_path() / ("compact_broken_" + std::to_string(tick));
  fs::copy(source, broken, fs::copy_options::recursive);
  const auto id = readJson(broken / "rgbd_pairs.jsonl").at("pair_id").get<std::string>();
  const auto pair = loadPreparedPair(broken, id);
  expect(pair.ok(), pair.message);
  {std::ofstream file(*pair.pair->depth_media, std::ios::binary); file << "broken";}
  const auto failed_target = broken.string() + "_out";
  const auto failed = compactPreparation(broken, failed_target);
  expect(!failed.ok() && !fs::exists(failed_target) && !failed.work_directory.empty() &&
    fs::is_directory(failed.work_directory) && fs::is_regular_file(*pair.pair->depth_media),
    "Failed compaction published a partial dataset or changed the source");
}

/** @brief 组装元数据的正常返回、失败状态、精度、隔离与生命周期回归。 */
void assemblyMetadataCases(const fs::path & source)
{
  const auto base = readLines(source / "rgbd_pairs.jsonl");
  const auto id = base.front().at("pair_id").get<std::string>();
  const auto expected = loadPreparedPair(source, id);
  expect(expected.ok(), expected.message);
  const auto & p = *expected.pair;
  const auto manifest = readJson(source / "bag_manifest.json");
  const auto & options = manifest.at("effective_options");
  expect(p.sync_tolerance_ns == std::stoll(options.at("sync_tolerance_ns").get<std::string>()) &&
    p.body_frame == options.at("body_frame") && p.tf_max_gap_ns == std::stoll(options.at("tf_max_gap_ns").get<std::string>()) &&
    p.source_manifest_sha256.size() == 64 && p.rgb_calibration.original && p.depth_calibration.original &&
    p.rgb_calibration.selection.image_topic == p.rgb_image.topic && p.depth_calibration.selection.image_topic == p.depth_image.topic &&
    p.rgb_calibration.calibration_id == p.rgb_calibration_id && p.depth_calibration.calibration_id == p.depth_calibration_id &&
    p.depth_storage && p.rgb_media_relative && p.depth_media_relative,
    "Incomplete assembly metadata");
  expect(fs::canonical(source / *p.rgb_media_relative) == *p.rgb_media &&
    fs::canonical(source / *p.depth_media_relative) == *p.depth_media, "Relative media differs");
  const auto messages = readLines(source / "messages.jsonl");
  std::set<std::string> references{p.rgb_message_id, p.depth_message_id, p.rgb_calibration.source_message_id,
    p.depth_calibration.source_message_id};
  for (const auto * pose : {&p.rgb_pose, &p.depth_pose, &p.body_pose}) {
    if (!*pose) {continue;}
    for (const auto & edge : (*pose)->time.edges) {
      references.insert(edge.before.source_message_id);
      if (edge.after) {references.insert(edge.after->source_message_id);}
    }
  }
  expect(references.size() == p.message_summaries.size(), "Message closure lost/doubled references");
  std::uint64_t previous = 0;
  for (const auto & summary : p.message_summaries) {
    const auto row = std::find_if(messages.begin(), messages.end(), [&summary](const Json & value) {
        return value.at("message_id") == summary.message_id;
      });
    expect(row != messages.end() && references.count(summary.message_id) &&
      summary.topic == row->at("topic") && summary.file_index == row->at("file_index") &&
      summary.topic_ordinal == std::stoull(row->at("topic_ordinal").get<std::string>()) &&
      summary.message_order == std::stoull(row->at("message_order").get<std::string>()) &&
      summary.bag_record_time_ns == std::stoll(row->at("bag_record_time_ns").get<std::string>()) &&
      summary.serialized_bytes == row->at("serialized_bytes") && summary.payload_sha256 == row->at("payload_sha256") &&
      summary.raw_path.has_value() == row->at("raw_path").is_string() && summary.message_order >= previous,
      "Message summary changed or is not source ordered");
    previous = summary.message_order;
  }
  auto opened = openPreparation(source);
  auto survivor = opened.reader->loadPair(id);
  opened.reader.reset();
  expect(survivor.ok(), survivor.message);
  comparePair(*survivor.pair, p);

  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = source.parent_path() / ("assembly_metadata_" + std::to_string(tick));
  fs::copy(source, root, fs::copy_options::recursive);
  const auto pairs_path = root / "rgbd_pairs.jsonl";
  const auto frames_path = root / "frames.jsonl";
  const auto cal_path = root / "camera_info.jsonl";
  const auto original_frames = readLines(frames_path);
  const auto original_cal = readLines(cal_path);

  auto sync_manifest = manifest;
  sync_manifest["effective_options"]["rgb_camera"]["time_policy"] = "synchronized";
  sync_manifest["effective_options"]["rgb_camera"]["max_delta_ns"] = "0";
  auto sync_calibrations = original_cal;
  for (auto & row : sync_calibrations) {
    if (row.at("calibration_id") == p.rgb_calibration_id) {
      row["stamp"] = base.front().at("rgb_stamp");
    }
  }
  auto sync_frames = original_frames;
  for (auto & row : sync_frames) {
    if (row.at("source").at("message_id") == p.rgb_message_id) {
      row["calibration"]["used_static_calibration"] = false;
      row["calibration"]["timing"] = {{"code", "matched"}, {"delta_ns", "0"}};
    }
  }
  {std::ofstream file(root / "bag_manifest.json"); file << sync_manifest.dump();}
  writeLines(cal_path, sync_calibrations);
  writeLines(frames_path, sync_frames);
  const auto synchronized = loadPreparedPair(root, id);
  expect(synchronized.ok() && synchronized.pair->rgb_calibration.timing &&
    synchronized.pair->rgb_calibration.timing->delta_ns == 0 &&
    !synchronized.pair->rgb_calibration.used_static_calibration, "Synchronized selection metadata changed");
  for (auto & row : sync_frames) {
    if (row.at("source").at("message_id") == p.rgb_message_id) {row["calibration"]["timing"]["delta_ns"] = "1";}
  }
  writeLines(frames_path, sync_frames);
  expectFailure(root, id, PreparationReadCode::invalid_reference);
  {std::ofstream file(root / "bag_manifest.json"); file << manifest.dump();}
  writeLines(cal_path, original_cal);
  writeLines(frames_path, original_frames);

  auto pairs = base;
  pairs.front()["sync_tolerance_ns"] = "1";
  writeLines(pairs_path, pairs);
  opened = openPreparation(root);
  expect(opened.ok(), "A single contradictory pair must not invalidate global index: " + opened.message);
  const auto bad_tolerance = opened.reader->loadPair(id);
  expect(!bad_tolerance.pair && bad_tolerance.code == PreparationReadCode::invalid_reference, "Pair tolerance mismatch accepted");
  expect(opened.reader->loadPair(base.back().at("pair_id").get<std::string>()).ok(), "Pair timing error was not isolated");
  writeLines(pairs_path, base);

  auto frames = original_frames;
  auto rgb = std::find_if(frames.begin(), frames.end(), [&p](const Json & row) {
      return row.at("source").at("message_id") == p.rgb_message_id;
    });
  rgb->at("camera_model")["k"][0] = rgb->at("camera_model").at("k").at(0).get<double>() + 1;
  writeLines(frames_path, frames);
  expectFailure(root, id, PreparationReadCode::invalid_reference);
  writeLines(frames_path, original_frames);

  frames = original_frames;
  rgb = std::find_if(frames.begin(), frames.end(), [&p](const Json & row) {
      return row.at("source").at("message_id") == p.rgb_message_id;
    });
  (*rgb)["camera_model"]["k"].push_back(0.0);
  writeLines(frames_path, frames);
  expectFailure(root, id, PreparationReadCode::invalid_metadata);
  writeLines(frames_path, original_frames);

  auto calibrations = original_cal;
  auto selected = std::find_if(calibrations.begin(), calibrations.end(), [&p](const Json & row) {
      return row.at("calibration_id") == p.rgb_calibration_id;
    });
  (*selected)["normalized_code"] = "invalid_distortion";
  writeLines(cal_path, calibrations);
  expectFailure(root, id, PreparationReadCode::invalid_reference);
  writeLines(cal_path, original_cal);

  calibrations = original_cal;
  selected = std::find_if(calibrations.begin(), calibrations.end(), [&p](const Json & row) {
      return row.at("calibration_id") == p.rgb_calibration_id;
    });
  (*selected)["k"][0] = nullptr;
  (*selected)["source_nonfinite"] = true;
  (*selected)["validation_code"] = "nonfinite_parameters";
  (*selected)["normalized_code"] = "nonfinite_parameters";
  (*selected)["normalization_applied"] = false;
  frames = original_frames;
  rgb = std::find_if(frames.begin(), frames.end(), [&p](const Json & row) {
      return row.at("source").at("message_id") == p.rgb_message_id;
    });
  (*rgb)["camera_model"]["code"] = "invalid_model";
  (*rgb)["calibration"]["code"] = "invalid_camera_info";
  (*rgb)["calibration"]["calibration_code"] = "nonfinite_parameters";
  (*rgb)["calibration"]["used_static_calibration"] = false;
  (*rgb)["calibration"].erase("timing");
  pairs = base;
  pairs.front()["status"] = "pending";
  pairs.front()["reasons"] = Json::array({"rgb_camera_info_unavailable"});
  writeLines(cal_path, calibrations);
  writeLines(frames_path, frames);
  writeLines(pairs_path, pairs);
  const auto unavailable = loadPreparedPair(root, id);
  expect(unavailable.ok() && !unavailable.pair->rgb_model && !unavailable.pair->rgb_calibration.original &&
    unavailable.pair->rgb_calibration.source_nonfinite &&
    unavailable.pair->rgb_calibration.original_unavailable_reason == "nonfinite_source_parameters_not_representable" &&
    unavailable.pair->rgb_calibration.source_message_id == p.rgb_calibration.source_message_id,
    "Nonfinite original calibration was invented or discarded with source");
  auto malformed = calibrations;
  auto malformed_selected = std::find_if(malformed.begin(), malformed.end(), [&p](const Json & row) {
      return row.at("calibration_id") == p.rgb_calibration_id;
    });
  malformed_selected->erase("stamp");
  writeLines(cal_path, malformed);
  expectFailure(root, id, PreparationReadCode::invalid_metadata);
  writeLines(cal_path, original_cal);

  (*rgb)["calibration"]["calibration_id"] = "";
  (*rgb)["calibration"]["code"] = "missing_calibration";
  (*rgb)["calibration"]["calibration_code"] = "none";
  pairs.front()["rgb_calibration_id"] = "";
  writeLines(frames_path, frames);
  writeLines(pairs_path, pairs);
  const auto missing = loadPreparedPair(root, id);
  expect(missing.ok() && !missing.pair->rgb_calibration.original &&
    missing.pair->rgb_calibration.selection_code == "missing_calibration" &&
    missing.pair->rgb_calibration.selection.camera_info_topic == p.rgb_calibration.selection.camera_info_topic &&
    missing.pair->rgb_calibration.selection.binding_basis == p.rgb_calibration.selection.binding_basis,
    "Missing candidate lost selection configuration");
  writeLines(frames_path, original_frames);
  writeLines(pairs_path, base);

  frames = original_frames;
  rgb = std::find_if(frames.begin(), frames.end(), [&p](const Json & row) {
      return row.at("source").at("message_id") == p.rgb_message_id;
    });
  (*rgb)["media_path"] = rgb->at("media_path").get<std::string>() + std::string(1, '\0') + "suffix";
  writeLines(frames_path, frames);
  expectFailure(root, id, PreparationReadCode::invalid_reference);
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
  assemblyMetadataCases(normal);
  const auto png = readPair(root / "normal_png_out", id);
  indexedDatasetCase(normal);
  indexedDatasetCase(root / "normal_no_cdr_out");
  compareMedia(npy, readPair(root / "normal_no_cdr_out", id));
  indexedDatasetCase(root / "normal_png_out");
  metadataSnapshotCase(root / "normal_png_out");
  indexedFailureCases(normal);
  metadataBoundsCase(normal);
  rawDigestCase(normal);
  mediaIntegrityCase(normal);
  cdrPolicyCases(normal);
  compactionCases(normal);
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
  removeMediaSeal(copy, manifest);
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
  compactionCases(copy);

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
