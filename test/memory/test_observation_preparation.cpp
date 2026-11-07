#include <vln_memory/engine/build_input.hpp>
#include <vln_memory/engine/observation_builder.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>

namespace
{
using namespace vln_memory;
namespace fs = std::filesystem;

/** @brief 拒绝真实适配结果与独立 preparation reader 不一致。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 时间/网格/话题全部保留，不能只比较浮点秒。 */
bool sameImage(const CameraImageReference & a, const CameraImageReference & b)
{
  return a.topic == b.topic && a.frame_id == b.frame_id && a.width == b.width && a.height == b.height &&
    a.stamp.sec == b.stamp.sec && a.stamp.nanosec == b.stamp.nanosec;
}

/** @brief 数值模型必须等于来源 reader 恢复的全部投影参数。 */
void compareModel(const std::optional<CameraModel> & a, const std::optional<CameraModel> & b)
{
  expect(a.has_value() == b.has_value(), "Model presence differs");
  if (!a) {return;}
  expect(a->k == b->k && a->d == b->d && a->r == b->r && a->p == b->p &&
    a->frame_id == b->frame_id && a->width == b->width && a->height == b->height &&
    a->image_domain == b->image_domain && a->normalization_applied == b->normalization_applied,
    "Effective model changed");
}

/** @brief 数值、查询时刻及坐标链必须等于独立 reader 输出。 */
void comparePose(const std::optional<TfPoseResult> & a, const std::optional<TfPoseResult> & b)
{
  expect(a.has_value() == b.has_value(), "Pose presence differs");
  if (!a) {return;}
  expect(a->pose && b->pose && a->pose->translation_m == b->pose->translation_m &&
    a->pose->rotation_xyzw == b->pose->rotation_xyzw && a->time.frames == b->time.frames &&
    a->time.stamp.sec == b->time.stamp.sec && a->time.stamp.nanosec == b->time.stamp.nanosec,
    "Pose changed");
}
/** @brief 用实际选择入口的已选配对调用生产 Builder，并核对顺序、批次和来源。 */
void checkSelection(
  const fs::path & source, const fs::path & output, const PreparationReader & reader,
  const ObservationBuildContext & context, const std::map<std::string, std::string> & expected_ids)
{
  BuildInputOptions options;
  options.batch_size = 3;
  const auto selection = buildInputManifest(source, output, options);
  expect(selection.ok(), selection.message);
  std::size_t selected = 0;
  for (const auto & frame : selection.frames) {
    if (!frame.selected) {continue;}
    expect(frame.selection_index == selected && frame.batch_index == selected / options.batch_size,
      "Selection order/batch changed");
    const auto built = buildObservation(reader, frame.pair_id, context);
    expect(built.ok(), built.message);
    const auto & o = *built.observation;
    expect(o.id == expected_ids.at(frame.pair_id) && o.source_pending_reasons == frame.pending_reasons &&
      o.rgb_image.stamp.sec == frame.rgb_stamp.sec && o.rgb_image.stamp.nanosec == frame.rgb_stamp.nanosec &&
      o.depth_image.stamp.sec == frame.depth_stamp.sec && o.depth_image.stamp.nanosec == frame.depth_stamp.nanosec,
      "Selected observation identity or source metadata differs");
    ++selected;
  }
  expect(selected == expected_ids.size() && selection.selected == selected, "Selected observations lost");
  const auto missing = buildObservation(reader, "", context);
  expect(!missing.ok() && missing.preparation_error == PreparationReadCode::pair_not_found &&
    !missing.observation_error, "Builder lost missing pair read error");
}

/** @brief 修改独立的元数据副本，验证变化时保持 Reader 的错误语义。 */
void checkSourceChanged(
  const fs::path & source, const fs::path & copy, const std::string & pair_id,
  const ObservationBuildContext & context)
{
  fs::create_directory(copy);
  for (const auto * file : {"bag_manifest.json", "rgbd_pairs.jsonl", "frames.jsonl",
    "camera_info.jsonl", "tf.jsonl", "messages.jsonl"}) {
    fs::copy_file(source / file, copy / file);
  }
  if (fs::exists(source / "media_sha256.jsonl")) {
    fs::copy_file(source / "media_sha256.jsonl", copy / "media_sha256.jsonl");
  }
  auto opened = openPreparation(copy);
  expect(opened.ok(), opened.message);
  std::ofstream changed(copy / "rgbd_pairs.jsonl", std::ios::app);
  changed << '\n';
  changed.close();
  expect(static_cast<bool>(changed), "Cannot change fixture metadata");
  const auto built = buildObservation(*opened.reader, pair_id, context);
  expect(!built.ok() && built.preparation_error == PreparationReadCode::source_changed &&
    !built.observation_error, "Builder lost source_changed read error");
}

}  // namespace

/** @brief 适配测试入口：DATASET INPUT_JSONL ROUNDTRIP_JSONL SOURCE_MANIFEST_SHA256。 */
int main(int argc, char ** argv)
{
  try {
    expect(argc == 5, "Expected DATASET INPUT_JSONL ROUNDTRIP_JSONL MANIFEST_SHA256");
    auto opened = vln_memory::openPreparation(argv[1]);
    expect(opened.ok(), opened.message);
    std::ifstream input(argv[2]);
    std::ofstream output(argv[3]);
    expect(input.is_open() && output.is_open(), "Cannot open test JSONL");
    const fs::path output_path = argv[3];
    std::map<std::string, std::string> ids;
    std::map<std::string, std::string> rgb_ids;
    ObservationBuildContext context;
    for (std::string line; std::getline(input, line);) {
      auto parsed = vln_memory::deserializeObservation(line);
      expect(parsed.ok(), vln_memory::observationCodeName(parsed.code) + std::string(": ") + parsed.message);
      const auto & o = *parsed.observation;
      expect(ids.emplace(o.pair_id, o.id).second, "Duplicate observation pair");
      if (o.rgb_media) {rgb_ids.emplace(o.pair_id, o.id);}
      context = {o.map_id, o.geometry_revision, o.map_frame, o.map_basis, o.revision,
        {"physical_geometry_unverified"}};
      expect(o.source_manifest_sha256 == argv[4], "Manifest digest changed");
      auto loaded = opened.reader->loadPair(o.pair_id);
      expect(loaded.ok(), loaded.message);
      const auto & pair = *loaded.pair;
      expect(o.bag_id == pair.bag_id && o.rgb_message_id == pair.rgb_message_id &&
        o.depth_message_id == pair.depth_message_id && o.delta_ns == pair.delta_ns &&
        sameImage(o.rgb_image, pair.rgb_image) && sameImage(o.depth_image, pair.depth_image) &&
        o.clock_id == pair.time_scope.clock_id && o.segment_id == pair.time_scope.segment_id &&
        o.clock_basis == pair.clock_basis && o.segment_basis == pair.segment_basis &&
        o.map_frame == pair.map_frame && o.capture_sync_status == pair.capture_sync_status &&
        o.source_pair_status == pair.status && o.source_pending_reasons == pair.pending_reasons,
        "Source identity/time/status changed");
      compareModel(o.rgb_model, pair.rgb_model);
      compareModel(o.depth_model, pair.depth_model);
      comparePose(o.camera_pose_map, pair.rgb_pose);
      comparePose(o.depth_camera_pose_map, pair.depth_pose);
      comparePose(o.body_pose_map, pair.body_pose);
      expect(o.rgb_media.has_value() == pair.rgb_media.has_value() &&
        o.depth_media.has_value() == pair.depth_media.has_value(), "Media presence changed");
      if (o.rgb_media) {
        expect(std::filesystem::canonical(std::filesystem::path(argv[1]) / *o.rgb_media) == *pair.rgb_media,
          "RGB media changed");
      }
      if (o.depth_media) {
        expect(std::filesystem::canonical(std::filesystem::path(argv[1]) / *o.depth_media) == *pair.depth_media,
          "Depth media changed");
        expect(o.depth_storage && pair.depth && o.depth_storage->valid_pixels == pair.depth->valid_pixels &&
          o.depth_storage->invalid_pixels == pair.depth->invalid_pixels &&
          o.depth_storage->source_unit == pair.depth->source_unit &&
          o.depth_storage->source_scale_to_m == pair.depth->source_scale_to_m, "Depth restoration metadata changed");
      }
      const auto assembled = buildObservation(pair, context);
      expect(assembled.ok(), assembled.message);
      const auto encoded = vln_memory::serializeObservation(*assembled.observation);
      const auto expected = vln_memory::serializeObservation(o);
      expect(encoded.ok(), encoded.message);
      expect(expected.ok() && encoded.json == expected.json, "Typed reader assembly differs from independent JSONL oracle");
      output << encoded.json << '\n';
    }
    expect(input.eof() && !input.bad() && ids.size() == opened.reader->pairs().size(), "Incomplete JSONL/pair coverage");
    output.close();
    expect(static_cast<bool>(output), "Cannot write round-trip JSONL");
    if (!ids.empty()) {
      checkSelection(argv[1], output_path.string() + ".selection", *opened.reader, context, rgb_ids);
      checkSourceChanged(argv[1], output_path.string() + ".changed", ids.begin()->first, context);
    }
    std::cout << "Preparation/Observation comparison passed: " << ids.size() << " pairs, "
              << rgb_ids.size() << " selected RGB pairs\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
