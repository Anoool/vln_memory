#include <vln_memory/engine/build_input.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <nlohmann/json.hpp>

namespace
{
using namespace vln_memory;
namespace fs = std::filesystem;

/** @brief Release 构建也执行的断言，失败说明直接交给 CTest。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 生成仅用于采样算法的显式同地图候选，不声称为真实观测。 */
BuildInputFrame candidate(std::size_t index, double x, double yaw = 0)
{
  BuildInputFrame frame;
  frame.source_index = index;
  frame.pair_id = "pair_" + std::to_string(index);
  frame.bag_id = "test_bag";
  frame.map_frame = "map";
  frame.clock_id = "clock";
  frame.segment_id = "segment";
  frame.rgb_stamp.sec = static_cast<std::int32_t>(index);
  frame.eligible = true;
  frame.body_pose_map = TfPose{{x, 0, 0}, {0, 0, std::sin(yaw / 2), std::cos(yaw / 2)}};
  return frame;
}

/** @brief 要求边界校验在修改候选前抛出可解释错误。 */
void invalid(std::vector<BuildInputFrame> frames, const BuildInputOptions & options)
{
  bool failed = false;
  try {selectBuildInputFrames(frames, options);}
  catch (const std::invalid_argument &) {failed = true;}
  expect(failed, "Invalid input accepted");
}

/** @brief 验证位移累积、固定分批及重复执行覆盖选择状态。 */
void motionCases()
{
  BuildInputOptions options;
  options.distance_m = 1;
  options.batch_size = 2;
  std::vector<BuildInputFrame> frames{
    candidate(0, 0), candidate(1, 0.4), candidate(2, 1), candidate(3, 1.4), candidate(4, 2)};
  selectBuildInputFrames(frames, options);
  expect(frames[0].selected && !frames[1].selected && frames[2].selected &&
    !frames[3].selected && frames[4].selected, "Distance must use last selected keyframe");
  expect(frames[2].selection_index == 1 && frames[2].batch_index == 0 &&
    frames[4].batch_index == 1, "Batch membership incorrect");
  options.distance_m.reset();
  selectBuildInputFrames(frames, options);
  expect(frames[3].selected && frames[3].selection_index == 3, "Selection state not replaced");
}

/** @brief 验证 yaw 环绕、缺姿态断点和明确步进上限，不把跳变当正常关键帧。 */
void poseCases()
{
  const double degree = std::acos(-1.0) / 180;
  BuildInputOptions options;
  options.yaw_rad = 5 * degree;
  std::vector<BuildInputFrame> frames{
    candidate(0, 0, 179 * degree), candidate(1, 0, -179 * degree), candidate(2, 0, -170 * degree)};
  selectBuildInputFrames(frames, options);
  expect(frames[0].selected && !frames[1].selected && frames[2].selected, "Yaw wrap incorrect");
  options.yaw_rad.reset();
  options.distance_m = 1;
  frames = {candidate(0, 0), candidate(1, 0), candidate(2, 0.1), candidate(3, 0.2)};
  frames[1].body_pose_map.reset();
  selectBuildInputFrames(frames, options);
  expect(frames[0].selected && !frames[1].selected && frames[2].selected &&
    !frames[3].selected, "Missing pose must break sampling continuity");
  frames[1].eligible = false;
  selectBuildInputFrames(frames, options);
  expect(frames[2].selected, "Ineligible gap must break continuity");
  options.max_step_m = 1;
  frames = {candidate(0, 0), candidate(1, 1), candidate(2, 2.01), candidate(3, 2.02)};
  selectBuildInputFrames(frames, options);
  expect(frames[1].selected && frames[2].selection_reason == "pose_jump" &&
    frames[3].selection_reason == "trajectory_discontinuity" && !frames[3].selected,
    "Step equality/jump handling incorrect");
  options.max_step_m.reset();
  frames = {candidate(0, 0)};
  frames[0].body_pose_map->rotation_xyzw = {0, 0, 0, 0};
  selectBuildInputFrames(frames, options);
  expect(!frames[0].selected, "Invalid quaternion accepted");
}

/** @brief 验证稳定参考时间顺序、零/负时间、配置和上下文边界。 */
void boundaryCases()
{
  BuildInputOptions options;
  auto first = candidate(0, 0);
  auto second = candidate(1, 0);
  auto third = candidate(2, 0);
  first.rgb_stamp = {0, 0};
  second.rgb_stamp = {-1, 999999999};
  third.rgb_stamp = {0, 0};
  std::vector<BuildInputFrame> frames{first, second, third};
  selectBuildInputFrames(frames, options);
  expect(frames[0].pair_id == second.pair_id && frames[1].pair_id == first.pair_id &&
    frames[2].pair_id == third.pair_id, "Reference ordering lost exact stamps/source ties");
  invalid({first, first}, options);
  second.clock_id = "other_clock";
  invalid({first, second}, options);
  second = candidate(1, 0);
  second.rgb_stamp.nanosec = 1000000000;
  invalid({second}, options);
  second.rgb_stamp.nanosec = 0;
  second.depth_stamp.nanosec = 1000000000;
  invalid({second}, options);
  options.batch_size = 0;
  invalid({}, options);
  options.batch_size = 1;
  options.distance_m = std::numeric_limits<double>::quiet_NaN();
  invalid({}, options);
  options.distance_m = -1;
  invalid({}, options);
  options.distance_m.reset();
  options.yaw_rad = 4;
  invalid({}, options);
  options.yaw_rad.reset();
  options.purpose = static_cast<BuildInputPurpose>(99);
  invalid({}, options);
}

/** @brief 读取独立发布文件，验证磁盘契约。 */
nlohmann::json readJson(const fs::path & path)
{
  std::ifstream file(path);
  nlohmann::json value;
  file >> value;
  return value;
}

/** @brief 公开入口消费实际 preparation，核对来源、用途、批次与不覆盖发布。 */
void datasetCases(const fs::path & input, const fs::path & output)
{
  BuildInputOptions options;
  options.batch_size = 3;
  auto opened = openPreparation(input);
  expect(opened.ok(), opened.message);
  const auto result = buildInputManifest(input, output / "rgb", options);
  expect(result.ok(), result.message);
  expect(result.frames.size() == opened.reader->pairs().size(), "Candidates lost");
  std::size_t selected = 0;
  for (const auto & frame : result.frames) {
    const auto & source = opened.reader->pairs().at(frame.source_index);
    expect(frame.pair_id == source.pair_id && frame.delta_ns == source.delta_ns &&
      frame.pending_reasons == source.pending_reasons &&
      frame.rgb_stamp.sec == source.rgb_stamp.sec &&
      frame.rgb_stamp.nanosec == source.rgb_stamp.nanosec, "Source decisions changed");
    const auto loaded = opened.reader->loadPair(frame.pair_id);
    expect(frame.eligible == (loaded.ok() && loaded.pair->rgb_media.has_value()), "RGB eligibility incorrect");
    if (frame.selected) {++selected;}
  }
  const auto manifest = readJson(result.output_directory / "build_input_manifest.json");
  expect(manifest.at("selected") == selected && manifest.at("frames").size() == result.frames.size() &&
    manifest.at("batches").size() == (selected + 2) / 3, "Published counts differ");
  const auto repeat_fresh = buildInputManifest(input, output / "rgb_repeat", options);
  expect(repeat_fresh.ok() && readJson(repeat_fresh.output_directory / "build_input_manifest.json") == manifest,
    "Same source/config must reproduce identical manifest");
  const auto repeated = buildInputManifest(input, output / "rgb", options);
  expect(!repeated.ok() && repeated.code == "output_exists", "Existing output replaced");
  const auto overlap = buildInputManifest(input, input / "selection_must_not_exist", options);
  expect(!overlap.ok() && overlap.code == "invalid_output" &&
    !fs::exists(input / "selection_must_not_exist"), "Overlapping output accepted");
  options.purpose = BuildInputPurpose::depth_map;
  const auto geometry = buildInputManifest(input, output / "depth_map", options);
  expect(geometry.ok(), geometry.message);
  for (const auto & frame : geometry.frames) {
    const auto loaded = opened.reader->loadPair(frame.pair_id);
    const bool expected = loaded.ok() && loaded.pair->depth && loaded.pair->depth_model &&
      loaded.pair->depth_pose && loaded.pair->depth_pose->ok();
    expect(frame.eligible == expected, "Depth map requirements incorrect");
  }
  options.purpose = BuildInputPurpose::prepared_rgbd;
  const auto prepared = buildInputManifest(input, output / "prepared", options);
  expect(prepared.ok(), prepared.message);
  for (const auto & frame : prepared.frames) {
    const auto loaded = opened.reader->loadPair(frame.pair_id);
    const bool expected = loaded.ok() && loaded.pair->status == "prepared_for_geometry" &&
      loaded.pair->rgb_media && loaded.pair->depth && loaded.pair->rgb_model && loaded.pair->depth_model &&
      loaded.pair->rgb_pose && loaded.pair->rgb_pose->ok() &&
      loaded.pair->depth_pose && loaded.pair->depth_pose->ok();
    expect(frame.eligible == expected && frame.selected == expected,
      "Prepared geometry must be selected; pending or missing fields must not");
  }
  options.batch_size = 0;
  const auto invalid_result = buildInputManifest(input, output / "invalid", options);
  expect(!invalid_result.ok() && !fs::exists(output / "invalid"), "Invalid options published");
}

/** @brief 针对独立副本破坏媒体，只记录读取失败并保留其他候选，不修改原始数据。 */
void damagedDatasetCase(const fs::path & input, const fs::path & output)
{
  const auto copy = output / "damaged_source";
  fs::copy(input, copy, fs::copy_options::recursive);
  auto opened = openPreparation(copy);
  expect(opened.ok(), opened.message);
  if (opened.reader->pairs().empty()) {return;}
  const auto loaded = opened.reader->loadPair(opened.reader->pairs().front().pair_id);
  if (!loaded.ok() || !loaded.pair->rgb_media) {return;}
  fs::remove(*loaded.pair->rgb_media);
  const auto result = buildInputManifest(copy, output / "damaged_result", {});
  expect(result.ok(), result.message);
  bool found = false;
  for (const auto & frame : result.frames) {
    if (frame.pair_id == loaded.pair->pair_id) {
      found = true;
      expect(!frame.eligible && !frame.selected && frame.read_code != PreparationReadCode::ready,
        "Damaged media silently selected");
    }
  }
  expect(found, "Damaged candidate lost");
}
}  // namespace

/** @brief 默认运行纯采样回归；可额外传 preparation 和全新产物父目录做磁盘集成。 */
int main(int argc, char ** argv)
{
  try {
    motionCases();
    poseCases();
    boundaryCases();
    if (argc == 3) {
      fs::path input = argv[1];
      fs::path output = argv[2];
      if (input == "--bag-fixture") {
        std::ifstream latest(output / "latest.txt");
        std::string fixture;
        std::getline(latest, fixture);
        expect(!fixture.empty(), "Missing bag fixture pointer");
        input = fs::path(fixture) / "normal_no_cdr_out";
        output = fs::path(fixture) / "build_input_checks";
      }
      fs::create_directories(output);
      datasetCases(input, output);
      damagedDatasetCase(input, output);
    } else if (argc != 1) {throw std::runtime_error("Expected no args or DATASET NEW_OUTPUT_PARENT");}
    std::cout << "Build input selection passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
