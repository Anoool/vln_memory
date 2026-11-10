#include <vln_memory/bag/builder/bag_builder.hpp>
#include <vln_memory/bag/builder/rgbd_sync.hpp>

#include <charconv>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "build_input_cli.hpp"

namespace
{
/** @brief 说明项目 YAML 入口和历史 preparation 的调用方式。 */
void usage()
{
  std::cout << "Usage: ros2 run vln_memory bag_builder --config config/vln_memory.yaml\n"
    << "  Legacy preparation: --config config/camera_record.yaml --format preparation\n";
}

/** @brief 将显式整数毫秒转换为纳秒，拒绝负值、尾随字符和乘法溢出。 */
std::int64_t toleranceNs(const std::string & value, const char * key)
{
  constexpr std::int64_t ns_per_ms = 1000000;
  std::int64_t milliseconds{0};
  const auto end = value.data() + value.size();
  const auto parsed = std::from_chars(value.data(), end, milliseconds);
  if (parsed.ec != std::errc{} || parsed.ptr != end || value.front() == '-' ||
    milliseconds > std::numeric_limits<std::int64_t>::max() / ns_per_ms) {
    throw std::invalid_argument(std::string(key) + " requires nonnegative integer milliseconds");
  }
  return milliseconds * ns_per_ms;
}

/** @brief 为当前录包绑定 raw CameraInfo，复用段内固定标定和零尾 D8 兼容。 */
vln_memory::CameraInfoMatchOptions projectCamera(
  const std::string & image_topic, const std::string & info_topic, const std::string & frame,
  const char * basis)
{
  using namespace vln_memory;
  CameraInfoMatchOptions camera;
  camera.image_topic = image_topic;
  camera.camera_info_topic = info_topic;
  camera.optical_frame = frame;
  camera.binding_basis = basis;
  camera.image_domain = CameraImageDomain::raw;
  camera.time_policy = CameraInfoTimePolicy::static_calibration;
  camera.compatibility = CameraInfoCompatibility::plumb_bob_zero_tail_d8;
  camera.clock_relation = SensorClockRelation::same_clock;
  return camera;
}

/** @brief 读取项目 YAML 并发布观测，options 返回本次生效的录包设置。 */
vln_memory::BagBuildResult runProject(const std::string & path, vln_memory::BagBuildOptions & options)
{
  using namespace vln_memory;
  const auto config = YAML::LoadFile(path);
  const auto dataset = config["dataset"];
  const auto topics = config["topics"];
  const auto frames = config["frames"];
  const auto preprocess = config["preprocess"];

  options.bag_uri = dataset["bag_path"].as<std::string>();
  options.output_directory = dataset["memory_dir"].as<std::string>();
  options.map_frame = frames["map"].as<std::string>();
  options.body_frame = frames["body"].as<std::string>();
  options.tf_topic = topics["tf"].as<std::string>();
  options.tf_static_topic = topics["tf_static"].as<std::string>();
  options.clock_id = "recorded_ros_header_epoch_hypothesis";
  options.clock_relation = SensorClockRelation::same_clock;
  options.clock_basis = "Diagnostic same-clock hypothesis for supplied capture; original headers "
    "preserved; physical synchronization not verified";
  options.segment_id = "camera_record_single_recording";
  options.segment_basis = "Supplied capture: prior full scan found no backwards image/CameraInfo "
    "headers; no automatic cross-reset pairing";
  options.rgb_camera = projectCamera(
    topics["rgb"].as<std::string>(), topics["rgb_camera_info"].as<std::string>(),
    frames["rgb_optical"].as<std::string>(),
    "Observed actual topic and header frame; geometry still requires audit; explicit fixed-model "
    "reuse: prior full-bag audit found one model per stream; current index rechecks entire segment consistency");
  options.depth_camera = projectCamera(
    topics["depth"].as<std::string>(), topics["depth_camera_info"].as<std::string>(),
    frames["depth_optical"].as<std::string>(),
    "Observed actual topic and header frame; registration/unit not yet confirmed; explicit fixed-model "
    "reuse: prior full-bag audit found one model per stream; current index rechecks entire segment consistency");
  options.depth_storage = BagDepthStorage::source_png16;
  options.depth.convention = DepthConvention::optical_z;
  options.depth.alignment_basis = "Not accepted as registered solely from equal dimensions/frame IDs";
  const auto scale = preprocess["depth_scale_to_m"].as<double>();
  if (scale == 0.001) {options.depth.input_unit = DepthUnit::millimeters;}
  else if (scale == 1.0) {options.depth.input_unit = DepthUnit::meters;}
  else {throw std::invalid_argument("depth_scale_to_m must be 0.001 (mm) or 1 (m)");}
  options.sync_tolerance_ns = toleranceNs(
    preprocess["sync_tolerance_ms"].as<std::string>(), "sync_tolerance_ms");

  const ObservationBuildContext context{
    dataset["map_id"].as<std::string>(), dataset["geometry_revision"].as<std::string>(),
    options.map_frame, dataset["map_basis"].as<std::string>(), 1, {}};
  BuildInputOptions selection;
  selection.batch_size = 4;
  if (const auto distance = preprocess["keyframe_distance_m"]) {selection.distance_m = distance.as<double>();}
  if (const auto yaw = preprocess["keyframe_yaw_deg"]) {selection.yaw_rad = yaw.as<double>() * std::acos(-1.0) / 180;}
  return buildBagObservations(options, context, selection);
}

/** @brief 保留旧 bag 配置及参数覆盖，兼容历史 preparation 和原有脚本。 */
vln_memory::BagBuildResult runLegacy(
  vln_memory::cli::Arguments args, vln_memory::BagBuildOptions & options)
{
  namespace cli = vln_memory::cli;
  options = vln_memory::loadBagBuildOptions(cli::required(args, "--config"));
  if (args.count("--bag")) {options.bag_uri = cli::required(args, "--bag");}
  if (args.count("--output")) {options.output_directory = cli::required(args, "--output");}
  if (args.count("--sync-tolerance-ms")) {
    options.sync_tolerance_ns = toleranceNs(cli::required(args, "--sync-tolerance-ms"), "--sync-tolerance-ms");
  }
  const bool preparation = args.count("--format") && args.at("--format") == "preparation";
  if (args.count("--format") && !preparation && args.at("--format") != "observations") {
    throw std::invalid_argument("Expected --format observations or preparation");
  }
  if (preparation) {
    for (const auto * flag : {"--map-id", "--geometry-revision", "--map-basis", "--purpose", "--batch-size",
        "--distance-m", "--yaw-deg", "--max-step-m", "--max-step-yaw-deg"}) {
      if (args.count(flag)) {throw std::invalid_argument("Observation options require direct output");}
    }
    return vln_memory::buildBag(options);
  }
  const vln_memory::ObservationBuildContext context{cli::required(args, "--map-id"),
    cli::required(args, "--geometry-revision"), options.map_frame, cli::required(args, "--map-basis"), 1, {}};
  args.try_emplace("--purpose", "rgb");
  args.try_emplace("--batch-size", "4");
  return vln_memory::buildBagObservations(options, context, cli::selectionOptions(args));
}
}

/** @brief 薄离线入口；0=完整发布（可带 issues），1=构建失败，2=参数/配置错误。 */
int main(int argc, char ** argv)
{
  try {
    namespace cli = vln_memory::cli;
    cli::Arguments args;
    const std::set<std::string> allowed{"--config", "--bag", "--output", "--sync-tolerance-ms",
      "--format", "--map-id", "--geometry-revision", "--map-basis", "--purpose", "--batch-size",
      "--distance-m", "--yaw-deg", "--max-step-m", "--max-step-yaw-deg"};
    for (int index = 1; index < argc; ++index) {
      const std::string flag(argv[index]);
      if (flag == "--help" || flag == "-h") {usage(); return 0;}
      if (!allowed.count(flag) || index + 1 >= argc || !args.emplace(flag, argv[++index]).second) {
        usage(); return 2;
      }
    }
    const auto & path = cli::required(args, "--config");
    vln_memory::BagBuildOptions options;
    const auto result = args.size() == 1 ? runProject(path, options) : runLegacy(args, options);
    const nlohmann::json report{{"status", vln_memory::bagBuildStatusName(result.status)},
      {"code", result.code}, {"message", result.message}, {"output_directory", result.output_directory},
      {"work_directory", result.work_directory}, {"rgb_frames", result.rgb_frames},
      {"depth_frames", result.depth_frames}, {"decoded_frames", result.decoded_frames},
      {"observations", result.observations}, {"pending_observations", result.pending_observations},
      {"camera_models", result.camera_models}, {"pairs", result.pairs}, {"prepared_pairs", result.prepared_pairs}, {"issue_count", result.issue_count},
      {"timestamp_match_policy", vln_memory::rgbdSyncPolicyName(options.sync_tolerance_ns)},
      {"sync_tolerance_ns", std::to_string(options.sync_tolerance_ns)},
      {"timestamp_match_status", result.timestamp_match_status}, {"capture_sync_status", "unknown"}};
    std::cout << report.dump(2) << '\n';
    if (!result.ok()) {return 1;}
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
