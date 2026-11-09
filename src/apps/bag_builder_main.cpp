#include <vln_memory/bag/builder/bag_builder.hpp>
#include <vln_memory/bag/builder/rgbd_sync.hpp>

#include <charconv>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "build_input_cli.hpp"

namespace
{
/** @brief 说明配置及本次运行的路径和配对门限覆盖。 */
void usage()
{
  std::cout << "Usage: ros2 run vln_memory bag_builder --config FILE [--bag INPUT] [--output DIRECTORY]"
    << " [--sync-tolerance-ms NONNEGATIVE_INTEGER]\n"
    << "  Direct output (default): --map-id ID --geometry-revision REV --map-basis TEXT\n"
    << "  [--purpose rgb|depth_map|prepared_rgbd] [--batch-size N] [--distance-m M] [--yaw-deg D]\n"
    << "  [--max-step-m M] [--max-step-yaw-deg D]\n"
    << "  Legacy output: --format preparation\n";
}

/** @brief 将显式整数毫秒转换为纳秒，拒绝负值、尾随字符和乘法溢出。 */
std::int64_t toleranceNs(const std::string & value)
{
  constexpr std::int64_t ns_per_ms = 1000000;
  std::int64_t milliseconds{0};
  const auto end = value.data() + value.size();
  const auto parsed = std::from_chars(value.data(), end, milliseconds);
  if (parsed.ec != std::errc{} || parsed.ptr != end || value.front() == '-' ||
    milliseconds > std::numeric_limits<std::int64_t>::max() / ns_per_ms) {
    throw std::invalid_argument("--sync-tolerance-ms requires nonnegative integer milliseconds within nanosecond range");
  }
  return milliseconds * ns_per_ms;
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
    auto options = vln_memory::loadBagBuildOptions(cli::required(args, "--config"));
    if (args.count("--bag")) {options.bag_uri = cli::required(args, "--bag");}
    if (args.count("--output")) {options.output_directory = cli::required(args, "--output");}
    if (args.count("--sync-tolerance-ms")) {
      options.sync_tolerance_ns = toleranceNs(cli::required(args, "--sync-tolerance-ms"));
    }
    const bool preparation = args.count("--format") && args.at("--format") == "preparation";
    if (args.count("--format") && !preparation && args.at("--format") != "observations") {
      throw std::invalid_argument("Expected --format observations or preparation");
    }
    vln_memory::BagBuildResult result;
    if (preparation) {
      for (const auto * flag : {"--map-id", "--geometry-revision", "--map-basis", "--purpose", "--batch-size",
          "--distance-m", "--yaw-deg", "--max-step-m", "--max-step-yaw-deg"}) {
        if (args.count(flag)) {throw std::invalid_argument("Observation options require direct output");}
      }
      result = vln_memory::buildBag(options);
    } else {
      const vln_memory::ObservationBuildContext context{cli::required(args, "--map-id"),
        cli::required(args, "--geometry-revision"), options.map_frame, cli::required(args, "--map-basis"), 1, {}};
      args.try_emplace("--purpose", "rgb");
      args.try_emplace("--batch-size", "4");
      result = vln_memory::buildBagObservations(options, context, cli::selectionOptions(args));
    }
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
