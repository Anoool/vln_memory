#include <vln_memory/bag/builder/bag_builder.hpp>
#include <vln_memory/bag/builder/rgbd_sync.hpp>

#include <charconv>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

namespace
{
/** @brief 说明配置及本次运行的路径和配对门限覆盖。 */
void usage()
{
  std::cout << "Usage: ros2 run vln_memory bag_builder --config FILE [--bag INPUT] [--output DIRECTORY]"
    << " [--sync-tolerance-ms NONNEGATIVE_INTEGER]\n";
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
    std::string config;
    std::string input;
    std::string output;
    std::optional<std::int64_t> tolerance;
    for (int index = 1; index < argc; ++index) {
      const std::string flag(argv[index]);
      if (flag == "--help" || flag == "-h") {usage(); return 0;}
      if ((flag != "--config" && flag != "--bag" && flag != "--output" && flag != "--sync-tolerance-ms") ||
        index + 1 >= argc) {
        usage(); return 2;
      }
      const std::string value(argv[++index]);
      if (flag == "--config") {config = value;}
      else if (flag == "--bag") {input = value;}
      else if (flag == "--output") {output = value;}
      else {tolerance = toleranceNs(value);}
    }
    if (config.empty()) {usage(); return 2;}
    auto options = vln_memory::loadBagBuildOptions(config);
    if (!input.empty()) {options.bag_uri = input;}
    if (!output.empty()) {options.output_directory = output;}
    if (tolerance) {options.sync_tolerance_ns = *tolerance;}
    const auto result = vln_memory::buildBag(options);
    const nlohmann::json report{{"status", vln_memory::bagBuildStatusName(result.status)},
      {"code", result.code}, {"message", result.message}, {"output_directory", result.output_directory},
      {"work_directory", result.work_directory}, {"rgb_frames", result.rgb_frames},
      {"depth_frames", result.depth_frames}, {"decoded_frames", result.decoded_frames},
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
