#include <vln_memory/bag/builder/camera_info_index.hpp>

#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace
{
using namespace vln_memory;

static_assert(!std::is_constructible_v<CameraInfoIndex, std::vector<BagCameraInfoRecord> &&>,
  "CameraInfoIndex must reject temporary records");

/** @brief Release 也验证契约，异常指出失败语义。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 两像素完整模型，单独设置时间/来源便于测试候选选择。 */
BagCameraInfoRecord calibration(std::int32_t sec, std::uint64_t order)
{
  BagCameraInfoRecord record;
  record.source.message_id = "calibration_" + std::to_string(order);
  record.source.message_order = order;
  record.data.topic = "/info";
  record.data.frame_id = "camera";
  record.data.stamp = {sec, 0};
  record.data.width = 2;
  record.data.height = 1;
  record.data.distortion_model = "plumb_bob";
  record.data.d = {0, 0, 0, 0, 0};
  record.data.k = {1, 0, 1, 0, 1, 0, 0, 0, 1};
  record.data.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  record.data.p = {1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  return record;
}

/** @brief 构造显式流绑定和充足纳秒容差，不替设备猜标定含义。 */
CameraInfoMatchOptions options()
{
  CameraInfoMatchOptions config;
  config.camera_info_topic = "/info";
  config.image_topic = "/rgb";
  config.optical_frame = "camera";
  config.binding_basis = "fixture profile";
  config.image_domain = CameraImageDomain::raw;
  config.clock_relation = SensorClockRelation::same_clock;
  config.max_delta_ns = 2000000000LL;
  return config;
}

/** @brief 同刻和等距规则、静态版本变化、坏候选不回退、类型尺寸仍复用核心。 */
void selectionCases()
{
  std::vector<BagCameraInfoRecord> records{calibration(3, 1), calibration(1, 0)};
  const CameraImageReference image{"/rgb", 2, 1, "camera", {2, 0}};
  auto config = options();
  auto selected = CameraInfoIndex(records).select(image, config);
  expect(selected.matched() && selected.calibration_id == "calibration_0", "Tie must prefer earlier time");
  records[0].data.k[0] = 2;
  config.time_policy = CameraInfoTimePolicy::static_calibration;
  expect(
    CameraInfoIndex(records).select(
      image,
      config).code == "static_calibration_changed", "Static model change hidden");
  records[0].data.stamp = {1, 0};
  config.time_policy = CameraInfoTimePolicy::synchronized;
  expect(CameraInfoIndex(records).select(image, config).code == "ambiguous_calibration", "Same-stamp conflict hidden");
  records = {calibration(1, 0), calibration(2, 1)};
  records[1].data.k[0] = 0;
  selected = CameraInfoIndex(records).select(image, config);
  expect(
    selected.code == "invalid_camera_info" && selected.calibration_id == "calibration_1",
    "Invalid nearest candidate must not fall back to older valid model");
  auto nonfinite = calibration(1, 0);
  nonfinite.data.d[0] = std::numeric_limits<double>::quiet_NaN();
  const std::vector<BagCameraInfoRecord> single_bad{nonfinite};
  auto fixed = config;
  fixed.time_policy = CameraInfoTimePolicy::static_calibration;
  expect(
    CameraInfoIndex(single_bad).select(image, fixed).code == "invalid_camera_info",
    "One invalid model must not be reported as a model change");
  auto raw_d8 = calibration(1, 0);
  raw_d8.data.d.resize(8, 0);
  const auto policy = CameraInfoCompatibility::plumb_bob_zero_tail_d8;
  const auto normalized = normalizeCameraInfo(raw_d8.data, policy);
  raw_d8.normalized = normalized.info;
  raw_d8.normalization_applied = normalized.normalization_applied;
  raw_d8.compatibility = policy;
  const std::vector<BagCameraInfoRecord> compatible_records{raw_d8};
  expect(!CameraInfoIndex(compatible_records).select(image, config).matched(),
    "Stored derived model bypassed strict caller policy");
  auto compatible_options = config;
  compatible_options.compatibility = policy;
  const auto accepted = CameraInfoIndex(compatible_records).select(image, compatible_options);
  expect(accepted.matched() && accepted.info != nullptr && accepted.info->d.size() == 5 &&
    compatible_records[0].data.d.size() == 8 && accepted.normalization_applied,
    "Index lost raw/derived contract or explicit compatibility");
  auto bare_d8 = calibration(1, 0);
  bare_d8.data.d.resize(8, 0);
  const std::vector<BagCameraInfoRecord> bare_records{bare_d8};
  const auto direct = CameraInfoIndex(bare_records).select(image, compatible_options);
  expect(direct.matched() && direct.normalization_applied,
    "Direct D8 index call lost normalization provenance");
  records[1].data.stamp.nanosec = 1000000000U;
  expect(
    CameraInfoIndex(records).select(
      image,
      config).code == "invalid_calibration_stamp",
    "Unknown version boundary ignored");
}
}

/** @brief 运行独立标定索引核心测试，无 ROS/JSON/YAML 依赖。 */
int main()
{
  try {
    selectionCases();
    std::cout << "Calibration candidate index passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
