#include "vln_memory/bag/camera_info/camera_info.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace vln_memory;
/** @brief 检查条件，失败抛异常以便 Release 构建仍执行断言。 */
void expect(bool value, const std::string & reason)
{
  if (!value) {throw std::runtime_error(reason);}
}

/** @brief 构造完整标定，K/P 故意不同并保留双目平移，返回值独立持有数据。 */
CameraInfoData calibration()
{
  CameraInfoData data;
  data.topic = "/camera/camera_info";
  data.frame_id = "camera_optical";
  data.stamp = {7, 100};
  data.width = 640;
  data.height = 480;
  data.distortion_model = "plumb_bob";
  data.d = {0, 0, 0, 0, 0};
  data.k = {500, 0, 320, 0, 510, 240, 0, 0, 1};
  data.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // 不同的 P 焦距和双目基线均合法，不能被检查器改写成 K。
  data.p = {490, 0, 315, -49, 0, 500, 239, 0, 0, 0, 1, 0};
  return data;
}

/** @brief 逐项填写显式绑定、图像域和纳秒容差，供各组用例复用。 */
CameraInfoMatchOptions options()
{
  CameraInfoMatchOptions config;
  config.camera_info_topic = "/camera/camera_info";
  config.image_topic = "/camera/image_raw";
  config.optical_frame = "camera_optical";
  config.binding_basis = "device_profile_v1";
  config.image_domain = CameraImageDomain::raw;
  config.clock_relation = SensorClockRelation::same_clock;
  config.max_delta_ns = 1000;
  return config;
}

/** @brief 构造实际图像的扁平引用元数据，不包含像素或借用引用。 */
CameraImageReference image()
{
  CameraImageReference reference;
  reference.topic = "/camera/image_raw";
  reference.width = 640;
  reference.height = 480;
  reference.frame_id = "camera_optical";
  reference.stamp = {7, 100};
  return reference;
}

/** @brief 核对坏标定的错误码、说明及失败无数据约定。 */
void rejects(const CameraInfoData & input, CameraInfoErrorCode expected)
{
  const auto result = decodeCameraInfo(input);
  expect(!result.ok() && !result.info && !result.message.empty() && result.code == expected,
    std::string("Expected ") + cameraInfoErrorCodeName(expected) + ", got " + result.message);
}

/** @brief 检查标定边界、三种畸变模型和输出所有权。 */
void testCalibration()
{
  auto input = calibration();
  const auto decoded = decodeCameraInfo(input);
  expect(decoded.ok(), decoded.message);
  input.d[0] = 123;
  input.topic = "/changed";
  expect(decoded.info->d[0] == 0 && decoded.info->topic == "/camera/camera_info" &&
    decoded.info->p[3] == -49 && decoded.info->k[0] != decoded.info->p[0],
    "Output lost source fields or retained input references");
  for (const auto & model : {std::string("equidistant"), std::string("rational_polynomial")}) {
    input = calibration();
    input.distortion_model = model;
    input.d.assign(model == "equidistant" ? 4U : 8U, 0.0);
    expect(decodeCameraInfo(input).ok(), "Supported model was rejected");
  }
  input = calibration();
  input.k[0] = 0;
  rejects(input, CameraInfoErrorCode::uncalibrated);
  input = calibration();
  input.k[4] = -1;
  rejects(input, CameraInfoErrorCode::invalid_intrinsics);
  input = calibration();
  input.k[1] = 0.1;
  rejects(input, CameraInfoErrorCode::invalid_intrinsics);
  input = calibration();
  input.r[0] = -1;
  rejects(input, CameraInfoErrorCode::invalid_rectification);
  input = calibration();
  input.r[0] = 0;
  rejects(input, CameraInfoErrorCode::invalid_rectification);
  input = calibration();
  input.p[10] = 0;
  rejects(input, CameraInfoErrorCode::invalid_projection);
  input = calibration();
  input.d.clear();
  rejects(input, CameraInfoErrorCode::invalid_distortion);
  input = calibration();
  input.distortion_model = "unknown";
  rejects(input, CameraInfoErrorCode::unsupported_distortion_model);
  input = calibration();
  input.d[0] = std::numeric_limits<double>::quiet_NaN();
  rejects(input, CameraInfoErrorCode::nonfinite_parameters);
  input = calibration();
  input.p[0] = std::numeric_limits<double>::infinity();
  rejects(input, CameraInfoErrorCode::nonfinite_parameters);
  input = calibration();
  input.width = 0;
  rejects(input, CameraInfoErrorCode::invalid_dimensions);
  input = calibration();
  input.frame_id.clear();
  rejects(input, CameraInfoErrorCode::invalid_frame);
  input = calibration();
  input.stamp.nanosec = 1000000000U;
  rejects(input, CameraInfoErrorCode::invalid_stamp);
  for (const auto & topic : {"camera_info", "/camera//info", "/camera/", "/1camera/info", "/camera/info bad"}) {
    input = calibration();
  input.topic = topic;
    rejects(input, CameraInfoErrorCode::invalid_topic);
  }
  input = calibration();
  input.topic = "/" + std::string(247, 'a');
  rejects(input, CameraInfoErrorCode::invalid_topic);
}

/** @brief 检查 ROI/binning 网格、显式绑定和公开可变输入的重新校验。 */
void testGridAndBinding()
{
  auto input = calibration();
  auto config = options();
  auto reference = image();
  expect(checkCameraInfoImage(input, reference, config).matched(), "Default ROI/binning should match");
  // 标定仍是 640x480，但实际 ROI 320x240、2x3 binning 的图像是 160x80。
  input.roi = {100, 60, 320, 240, false};
  input.binning_x = 2;
  input.binning_y = 3;
  reference.width = 160;
  reference.height = 80;
  const auto cropped = checkCameraInfoImage(input, reference, config);
  expect(cropped.matched() && cropped.expected_width == 160 && cropped.expected_height == 80,
    "ROI/binning geometry not applied");
  reference.width = 640;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::size_mismatch,
    "Calibration width was confused with actual image width");
  input.roi.do_rectify = true;
  config.image_domain = CameraImageDomain::rectified;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::unsupported_rectified_roi,
    "Raw ROI was silently used for a rectified image");
  input = calibration();
  input.roi = {630, 0, 20, 30, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration();
  input.roi = {1, 0, 0, 0, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration();
  input.roi = {0, 0, std::numeric_limits<std::uint32_t>::max(), 1, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration();
  input.binning_x = 3;
  rejects(input, CameraInfoErrorCode::unsupported_binning);
  input = calibration();
  config = options();
  reference = image();
  reference.topic = "/depth/image_raw";
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::topic_mismatch,
    "Matching dimensions must not bind unrelated topics");
  reference = image();
  reference.frame_id = "depth_optical";
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::frame_mismatch,
    "Different optical frames were accepted");
  reference = image();
  config.binding_basis.clear();
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::invalid_options,
    "Missing binding evidence was accepted");
  config = options();
  config.image_domain = CameraImageDomain::unspecified;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::invalid_options,
    "Raw versus rectified image domain was guessed");
  config = options();
  input.k[0] = 0;
  const auto invalid = checkCameraInfoImage(input, reference, config);
  expect(invalid.code == CameraInfoMatchCode::invalid_camera_info &&
    invalid.calibration_code == CameraInfoErrorCode::uncalibrated,
    "Public mutable input bypassed calibration validation or lost the structured error");
}

/** @brief 检查有符号时差、容差边界、未知时钟及固定标定语义。 */
void testTiming()
{
  auto input = calibration();
  auto reference = image();
  auto config = options();
  input.stamp.nanosec += 1000;
  const auto edge = checkCameraInfoImage(input, reference, config);
  expect(edge.matched() && edge.timing && edge.timing->delta_ns == 1000,
    "Image should be the timestamp reference");
  input.stamp.nanosec += 1;
  const auto late = checkCameraInfoImage(input, reference, config);
  expect(!late.matched() && late.timing && late.timing->code == SensorTimeCode::outside_tolerance,
    "Out-of-time calibration was accepted");
  config.clock_relation = SensorClockRelation::unknown;
  expect(checkCameraInfoImage(input, reference, config).timing->code == SensorTimeCode::clock_unknown,
    "Clock domain was inferred");
  config.time_policy = CameraInfoTimePolicy::static_calibration;
  input.stamp = {0, 0};
  const auto fixed = checkCameraInfoImage(input, reference, config);
  expect(fixed.matched() && fixed.used_static_calibration && !fixed.timing && input.stamp.sec == 0,
    "Fixed calibration must not fabricate a synchronized stamp");
  reference.stamp.nanosec = 1000000000U;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::invalid_image,
    "Static policy must still reject malformed image stamp");
}
}

/** @brief 运行 CameraInfo 回归，失败输出原因并返回非零状态。 */
int main()
{
  try {
    testCalibration();
    testGridAndBinding();
    testTiming();
    std::cout << "CameraInfo calibration, stream/grid and timing checks: passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
