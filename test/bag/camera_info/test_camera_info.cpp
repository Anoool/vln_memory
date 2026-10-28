#include "vln_memory/bag/camera_info/camera_info.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace vln_memory;
void expect(bool value, const std::string & reason)
{
  if (!value) {throw std::runtime_error(reason);}
}

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

CameraInfoMatchOptions options()
{
  CameraInfoMatchOptions config;
  config.binding = {"/camera/camera_info", "/camera/image_raw", "camera_optical",
    "device_profile_v1", CameraImageDomain::raw};
  config.time = {SensorClockRelation::same_clock, 1000};
  return config;
}

CameraImageReference image()
{
  return {"/camera/image_raw", {640, 480, "camera_optical"}, {7, 100}};
}

void rejects(const CameraInfoData & input, CameraInfoErrorCode expected)
{
  const auto result = decodeCameraInfo(input);
  expect(!result.ok() && !result.info && !result.message.empty() && result.code == expected,
    std::string("Expected ") + cameraInfoErrorCodeName(expected) + ", got " + result.message);
}

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
  input = calibration(); input.k[0] = 0;
  rejects(input, CameraInfoErrorCode::uncalibrated);
  input = calibration(); input.k[4] = -1;
  rejects(input, CameraInfoErrorCode::invalid_intrinsics);
  input = calibration(); input.k[1] = 0.1;
  rejects(input, CameraInfoErrorCode::invalid_intrinsics);
  input = calibration(); input.r[0] = -1;
  rejects(input, CameraInfoErrorCode::invalid_rectification);
  input = calibration(); input.r[0] = 0;
  rejects(input, CameraInfoErrorCode::invalid_rectification);
  input = calibration(); input.p[10] = 0;
  rejects(input, CameraInfoErrorCode::invalid_projection);
  input = calibration(); input.d.clear();
  rejects(input, CameraInfoErrorCode::invalid_distortion);
  input = calibration(); input.distortion_model = "unknown";
  rejects(input, CameraInfoErrorCode::unsupported_distortion_model);
  input = calibration(); input.d[0] = std::numeric_limits<double>::quiet_NaN();
  rejects(input, CameraInfoErrorCode::nonfinite_parameters);
  input = calibration(); input.p[0] = std::numeric_limits<double>::infinity();
  rejects(input, CameraInfoErrorCode::nonfinite_parameters);
  input = calibration(); input.width = 0;
  rejects(input, CameraInfoErrorCode::invalid_dimensions);
  input = calibration(); input.frame_id.clear();
  rejects(input, CameraInfoErrorCode::invalid_frame);
  input = calibration(); input.stamp.nanosec = 1000000000U;
  rejects(input, CameraInfoErrorCode::invalid_stamp);
  for (const auto & topic : {"camera_info", "/camera//info", "/camera/", "/1camera/info", "/camera/info bad"}) {
    input = calibration(); input.topic = topic;
    rejects(input, CameraInfoErrorCode::invalid_topic);
  }
}

void testGridAndBinding()
{
  auto input = calibration();
  auto config = options();
  auto reference = image();
  expect(checkCameraInfoImage(input, reference, config).matched(), "Default ROI/binning should match");
  // 标定仍是 640x480，但实际 ROI 320x240、2x3 binning 的图像是 160x80。
  input.roi = {100, 60, 320, 240, false};
  input.binning_x = 2; input.binning_y = 3;
  reference.geometry = {160, 80, "camera_optical"};
  const auto cropped = checkCameraInfoImage(input, reference, config);
  expect(cropped.matched() && cropped.expected_width == 160 && cropped.expected_height == 80,
    "ROI/binning geometry not applied");
  reference.geometry.width = 640;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::size_mismatch,
    "Calibration width was confused with actual image width");
  input.roi.do_rectify = true; config.binding.image_domain = CameraImageDomain::rectified;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::unsupported_rectified_roi,
    "Raw ROI was silently used for a rectified image");
  input = calibration(); input.roi = {630, 0, 20, 30, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration(); input.roi = {1, 0, 0, 0, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration(); input.roi = {0, 0, std::numeric_limits<std::uint32_t>::max(), 1, false};
  rejects(input, CameraInfoErrorCode::invalid_roi);
  input = calibration(); input.binning_x = 3;
  rejects(input, CameraInfoErrorCode::unsupported_binning);
  input = calibration(); config = options(); reference = image();
  reference.topic = "/depth/image_raw";
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::topic_mismatch,
    "Matching dimensions must not bind unrelated topics");
  reference = image(); reference.geometry.frame_id = "depth_optical";
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::frame_mismatch,
    "Different optical frames were accepted");
  reference = image(); config.binding.basis.clear();
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::invalid_options,
    "Missing binding evidence was accepted");
  config = options(); config.binding.image_domain = CameraImageDomain::unspecified;
  expect(checkCameraInfoImage(input, reference, config).code == CameraInfoMatchCode::invalid_options,
    "Raw versus rectified image domain was guessed");
  config = options(); input.k[0] = 0;
  const auto invalid = checkCameraInfoImage(input, reference, config);
  expect(invalid.code == CameraInfoMatchCode::invalid_camera_info &&
    invalid.calibration_code == CameraInfoErrorCode::uncalibrated,
    "Public mutable input bypassed calibration validation or lost the structured error");
}

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
  config.time.clock_relation = SensorClockRelation::unknown;
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
