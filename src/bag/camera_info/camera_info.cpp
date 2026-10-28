#include "vln_memory/bag/camera_info/camera_info.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

namespace vln_memory
{
namespace
{
bool letter(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

/** 只接受 bag 中已解析的绝对 ROS topic；不在本模块处理 ~、替换或 remap。 */
bool validTopic(std::string_view topic)
{
  if (topic.size() < 2 || topic.front() != '/' || topic.back() == '/') {return false;}
  bool first = true;
  for (std::size_t i = 1; i < topic.size(); ++i) {
    const char c = topic[i];
    if (c == '/') {
      if (first) {return false;}
      first = true;
    } else {
      if (!(letter(c) || (!first && c >= '0' && c <= '9'))) {return false;}
      first = false;
    }
  }
  return !first;
}

bool validFrame(std::string_view frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(frame.begin(), frame.end(), [](char c) {return c <= ' ' || c == '\x7f';});
}

template<typename Values>
bool finite(const Values & values)
{
  return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
  return std::abs(actual - expected) <= tolerance;
}

/** 校验 R 属于旋转矩阵；允许常见标定文件的小数舍入，但拒绝反射矩阵。 */
bool rotation(const std::array<double, 9> & r)
{
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t other = 0; other < 3; ++other) {
      double dot = 0.0;
      for (std::size_t col = 0; col < 3; ++col) {dot += r[row * 3 + col] * r[other * 3 + col];}
      if (!near(dot, row == other ? 1.0 : 0.0, 1e-5)) {return false;}
    }
  }
  const double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) -
    r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
  return near(determinant, 1.0, 1e-5);
}

struct Validation
{
  CameraInfoErrorCode code{CameraInfoErrorCode::none};
  const char * message{""};
  std::uint32_t image_width{0};
  std::uint32_t image_height{0};
};

Validation validate(const CameraInfoData & input)
{
  using Code = CameraInfoErrorCode;
  // 第一段：采样身份与标定数值。未标定不是可用于投影的默认单位相机。
  if (!validTopic(input.topic)) {
    return {Code::invalid_topic, "Expected a resolved absolute ROS topic"};
  }
  if (!validFrame(input.frame_id)) {
    return {Code::invalid_frame, "Expected a non-empty optical frame ID without a leading slash"};
  }
  if (input.stamp.nanosec >= 1000000000U) {
    return {Code::invalid_stamp, "Stamp nanosec must be less than 1000000000"};
  }
  if (input.width == 0 || input.height == 0) {
    return {Code::invalid_dimensions, "Calibration dimensions must be positive"};
  }
  if (!finite(input.k) || !finite(input.r) || !finite(input.p) || !finite(input.d)) {
    return {Code::nonfinite_parameters, "Calibration contains NaN or infinity"};
  }
  if (input.k[0] == 0.0) {
    return {Code::uncalibrated, "K[0]==0 denotes an uncalibrated camera"};
  }
  std::size_t coefficients = 0;
  if (input.distortion_model == "plumb_bob") {
    coefficients = 5;
  } else if (input.distortion_model == "rational_polynomial") {
    coefficients = 8;
  } else if (input.distortion_model == "equidistant") {
    coefficients = 4;
  } else {
    return {Code::unsupported_distortion_model, "Unsupported or missing distortion model"};
  }
  if (input.d.size() != coefficients) {
    return {Code::invalid_distortion, "Distortion coefficient count does not match model"};
  }

  // K 和 P 分别检查，不要求两者相等，也不要求裁剪后的主点仍在图像内部。
  const auto & k = input.k;
  if (k[0] <= 0.0 || k[4] <= 0.0 || !near(k[1], 0) || !near(k[3], 0) ||
    !near(k[6], 0) || !near(k[7], 0) || !near(k[8], 1))
  {
    return {Code::invalid_intrinsics, "K must have positive focal lengths and canonical pinhole structure"};
  }
  if (!rotation(input.r)) {
    return {Code::invalid_rectification, "R must be a finite proper rotation (tolerance 1e-5)"};
  }
  const auto & p = input.p;
  if (p[0] <= 0.0 || p[5] <= 0.0 || !near(p[1], 0) || !near(p[4], 0) ||
    !near(p[8], 0) || !near(p[9], 0) || !near(p[10], 1) || !near(p[11], 0))
  {
    return {Code::invalid_projection, "P must have positive focal lengths and canonical projection structure"};
  }

  // 第二段：ROI 坐标始终是未 binning 的完整分辨率；减法检查避免 uint32 加法溢出。
  const auto & roi = input.roi;
  std::uint32_t width = roi.width;
  std::uint32_t height = roi.height;
  if (width == 0 && height == 0 && roi.x_offset == 0 && roi.y_offset == 0) {
    width = input.width;
    height = input.height;
  } else if (width == 0 || height == 0 || roi.x_offset >= input.width ||
    roi.y_offset >= input.height || width > input.width - roi.x_offset ||
    height > input.height - roi.y_offset)
  {
    return {Code::invalid_roi, "ROI must be default full frame or a non-empty window inside calibration dimensions"};
  }
  const auto bx = input.binning_x == 0 ? 1U : input.binning_x;
  const auto by = input.binning_y == 0 ? 1U : input.binning_y;
  if (width % bx != 0 || height % by != 0) {
    return {Code::unsupported_binning, "ROI dimensions must divide exactly by binning; driver rounding is not assumed"};
  }
  return {Code::none, "", width / bx, height / by};
}

}  // namespace

const char * cameraInfoErrorCodeName(CameraInfoErrorCode code) noexcept
{
  switch (code) {
    case CameraInfoErrorCode::none: return "none";
    case CameraInfoErrorCode::invalid_topic: return "invalid_topic";
    case CameraInfoErrorCode::invalid_frame: return "invalid_frame";
    case CameraInfoErrorCode::invalid_stamp: return "invalid_stamp";
    case CameraInfoErrorCode::invalid_dimensions: return "invalid_dimensions";
    case CameraInfoErrorCode::uncalibrated: return "uncalibrated";
    case CameraInfoErrorCode::nonfinite_parameters: return "nonfinite_parameters";
    case CameraInfoErrorCode::unsupported_distortion_model: return "unsupported_distortion_model";
    case CameraInfoErrorCode::invalid_distortion: return "invalid_distortion";
    case CameraInfoErrorCode::invalid_intrinsics: return "invalid_intrinsics";
    case CameraInfoErrorCode::invalid_rectification: return "invalid_rectification";
    case CameraInfoErrorCode::invalid_projection: return "invalid_projection";
    case CameraInfoErrorCode::invalid_roi: return "invalid_roi";
    case CameraInfoErrorCode::unsupported_binning: return "unsupported_binning";
  }
  return "unknown";
}

const char * cameraInfoMatchCodeName(CameraInfoMatchCode code) noexcept
{
  switch (code) {
    case CameraInfoMatchCode::matched: return "matched";
    case CameraInfoMatchCode::invalid_camera_info: return "invalid_camera_info";
    case CameraInfoMatchCode::invalid_options: return "invalid_options";
    case CameraInfoMatchCode::invalid_image: return "invalid_image";
    case CameraInfoMatchCode::topic_mismatch: return "topic_mismatch";
    case CameraInfoMatchCode::frame_mismatch: return "frame_mismatch";
    case CameraInfoMatchCode::size_mismatch: return "size_mismatch";
    case CameraInfoMatchCode::unsupported_rectified_roi: return "unsupported_rectified_roi";
    case CameraInfoMatchCode::time_mismatch: return "time_mismatch";
  }
  return "unknown";
}

CameraInfoResult decodeCameraInfo(CameraInfoData input)
{
  const auto validation = validate(input);
  if (validation.code != CameraInfoErrorCode::none) {
    return {validation.code, validation.message, std::nullopt};
  }
  return {CameraInfoErrorCode::none, "", std::move(input)};
}

CameraInfoMatchResult checkCameraInfoImage(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options)
{
  using Code = CameraInfoMatchCode;
  CameraInfoMatchResult result;
  const auto fail = [&result](Code code, const char * message) {
      result.code = code;
      result.message = message;
      return result;
    };
  const auto validation = validate(info);
  if (validation.code != CameraInfoErrorCode::none) {
    result.calibration_code = validation.code;
    return fail(Code::invalid_camera_info, validation.message);
  }
  result.expected_width = validation.image_width;
  result.expected_height = validation.image_height;
  const auto & binding = options.binding;
  if (!validTopic(binding.camera_info_topic) || !validTopic(binding.image_topic) ||
    !validFrame(binding.optical_frame) || binding.basis.empty() ||
    (binding.image_domain != CameraImageDomain::raw &&
    binding.image_domain != CameraImageDomain::rectified) ||
    (options.time_policy != CameraInfoTimePolicy::synchronized &&
    options.time_policy != CameraInfoTimePolicy::static_calibration))
  {
    return fail(Code::invalid_options, "Explicit topics, optical frame, domain, binding basis and time policy are required");
  }
  if (!validTopic(image.topic) || !validFrame(image.geometry.frame_id) ||
    image.geometry.width == 0 || image.geometry.height == 0 || image.stamp.nanosec >= 1000000000U)
  {
    return fail(Code::invalid_image, "Image reference has invalid topic, frame, dimensions or stamp");
  }
  if (info.topic != binding.camera_info_topic || image.topic != binding.image_topic) {
    return fail(Code::topic_mismatch, "Actual topics do not match the configured stream binding");
  }
  if (info.frame_id != binding.optical_frame || image.geometry.frame_id != binding.optical_frame) {
    return fail(Code::frame_mismatch, "CameraInfo and image must use the declared optical frame");
  }
  if (binding.image_domain == CameraImageDomain::rectified && info.roi.do_rectify) {
    return fail(Code::unsupported_rectified_roi, "Rectified ROI requires geometric ROI transformation before matching");
  }
  if (image.geometry.width != result.expected_width || image.geometry.height != result.expected_height) {
    return fail(Code::size_mismatch, "Image dimensions differ from ROI/binning-adjusted calibration grid");
  }

  // 固定标定是明确的流配置承诺；它跳过时差，不伪造 matched 时间结果。
  if (options.time_policy == CameraInfoTimePolicy::static_calibration) {
    result.used_static_calibration = true;
  } else {
    result.timing = checkSensorTimes(image.stamp, info.stamp, options.time);
    if (!result.timing->matched()) {
      return fail(Code::time_mismatch,
        "CameraInfo sampling time/clock check failed; inspect timing.code");
    }
  }
  result.code = Code::matched;
  return result;
}

}  // namespace vln_memory
