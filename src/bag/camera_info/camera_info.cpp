#include <vln_memory/bag/camera_info/camera_info.hpp>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

#include <vln_memory/bag/common/sensor_names.hpp>

/**
 * @file
 * 先验证完整标定，再核对调用者给定的图像话题、光学帧、网格和时间策略。
 * 选择哪条标定由上层负责；这里不按“看起来相近”推断相机、时钟或标定版本。
 */
namespace vln_memory
{
namespace
{
/** @brief 检查非空 frame 名称，不允许前导 /、空白或控制字符；不推断实际光学轴。 */
bool isValidFrame(std::string_view frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(frame.begin(), frame.end(), [](char character) {
      return character <= ' ' || character == '\x7f';
    });
}

/** @brief 检查只读系数序列是否全为有限数；不复制输入，任一 NaN/Inf 返回 false。 */
template<typename Values>
bool hasFiniteValues(const Values & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

/** @brief 用绝对容差比较矩阵元素；默认 1e-9，旋转检查显式使用 1e-5。 */
bool isNear(double actual, double expected, double tolerance = 1e-9)
{
  return std::abs(actual - expected) <= tolerance;
}

/** @brief 校验 row-major R 的正交性和 det=+1；允许标定舍入，拒绝反射矩阵。 */
bool isRotationMatrix(const std::array<double, 9> & matrix)
{
  for (std::size_t first_row = 0; first_row < 3; ++first_row) {
    for (std::size_t second_row = 0; second_row < 3; ++second_row) {
      double dot_product = 0.0;
      for (std::size_t column = 0; column < 3; ++column) {
        dot_product += matrix[first_row * 3 + column] * matrix[second_row * 3 + column];
      }
      const double expected_dot_product = first_row == second_row ? 1.0 : 0.0;
      if (!isNear(dot_product, expected_dot_product, 1e-5)) {
        return false;
      }
    }
  }
  const double determinant =
    matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
    matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
    matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
  return isNear(determinant, 1.0, 1e-5);
}

/** @brief 内部扁平校验结果；尺寸只在全部标定检查通过后填写，无输入所有权。 */
struct CameraInfoValidation
{
  CameraInfoErrorCode code{CameraInfoErrorCode::none};
  const char * message{""};
  std::uint32_t expected_width{0};
  std::uint32_t expected_height{0};
};

/** @brief 核对畸变模型和精确系数数目；有限性已由入口检查，返回原有错误码。 */
CameraInfoValidation validateDistortion(const CameraInfoData & info)
{
  std::size_t expected_coefficients = 0;
  if (info.distortion_model == "plumb_bob") {
    expected_coefficients = 5;
  } else if (info.distortion_model == "rational_polynomial") {
    expected_coefficients = 8;
  } else if (info.distortion_model == "equidistant") {
    expected_coefficients = 4;
  } else {
    return {CameraInfoErrorCode::unsupported_distortion_model, "Unsupported or missing distortion model"};
  }
  if (info.d.size() != expected_coefficients) {
    return {CameraInfoErrorCode::invalid_distortion, "Distortion coefficient count does not match model"};
  }
  return {};
}

/** @brief 按 K、R、P 顺序校验矩阵结构；不要求 K=P，不改写双目平移或主点。 */
CameraInfoValidation validateMatrices(const CameraInfoData & info)
{
  using Code = CameraInfoErrorCode;
  const auto & intrinsics = info.k;
  if (intrinsics[0] <= 0.0 || intrinsics[4] <= 0.0 ||
    !isNear(intrinsics[1], 0) || !isNear(intrinsics[3], 0) ||
    !isNear(intrinsics[6], 0) || !isNear(intrinsics[7], 0) || !isNear(intrinsics[8], 1))
  {
    return {Code::invalid_intrinsics, "K must have positive focal lengths and canonical pinhole structure"};
  }
  // 当前要求完整 CameraInfo；原始图也不替空 R/P 补默认矩阵。
  if (!isRotationMatrix(info.r)) {
    return {Code::invalid_rectification, "R must be a finite proper rotation (tolerance 1e-5)"};
  }
  const auto & projection = info.p;
  if (projection[0] <= 0.0 || projection[5] <= 0.0 ||
    !isNear(projection[1], 0) || !isNear(projection[4], 0) ||
    !isNear(projection[8], 0) || !isNear(projection[9], 0) ||
    !isNear(projection[10], 1) || !isNear(projection[11], 0))
  {
    return {Code::invalid_projection, "P must have positive focal lengths and canonical projection structure"};
  }
  return {};
}

/** @brief 用原始 ROI/binning 推导期望网格；越界或不整除失败，不修改 K/P 或输入 ROI。 */
CameraInfoValidation validateImageGrid(const CameraInfoData & info)
{
  using Code = CameraInfoErrorCode;
  const auto & roi = info.roi;
  std::uint32_t roi_width = roi.width;
  std::uint32_t roi_height = roi.height;
  const bool full_frame_roi = roi_width == 0 && roi_height == 0 &&
    roi.x_offset == 0 && roi.y_offset == 0;
  if (full_frame_roi) {
    roi_width = info.width;
    roi_height = info.height;
  } else if (roi_width == 0 || roi_height == 0 || roi.x_offset >= info.width ||
    roi.y_offset >= info.height || roi_width > info.width - roi.x_offset ||
    roi_height > info.height - roi.y_offset)
  {
    // 先确认 offset 在范围内，再用减法检查窗口，避免 uint32 加法溢出。
    return {Code::invalid_roi,
      "ROI must be default full frame or a non-empty window inside calibration dimensions"};
  }
  const auto binning_x = info.binning_x == 0 ? 1U : info.binning_x;
  const auto binning_y = info.binning_y == 0 ? 1U : info.binning_y;
  if (roi_width % binning_x != 0 || roi_height % binning_y != 0) {
    return {Code::unsupported_binning,
      "ROI dimensions must divide exactly by binning; driver rounding is not assumed"};
  }
  return {Code::none, "", roi_width / binning_x, roi_height / binning_y};
}

/** @brief 顺序检查来源、数值、畸变、矩阵和网格；只读输入，保留首个失败及原始错误文本。 */
CameraInfoValidation validateCameraInfo(const CameraInfoData & info)
{
  using Code = CameraInfoErrorCode;
  if (!isResolvedTopicName(info.topic)) {
    return {Code::invalid_topic, "Expected a resolved absolute ROS topic"};
  }
  if (!isValidFrame(info.frame_id)) {
    return {Code::invalid_frame, "Expected a non-empty optical frame ID without a leading slash"};
  }
  if (info.stamp.nanosec >= 1000000000U) {
    return {Code::invalid_stamp, "Stamp nanosec must be less than 1000000000"};
  }
  if (info.width == 0 || info.height == 0) {
    return {Code::invalid_dimensions, "Calibration dimensions must be positive"};
  }
  if (!hasFiniteValues(info.k) || !hasFiniteValues(info.r) ||
    !hasFiniteValues(info.p) || !hasFiniteValues(info.d))
  {
    return {Code::nonfinite_parameters, "Calibration contains NaN or infinity"};
  }
  if (info.k[0] == 0.0) {
    return {Code::uncalibrated, "K[0]==0 denotes an uncalibrated camera"};
  }
  const auto distortion = validateDistortion(info);
  if (distortion.code != Code::none) {
    return distortion;
  }
  const auto matrices = validateMatrices(info);
  if (matrices.code != Code::none) {
    return matrices;
  }
  return validateImageGrid(info);
}

/** @brief 检查显式流绑定与策略；时钟/容差留到逐帧时间检查，固定策略不检查时差。 */
bool hasValidMatchOptions(const CameraInfoMatchOptions & options)
{
  const bool valid_image_domain = options.image_domain == CameraImageDomain::raw ||
    options.image_domain == CameraImageDomain::rectified;
  const bool valid_time_policy = options.time_policy == CameraInfoTimePolicy::synchronized ||
    options.time_policy == CameraInfoTimePolicy::static_calibration;
  return isResolvedTopicName(options.camera_info_topic) && isResolvedTopicName(options.image_topic) &&
         isValidFrame(options.optical_frame) && !options.binding_basis.empty() &&
         valid_image_domain && valid_time_policy;
}

/** @brief 核对候选图像的来源、frame、非零尺寸和 stamp 格式；不读取或复制像素。 */
bool hasValidImageReference(const CameraImageReference & image)
{
  return isResolvedTopicName(image.topic) && isValidFrame(image.frame_id) &&
         image.width > 0 && image.height > 0 && image.stamp.nanosec < 1000000000U;
}

}  // namespace

/** @brief 返回错误的稳定静态名称；未知枚举返回 unknown，调用方不负责释放。 */
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

/** @brief 返回匹配状态的稳定静态名称；未知枚举返回 unknown，调用方不负责释放。 */
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

/** @brief 校验按值输入后移动为拥有型结果；失败无部分标定，分配异常继续向外传播。 */
CameraInfoResult decodeCameraInfo(CameraInfoData input)
{
  const auto validation = validateCameraInfo(input);
  if (validation.code != CameraInfoErrorCode::none) {
    return {validation.code, validation.message, std::nullopt};
  }
  return {CameraInfoErrorCode::none, "", std::move(input)};
}

/** @brief 按标定、配置、图像、绑定、网格、时间的顺序检查，返回首个错误及已有诊断。 */
CameraInfoMatchResult checkCameraInfoImage(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options)
{
  using Code = CameraInfoMatchCode;
  CameraInfoMatchResult result;
  // 保留已计算的尺寸、标定细分错误与时间诊断，只设置当前匹配错误。
  const auto fail = [&result](Code code, const char * message) {
      result.code = code;
      result.message = message;
      return result;
    };
  // 公开结构可被调用者修改，所以不能仅凭“曾经 decode 成功”跳过当前数据检查。
  const auto validation = validateCameraInfo(info);
  if (validation.code != CameraInfoErrorCode::none) {
    result.calibration_code = validation.code;
    return fail(Code::invalid_camera_info, validation.message);
  }
  result.expected_width = validation.expected_width;
  result.expected_height = validation.expected_height;
  if (!hasValidMatchOptions(options)) {
    return fail(Code::invalid_options,
      "Explicit topics, optical frame, domain, binding basis and time policy are required");
  }
  if (!hasValidImageReference(image)) {
    return fail(Code::invalid_image, "Image reference has invalid topic, frame, dimensions or stamp");
  }
  // 从实际来源验证显式流绑定，再核对网格；尺寸相同的两台相机不能误关联。
  if (info.topic != options.camera_info_topic || image.topic != options.image_topic) {
    return fail(Code::topic_mismatch, "Actual topics do not match the configured stream binding");
  }
  if (info.frame_id != options.optical_frame || image.frame_id != options.optical_frame) {
    return fail(Code::frame_mismatch, "CameraInfo and image must use the declared optical frame");
  }
  if (options.image_domain == CameraImageDomain::rectified && info.roi.do_rectify) {
    return fail(Code::unsupported_rectified_roi,
      "Rectified ROI requires geometric ROI transformation before matching");
  }
  if (image.width != result.expected_width ||
    image.height != result.expected_height)
  {
    return fail(Code::size_mismatch, "Image dimensions differ from ROI/binning-adjusted calibration grid");
  }

  // 固定标定是明确的流配置承诺；它跳过时差，不伪造 matched 时间结果。
  if (options.time_policy == CameraInfoTimePolicy::static_calibration) {
    result.used_static_calibration = true;
  } else {
    const SensorTimeOptions time_options{options.clock_relation, options.max_delta_ns};
    result.timing = checkSensorTimes(image.stamp, info.stamp, time_options);
    if (!result.timing->matched()) {
      return fail(Code::time_mismatch,
        "CameraInfo sampling time/clock check failed; inspect timing.code");
    }
  }
  result.code = Code::matched;
  return result;
}

}  // namespace vln_memory
