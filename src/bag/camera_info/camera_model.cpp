#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <algorithm>
#include <cmath>
#include <utility>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace vln_memory
{
namespace
{
/** @brief 仅支持独立单相机的校正像素；双目平移需要额外坐标语义。 */
bool supportedProjection(const CameraModel & model)
{
  return model.image_domain != CameraImageDomain::rectified ||
         (model.p[3] == 0.0 && model.p[7] == 0.0);
}
/** @brief 检查公开可变模型；K/P 已在实际网格中，不重复应用 ROI。 */
bool validModel(const CameraModel & model)
{
  if (model.image_domain != CameraImageDomain::raw &&
    model.image_domain != CameraImageDomain::rectified)
  {
    return false;
  }
  CameraInfoData info;
  info.topic = "/effective_camera_info";
  info.frame_id = model.frame_id;
  info.width = model.width;
  info.height = model.height;
  info.distortion_model = model.distortion_model;
  info.d = model.d;
  info.k = model.k;
  info.r = model.r;
  info.p = model.p;
  return decodeCameraInfo(std::move(info)).ok();
}
/** @brief 先裁剪后 binning；原始矩阵与参数均不修改。 */
void adjustGrid(CameraModel & model, const CameraInfoData & info)
{
  const double bx = info.binning_x == 0 ? 1.0 : static_cast<double>(info.binning_x);
  const double by = info.binning_y == 0 ? 1.0 : static_cast<double>(info.binning_y);
  model.k[2] -= info.roi.x_offset;
  model.k[5] -= info.roi.y_offset;
  model.p[2] -= info.roi.x_offset;
  model.p[6] -= info.roi.y_offset;
  for (std::size_t column = 0; column < 3; ++column) {
    model.k[column] /= bx;
    model.k[3 + column] /= by;
  }
  for (std::size_t column = 0; column < 4; ++column) {
    model.p[column] /= bx;
    model.p[4 + column] /= by;
  }
}
/** @brief 将 z=1 的归一化射线按 raw 模型正投影，输出有限的浮点像素。 */
std::optional<cv::Point2d> projectRawRay(const CameraModel & model, const cv::Point2d & ray)
{
  if (!std::isfinite(ray.x) || !std::isfinite(ray.y)) {return std::nullopt;}
  const cv::Matx33d k(model.k.data());
  const cv::Mat d(model.d, true);
  std::vector<cv::Point2d> projected;
  if (model.distortion_model == "equidistant") {
    cv::fisheye::distortPoints(std::vector<cv::Point2d>{ray}, projected, k, d);
  } else {
    const std::vector<cv::Point3d> points{{ray.x, ray.y, 1.0}};
    cv::projectPoints(points, cv::Vec3d::all(0), cv::Vec3d::all(0), k, d, projected);
  }
  if (projected.size() != 1 || !std::isfinite(projected[0].x) || !std::isfinite(projected[0].y)) {
    return std::nullopt;
  }
  return projected[0];
}

/** @brief 用 OpenCV 逆畸变，并正向回投影核验残差；失败不返回近似假射线。 */
std::optional<cv::Point2d> undistortPixel(const CameraModel & model, double u, double v)
{
  const cv::Matx33d k(model.k.data());
  const cv::Mat d(model.d, true);
  const std::vector<cv::Point2d> input{{u, v}};
  std::vector<cv::Point2d> output;
  if (model.distortion_model == "equidistant") {
    cv::fisheye::undistortPoints(input, output, k, d);
  } else {
    cv::undistortPoints(input, output, k, d, cv::noArray(), cv::noArray(),
      cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 1e-12));
  }
  if (output.size() != 1 || !std::isfinite(output[0].x) || !std::isfinite(output[0].y)) {
    return std::nullopt;
  }
  const auto projected = projectRawRay(model, output[0]);
  if (!projected || std::hypot(projected->x - u, projected->y - v) > 1e-4) {
    return std::nullopt;
  }
  return output[0];
}
/** @brief 校正像素经 P 的内参反投影，用 R 转置返回原相机光学坐标。 */
std::optional<std::array<double, 3>> rectifiedRay(const CameraModel & model, double u, double v)
{
  const std::array<double, 3> rectified{
    (u - model.p[2]) / model.p[0], (v - model.p[6]) / model.p[5], 1.0};
  std::array<double, 3> original{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      original[row] += model.r[column * 3 + row] * rectified[column];
    }
  }
  if (!std::isfinite(original[2]) || original[2] <= 0.0) {return std::nullopt;}
  original[0] /= original[2];
  original[1] /= original[2];
  original[2] = 1.0;
  if (!std::isfinite(original[0]) || !std::isfinite(original[1])) {return std::nullopt;}
  return original;
}
}  // namespace

/** @brief 返回不依赖异常文本的稳定几何状态名，无分配。 */
const char * cameraGeometryCodeName(CameraGeometryCode code) noexcept
{
  switch (code) {
    case CameraGeometryCode::ready: return "ready";
    case CameraGeometryCode::camera_mismatch: return "camera_mismatch";
    case CameraGeometryCode::unsupported_projection: return "unsupported_projection";
    case CameraGeometryCode::invalid_model: return "invalid_model";
    case CameraGeometryCode::invalid_pixel: return "invalid_pixel";
    case CameraGeometryCode::numerical_failure: return "numerical_failure";
    case CameraGeometryCode::point_behind_camera: return "point_behind_camera";
  }
  return "unknown";
}
/** @brief 统一匹配和标定解释后生成独立模型，失败不填默认内参。 */
CameraModelResult buildCameraModel(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options)
{
  const auto match = checkCameraInfoImage(info, image, options);
  if (!match.matched()) {
    return {CameraGeometryCode::camera_mismatch, match.message, std::nullopt};
  }
  const auto normalized = normalizeCameraInfo(info, options.compatibility);
  if (!normalized.ok()) {
    return {CameraGeometryCode::invalid_model, normalized.message, std::nullopt};
  }
  const auto & data = *normalized.info;
  CameraModel model;
  model.width = image.width;
  model.height = image.height;
  model.frame_id = image.frame_id;
  model.image_domain = options.image_domain;
  model.distortion_model = data.distortion_model;
  model.d = data.d;
  model.k = data.k;
  model.r = data.r;
  model.p = data.p;
  model.normalization_applied = normalized.normalization_applied;
  adjustGrid(model, data);
  if (!validModel(model)) {
    return {CameraGeometryCode::invalid_model, "Invalid adjusted intrinsics", std::nullopt};
  }
  if (!supportedProjection(model)) {
    return {CameraGeometryCode::unsupported_projection,
      "Rectified stereo Tx/Ty needs an explicit projection coordinate frame", std::nullopt};
  }
  return {CameraGeometryCode::ready, "", std::move(model)};
}
/** @brief 验证模型和像素；OpenCV 数值失败返回稳定码，内存异常继续向外传播。 */
CameraRayResult pixelToCameraRay(const CameraModel & model, double u, double v)
{
  if (!validModel(model)) {
    return {CameraGeometryCode::invalid_model, "Invalid effective model", std::nullopt};
  }
  if (!supportedProjection(model)) {
    return {CameraGeometryCode::unsupported_projection, "Unsupported stereo projection", std::nullopt};
  }
  if (!std::isfinite(u) || !std::isfinite(v) || u < 0 || v < 0 ||
    u >= model.width || v >= model.height)
  {
    return {CameraGeometryCode::invalid_pixel, "Pixel outside actual image grid", std::nullopt};
  }
  std::optional<std::array<double, 3>> ray;
  try {
    if (model.image_domain == CameraImageDomain::raw) {
      const auto point = undistortPixel(model, u, v);
      if (point) {ray = std::array<double, 3>{point->x, point->y, 1.0};}
    } else {
      ray = rectifiedRay(model, u, v);
    }
  } catch (const cv::Exception & error) {
    if (error.code == cv::Error::StsNoMem) {throw;}
    return {CameraGeometryCode::numerical_failure, "OpenCV could not invert pixel", std::nullopt};
  }
  if (!ray) {
    return {CameraGeometryCode::numerical_failure, "No verified forward-facing ray", std::nullopt};
  }
  return {CameraGeometryCode::ready, "", ray};
}

/** @brief 应用实际 raw 或 rectified 模型并检查相机前方和图像边界。 */
CameraPixelResult cameraPointToPixel(
  const CameraModel & model, const std::array<double, 3> & point_camera_m)
{
  using Code = CameraGeometryCode;
  if (!validModel(model)) {return {Code::invalid_model, std::nullopt};}
  if (!supportedProjection(model)) {return {Code::unsupported_projection, std::nullopt};}
  if (!std::all_of(point_camera_m.begin(), point_camera_m.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return {Code::numerical_failure, std::nullopt};
  }
  if (point_camera_m[2] <= 0.0) {return {Code::point_behind_camera, std::nullopt};}

  cv::Point2d pixel;
  if (model.image_domain == CameraImageDomain::raw) {
    const double z_m = point_camera_m[2];
    try {
      const auto projected = projectRawRay(
        model, {point_camera_m[0] / z_m, point_camera_m[1] / z_m});
      if (!projected) {return {Code::numerical_failure, std::nullopt};}
      pixel = *projected;
    } catch (const cv::Exception & error) {
      if (error.code == cv::Error::StsNoMem) {throw;}
      return {Code::numerical_failure, std::nullopt};
    }
  } else {
    const cv::Matx33d r(model.r.data());
    const cv::Vec3d rectified = r * cv::Vec3d(point_camera_m.data());
    if (!std::isfinite(rectified[2])) {return {Code::numerical_failure, std::nullopt};}
    if (rectified[2] <= 0.0) {return {Code::point_behind_camera, std::nullopt};}
    pixel.x = model.p[0] * (rectified[0] / rectified[2]) + model.p[2];
    pixel.y = model.p[5] * (rectified[1] / rectified[2]) + model.p[6];
  }
  if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) {
    return {Code::numerical_failure, std::nullopt};
  }
  if (pixel.x < 0.0 || pixel.y < 0.0 || pixel.x >= model.width || pixel.y >= model.height) {
    return {Code::invalid_pixel, std::nullopt};
  }
  return {Code::ready, std::array<double, 2>{pixel.x, pixel.y}};
}
}  // namespace vln_memory
