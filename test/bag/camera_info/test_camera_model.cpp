#include <vln_memory/bag/camera_info/camera_model.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace vln_memory;
/** @brief Release 中也检查断言，失败给出可复现说明。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
/** @brief 比较独立已知答案，容差按归一化射线而非像素指定。 */
void near(double actual, double expected)
{
  expect(std::abs(actual - expected) < 1e-9, "Geometry differs from known answer");
}
/** @brief K 与 P 故意不同，避免 raw/rectified 误用在零畸变数据中漏检。 */
CameraInfoData calibration()
{
  CameraInfoData info;
  info.topic = "/info";
  info.frame_id = "optical";
  info.width = 640; info.height = 480;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {500, 0, 320, 0, 510, 240, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {400, 0, 300, 0, 0, 420, 230, 0, 0, 0, 1, 0};
  return info;
}
/** @brief 显式固定标定和图像域，时间不被改写或伪造为同步。 */
CameraInfoMatchOptions options()
{
  CameraInfoMatchOptions value;
  value.camera_info_topic = "/info"; value.image_topic = "/image";
  value.optical_frame = "optical"; value.binding_basis = "known generated model";
  value.image_domain = CameraImageDomain::raw;
  value.time_policy = CameraInfoTimePolicy::static_calibration;
  return value;
}
/** @brief 实际网格引用，不含像素所有权。 */
CameraImageReference image()
{
  return {"/image", 640, 480, "optical", {7, 0}};
}
/** @brief 校验显式 D8 范围、原始所有权、未知策略以及直接匹配/模型入口一致。 */
void compatibilityCases()
{
  auto info = calibration();
  info.d.resize(8, 0);
  const auto policy = CameraInfoCompatibility::plumb_bob_zero_tail_d8;
  expect(!decodeCameraInfo(info).ok(), "Strict decoder accepted D8");
  auto result = normalizeCameraInfo(info, policy);
  expect(result.ok() && result.normalization_applied && result.info->d.size() == 5 && info.d.size() == 8,
    "Bounded normalization failed or changed raw");
  auto config = options(); config.compatibility = policy;
  expect(checkCameraInfoImage(info, image(), config).matched(), "Direct matching rejected compatible D8");
  const auto model = buildCameraModel(info, image(), config);
  expect(model.ok() && model.model->normalization_applied, "Model lost normalization");
  info.d[7] = 1e-15;
  expect(!normalizeCameraInfo(info, policy).ok(), "Nonzero tail truncated");
  info.d[7] = std::numeric_limits<double>::quiet_NaN();
  expect(!normalizeCameraInfo(info, policy).ok(), "Nonfinite tail truncated");
  expect(!normalizeCameraInfo(calibration(), static_cast<CameraInfoCompatibility>(99)).ok(),
    "Unknown compatibility accepted");
}
/** @brief 比较 raw K、rectified P、R 逆变换及 ROI/binning 的独立数值答案。 */
void modelCases()
{
  auto info = calibration(); auto config = options(); auto reference = image();
  auto built = buildCameraModel(info, reference, config);
  expect(built.ok(), "Raw model failed");
  auto ray = pixelToCameraRay(*built.model, 400, 300);
  expect(ray.ok(), "Raw inverse failed"); near((*ray.ray)[0], 0.16); near((*ray.ray)[1], 60.0 / 510);
  config.image_domain = CameraImageDomain::rectified;
  built = buildCameraModel(info, reference, config);
  ray = pixelToCameraRay(*built.model, 400, 300);
  expect(ray.ok(), "Rectified inverse failed"); near((*ray.ray)[0], 0.25); near((*ray.ray)[1], 1.0 / 6);
  const double angle = 0.1;
  info.r = {std::cos(angle), 0, std::sin(angle), 0, 1, 0, -std::sin(angle), 0, std::cos(angle)};
  built = buildCameraModel(info, reference, config);
  ray = pixelToCameraRay(*built.model, 300, 230);
  expect(ray.ok(), "Rectification rotation failed"); near((*ray.ray)[0], -std::tan(angle));
  info = calibration(); config = options();
  info.roi = {100, 60, 320, 240, false}; info.binning_x = 2; info.binning_y = 3;
  reference.width = 160; reference.height = 80;
  built = buildCameraModel(info, reference, config);
  expect(built.ok(), "Cropped/binned model failed");
  near(built.model->k[0], 250); near(built.model->k[2], 110);
  near(built.model->k[4], 170); near(built.model->k[5], 60);
  expect(info.k[0] == 500 && info.roi.x_offset == 100, "Raw parameters overwritten");
  expect(pixelToCameraRay(*built.model, 160, 0).code == CameraGeometryCode::invalid_pixel,
    "Out of bounds pixel accepted");
  built.model->k[0] = 0;
  expect(pixelToCameraRay(*built.model, 0, 0).code == CameraGeometryCode::invalid_model,
    "Mutable model bypassed validation");
  info = calibration(); info.p[3] = -40; config.image_domain = CameraImageDomain::rectified;
  expect(buildCameraModel(info, image(), config).code == CameraGeometryCode::unsupported_projection,
    "Stereo projection silently treated as monocular");
  info = calibration(); info.roi.do_rectify = true;
  expect(!buildCameraModel(info, image(), config).ok(), "Uncomputed rectified ROI accepted");
}
/** @brief 用独立解析答案检查 raw 畸变和 rectified 正旋转的投影方向。 */
void forwardProjectionCases()
{
  auto info = calibration();
  auto config = options();
  info.d[0] = 0.1;
  auto built = buildCameraModel(info, image(), config);
  expect(built.ok(), "Fixture raw model failed");
  const auto raw = cameraPointToPixel(*built.model, {0.4, 0.2, 2.0});
  expect(raw.ok(), "Raw projection failed");
  // x=0.2, y=0.1, 1+k1*(x*x+y*y)=1.005。
  near((*raw.pixel_uv)[0], 420.5);
  near((*raw.pixel_uv)[1], 291.255);

  config.image_domain = CameraImageDomain::rectified;
  const double cosine = 1.0 / std::sqrt(1.04);
  const double sine = 0.2 * cosine;
  info.r = {cosine, 0, sine, 0, 1, 0, -sine, 0, cosine};
  built = buildCameraModel(info, image(), config);
  expect(built.ok(), "Fixture rectified model failed");
  const auto rectified = cameraPointToPixel(*built.model, {0, 0, 2});
  expect(rectified.ok(), "Rectified projection failed");
  near((*rectified.pixel_uv)[0], 380.0);
  near((*rectified.pixel_uv)[1], 230.0);
  expect(cameraPointToPixel(*built.model, {10, 0, 1}).code ==
    CameraGeometryCode::point_behind_camera, "Nonpositive rectified z accepted");
  expect(cameraPointToPixel(*built.model, {0, 0, 0}).code ==
    CameraGeometryCode::point_behind_camera, "Point at camera center accepted");
  auto unsupported = *built.model;
  unsupported.p[3] = -40;
  expect(cameraPointToPixel(unsupported, {0, 0, 2}).code ==
    CameraGeometryCode::unsupported_projection, "Stereo offset silently ignored");
}

/** @brief 三种畸变用独立正向公式生成像素，验证逆解而不是只测试零畸变。 */
void distortionCases()
{
  for (const auto & name : {"plumb_bob", "rational_polynomial", "equidistant"}) {
    auto info = calibration(); info.distortion_model = name;
    double x = 0.2, y = 0.1;
    const double radius2 = x * x + y * y;
    double scale = 1.0 + 0.1 * radius2;
    info.d = {0.1, 0, 0, 0, 0};
    if (info.distortion_model == "rational_polynomial") {info.d.resize(8, 0);}
    if (info.distortion_model == "equidistant") {
      info.d = {0.1, 0, 0, 0};
      const double radius = std::sqrt(radius2), theta = std::atan(radius);
      scale = theta * (1.0 + 0.1 * theta * theta) / radius;
    }
    const double u = 500 * x * scale + 320, v = 510 * y * scale + 240;
    const auto built = buildCameraModel(info, image(), options());
    expect(built.ok(), "Distorted model failed");
    const auto ray = pixelToCameraRay(*built.model, u, v);
    expect(ray.ok(), "Distorted inverse failed"); near((*ray.ray)[0], x); near((*ray.ray)[1], y);
  }
}
}
/** @brief 覆盖兼容、网格、图像域、逆畸变和拒绝路径，失败返回非零。 */
int main()
{
  try {
    compatibilityCases(); modelCases(); distortionCases(); forwardProjectionCases();
    std::cout << "Camera models, raw/rectified rays and bounded D8 passed\n";
    return 0;
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
