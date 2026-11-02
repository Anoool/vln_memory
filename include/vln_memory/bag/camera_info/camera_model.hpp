#pragma once
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <vln_memory/bag/camera_info/camera_info.hpp>

namespace vln_memory
{
/** @brief 实际图像的拥有型模型；K/P 已应用 ROI/binning，R 保留原始校正旋转。 */
struct CameraModel
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string frame_id;
  CameraImageDomain image_domain{CameraImageDomain::unspecified};
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};
  bool normalization_applied{false};
};
enum class CameraGeometryCode
{
  ready, camera_mismatch, unsupported_projection, invalid_model, invalid_pixel, numerical_failure
};
struct CameraModelResult
{
  CameraGeometryCode code{CameraGeometryCode::invalid_model};
  std::string message;
  std::optional<CameraModel> model;
  /** @brief 仅表示模型生成成功，不包含注册、TF 或物理精度验收。 */
  bool ok() const noexcept {return code == CameraGeometryCode::ready && model.has_value();}
};
struct CameraRayResult
{
  CameraGeometryCode code{CameraGeometryCode::invalid_model};
  std::string message;
  // 原始 frame_id 光学坐标中 z=1 的方向；不是单位向量，也不含深度。
  std::optional<std::array<double, 3>> ray;
  /** @brief 数值求解与像素检查成功才返回可用方向。 */
  bool ok() const noexcept {return code == CameraGeometryCode::ready && ray.has_value();}
};
/** @brief 返回稳定静态状态名，未知枚举为 unknown，无需释放。 */
const char * cameraGeometryCodeName(CameraGeometryCode code) noexcept;
/** @brief 绑定/时间/网格检查后生成模型；不修改图像、原始标定或时间。 */
CameraModelResult buildCameraModel(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options);
/** @brief 实际像素转原光学 frame 的 z=1 射线；raw 去畸变，rectified 逆转 R。 */
CameraRayResult pixelToCameraRay(const CameraModel & model, double u, double v);
}  // namespace vln_memory
