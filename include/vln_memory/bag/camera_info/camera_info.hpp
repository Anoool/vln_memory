#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <vln_memory/bag/common/sensor_time.hpp>

/**
 * @file
 * CameraInfo 契约：保留完整来源标定，再验证它与指定图像候选的绑定。
 * 校验结果是模型/元数据的一致性，不包含标定版本检索、去畸变或有效内参求解。
 */
namespace vln_memory
{

/** @brief 完整未 binning 像素坐标中的 ROI；原值保留，不覆盖 ROS 的零值约定。 */
struct CameraRoi
{
  std::uint32_t x_offset{0};
  std::uint32_t y_offset{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool do_rectify{false};
};

/**
 * @brief 独立持有的 CameraInfo 字段与真实来源话题，矩阵均为 row-major。
 * @note topic 由 bag 元数据/订阅者提供，消息本身没有 topic 字段。
 *       width/height 是标定分辨率；K 用于原始图，P 用于校正图，二者不能混用。
 *       保留原始 ROI/binning、时间与双目 Tx/Ty，不自行缩放图像或改写标定。
 */
struct CameraInfoData
{
  SensorStamp stamp;
  std::string topic;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{};
  std::array<double, 12> p{};
  std::uint32_t binning_x{0};
  std::uint32_t binning_y{0};
  CameraRoi roi;
};

enum class CameraInfoCompatibility {strict, plumb_bob_zero_tail_d8};

enum class CameraInfoErrorCode
{
  none, invalid_topic, invalid_frame, invalid_stamp, invalid_dimensions, uncalibrated,
  nonfinite_parameters, unsupported_distortion_model, invalid_distortion,
  invalid_intrinsics, invalid_rectification, invalid_projection, invalid_roi, unsupported_binning, invalid_options
};

/** @brief 只返回完整可用的标定；失败不返回半成功数据，原始消息由上层保留为证据。 */
struct CameraInfoResult
{
  CameraInfoErrorCode code{CameraInfoErrorCode::none};
  std::string message;
  std::optional<CameraInfoData> info;
  bool normalization_applied{false};  // 原始数据由调用方保存；这里只标记派生 D5。
  /** @brief 仅完整校验通过且存在拥有型标定时返回 true，不代表物理精度已验证。 */
  bool ok() const noexcept
  {
    return code == CameraInfoErrorCode::none && info.has_value();
  }
};

enum class CameraImageDomain {unspecified, raw, rectified};
enum class CameraInfoTimePolicy {synchronized, static_calibration};

/**
 * @brief 指定图像的扁平元数据；由 RgbFrame/DepthFrame 的公开字段和真实话题逐项填写。
 * @note 独立持有字符串，stamp 保留原始采样时刻；不引用源图像或复制像素。
 */
struct CameraImageReference
{
  std::string topic;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string frame_id;
  SensorStamp stamp;
};

/**
 * @brief 标定与图像的显式匹配配置；绑定和时间参数直接位于同一层。
 * @note optical_frame 由设备配置/URDF 声明，名称不证明实际光学轴或安装方向。
 *       binding_basis 记录配置/标定依据；固定策略还须证明该段内模型/ROI/binning 不变。
 */
struct CameraInfoMatchOptions
{
  std::string camera_info_topic;
  std::string image_topic;
  std::string optical_frame;
  std::string binding_basis;
  CameraImageDomain image_domain{CameraImageDomain::unspecified};
  CameraInfoTimePolicy time_policy{CameraInfoTimePolicy::synchronized};
  SensorClockRelation clock_relation{SensorClockRelation::unknown};
  // 非负整数纳秒，0 表示严格相等；固定标定策略不检查采样时差。
  std::int64_t max_delta_ns{0};
  CameraInfoCompatibility compatibility{CameraInfoCompatibility::strict};
};

enum class CameraInfoMatchCode
{
  matched, invalid_camera_info, invalid_options, invalid_image, topic_mismatch,
  frame_mismatch, size_mismatch, unsupported_rectified_roi, time_mismatch
};

struct CameraInfoMatchResult
{
  CameraInfoMatchCode code{CameraInfoMatchCode::invalid_options};
  std::string message;
  // invalid_camera_info 时保留具体标定错误，报告层无需解析 message 文本。
  CameraInfoErrorCode calibration_code{CameraInfoErrorCode::none};
  std::uint32_t expected_width{0};
  std::uint32_t expected_height{0};
  // 固定标定策略不比较采样时差，timing 为空；这不表示两条消息时间相同。
  std::optional<SensorTimeCheck> timing;
  bool used_static_calibration{false};
  bool normalization_applied{false};
  /** @brief 只确认指定候选的关联条件通过，不代表已生成有效内参或完成深度注册。 */
  bool matched() const noexcept
  {
    return code == CameraInfoMatchCode::matched;
  }
};

/** @brief 返回标定错误的稳定静态名称；未知枚举返回 unknown，无所有权转移。 */
const char * cameraInfoErrorCodeName(CameraInfoErrorCode code) noexcept;
/** @brief 返回匹配状态的稳定静态名称；未知枚举返回 unknown，无所有权转移。 */
const char * cameraInfoMatchCodeName(CameraInfoMatchCode code) noexcept;

/**
 * @brief 校验后独立持有 CameraInfo；传左值会复制，传 std::move(data) 可转移所有权。
 * @note 支持 plumb_bob(5)、rational_polynomial(8)、equidistant(4)。
 *       校验数值/结构一致性，不证明物理精度；系统分配异常通过 C++ 异常上报。
 */
CameraInfoResult decodeCameraInfo(CameraInfoData input);

/** @brief 按显式策略派生后严格校验；仅零尾 D8 可转 D5，原输入不被改写。 */
CameraInfoResult normalizeCameraInfo(
  CameraInfoData input, CameraInfoCompatibility compatibility = CameraInfoCompatibility::strict);

/** @brief 返回静态策略名，未知枚举返回 unknown，无需释放。 */
const char * cameraInfoCompatibilityName(CameraInfoCompatibility compatibility) noexcept;

/**
 * @brief 核对标定与指定图像的话题、光学 frame、ROI/binning 后尺寸和时间策略。
 * @note 重新校验公开可变的 info，避免手动构造或修改绕过检查。
 *       不选取消息、不重投影、不证明 Depth 已注册；rectified + roi.do_rectify
 *       需要计算校正后的 ROI，当前明确拒绝，防止用原始 ROI 冒充校正网格。
 */
CameraInfoMatchResult checkCameraInfoImage(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options);

}  // namespace vln_memory
