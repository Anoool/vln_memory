#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vln_memory/bag/common/sensor_types.hpp"

namespace vln_memory
{

/** @brief 源值的单位必须由设备配置/标定明确给出，不从 encoding 静默推断。 */
enum class DepthUnit {unspecified, meters, millimeters};
/** @brief optical_z 是光学轴深度；radial_range 需要内参才能转换，当前不接受。 */
enum class DepthConvention {unspecified, optical_z, radial_range};
enum class DepthNumericType {unknown, uint16, float32};
enum class DepthAlignment {unknown, not_aligned, aligned_to_rgb};

/**
 * @brief 上游对 RGB 注册的声明，不是本模块从像素或尺寸推断出的结论。
 * @note aligned_to_rgb 必须提供匹配的 RGB 网格和非空依据，例如注册流配置 ID。
 *       unknown/not_aligned 的 rgb_grid 必须为空；basis 可记录未确定/未注册原因。
 */
struct DepthAlignmentInfo
{
  DepthAlignment state{DepthAlignment::unknown};
  ImageGeometry rgb_grid;
  std::string basis;
};

/**
 * @brief 借用未压缩单通道 Image 数据，所有引用只需在 decodeDepth 调用期间有效。
 * @note 调用期间不得并发修改输入；成功输出独立拥有数据，不保留借用引用。
 */
struct DepthImageView
{
  // 原始采样身份，不转换时钟域，也不改写光学帧。
  SensorStamp stamp;
  std::string_view frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string_view encoding;
  // step 包含行填充；data_size 必须严格等于 step*height。
  std::uint32_t step{0};
  std::uint8_t is_bigendian{0};
  const std::uint8_t * data{nullptr};
  std::size_t data_size{0};
};

/** @brief 必填语义与可选范围过滤；默认对象缺少单位/深度含义，不能直接解码。 */
struct DepthOptions
{
  DepthUnit input_unit{DepthUnit::unspecified};
  DepthConvention convention{DepthConvention::unspecified};
  DepthAlignmentInfo alignment;
  // 有效像素必须正数，且位于 [min_depth_m, max_depth_m]；默认不截断正有限值。
  double min_depth_m{0.0};
  double max_depth_m{std::numeric_limits<double>::infinity()};
};

/** @brief 约束本次转换的像素数与源字节数，不限制此前 ROS 反序列化的内存。 */
struct DepthLimits
{
  std::size_t max_pixels{16U * 1024U * 1024U};
  std::size_t max_input_bytes{128U * 1024U * 1024U};
};

/** @brief 无效原因互斥计数；invalid_pixels 等于五类原因之和。 */
struct DepthStatistics
{
  std::size_t valid_pixels{0};
  std::size_t invalid_pixels{0};
  std::size_t zero_pixels{0};
  std::size_t negative_pixels{0};
  std::size_t nonfinite_pixels{0};
  std::size_t out_of_range_pixels{0};
  std::size_t unrepresentable_pixels{0};
  double valid_ratio{0.0};
};

/**
 * @brief 独立持有的 float32 米制 optical-z 深度，行优先、无填充。
 * @note 无效像素统一为 quiet NaN；全无效图像仍是成功解码，valid_ratio=0。
 *       不缩放、不填洞、不做配准；原始字节由未来 bag 层另外保留为证据。
 */
struct DepthFrame
{
  SensorStamp stamp;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding{"32FC1"};
  std::uint32_t step{0};
  DepthUnit unit{DepthUnit::meters};
  DepthConvention convention{DepthConvention::optical_z};

  // 源类型、单位与实际采用比例分别记录，防止后续重复换算毫米。
  std::string source_encoding;
  DepthNumericType source_numeric_type{DepthNumericType::unknown};
  DepthUnit source_unit{DepthUnit::unspecified};
  double source_scale_to_m{0.0};
  std::uint32_t source_step{0};
  bool source_is_bigendian{false};
  DepthAlignmentInfo alignment;
  double min_depth_m{0.0};
  double max_depth_m{std::numeric_limits<double>::infinity()};
  DepthStatistics statistics;
  std::vector<float> depth_m;
};

enum class DepthErrorCode
{
  none,
  // 布局、数据和资源约束。
  invalid_limits,
  invalid_dimensions,
  invalid_stamp,
  unsupported_encoding,
  invalid_endianness,
  invalid_step,
  size_overflow,
  size_limit_exceeded,
  invalid_data,
  // 解释数值所必需的语义，以及额外配置的一致性。
  missing_unit,
  unsupported_unit,
  missing_convention,
  unsupported_convention,
  invalid_range,
  invalid_alignment
};

/** @brief 成功时 code=none 且 frame 有值；调用者统一用 ok() 判断。 */
struct DepthResult
{
  DepthErrorCode code{DepthErrorCode::none};
  std::string message;
  std::optional<DepthFrame> frame;
  bool ok() const noexcept {return code == DepthErrorCode::none && frame.has_value();}
};

enum class DepthAlignmentStatus
{
  unknown,
  not_aligned,
  aligned,
  invalid_reference,
  inconsistent_declaration,
  size_mismatch,
  frame_mismatch
};

/** @brief aligned 仅表示声明与实际 RGB 元数据一致，不证明标定或时间同步正确。 */
struct DepthAlignmentResult
{
  DepthAlignmentStatus status{DepthAlignmentStatus::unknown};
  std::string message;
  bool alignedToRgb() const noexcept {return status == DepthAlignmentStatus::aligned;}
};

/** @brief 稳定名称用于后续 JSONL 报告；不以数值枚举下标作为持久格式。 */
const char * depthErrorCodeName(DepthErrorCode code) noexcept;
const char * depthUnitName(DepthUnit unit) noexcept;
const char * depthNumericTypeName(DepthNumericType type) noexcept;
const char * depthAlignmentStatusName(DepthAlignmentStatus status) noexcept;

/**
 * @brief 校验 16UC1/32FC1，并按显式单位转为米制 optical-z。
 * @param input 调用期间只读且有效的原始图像视图。
 * @param options 必须指定 input_unit 与 optical_z；对齐声明与范围可选。
 * @return 输入错误返回空 frame；系统分配异常仍通过 C++ 异常上报。
 * @note 支持大小端和行填充，不使用未对齐的类型指针；无共享可变状态。
 */
DepthResult decodeDepth(
  const DepthImageView & input, const DepthOptions & options, const DepthLimits & limits = {});

/**
 * @brief 用真正配对的 RGB 网格检查 Depth 的上游注册声明。
 * @note 相同尺寸或 frame_id 不能自行产生 aligned 结论；本函数不做重投影、
 *       校准或时间同步。调用方可从 RgbFrame 的宽高/frame_id 构造 ImageGeometry。
 */
DepthAlignmentResult checkDepthRgbAlignment(const DepthFrame & depth, const ImageGeometry & rgb);

}  // namespace vln_memory
