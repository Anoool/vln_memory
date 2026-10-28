#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vln_memory/sensor_types.hpp"

namespace vln_memory
{

/**
 * @brief 原始图像的只读借用视图，字段对应 sensor_msgs/msg/Image。
 * @note data、encoding、frame_id 必须在 decodeRgb 返回前保持有效且不被并发修改。
 *       核心不会保存任何借用引用。step 包含行尾填充，data_size 必须等于 step*height。
 */
struct RgbImageView
{
  // 采样身份：保留来源时间和光学帧，不在 RGB 层补定位信息。
  SensorStamp stamp;
  std::string_view frame_id;
  // 输入布局：step 是一整行的字节数，可能大于有效像素所需长度。
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string_view encoding;
  std::uint32_t step{0};
  std::uint8_t is_bigendian{0};
  // 借用的缓冲区与长度必须来自同一张图像，调用期间由上层保持有效。
  const std::uint8_t * data{nullptr};
  std::size_t data_size{0};
};

/** @brief 单帧处理上限；约束本次转换，不是 bag 反序列化或进程总内存上限。 */
struct RgbLimits
{
  // 像素上限约束标准输出大小；输入字节上限同时计入源行填充。
  std::size_t max_pixels{16U * 1024U * 1024U};
  std::size_t max_input_bytes{128U * 1024U * 1024U};
};

/**
 * @brief 拥有自身数据的标准 RGB 帧，适合后续保存 PNG 或与深度配对。
 * @note pixels 按行紧密排列为 R,G,B，无填充；大小为 step*height，step=width*3。
 *       不缩放、不裁剪，因此像素坐标与输入一致；源时间和 frame_id 原样保留。
 *       空 frame_id 也保留，由后续 TF/组装层决定该帧是否可定位。
 */
struct RgbFrame
{
  // 以下字段描述标准输出，width/height 和采样身份与输入一致。
  SensorStamp stamp;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding{"rgb8"};
  std::uint32_t step{0};
  // 源格式单独保留，供后续导出和排查使用，不能当作输出像素格式。
  std::string source_encoding;
  std::uint32_t source_step{0};
  bool source_is_bigendian{false};
  // 独立持有的数据，可在源 ROS 消息销毁后交给保存或同步模块。
  std::vector<std::uint8_t> pixels;
};

/** @brief 可预期的输入错误；系统资源异常仍通过标准 C++ 异常上报。 */
enum class RgbErrorCode
{
  none,
  invalid_limits,
  invalid_dimensions,
  invalid_stamp,
  unsupported_encoding,
  invalid_endianness,
  invalid_step,
  size_overflow,
  size_limit_exceeded,
  invalid_data
};

/** @brief 成功时 frame 有值且 code=none；失败时 frame 为空、message 说明原因。 */
struct RgbResult
{
  RgbErrorCode code{RgbErrorCode::none};
  std::string message;
  std::optional<RgbFrame> frame;

  /**
   * @brief 调用者须先检查 ok()，再访问 frame。
   * @note 默认构造尚无图像，code=none 本身不足以说明处理成功。
   */
  bool ok() const noexcept {return code == RgbErrorCode::none && frame.has_value();}
};

/** @brief 稳定的错误码名称，供未来 JSONL 报告层使用。 */
const char * rgbErrorCodeName(RgbErrorCode code) noexcept;

/**
 * @brief 校验并把 rgb8/bgr8/mono8 转成独立持有的紧密 rgb8。
 * @param input 未压缩原始图像；输入内存仍由调用者拥有。
 * @param limits 正数的单帧资源上限。
 * @return 输入错误返回结构化失败，无部分成功帧；成功帧不再依赖输入生命周期。
 * @note 三种支持编码均为 8 位通道，大小端不影响像素值，但标志仅允许 0/1。
 *       本函数无共享可变状态，可并发调用；分配失败等系统异常不转换为坏帧。
 */
RgbResult decodeRgb(const RgbImageView & input, const RgbLimits & limits = {});

}  // namespace vln_memory
