#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vln_memory/bag/common/sensor_time.hpp"

namespace vln_memory
{

/** @brief 拥有型消息引用，无像素；order 是流内唯一的原始稳定扫描顺序。 */
struct RgbdFrameRef
{
  std::string message_id;
  std::uint64_t message_order{0};
  SensorStamp stamp;
};

/** @brief 一个明确时钟段内的单一图像话题；输入排列可乱序，来源顺序不变。 */
struct RgbdStreamIndex
{
  std::string topic;
  std::string clock_id;
  std::string segment_id;
  std::vector<RgbdFrameRef> frames;
};

/** @brief 显式同钟依据、整数纳秒容差及两流合计条数上限；不校正时钟。 */
struct RgbdSyncOptions
{
  SensorClockRelation clock_relation{SensorClockRelation::unknown};
  std::string clock_basis;
  std::int64_t max_delta_ns{0};
  std::size_t max_frames{1000000};
};

/** @brief 两个独立拥有的来源引用，delta_ns=Depth-RGB；不代表空间注册。 */
struct RgbdPair
{
  RgbdFrameRef rgb;
  RgbdFrameRef depth;
  std::int64_t delta_ns{0};
};

enum class RgbdUnmatchedReason {empty_depth, outside_tolerance, unavailable_by_policy};

/** @brief 未配对 RGB 及可机器读取原因；不等同于图像解码失败。 */
struct RgbdUnmatched
{
  RgbdFrameRef rgb;
  RgbdUnmatchedReason reason{RgbdUnmatchedReason::outside_tolerance};
};

/** @brief 成功算法报告，空流也可成功；输入数量等于已配对加各自未配对。 */
struct RgbdSyncReport
{
  std::size_t rgb_count{0};
  std::size_t depth_count{0};
  std::size_t exact_count{0};
  std::uint64_t max_abs_delta_ns{0};
  std::vector<RgbdPair> pairs;
  std::vector<RgbdUnmatched> unmatched_rgb;
  std::vector<RgbdFrameRef> unused_depth;
};

enum class RgbdSyncCode {ok, invalid_options, invalid_stream, invalid_reference, limit_exceeded};

/** @brief 配置/输入错误没有部分报告；检查 ok() 后才能消费 report。 */
struct RgbdSyncResult
{
  RgbdSyncCode code{RgbdSyncCode::invalid_options};
  std::string message;
  std::optional<RgbdSyncReport> report;

  /** @brief 只表示算法完整执行，不代表匹配率、图像质量或几何可用。 */
  bool ok() const noexcept {return code == RgbdSyncCode::ok && report.has_value();}
};

/** @brief 返回稳定静态错误名，无分配；未知枚举返回 unknown。 */
const char * rgbdSyncCodeName(RgbdSyncCode code) noexcept;
/** @brief 返回未配对原因静态名；调用方无需释放。 */
const char * rgbdUnmatchedReasonName(RgbdUnmatchedReason reason) noexcept;

/**
 * @brief RGB 基准、单调、一对一的最近候选贪心匹配，输入不被修改。
 * @note 等距选较早 Depth，同刻选较小 order；不是全局最优匹配。
 *       两侧必须同 clock/segment，消息 ID 全局唯一，order 在各流内唯一。
 *       输出拥有引用数据，不依赖输入生命周期；分配异常直接传播。
 */
RgbdSyncResult pairRgbDepth(
  const RgbdStreamIndex & rgb, const RgbdStreamIndex & depth, const RgbdSyncOptions & options);

}  // namespace vln_memory
