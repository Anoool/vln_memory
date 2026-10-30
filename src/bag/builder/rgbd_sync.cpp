#include "vln_memory/bag/builder/rgbd_sync.hpp"

#include <algorithm>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "vln_memory/bag/common/sensor_names.hpp"

namespace vln_memory
{
namespace
{
using FramePointers = std::vector<const RgbdFrameRef *>;

/** @brief 将合法 ROS stamp 转整数纳秒，完整 int32 秒范围可安全表示。 */
std::int64_t stampNs(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

/** @brief 两个合法 SensorStamp 的差不会达到 INT64_MIN，可安全取绝对值。 */
std::uint64_t distanceNs(const SensorStamp & left, const SensorStamp & right)
{
  const auto delta_ns = stampNs(left) - stampNs(right);
  if (delta_ns < 0) {return static_cast<std::uint64_t>(-delta_ns);}
  return static_cast<std::uint64_t>(delta_ns);
}

/** @brief 构造无报告失败；拥有错误文本，不借用局部对象。 */
RgbdSyncResult failure(RgbdSyncCode code, const char * message)
{
  return {code, message, std::nullopt};
}

/** @brief 验证流身份和同段声明；不会根据数值相近推断同钟。 */
bool validStreams(const RgbdStreamIndex & rgb, const RgbdStreamIndex & depth)
{
  return isResolvedTopicName(rgb.topic) && isResolvedTopicName(depth.topic) &&
         rgb.topic != depth.topic && !rgb.clock_id.empty() && !rgb.segment_id.empty() &&
         rgb.clock_id == depth.clock_id && rgb.segment_id == depth.segment_id;
}

/** @brief 验证引用唯一性；借用的 string_view 仅存活于输入保持不变的本次调用。 */
bool validReferences(
  const RgbdStreamIndex & stream, std::unordered_set<std::string_view> & message_ids)
{
  std::unordered_set<std::uint64_t> orders;
  for (const auto & frame : stream.frames) {
    if (frame.message_id.empty() || frame.stamp.nanosec >= 1000000000U ||
      !message_ids.insert(frame.message_id).second || !orders.insert(frame.message_order).second)
    {
      return false;
    }
  }
  return true;
}

/** @brief 排序借用指针，避免复制输入字符串；结果不离开本次算法调用。 */
FramePointers sortedFrames(const RgbdStreamIndex & stream)
{
  FramePointers frames;
  frames.reserve(stream.frames.size());
  for (const auto & frame : stream.frames) {
    frames.push_back(&frame);
  }
  std::sort(
    frames.begin(), frames.end(), [](const auto * left, const auto * right) {
      if (stampNs(left->stamp) != stampNs(right->stamp)) {
        return stampNs(left->stamp) < stampNs(right->stamp);
      }
      return left->message_order < right->message_order;
    });
  return frames;
}

/** @brief 在允许后缀二分找最近时刻；同刻取最早来源，等距保留较早时间。 */
std::size_t nearestDepth(
  const FramePointers & depth, std::size_t first_allowed, const SensorStamp & rgb_stamp)
{
  const auto begin = depth.begin() + static_cast<FramePointers::difference_type>(first_allowed);
  const auto query_ns = stampNs(rgb_stamp);
  const auto next = std::lower_bound(
    begin, depth.end(), query_ns,
    [](const auto * frame, std::int64_t value) {return stampNs(frame->stamp) < value;});
  auto best = next;
  if (next != begin) {
    const auto previous_ns = stampNs((*std::prev(next))->stamp);
    const auto previous = std::lower_bound(
      begin, next, previous_ns,
      [](const auto * frame, std::int64_t value) {return stampNs(frame->stamp) < value;});
    if (best == depth.end() ||
      distanceNs((*previous)->stamp, rgb_stamp) <= distanceNs((*best)->stamp, rgb_stamp))
    {
      best = previous;
    }
  }
  return static_cast<std::size_t>(std::distance(depth.begin(), best));
}

/** @brief 区分没有深度、确实超差和已被单调/一对一策略排除，额外查找仍为二分。 */
RgbdUnmatchedReason unmatchedReason(
  const FramePointers & depth, const SensorStamp & rgb_stamp, std::uint64_t tolerance_ns)
{
  if (depth.empty()) {return RgbdUnmatchedReason::empty_depth;}
  const auto nearest = nearestDepth(depth, 0, rgb_stamp);
  if (distanceNs(depth[nearest]->stamp, rgb_stamp) <= tolerance_ns) {
    return RgbdUnmatchedReason::unavailable_by_policy;
  }
  return RgbdUnmatchedReason::outside_tolerance;
}

/** @brief 消费已验证/排序的引用，输出拥有型报告；每帧只二分查询候选。 */
RgbdSyncReport matchFrames(
  const FramePointers & rgb, const FramePointers & depth, const RgbdSyncOptions & options)
{
  RgbdSyncReport report;
  report.rgb_count = rgb.size();
  report.depth_count = depth.size();
  std::vector<bool> selected(depth.size(), false);
  std::size_t first_allowed = 0;
  const SensorTimeOptions time{options.clock_relation, options.max_delta_ns};
  for (const auto * frame : rgb) {
    const auto candidate = nearestDepth(depth, first_allowed, frame->stamp);
    if (candidate == depth.size()) {
      report.unmatched_rgb.push_back(
        {*frame, unmatchedReason(
            depth, frame->stamp,
            static_cast<std::uint64_t>(options.max_delta_ns))});
      continue;
    }
    const auto timing = checkSensorTimes(frame->stamp, depth[candidate]->stamp, time);
    if (!timing.matched()) {
      report.unmatched_rgb.push_back(
        {*frame, unmatchedReason(
            depth, frame->stamp,
            static_cast<std::uint64_t>(options.max_delta_ns))});
      continue;
    }
    report.pairs.push_back({*frame, *depth[candidate], *timing.delta_ns});
    selected[candidate] = true;
    first_allowed = candidate + 1;
    if (*timing.delta_ns == 0) {++report.exact_count;}
    report.max_abs_delta_ns = std::max(
      report.max_abs_delta_ns,
      distanceNs(frame->stamp, depth[candidate]->stamp));
  }
  for (std::size_t index = 0; index < depth.size(); ++index) {
    if (!selected[index]) {report.unused_depth.push_back(*depth[index]);}
  }
  return report;
}

}  // namespace

/** @brief 验证后排序并匹配，全部成功才提供报告；系统异常不伪装成坏数据。 */
RgbdSyncResult pairRgbDepth(
  const RgbdStreamIndex & rgb, const RgbdStreamIndex & depth, const RgbdSyncOptions & options)
{
  if (options.clock_relation != SensorClockRelation::same_clock || options.clock_basis.empty() ||
    options.max_delta_ns < 0 || options.max_frames == 0)
  {
    return failure(RgbdSyncCode::invalid_options, "Explicit same clock, basis and valid limits required");
  }
  if (!validStreams(rgb, depth)) {
    return failure(RgbdSyncCode::invalid_stream, "Distinct topics and identical explicit clock/segment required");
  }
  if (rgb.frames.size() > options.max_frames || depth.frames.size() > options.max_frames - rgb.frames.size()) {
    return failure(RgbdSyncCode::limit_exceeded, "Combined frame index limit exceeded");
  }
  std::unordered_set<std::string_view> message_ids;
  if (!validReferences(rgb, message_ids) || !validReferences(depth, message_ids)) {
    return failure(RgbdSyncCode::invalid_reference, "Invalid stamp, duplicate/empty ID or duplicate stream order");
  }
  return {RgbdSyncCode::ok, {}, matchFrames(sortedFrames(rgb), sortedFrames(depth), options)};
}

/** @brief 返回稳定错误名，不分配内存。 */
const char * rgbdSyncCodeName(RgbdSyncCode code) noexcept
{
  switch (code) {
    case RgbdSyncCode::ok: return "ok";
    case RgbdSyncCode::invalid_options: return "invalid_options";
    case RgbdSyncCode::invalid_stream: return "invalid_stream";
    case RgbdSyncCode::invalid_reference: return "invalid_reference";
    case RgbdSyncCode::limit_exceeded: return "limit_exceeded";
  }
  return "unknown";
}

/** @brief 返回稳定未配对原因，不把未使用帧标成解码失败。 */
const char * rgbdUnmatchedReasonName(RgbdUnmatchedReason reason) noexcept
{
  switch (reason) {
    case RgbdUnmatchedReason::empty_depth: return "empty_depth";
    case RgbdUnmatchedReason::outside_tolerance: return "outside_tolerance";
    case RgbdUnmatchedReason::unavailable_by_policy: return "unavailable_by_policy";
  }
  return "unknown";
}

}  // namespace vln_memory
