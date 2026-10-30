#include <vln_memory/bag/builder/rgbd_sync.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace
{
using namespace vln_memory;

/** @brief Release 也检查断言，失败给出用例原因。 */
void expect(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 从演示整数纳秒生成正秒范围内的独立来源引用。 */
RgbdFrameRef reference(const std::string & prefix, std::uint64_t order, std::uint32_t nanosec)
{
  return {prefix + std::to_string(order), order, {0, nanosec}};
}

/** @brief 构造明确同钟/同段的流，stamp 列表顺序同时作为原始 order。 */
RgbdStreamIndex stream(const std::string & name, const std::vector<std::uint32_t> & stamps)
{
  RgbdStreamIndex result;
  result.topic = "/" + name;
  result.clock_id = "sensor_clock";
  result.segment_id = "segment";
  for (std::size_t index = 0; index < stamps.size(); ++index) {
    result.frames.push_back(reference(name, index, stamps[index]));
  }
  return result;
}

/** @brief 生成显式同钟选项；演示容差单位仍是整数纳秒。 */
RgbdSyncOptions options(std::int64_t tolerance_ns)
{
  RgbdSyncOptions result;
  result.clock_relation = SensorClockRelation::same_clock;
  result.clock_basis = "test fixture";
  result.max_delta_ns = tolerance_ns;
  return result;
}

/** @brief 验证边界、等距、重复时间、单调及贪心非全局最优语义。 */
void knownAnswers()
{
  auto result = pairRgbDepth(stream("rgb", {4, 6}), stream("depth", {0, 5}), options(6));
  expect(result.ok() && result.report->pairs.size() == 1, "Greedy policy changed");
  expect(result.report->pairs[0].depth.stamp.nanosec == 5, "Nearest depth not selected");
  expect(
    result.report->unmatched_rgb[0].reason == RgbdUnmatchedReason::unavailable_by_policy,
    "Policy exclusion not reported");
  result = pairRgbDepth(stream("rgb", {5, 5}), stream("depth", {4, 6, 6}), options(1));
  expect(result.ok() && result.report->pairs.size() == 2, "Tie/boundary pairing failed");
  expect(
    result.report->pairs[0].delta_ns == -1 && result.report->pairs[1].depth.message_order == 1,
    "Tie should select earlier time then earlier source");
  expect(
    result.report->unused_depth.size() == 1 && result.report->max_abs_delta_ns == 1,
    "Conservation/statistics failed");
  result = pairRgbDepth(stream("rgb", {1}), stream("depth", {2}), options(0));
  expect(result.ok() && result.report->pairs.empty(), "Exact policy accepted unequal stamps");
  result = pairRgbDepth(stream("rgb", {}), stream("depth", {}), options(0));
  expect(result.ok() && result.report->pairs.empty(), "Empty streams should return report");
}

/** @brief 零容差仍接受真实零时刻；重复同刻消息按来源一对一，未知时钟不凭相等放行。 */
void strictEquality()
{
  const auto rgb = stream("rgb", {0, 5, 5, 7});
  const auto depth = stream("depth", {0, 5, 7});
  const auto exact = pairRgbDepth(rgb, depth, options(0));
  expect(exact.ok() && exact.report->pairs.size() == 3, "Exact/duplicate stamp matching failed");
  expect(exact.report->exact_count == 3 && exact.report->max_abs_delta_ns == 0, "Nonzero strict delta");
  expect(exact.report->unmatched_rgb.size() == 1 && exact.report->unused_depth.empty(), "Depth reused");
  expect(exact.report->pairs.front().rgb.stamp.nanosec == 0, "Real zero timestamp was discarded");
  auto unknown = options(0);
  unknown.clock_relation = SensorClockRelation::unknown;
  expect(!pairRgbDepth(rgb, depth, unknown).ok(), "Equal timestamps proved an unknown clock");
  auto other_clock = depth;
  other_clock.clock_id = "other_clock";
  expect(!pairRgbDepth(rgb, other_clock, options(0)).ok(), "Cross-clock exact stamps accepted");
}

/** @brief 验证坏引用、资源限制、不同域及完整 ROS 时间边界。 */
void inputBoundaries()
{
  auto rgb = stream("rgb", {1});
  auto depth = stream("depth", {1});
  rgb.frames[0].stamp = {std::numeric_limits<std::int32_t>::min(), 0};
  depth.frames[0].stamp = {std::numeric_limits<std::int32_t>::max(), 999999999};
  auto result = pairRgbDepth(rgb, depth, options(std::numeric_limits<std::int64_t>::max()));
  expect(
    result.ok() && result.report->pairs.size() == 1 && result.report->pairs[0].delta_ns > 0,
    "Full stamp range overflowed");
  rgb.frames[0].stamp.nanosec = 1000000000U;
  result = pairRgbDepth(rgb, depth, options(0));
  expect(
    !result.ok() && !result.report && result.code == RgbdSyncCode::invalid_reference,
    "Invalid nanosec accepted");
  rgb = stream("rgb", {1, 2});
  depth = stream("depth", {1});
  rgb.frames[1].message_order = 0;
  expect(!pairRgbDepth(rgb, depth, options(1)).ok(), "Duplicate order accepted");
  rgb = stream("rgb", {1});
  depth.frames[0].message_id = rgb.frames[0].message_id;
  expect(!pairRgbDepth(rgb, depth, options(1)).ok(), "Cross-stream duplicate ID accepted");
  depth = stream("depth", {1});
  depth.segment_id = "other";
  expect(!pairRgbDepth(rgb, depth, options(1)).ok(), "Different segment accepted");
  depth.segment_id = rgb.segment_id;
  auto limits = options(1);
  limits.max_frames = 1;
  expect(pairRgbDepth(rgb, depth, limits).code == RgbdSyncCode::limit_exceeded, "Limit ignored");
  expect(!pairRgbDepth(rgb, depth, options(-1)).ok(), "Negative tolerance accepted");
}

/** @brief 小数据全扫描参考算法，仅用于独立核对相同贪心目标。 */
std::vector<std::string> oracle(
  RgbdStreamIndex rgb, RgbdStreamIndex depth, std::uint32_t tolerance_ns)
{
  const auto earlier = [](const auto & left, const auto & right) {
      if (left.stamp.nanosec != right.stamp.nanosec) {return left.stamp.nanosec < right.stamp.nanosec;}
      return left.message_order < right.message_order;
    };
  std::sort(rgb.frames.begin(), rgb.frames.end(), earlier);
  std::sort(depth.frames.begin(), depth.frames.end(), earlier);
  std::size_t first = 0;
  std::vector<std::string> pairs;
  for (const auto & frame : rgb.frames) {
    std::size_t best = depth.frames.size();
    std::uint32_t best_distance = std::numeric_limits<std::uint32_t>::max();
    for (std::size_t index = first; index < depth.frames.size(); ++index) {
      const auto delta = static_cast<std::int64_t>(depth.frames[index].stamp.nanosec) - frame.stamp.nanosec;
      const auto distance = static_cast<std::uint32_t>(std::abs(delta));
      if (distance <= tolerance_ns && distance < best_distance) {
        best_distance = distance;
        best = index;
      }
    }
    if (best != depth.frames.size()) {
      pairs.push_back(frame.message_id + ":" + depth.frames[best].message_id);
      first = best + 1;
    }
  }
  return pairs;
}

/** @brief 固定种子生成乱序/重复时间，用朴素 oracle 核对配对并检查所有权。 */
void generatedCases()
{
  std::mt19937 generator(42);
  for (int iteration = 0; iteration < 600; ++iteration) {
    auto rgb = stream("rgb", {});
    auto depth = stream("depth", {});
    const auto rgb_count = generator() % 15;
    const auto depth_count = generator() % 15;
    for (std::uint64_t index = 0; index < rgb_count; ++index) {
      rgb.frames.push_back(reference("rgb", index, static_cast<std::uint32_t>(generator() % 30)));
    }
    for (std::uint64_t index = 0; index < depth_count; ++index) {
      depth.frames.push_back(reference("depth", index, static_cast<std::uint32_t>(generator() % 30)));
    }
    const auto tolerance = static_cast<std::uint32_t>(generator() % 10);
    const auto expected = oracle(rgb, depth, tolerance);
    std::shuffle(rgb.frames.begin(), rgb.frames.end(), generator);
    std::shuffle(depth.frames.begin(), depth.frames.end(), generator);
    const auto result = pairRgbDepth(rgb, depth, options(tolerance));
    expect(result.ok(), "Generated valid input rejected");
    std::vector<std::string> actual;
    for (const auto & pair : result.report->pairs) {
      actual.push_back(pair.rgb.message_id + ":" + pair.depth.message_id);
    }
    expect(actual == expected, "Binary search and scan oracle disagree");
    expect(rgb.frames.size() == actual.size() + result.report->unmatched_rgb.size(), "RGB count mismatch");
    expect(depth.frames.size() == actual.size() + result.report->unused_depth.size(), "Depth count mismatch");
  }
}
}

/** @brief 执行配对核心回归，不依赖 ROS 或真实录包。 */
int main()
{
  try {
    knownAnswers();
    strictEquality();
    inputBoundaries();
    generatedCases();
    std::cout << "RGB-D sync: 600 oracle cases and boundaries passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
