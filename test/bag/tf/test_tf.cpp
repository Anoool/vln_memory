#include <vln_memory/bag/tf/tf.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
using namespace vln_memory;
const TfScope scope{"ros_clock_epoch_1", "map_session_1"};

/** 检查预期条件；失败抛异常，测试在 NDEBUG 下仍有效。 */
void expect(bool value, const std::string & reason)
{
  if (!value) {throw std::runtime_error(reason);}
}

/** 生成带明确来源的原始样本；时间和几何由各用例按需设置。 */
TfTransform sample(
  const std::string & parent, const std::string & child, SensorStamp stamp = {},
  TfKind kind = TfKind::dynamic)
{
  TfTransform result;
  result.parent_frame = parent;
  result.child_frame = child;
  result.stamp = stamp;
  result.kind = kind;
  result.source_topic = kind == TfKind::dynamic ? "/tf" : "/tf_static";
  result.source_message_id = "bag_1_message_1";
  result.source_transform_index = 0;
  return result;
}

/** 插入测试样本，任何非接受状态都使当前用例失败。 */
void add(TfIndex & index, const TfTransform & transform)
{
  const auto inserted = index.insert(transform);
  expect(inserted.accepted(), inserted.message);
}

/** 构造同段查询配置；容差为纳秒，调用方可显式修改 clock/segment 测试拒绝路径。 */
TfTimeOptions timeOptions(std::int64_t max_interpolation_gap_ns)
{
  TfTimeOptions options;
  options.clock_id = scope.clock_id;
  options.segment_id = scope.segment_id;
  options.max_interpolation_gap_ns = max_interpolation_gap_ns;
  return options;
}

/** 验证坐标方向、共同祖先、上下文及独立持有的样本来源。 */
void testTopologyAndOwnership()
{
  TfIndex index(scope);
  auto fixed = sample("base", "camera", {50, 0}, TfKind::static_transform);
  fixed.translation_m = {0.1, 0.2, 0.3};
  add(index, fixed);
  fixed.translation_m[0] = 999;
  fixed.child_frame = "changed";
  fixed.source_topic = "/changed";
  fixed.source_message_id = "changed";
  fixed.source_transform_index = 99;
  add(index, sample("map", "odom", {1, 0}));
  add(index, sample("odom", "base", {1, 0}));
  const auto chain = index.inspectChain("map", "camera", {});
  expect(chain.clock_id == scope.clock_id && chain.segment_id == scope.segment_id,
    "Flat report lost the declared index context");
  expect(chain.chainExists() && chain.frames == std::vector<std::string>{"camera", "base", "odom", "map"},
    "TF chain direction or frames are incorrect");
  expect(chain.edges.size() == 3 && !chain.edges[0].inverted && !chain.edges[2].inverted,
    "Child->parent path should use original T_parent_child");
  const auto reverse = index.inspectChain("camera", "map", {});
  expect(reverse.chainExists() && reverse.edges[0].inverted && reverse.edges[2].inverted,
    "Parent->child edges must request inversion");
  add(index, sample("base", "depth", {}, TfKind::static_transform));
  const auto sibling = index.inspectChain("depth", "camera", {});
  expect(sibling.frames == std::vector<std::string>{"camera", "base", "depth"} &&
    !sibling.edges[0].inverted && sibling.edges[1].inverted, "Nearest common ancestor path failed");
  const auto selected = index.checkTime("base", "camera", {1, 0}, timeOptions(0));
  expect(selected.covered() && selected.edges[0].before.translation_m[0] == 0.1 &&
    selected.edges[0].before.stamp.sec == 50, "Static TF was aliased or given a synthetic timestamp");
  const auto & saved = selected.edges[0].before;
  expect(saved.source_topic == "/tf_static" && saved.source_message_id == "bag_1_message_1" &&
    saved.source_transform_index == 0, "Flat provenance was aliased to the input");
  expect(index.inspectChain("unknown", "camera", {}).code == TfChainCode::unknown_frame,
    "Missing frame was considered connected");
  expect(index.inspectChain("/map", "camera", {}).code == TfChainCode::invalid_frame,
    "Leading slash was silently normalized");
  add(index, sample("other_root", "other_leaf"));
  expect(index.inspectChain("other_root", "camera", {}).code == TfChainCode::disconnected,
    "Disconnected trees were connected by an invented transform");
  const auto identity = index.inspectChain("camera", "camera", {});
  expect(identity.chainExists() && identity.edges.empty() &&
    identity.coverage_status == TfCoverageStatus::static_only, "Known-frame identity chain failed");
}

/** 验证闭区间交集、内部空洞、精确端点及查询上下文拒绝。 */
void testCoverage()
{
  TfIndex index(scope);
  // 乱序插入：map->odom 的 [1,3] 与 [10,12] 中间有洞，第二条动态边覆盖 [2,11]。
  for (const auto sec : {12, 1, 10, 3}) {add(index, sample("map", "odom", {sec, 0}));}
  for (int sec = 2; sec <= 11; ++sec) {add(index, sample("odom", "base", {sec, 0}));}
  add(index, sample("base", "camera", {}, TfKind::static_transform));
  const TfCoverageOptions policy{2000000000LL};
  const auto query = timeOptions(policy.max_interpolation_gap_ns);
  const auto report = index.inspectChain("map", "camera", policy);
  expect(report.chainExists() && report.coverage_status == TfCoverageStatus::available &&
    report.intervals.size() == 2, "Common coverage must preserve internal gaps");
  expect(report.intervals[0].start.sec == 2 && report.intervals[0].end.sec == 3 &&
    report.intervals[1].start.sec == 10 && report.intervals[1].end.sec == 11,
    "Dynamic edge interval intersection is incorrect");
  const auto exact = index.checkTime("map", "camera", {3, 0}, query);
  expect(exact.covered(), "Exact sample failed at an interval boundary");
  const auto between = index.checkTime("map", "camera", {2, 500000000}, query);
  expect(between.covered() && between.edges.size() == 3 && between.edges[2].after &&
    between.edges[2].before.stamp.sec == 1 && between.edges[2].after->stamp.sec == 3,
    "Interpolation sample selection lost brackets or path order");
  const auto hole = index.checkTime("map", "camera", {5, 0}, query);
  expect(hole.code == TfTimeCode::gap_too_large && hole.failed_frame == "odom" && hole.edges.empty(),
    "A time within the raw envelope but inside a hole was accepted");
  expect(index.checkTime("map", "camera", {0, 0}, query).code == TfTimeCode::outside_range,
    "Past extrapolation was accepted");
  expect(index.checkTime("map", "camera", {13, 0}, query).code == TfTimeCode::outside_range,
    "Future extrapolation was accepted");
  expect(index.checkTime("map", "camera", {2, 500000000}, timeOptions(0)).code == TfTimeCode::gap_too_large,
    "Zero gap must mean exact samples only");
  expect(index.inspectChain("map", "camera", {-1}).code == TfChainCode::invalid_options,
    "Negative interpolation gap was accepted");
  auto other_clock = query;
  other_clock.clock_id = "other_clock";
  expect(index.checkTime("map", "camera", {}, other_clock).code ==
    TfTimeCode::clock_mismatch, "Different clocks were mixed");
  auto other_segment = query;
  other_segment.segment_id = "other_segment";
  expect(index.checkTime("map", "camera", {}, other_segment).code ==
    TfTimeCode::segment_mismatch, "Different map/clock segments were mixed");
  expect(index.checkTime("map", "camera", {}, {}).code == TfTimeCode::invalid_scope,
    "Unknown clock and segment were inferred");
  expect(index.checkTime("map", "camera", {0, 1000000000U}, query).code == TfTimeCode::invalid_stamp,
    "Malformed query stamp was accepted");

  TfIndex no_overlap(scope);
  add(no_overlap, sample("map", "base", {1, 0}));
  add(no_overlap, sample("base", "camera", {2, 0}));
  const auto disconnected_time = no_overlap.inspectChain("map", "camera", policy);
  expect(disconnected_time.chainExists() && disconnected_time.coverage_status == TfCoverageStatus::no_common_time &&
    disconnected_time.intervals.empty(), "Topology existence was confused with time availability");
  TfIndex fixed(scope);
  add(fixed, sample("base", "camera", {100, 0}, TfKind::static_transform));
  const auto fixed_report = fixed.inspectChain("base", "camera", {});
  expect(fixed_report.coverage_status == TfCoverageStatus::static_only && fixed_report.intervals.empty() &&
    !fixed_report.edges[0].dynamic_bounds, "Static TF must not get a false dynamic time range");
  expect(fixed.checkTime("base", "camera", {-200, 0}, timeOptions(0)).covered(),
    "Static TF should not require matching its publication stamp");
}

/** 验证冲突阻断、独立树可用性和资源上限，不允许静默覆盖。 */
void testConflictsAndLimits()
{
  for (const auto conflict_kind : {0, 1, 2, 3, 4}) {
    TfIndex index(scope);
    const auto kind = conflict_kind == 2 ? TfKind::static_transform : TfKind::dynamic;
    auto first = sample("map", "base", {}, kind);
    add(index, first);
    auto contradictory = first;
    TfInsertCode expected = TfInsertCode::duplicate_stamp_conflict;
    if (conflict_kind == 0) {
      contradictory.parent_frame = "other";
      expected = TfInsertCode::parent_conflict;
    }
    if (conflict_kind == 1) {
      contradictory.kind = TfKind::static_transform;
      expected = TfInsertCode::kind_conflict;
    }
    if (conflict_kind == 2) {
      contradictory.translation_m[0] = 1;
      expected = TfInsertCode::static_conflict;
    }
    if (conflict_kind == 3) {
      contradictory.translation_m[0] = 1;
    }
    if (conflict_kind == 4) {
      contradictory = sample("base", "map");
      expected = TfInsertCode::cycle_detected;
    }
    expect(index.insert(contradictory).code == expected, "TF contradiction was silently overwritten");
    const auto blocked = index.inspectChain("map", "base", {});
    expect(blocked.code == TfChainCode::topology_conflict &&
      blocked.failed_frame == (conflict_kind == 4 ? "map" : "base"),
      "Rejected contradictory TF left the old version usable or lost the conflicting frame");
    add(index, sample("separate_root", "separate_leaf", {}, TfKind::static_transform));
    expect(index.inspectChain("separate_root", "separate_leaf", {}).chainExists(),
      "A conflict disabled an unrelated tree");
  }
  TfIndex duplicate(scope);
  auto transform = sample("base", "camera", {8, 0}, TfKind::static_transform);
  add(duplicate, transform);
  transform.stamp = {99, 0};
  transform.rotation_xyzw[3] = -1;
  expect(duplicate.insert(transform).code == TfInsertCode::duplicate && duplicate.sampleCount() == 1,
    "Equivalent repeated static broadcasts expanded the index");
  TfIndex limited(scope, {1, 1});
  add(limited, sample("map", "base"));
  expect(limited.insert(sample("map", "base")).code == TfInsertCode::duplicate,
    "Duplicate at resource limit should remain accepted");
  expect(limited.insert(sample("map", "base", {1, 0})).code == TfInsertCode::limit_exceeded &&
    limited.sampleCount() == 1, "Sample limit failure changed the accepted index");
  expect(limited.insert(sample("base", "camera")).code == TfInsertCode::limit_exceeded &&
    limited.edgeCount() == 1, "Edge limit failure changed topology");
  TfIndex invalid_scope({});
  expect(invalid_scope.insert(sample("map", "base")).code == TfInsertCode::invalid_scope,
    "Index accepted an unspecified time context");
  TfIndex invalid_limits(scope, {0, 1});
  expect(invalid_limits.insert(sample("map", "base")).code == TfInsertCode::invalid_limits,
    "Zero resource limit was accepted");
}

/** 验证字段拒绝、输入失败不变性及整数时间的全范围边界。 */
void testInvalidDataAndTimeExtremes()
{
  TfIndex index(scope);
  auto transform = sample("map", "camera");
  const auto rejects = [&index](const TfTransform & input, TfInsertCode expected) {
      const auto result = index.insert(input);
      expect(result.code == expected && !result.accepted() && !result.message.empty(),
        std::string("Expected ") + tfInsertCodeName(expected));
      expect(index.sampleCount() == 0, "Invalid input mutated the index");
    };
  transform.child_frame = "map"; rejects(transform, TfInsertCode::self_transform);
  transform = sample("map", "camera");
  transform.parent_frame = "/map";
  rejects(transform, TfInsertCode::invalid_frame);
  transform = sample("map", "camera");
  transform.stamp.nanosec = 1000000000U;
  rejects(transform, TfInsertCode::invalid_stamp);
  transform = sample("map", "camera");
  transform.translation_m[0] = std::numeric_limits<double>::infinity();
  rejects(transform, TfInsertCode::invalid_translation);
  transform = sample("map", "camera");
  transform.rotation_xyzw = {0, 0, 0, 0};
  rejects(transform, TfInsertCode::invalid_quaternion);
  transform = sample("map", "camera");
  transform.rotation_xyzw[0] = std::numeric_limits<double>::quiet_NaN();
  rejects(transform, TfInsertCode::invalid_quaternion);
  transform = sample("map", "camera");
  transform.source_message_id.clear();
  rejects(transform, TfInsertCode::missing_provenance);
  transform = sample("map", "camera");
  transform.kind = static_cast<TfKind>(99);
  rejects(transform, TfInsertCode::invalid_kind);

  // 公开核心不能依赖调用者碰巧经过 ROS binding 校验；坏来源不得污染索引。
  for (const auto & topic : {std::string("/bad topic"), std::string("tf"),
    std::string("/a//b"), std::string("/1tf"), std::string("/bad\0topic", 10),
    "/" + std::string(247, 'a')})
  {
    transform = sample("map", "camera");
    transform.source_topic = topic;
    rejects(transform, TfInsertCode::missing_provenance);
    expect(index.edgeCount() == 0, "Invalid topic changed topology");
  }

  add(index, sample("map", "camera", {0, 0}));
  add(index, sample("map", "camera", {20, 0}));
  const auto zero = index.checkTime("map", "camera", {}, timeOptions(0));
  expect(zero.covered() && zero.edges[0].before.stamp.sec == 0 && !zero.edges[0].after,
    "Actual zero time was confused with latest or long history was discarded");
  TfIndex extremes(scope);
  add(extremes, sample("map", "camera", {std::numeric_limits<std::int32_t>::min(), 0}));
  add(extremes, sample("map", "camera", {std::numeric_limits<std::int32_t>::max(), 999999999U}));
  const TfCoverageOptions largest{std::numeric_limits<std::int64_t>::max()};
  expect(extremes.inspectChain("map", "camera", largest).intervals.size() == 1 &&
    extremes.checkTime("map", "camera", {-1, 500000000}, timeOptions(largest.max_interpolation_gap_ns)).covered(),
    "Integer TF timestamps overflowed across full int32 seconds range");
}

/** 独立的离散时间 oracle，不使用区间合并算法生成预期结果。 */
bool oracle(const std::set<std::uint32_t> & times, std::uint32_t t, std::uint32_t max_gap)
{
  if (times.count(t) != 0) {return true;}
  const auto next = times.lower_bound(t);
  return next != times.begin() && next != times.end() && *next - *std::prev(next) <= max_gap;
}

/** 验证多个输入同时错误时仍按原顺序返回第一个诊断。 */
void testQueryValidationOrder()
{
  TfIndex index(scope);
  auto query = timeOptions(-1);
  query.clock_id.clear();
  expect(index.checkTime("/bad", "unknown", {0, 1000000000U}, query).code ==
    TfTimeCode::invalid_stamp, "Query stamp must be checked before scope and frames");
  expect(index.checkTime("/bad", "unknown", {}, query).code == TfTimeCode::invalid_scope,
    "Missing query scope must be checked before domain mismatch");
  query.clock_id = "other_clock";
  query.segment_id = "other_segment";
  expect(index.checkTime("/bad", "unknown", {}, query).code == TfTimeCode::clock_mismatch,
    "Clock mismatch must be checked before segment mismatch");
  query.clock_id = scope.clock_id;
  expect(index.checkTime("/bad", "unknown", {}, query).code == TfTimeCode::segment_mismatch,
    "Segment mismatch must be checked before interpolation policy");
  query.segment_id = scope.segment_id;
  expect(index.checkTime("/bad", "unknown", {}, query).code == TfTimeCode::invalid_options,
    "Negative gap must be checked before chain discovery");
  expect(index.inspectChain("unknown", "unknown", {-1}).code == TfChainCode::unknown_frame,
    "Coverage inspection must preserve chain-first validation");
}

/** 使用可手算的旋转和平移核对父坐标系结果、反向链及门限。 */
void testNumericPose()
{
  TfIndex index(scope);
  auto before = sample("map", "base", {0, 0});
  auto after = sample("map", "base", {0, 50});
  before.rotation_xyzw = {0, 0, std::sin(0.125 * std::acos(-1.0)), std::cos(0.125 * std::acos(-1.0))};
  after.rotation_xyzw = {0, 0, -before.rotation_xyzw[3], -before.rotation_xyzw[2]};
  after.translation_m = {2, 0, 0};
  after.source_message_id = "later";
  add(index, before);
  add(index, after);
  after.stamp.nanosec = 100;
  add(index, after);
  after.stamp.nanosec = 300;
  add(index, after);
  auto fixed = sample("base", "camera", {}, TfKind::static_transform);
  fixed.translation_m = {1, 0, 0};
  add(index, fixed);
  expect(index.inferInterpolationGapNs("map", {"camera", "base"}) == 100,
    "Automatic gap ignored cadence or followed the outlier");
  expect(index.inferInterpolationGapNs("map", {"missing"}) == 0, "Unknown chain inferred a gap");
  const auto result = index.lookupPose("map", "camera", {0, 25}, timeOptions(100));
  expect(result.ok(), "Forward pose unavailable");
  const auto & t = result.pose->translation_m;
  const auto & q = result.pose->rotation_xyzw;
  expect(std::abs(t[0] - 1) < 1e-12 && std::abs(t[1] - 1) < 1e-12 &&
    std::abs(t[2]) < 1e-12 && std::abs(q[2] - std::sqrt(0.5)) < 1e-12,
    "Incorrect parent-space composition or quaternion interpolation");
  expect(result.time.edges[1].after->source_message_id == "later" &&
    result.time.edges[1].after->rotation_xyzw[3] < 0, "Raw evidence was rewritten");
  const auto reverse = index.lookupPose("camera", "map", {0, 25}, timeOptions(100));
  expect(reverse.ok() && std::abs(reverse.pose->translation_m[0] + 1) < 1e-12 &&
    std::abs(reverse.pose->translation_m[1] - 1) < 1e-12, "Reverse pose is not the inverse");
  const auto exact = index.lookupPose("map", "base", {}, timeOptions(0));
  expect(exact.ok() && exact.pose->translation_m[0] == 0.0, "Exact zero must use original sample");
  const auto identity = index.lookupPose("base", "base", {}, timeOptions(0));
  expect(identity.ok() && identity.pose->rotation_xyzw[3] == 1.0, "Known identity unavailable");
  const auto jump = index.lookupPose("map", "camera", {0, 25}, timeOptions(100), {1.0, {}});
  expect(!jump.ok() && jump.code == TfPoseCode::jump_too_large && jump.failed_frame == "base",
    "Translation jump was interpolated");
  const auto turn = index.lookupPose("map", "camera", {0, 25}, timeOptions(100), {{}, 0.5});
  expect(!turn.ok() && turn.code == TfPoseCode::jump_too_large, "Rotation jump was interpolated");
  const auto gap = index.lookupPose("map", "camera", {0, 150}, timeOptions(100));
  expect(!gap.ok() && gap.time.code == TfTimeCode::gap_too_large, "Long gap produced a pose");
  const auto outside = index.lookupPose("map", "camera", {0, 301}, timeOptions(100));
  expect(!outside.ok() && outside.time.code == TfTimeCode::outside_range, "Pose extrapolated");
}

/** 以固定种子生成动态历史，对照独立逐时刻 oracle，不放宽任何覆盖预期。 */
void testGeneratedIntervals()
{
  std::mt19937 random(20261029U);
  for (std::size_t trial = 0; trial < 200; ++trial) {
    TfIndex index(scope);
    std::set<std::uint32_t> a;
    std::set<std::uint32_t> b;
    for (std::size_t i = 0; i < 8; ++i) {
      a.insert(static_cast<std::uint32_t>(random() % 31U));
      b.insert(static_cast<std::uint32_t>(random() % 31U));
    }
    for (const auto t : a) {add(index, sample("map", "base", {0, t}));}
    for (const auto t : b) {add(index, sample("base", "camera", {0, t}));}
    const auto max_gap = static_cast<std::uint32_t>(random() % 12U);
    const auto report = index.inspectChain("map", "camera", {max_gap});
    for (std::uint32_t t = 0; t <= 32; ++t) {
      const bool expected = oracle(a, t, max_gap) && oracle(b, t, max_gap);
      const bool contained = std::any_of(report.intervals.begin(), report.intervals.end(),
        [t](const TfTimeInterval & interval) {return t >= interval.start.nanosec && t <= interval.end.nanosec;});
      const bool selected = index.checkTime("map", "camera", {0, t}, timeOptions(max_gap)).covered();
      expect(contained == expected && selected == expected,
        "Generated sample oracle disagrees with chain intervals or point-time selection");
    }
  }
}
}  // namespace

/** 执行全部核心回归；异常打印原因并返回失败状态。 */
int main()
{
  try {
    testTopologyAndOwnership();
    testCoverage();
    testConflictsAndLimits();
    testInvalidDataAndTimeExtremes();
    testQueryValidationOrder();
    testGeneratedIntervals();
    testNumericPose();
    std::cout << "TF topology, coverage, source selection and 200 generated interval cases: passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
