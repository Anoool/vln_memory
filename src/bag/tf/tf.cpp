#include "vln_memory/bag/tf/tf.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace vln_memory
{
namespace
{
std::int64_t nanoseconds(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool validScope(const TfScope & scope)
{
  return !scope.clock_id.empty() && !scope.segment_id.empty();
}

bool validFrame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(frame.begin(), frame.end(), [](char c) {return c <= ' ' || c == '\x7f';});
}

template<typename Values>
bool finite(const Values & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

double quaternionNorm(const std::array<double, 4> & q)
{
  return std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
}

/** 只在重复判定时比较归一化方向，q 与 -q 表示同一旋转；原始数据仍完整保留。 */
bool samePose(const TfTransform & left, const TfTransform & right)
{
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::abs(left.translation_m[i] - right.translation_m[i]) > 1e-9) {return false;}
  }
  const double ln = quaternionNorm(left.rotation_xyzw);
  const double rn = quaternionNorm(right.rotation_xyzw);
  bool same = true;
  bool opposite = true;
  for (std::size_t i = 0; i < 4; ++i) {
    const double a = left.rotation_xyzw[i] / ln;
    const double b = right.rotation_xyzw[i] / rn;
    same = same && std::abs(a - b) <= 1e-9;
    opposite = opposite && std::abs(a + b) <= 1e-9;
  }
  return same || opposite;
}

TfInsertResult validate(const TfTransform & input)
{
  using Code = TfInsertCode;
  if (input.kind != TfKind::dynamic && input.kind != TfKind::static_transform) {
    return {Code::invalid_kind, "Explicit dynamic/static TF kind is required"};
  }
  if (!validFrame(input.parent_frame) || !validFrame(input.child_frame)) {
    return {Code::invalid_frame, "TF frames must be non-empty and have no leading slash or whitespace"};
  }
  if (input.parent_frame == input.child_frame) {
    return {Code::self_transform, "A frame cannot be its own parent"};
  }
  if (input.stamp.nanosec >= 1000000000U) {
    return {Code::invalid_stamp, "Stamp nanosec must be less than 1000000000"};
  }
  if (!finite(input.translation_m)) {
    return {Code::invalid_translation, "Translation must be finite, in meters"};
  }
  if (!finite(input.rotation_xyzw) || std::abs(quaternionNorm(input.rotation_xyzw) - 1.0) > 1e-3) {
    return {Code::invalid_quaternion, "Quaternion must be finite and unit length (norm tolerance 1e-3)"};
  }
  if (input.source.topic.size() < 2 || input.source.topic.front() != '/' ||
    input.source.message_id.empty())
  {
    return {Code::missing_provenance, "Resolved source topic and unique source message ID are required"};
  }
  return {Code::inserted, ""};
}

std::vector<TfTimeInterval> intersect(
  const std::vector<TfTimeInterval> & left, const std::vector<TfTimeInterval> & right)
{
  std::vector<TfTimeInterval> result;
  std::size_t i = 0;
  std::size_t j = 0;
  while (i < left.size() && j < right.size()) {
    const auto start = nanoseconds(left[i].start) >= nanoseconds(right[j].start) ?
      left[i].start : right[j].start;
    const auto end = nanoseconds(left[i].end) <= nanoseconds(right[j].end) ? left[i].end : right[j].end;
    if (nanoseconds(start) <= nanoseconds(end)) {result.push_back({start, end});}
    if (nanoseconds(left[i].end) < nanoseconds(right[j].end)) {++i;} else {++j;}
  }
  return result;
}

}  // namespace

TfIndex::TfIndex(TfScope scope, TfLimits limits)
: scope_(std::move(scope)), limits_(limits)
{
}

TfInsertResult TfIndex::insert(TfTransform input)
{
  using Code = TfInsertCode;
  if (!validScope(scope_)) {return {Code::invalid_scope, "Index clock_id and segment_id are required"};}
  if (limits_.max_edges == 0 || limits_.max_samples == 0) {
    return {Code::invalid_limits, "TF index limits must be positive"};
  }
  const auto validated = validate(input);
  if (!validated.accepted()) {return validated;}
  if (blocked_frames_.count(input.child_frame) != 0) {
    return {Code::edge_blocked, "Child frame was blocked by an earlier TF contradiction"};
  }
  auto found = edges_.find(input.child_frame);
  // 矛盾不是普通丢帧：保守阻断节点，后续查询不能继续选用先到/后到的任一版本。
  const auto conflict = [this, &input](Code code, const char * reason) {
      blocked_frames_.insert(input.child_frame);
      return TfInsertResult{code, reason};
    };
  if (found != edges_.end()) {
    if (found->second.parent != input.parent_frame) {
      return conflict(Code::parent_conflict, "Child frame has multiple parents; split the data segment");
    }
    if (found->second.kind != input.kind) {
      return conflict(Code::kind_conflict, "Child frame appears in both dynamic and static TF");
    }
    auto & samples = found->second.samples;
    const auto previous = input.kind == TfKind::static_transform ? samples.begin() :
      samples.find(nanoseconds(input.stamp));
    if (previous != samples.end()) {
      if (samePose(previous->second, input)) {return {Code::duplicate, "Consistent duplicate TF"};}
      return conflict(input.kind == TfKind::static_transform ? Code::static_conflict :
        Code::duplicate_stamp_conflict, "Conflicting transform values; no last-message overwrite");
    }
  } else {
    // 已接受拓扑始终无环；向上走到根即可发现新增边是否把祖先接回自己的后代。
    std::string cursor = input.parent_frame;
    while (true) {
      if (cursor == input.child_frame) {
        return conflict(Code::cycle_detected, "TF edge would create a cycle");
      }
      const auto parent = edges_.find(cursor);
      if (parent == edges_.end()) {break;}
      cursor = parent->second.parent;
    }
    if (edges_.size() >= limits_.max_edges) {return {Code::limit_exceeded, "TF edge limit exceeded"};}
  }
  if (sample_count_ >= limits_.max_samples) {return {Code::limit_exceeded, "TF sample limit exceeded"};}

  // 所有业务检查先完成，再写入。系统分配异常应终止本次构建，不应捕获后继续使用索引。
  if (found == edges_.end()) {
    found = edges_.emplace(input.child_frame, Edge{input.parent_frame, input.kind, {}}).first;
  }
  frames_.insert(input.parent_frame);
  frames_.insert(input.child_frame);
  const auto stamp_ns = nanoseconds(input.stamp);
  found->second.samples.emplace(stamp_ns, std::move(input));
  ++sample_count_;
  return {Code::inserted, ""};
}

TfChainReport TfIndex::findChain(const std::string & target, const std::string & source) const
{
  TfChainReport report;
  report.scope = scope_;
  report.target_frame = target;
  report.source_frame = source;
  const auto fail = [&report](TfChainCode code, const char * message) {
      report.code = code;
      report.message = message;
      return report;
    };
  if (!validScope(scope_) || limits_.max_edges == 0 || limits_.max_samples == 0) {
    return fail(TfChainCode::invalid_options, "TF index requires a scope and positive resource limits");
  }
  if (!validFrame(target) || !validFrame(source)) {
    report.failed_frame = !validFrame(target) ? target : source;
    return fail(TfChainCode::invalid_frame, "Query frames are invalid");
  }
  if (frames_.count(target) == 0 || frames_.count(source) == 0) {
    report.failed_frame = frames_.count(target) == 0 ? target : source;
    return fail(TfChainCode::unknown_frame, "A requested frame is absent from accepted TF records");
  }
  // 两端分别走到根，寻找最近共同祖先；不依赖图中恰好存在 odom 或某个相机名称。
  const auto ancestors = [this, &report](std::string cursor, std::vector<std::string> & path) {
      while (true) {
        if (blocked_frames_.count(cursor) != 0) {
          report.failed_frame = cursor;
          return false;
        }
        path.push_back(cursor);
        const auto edge = edges_.find(cursor);
        if (edge == edges_.end()) {return true;}
        cursor = edge->second.parent;
      }
    };
  std::vector<std::string> source_path;
  std::vector<std::string> target_path;
  if (!ancestors(source, source_path) || !ancestors(target, target_path)) {
    return fail(TfChainCode::topology_conflict, "A query ancestor is blocked by contradictory TF records");
  }
  std::map<std::string, std::size_t> positions;
  for (std::size_t i = 0; i < source_path.size(); ++i) {positions.emplace(source_path[i], i);}
  std::size_t target_lca = 0;
  while (target_lca < target_path.size() && positions.count(target_path[target_lca]) == 0) {++target_lca;}
  if (target_lca == target_path.size()) {
    return fail(TfChainCode::disconnected, "Frames exist but belong to disconnected TF trees");
  }
  const auto source_lca = positions.at(target_path[target_lca]);
  const auto append = [this, &report](const std::string & child, bool inverted) {
      const auto & edge = edges_.at(child);
      TfChainEdge info{edge.parent, child, edge.kind, inverted, edge.samples.size(), std::nullopt};
      if (edge.kind == TfKind::dynamic) {
        info.dynamic_bounds = TfTimeInterval{
          edge.samples.begin()->second.stamp, edge.samples.rbegin()->second.stamp};
      }
      report.edges.push_back(std::move(info));
    };
  for (std::size_t i = 0; i <= source_lca; ++i) {report.frames.push_back(source_path[i]);}
  for (std::size_t i = 0; i < source_lca; ++i) {append(source_path[i], false);}
  for (std::size_t i = target_lca; i > 0; --i) {
    append(target_path[i - 1], true);
    report.frames.push_back(target_path[i - 1]);
  }
  report.code = TfChainCode::connected;
  return report;
}

TfChainReport TfIndex::inspectChain(
  const std::string & target, const std::string & source, const TfCoverageOptions & options) const
{
  auto report = findChain(target, source);
  if (!report.chainExists()) {return report;}
  if (options.max_interpolation_gap_ns < 0) {
    report.code = TfChainCode::invalid_options;
    report.message = "Maximum interpolation gap must be non-negative";
    return report;
  }
  bool first_dynamic = true;
  for (const auto & edge : report.edges) {
    if (edge.kind == TfKind::static_transform) {continue;}
    const auto & samples = edges_.at(edge.child_frame).samples;
    std::vector<TfTimeInterval> intervals;
    TfTimeInterval current{samples.begin()->second.stamp, samples.begin()->second.stamp};
    auto previous = samples.begin();
    for (auto next = std::next(previous); next != samples.end(); ++next) {
      if (next->first - previous->first > options.max_interpolation_gap_ns) {
        intervals.push_back(current);
        current.start = next->second.stamp;
      }
      current.end = next->second.stamp;
      previous = next;
    }
    intervals.push_back(current);
    report.intervals = first_dynamic ? std::move(intervals) : intersect(report.intervals, intervals);
    first_dynamic = false;
    if (report.intervals.empty()) {break;}
  }
  report.coverage_status = first_dynamic ? TfCoverageStatus::static_only :
    (report.intervals.empty() ? TfCoverageStatus::no_common_time : TfCoverageStatus::available);
  return report;
}

TfTimeCheck TfIndex::checkTime(
  const std::string & target, const std::string & source, const SensorStamp & stamp,
  const TfTimeOptions & options) const
{
  TfTimeCheck result;
  result.stamp = stamp;
  const auto fail = [&result](TfTimeCode code, const char * message, const std::string & child = "") {
      result.code = code;
      result.message = message;
      result.failed_frame = child;
      result.edges.clear();
      return result;
    };
  if (stamp.nanosec >= 1000000000U) {return fail(TfTimeCode::invalid_stamp, "Invalid query nanosec");}
  if (!validScope(options.scope)) {return fail(TfTimeCode::invalid_scope, "Query clock/segment must be explicit");}
  if (options.scope.clock_id != scope_.clock_id) {return fail(TfTimeCode::clock_mismatch, "Query clock differs from TF index");}
  if (options.scope.segment_id != scope_.segment_id) {return fail(TfTimeCode::segment_mismatch, "Query data segment differs from TF index");}
  if (options.coverage.max_interpolation_gap_ns < 0) {
    return fail(TfTimeCode::invalid_options, "Maximum interpolation gap must be non-negative");
  }
  const auto chain = findChain(target, source);
  result.chain_code = chain.code;
  if (!chain.chainExists()) {
    return fail(TfTimeCode::chain_unavailable, chain.message.c_str(), chain.failed_frame);
  }
  result.frames = chain.frames;
  const auto time_ns = nanoseconds(stamp);
  for (const auto & edge : chain.edges) {
    const auto & samples = edges_.at(edge.child_frame).samples;
    if (edge.kind == TfKind::static_transform) {
      result.edges.push_back({edge.inverted, samples.begin()->second, std::nullopt});
      continue;
    }
    const auto next = samples.lower_bound(time_ns);
    if (next != samples.end() && next->first == time_ns) {
      result.edges.push_back({edge.inverted, next->second, std::nullopt});
      continue;
    }
    if (next == samples.begin() || next == samples.end()) {
      return fail(TfTimeCode::outside_range, "Query is outside an edge's dynamic sample range", edge.child_frame);
    }
    const auto previous = std::prev(next);
    if (next->first - previous->first > options.coverage.max_interpolation_gap_ns) {
      return fail(TfTimeCode::gap_too_large, "Dynamic samples are too far apart for interpolation", edge.child_frame);
    }
    result.edges.push_back({edge.inverted, previous->second, next->second});
  }
  result.code = TfTimeCode::covered;
  return result;
}

const char * tfInsertCodeName(TfInsertCode code) noexcept
{
  switch (code) {
    case TfInsertCode::inserted: return "inserted";
    case TfInsertCode::duplicate: return "duplicate";
    case TfInsertCode::invalid_scope: return "invalid_scope";
    case TfInsertCode::invalid_limits: return "invalid_limits";
    case TfInsertCode::invalid_kind: return "invalid_kind";
    case TfInsertCode::invalid_frame: return "invalid_frame";
    case TfInsertCode::self_transform: return "self_transform";
    case TfInsertCode::invalid_stamp: return "invalid_stamp";
    case TfInsertCode::invalid_translation: return "invalid_translation";
    case TfInsertCode::invalid_quaternion: return "invalid_quaternion";
    case TfInsertCode::missing_provenance: return "missing_provenance";
    case TfInsertCode::limit_exceeded: return "limit_exceeded";
    case TfInsertCode::parent_conflict: return "parent_conflict";
    case TfInsertCode::kind_conflict: return "kind_conflict";
    case TfInsertCode::static_conflict: return "static_conflict";
    case TfInsertCode::duplicate_stamp_conflict: return "duplicate_stamp_conflict";
    case TfInsertCode::cycle_detected: return "cycle_detected";
    case TfInsertCode::edge_blocked: return "edge_blocked";
  }
  return "unknown";
}

const char * tfChainCodeName(TfChainCode code) noexcept
{
  switch (code) {
    case TfChainCode::connected: return "connected";
    case TfChainCode::invalid_options: return "invalid_options";
    case TfChainCode::invalid_frame: return "invalid_frame";
    case TfChainCode::unknown_frame: return "unknown_frame";
    case TfChainCode::disconnected: return "disconnected";
    case TfChainCode::topology_conflict: return "topology_conflict";
  }
  return "unknown";
}

const char * tfCoverageStatusName(TfCoverageStatus status) noexcept
{
  switch (status) {
    case TfCoverageStatus::not_evaluated: return "not_evaluated";
    case TfCoverageStatus::static_only: return "static_only";
    case TfCoverageStatus::available: return "available";
    case TfCoverageStatus::no_common_time: return "no_common_time";
  }
  return "unknown";
}

const char * tfTimeCodeName(TfTimeCode code) noexcept
{
  switch (code) {
    case TfTimeCode::covered: return "covered";
    case TfTimeCode::invalid_options: return "invalid_options";
    case TfTimeCode::invalid_stamp: return "invalid_stamp";
    case TfTimeCode::invalid_scope: return "invalid_scope";
    case TfTimeCode::clock_mismatch: return "clock_mismatch";
    case TfTimeCode::segment_mismatch: return "segment_mismatch";
    case TfTimeCode::chain_unavailable: return "chain_unavailable";
    case TfTimeCode::outside_range: return "outside_range";
    case TfTimeCode::gap_too_large: return "gap_too_large";
  }
  return "unknown";
}

}  // namespace vln_memory
