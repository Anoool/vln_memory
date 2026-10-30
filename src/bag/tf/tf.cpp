#include <vln_memory/bag/tf/tf.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

#include <vln_memory/bag/common/sensor_names.hpp>

/**
 * @file
 * 按 child 和整数 header 时间索引；依次检查输入、拓扑、覆盖和查询端点。
 * 这里只返回原始证据，不进行位姿插值/组合，也不使用 latest 填补历史。
 */
namespace vln_memory
{
namespace
{

/** 将已校验的 stamp 转为整数纳秒；完整 int32 秒范围及差值均可用 int64 表示。 */
std::int64_t stampNanoseconds(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

/** 只检查时钟/数据段 ID 是否显式给出，不证明硬件同步或数据段的实际范围。 */
bool hasExplicitScope(const TfScope & scope)
{
  return !scope.clock_id.empty() && !scope.segment_id.empty();
}

/** 检查 frame 原始名称；拒绝空白/前导斜杠，不自动修正或合并不同名称。 */
bool isValidFrame(const std::string & frame)
{
  return !frame.empty() && frame.front() != '/' &&
         std::none_of(frame.begin(), frame.end(), [](char c) {return c <= ' ' || c == '\x7f';});
}

/** 逐项检查固定几何数组，不修改原始平移或旋转分量。 */
template<typename Values>
bool allFinite(const Values & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

/** 用 hypot 求四元数范数，避免直接平方大分量时溢出；输入顺序为 xyzw。 */
double quaternionNorm(const std::array<double, 4> & rotation_xyzw)
{
  return std::hypot(
    std::hypot(rotation_xyzw[0], rotation_xyzw[1]),
    std::hypot(rotation_xyzw[2], rotation_xyzw[3]));
}

/** 比较已校验样本的几何；识别 q/-q 等价，归一化只用于比较，不改写来源证据。 */
bool sameGeometry(const TfTransform & left, const TfTransform & right)
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (std::abs(left.translation_m[axis] - right.translation_m[axis]) > 1e-9) {
      return false;
    }
  }
  const double left_norm = quaternionNorm(left.rotation_xyzw);
  const double right_norm = quaternionNorm(right.rotation_xyzw);
  bool same_rotation = true;
  bool opposite_rotation = true;
  for (std::size_t component = 0; component < 4; ++component) {
    const double left_value = left.rotation_xyzw[component] / left_norm;
    const double right_value = right.rotation_xyzw[component] / right_norm;
    same_rotation = same_rotation && std::abs(left_value - right_value) <= 1e-9;
    opposite_rotation = opposite_rotation && std::abs(left_value + right_value) <= 1e-9;
  }
  return same_rotation || opposite_rotation;
}

/** 按原有优先级检查单条输入；成功只代表字段合法，此时尚未写入索引。 */
TfInsertResult validateTransform(const TfTransform & input)
{
  using Code = TfInsertCode;
  if (input.kind != TfKind::dynamic && input.kind != TfKind::static_transform) {
    return {Code::invalid_kind, "Explicit dynamic/static TF kind is required"};
  }
  if (!isValidFrame(input.parent_frame) || !isValidFrame(input.child_frame)) {
    return {Code::invalid_frame, "TF frames must be non-empty and have no leading slash or whitespace"};
  }
  if (input.parent_frame == input.child_frame) {
    return {Code::self_transform, "A frame cannot be its own parent"};
  }
  if (input.stamp.nanosec >= 1000000000U) {
    return {Code::invalid_stamp, "Stamp nanosec must be less than 1000000000"};
  }
  if (!allFinite(input.translation_m)) {
    return {Code::invalid_translation, "Translation must be finite, in meters"};
  }
  if (!allFinite(input.rotation_xyzw) ||
    std::abs(quaternionNorm(input.rotation_xyzw) - 1.0) > 1e-3)
  {
    return {Code::invalid_quaternion, "Quaternion must be finite and unit length (norm tolerance 1e-3)"};
  }
  if (!isResolvedTopicName(input.source_topic) || input.source_message_id.empty()) {
    return {Code::missing_provenance, "Resolved source topic and unique source message ID are required"};
  }
  return {Code::inserted, ""};
}

/** 双指针求有序闭区间交集；保留单点相交和内部空洞，输出独立拥有端点。 */
std::vector<TfTimeInterval> intersectIntervals(
  const std::vector<TfTimeInterval> & left, const std::vector<TfTimeInterval> & right)
{
  std::vector<TfTimeInterval> intersection;
  std::size_t left_index = 0;
  std::size_t right_index = 0;
  while (left_index < left.size() && right_index < right.size()) {
    const auto & left_interval = left[left_index];
    const auto & right_interval = right[right_index];
    auto overlap_start = left_interval.start;
    if (stampNanoseconds(right_interval.start) > stampNanoseconds(overlap_start)) {
      overlap_start = right_interval.start;
    }
    auto overlap_end = left_interval.end;
    if (stampNanoseconds(right_interval.end) < stampNanoseconds(overlap_end)) {
      overlap_end = right_interval.end;
    }
    if (stampNanoseconds(overlap_start) <= stampNanoseconds(overlap_end)) {
      intersection.push_back({overlap_start, overlap_end});
    }
    if (stampNanoseconds(left_interval.end) < stampNanoseconds(right_interval.end)) {
      ++left_index;
    } else {
      ++right_index;
    }
  }
  return intersection;
}

/** 扫描一条非空动态历史，按最大间隔切出闭区间；单样本只覆盖精确时刻。 */
std::vector<TfTimeInterval> buildCoverageIntervals(
  const detail::TfEdgeHistory & history, std::int64_t max_interpolation_gap_ns)
{
  std::vector<TfTimeInterval> intervals;
  auto previous = history.samples.begin();
  TfTimeInterval current{previous->second.stamp, previous->second.stamp};
  for (auto next = std::next(previous); next != history.samples.end(); ++next) {
    if (next->first - previous->first > max_interpolation_gap_ns) {
      intervals.push_back(current);
      current.start = next->second.stamp;
    }
    current.end = next->second.stamp;
    previous = next;
  }
  intervals.push_back(current);
  return intervals;
}

/** 从非空历史选择拥有型原始端点；不插值、不外推，失败时不写 selected。 */
TfTimeCode selectEdgeSamples(
  const detail::TfEdgeHistory & history, bool inverted, const SensorStamp & stamp,
  std::int64_t max_interpolation_gap_ns, TfSelectedEdge & selected)
{
  if (history.kind == TfKind::static_transform) {
    selected = {inverted, history.samples.begin()->second, std::nullopt};
    return TfTimeCode::covered;
  }
  const auto query_ns = stampNanoseconds(stamp);
  const auto next = history.samples.lower_bound(query_ns);
  if (next != history.samples.end() && next->first == query_ns) {
    selected = {inverted, next->second, std::nullopt};
    return TfTimeCode::covered;
  }
  if (next == history.samples.begin() || next == history.samples.end()) {
    return TfTimeCode::outside_range;
  }
  const auto previous = std::prev(next);
  if (next->first - previous->first > max_interpolation_gap_ns) {
    return TfTimeCode::gap_too_large;
  }
  selected = {inverted, previous->second, next->second};
  return TfTimeCode::covered;
}

/** 原位写查询失败并清空部分链；保留查询 stamp、链状态和路径诊断，不复制整份结果。 */
void setTimeFailure(
  TfTimeCheck & result, TfTimeCode code, const char * message, const std::string & failed_frame = "")
{
  result.code = code;
  result.message = message;
  result.failed_frame = failed_frame;
  result.edges.clear();
}

/** 按 stamp、显式上下文、域匹配、容差的顺序验证查询；失败原因写入 result。 */
bool validateTimeQuery(
  const TfScope & index_scope, const SensorStamp & stamp,
  const TfTimeOptions & options, TfTimeCheck & result)
{
  if (stamp.nanosec >= 1000000000U) {
    setTimeFailure(result, TfTimeCode::invalid_stamp, "Invalid query nanosec");
    return false;
  }
  if (options.clock_id.empty() || options.segment_id.empty()) {
    setTimeFailure(result, TfTimeCode::invalid_scope, "Query clock/segment must be explicit");
    return false;
  }
  if (options.clock_id != index_scope.clock_id) {
    setTimeFailure(result, TfTimeCode::clock_mismatch, "Query clock differs from TF index");
    return false;
  }
  if (options.segment_id != index_scope.segment_id) {
    setTimeFailure(result, TfTimeCode::segment_mismatch, "Query data segment differs from TF index");
    return false;
  }
  if (options.max_interpolation_gap_ns < 0) {
    setTimeFailure(result, TfTimeCode::invalid_options, "Maximum interpolation gap must be non-negative");
    return false;
  }
  return true;
}

}  // namespace

/** 独立持有声明的时钟/数据段及上限；无效配置留到插入/查询时报告。 */
TfIndex::TfIndex(TfScope scope, TfLimits limits)
: scope_(std::move(scope)), limits_(limits)
{
}

/** 记录矛盾 child 并返回错误；不删除旧样本，后续涉及该节点的查询将失败。 */
TfInsertResult TfIndex::blockFrame(
  const std::string & child, TfInsertCode code, const char * message)
{
  blocked_frames_.insert(child);
  return {code, message};
}

/** 沿已接受的无环父链查找 child；只检查拟新增边，不修改任何索引状态。 */
bool TfIndex::wouldCreateCycle(const std::string & parent, const std::string & child) const
{
  std::string ancestor = parent;
  while (true) {
    if (ancestor == child) {
      return true;
    }
    const auto edge = edges_.find(ancestor);
    if (edge == edges_.end()) {
      return false;
    }
    ancestor = edge->second.parent_frame;
  }
}

/** 完成输入、冲突、资源检查后才写入；重复不增计数，系统分配异常交由调用方终止构建。 */
TfInsertResult TfIndex::insert(TfTransform input)
{
  using Code = TfInsertCode;
  if (!hasExplicitScope(scope_)) {
    return {Code::invalid_scope, "Index clock_id and segment_id are required"};
  }
  if (limits_.max_edges == 0 || limits_.max_samples == 0) {
    return {Code::invalid_limits, "TF index limits must be positive"};
  }
  const auto validated = validateTransform(input);
  if (!validated.accepted()) {
    return validated;
  }
  if (blocked_frames_.count(input.child_frame) != 0) {
    return {Code::edge_blocked, "Child frame was blocked by an earlier TF contradiction"};
  }

  // 一个 child 只对应一个 parent/类型；同刻动态或重复静态按几何去重。
  auto edge = edges_.find(input.child_frame);
  if (edge != edges_.end()) {
    if (edge->second.parent_frame != input.parent_frame) {
      return blockFrame(
        input.child_frame, Code::parent_conflict, "Child frame has multiple parents; split the data segment");
    }
    if (edge->second.kind != input.kind) {
      return blockFrame(
        input.child_frame, Code::kind_conflict, "Child frame appears in both dynamic and static TF");
    }
    const auto & samples = edge->second.samples;
    auto previous = samples.begin();
    if (input.kind == TfKind::dynamic) {
      previous = samples.find(stampNanoseconds(input.stamp));
    }
    if (previous != samples.end()) {
      if (sameGeometry(previous->second, input)) {
        return {Code::duplicate, "Consistent duplicate TF"};
      }
      auto conflict_code = Code::duplicate_stamp_conflict;
      if (input.kind == TfKind::static_transform) {
        conflict_code = Code::static_conflict;
      }
      return blockFrame(
        input.child_frame, conflict_code, "Conflicting transform values; no last-message overwrite");
    }
  } else {
    if (wouldCreateCycle(input.parent_frame, input.child_frame)) {
      return blockFrame(input.child_frame, Code::cycle_detected, "TF edge would create a cycle");
    }
    if (edges_.size() >= limits_.max_edges) {
      return {Code::limit_exceeded, "TF edge limit exceeded"};
    }
  }
  if (sample_count_ >= limits_.max_samples) {
    return {Code::limit_exceeded, "TF sample limit exceeded"};
  }

  // 全部业务检查通过后再修改拥有型容器；不在分配异常后继续使用半建索引。
  if (edge == edges_.end()) {
    edge = edges_.emplace(
      input.child_frame, detail::TfEdgeHistory{input.parent_frame, input.kind, {}}).first;
  }
  frames_.insert(input.parent_frame);
  frames_.insert(input.child_frame);
  const auto stamp_ns = stampNanoseconds(input.stamp);
  edge->second.samples.emplace(stamp_ns, std::move(input));
  ++sample_count_;
  return {Code::inserted, ""};
}

/** 从 frame 收集到根的完整祖先；遇阻断节点保留 failed_frame，不缩短到局部可用路径。 */
bool TfIndex::collectAncestors(
  std::string frame, std::vector<std::string> & path, std::string & failed_frame) const
{
  while (true) {
    if (blocked_frames_.count(frame) != 0) {
      failed_frame = frame;
      return false;
    }
    path.push_back(frame);
    const auto edge = edges_.find(frame);
    if (edge == edges_.end()) {
      return true;
    }
    frame = edge->second.parent_frame;
  }
}

/** 为已知 child 生成拥有型边摘要；动态首末包络不是完整覆盖，静态包络为空。 */
TfChainEdge TfIndex::describeEdge(const std::string & child, bool inverted) const
{
  const auto & history = edges_.at(child);
  TfChainEdge edge;
  edge.parent_frame = history.parent_frame;
  edge.child_frame = child;
  edge.kind = history.kind;
  edge.inverted = inverted;
  edge.sample_count = history.samples.size();
  if (history.kind == TfKind::dynamic) {
    edge.dynamic_bounds = TfTimeInterval{
      history.samples.begin()->second.stamp, history.samples.rbegin()->second.stamp};
  }
  return edge;
}

/** 验证查询两端并按最近共同祖先生成 source->target 路径；此步骤不计算时间覆盖。 */
TfChainReport TfIndex::findChain(const std::string & target, const std::string & source) const
{
  TfChainReport report;
  report.clock_id = scope_.clock_id;
  report.segment_id = scope_.segment_id;
  report.target_frame = target;
  report.source_frame = source;
  if (!hasExplicitScope(scope_) || limits_.max_edges == 0 || limits_.max_samples == 0) {
    report.code = TfChainCode::invalid_options;
    report.message = "TF index requires a scope and positive resource limits";
    return report;
  }
  if (!isValidFrame(target) || !isValidFrame(source)) {
    report.failed_frame = isValidFrame(target) ? source : target;
    report.code = TfChainCode::invalid_frame;
    report.message = "Query frames are invalid";
    return report;
  }
  if (frames_.count(target) == 0 || frames_.count(source) == 0) {
    report.failed_frame = frames_.count(target) == 0 ? target : source;
    report.code = TfChainCode::unknown_frame;
    report.message = "A requested frame is absent from accepted TF records";
    return report;
  }

  std::vector<std::string> source_path;
  std::vector<std::string> target_path;
  if (!collectAncestors(source, source_path, report.failed_frame) ||
    !collectAncestors(target, target_path, report.failed_frame))
  {
    report.code = TfChainCode::topology_conflict;
    report.message = "A query ancestor is blocked by contradictory TF records";
    return report;
  }

  std::map<std::string, std::size_t> source_positions;
  for (std::size_t index = 0; index < source_path.size(); ++index) {
    source_positions.emplace(source_path[index], index);
  }
  std::size_t target_common_index = 0;
  while (target_common_index < target_path.size() &&
    source_positions.count(target_path[target_common_index]) == 0)
  {
    ++target_common_index;
  }
  if (target_common_index == target_path.size()) {
    report.code = TfChainCode::disconnected;
    report.message = "Frames exist but belong to disconnected TF trees";
    return report;
  }
  const auto source_common_index = source_positions.at(target_path[target_common_index]);
  for (std::size_t index = 0; index <= source_common_index; ++index) {
    report.frames.push_back(source_path[index]);
  }
  for (std::size_t index = 0; index < source_common_index; ++index) {
    report.edges.push_back(describeEdge(source_path[index], false));
  }
  for (std::size_t index = target_common_index; index > 0; --index) {
    report.edges.push_back(describeEdge(target_path[index - 1], true));
    report.frames.push_back(target_path[index - 1]);
  }
  report.code = TfChainCode::connected;
  return report;
}

/** 先查链，再扫描各动态历史求区间交集；静态/identity 与无共同动态覆盖分别报告。 */
TfChainReport TfIndex::inspectChain(
  const std::string & target, const std::string & source, const TfCoverageOptions & options) const
{
  auto report = findChain(target, source);
  if (!report.chainExists()) {
    return report;
  }
  if (options.max_interpolation_gap_ns < 0) {
    report.code = TfChainCode::invalid_options;
    report.message = "Maximum interpolation gap must be non-negative";
    return report;
  }
  bool has_dynamic_edge = false;
  for (const auto & edge : report.edges) {
    if (edge.kind == TfKind::static_transform) {
      continue;
    }
    auto intervals = buildCoverageIntervals(
      edges_.at(edge.child_frame), options.max_interpolation_gap_ns);
    if (!has_dynamic_edge) {
      report.intervals = std::move(intervals);
    } else {
      report.intervals = intersectIntervals(report.intervals, intervals);
    }
    has_dynamic_edge = true;
    if (report.intervals.empty()) {
      break;
    }
  }
  if (!has_dynamic_edge) {
    report.coverage_status = TfCoverageStatus::static_only;
  } else if (report.intervals.empty()) {
    report.coverage_status = TfCoverageStatus::no_common_time;
  } else {
    report.coverage_status = TfCoverageStatus::available;
  }
  return report;
}

/** 校验时刻/上下文后按路径选择端点；失败清空部分链，成功仍不包含求解后的位姿。 */
TfTimeCheck TfIndex::checkTime(
  const std::string & target, const std::string & source, const SensorStamp & stamp,
  const TfTimeOptions & options) const
{
  TfTimeCheck result;
  result.stamp = stamp;
  if (!validateTimeQuery(scope_, stamp, options, result)) {
    return result;
  }
  const auto chain = findChain(target, source);
  result.chain_code = chain.code;
  if (!chain.chainExists()) {
    setTimeFailure(result, TfTimeCode::chain_unavailable, chain.message.c_str(), chain.failed_frame);
    return result;
  }
  result.frames = chain.frames;
  for (const auto & edge : chain.edges) {
    TfSelectedEdge selected;
    const auto code = selectEdgeSamples(
      edges_.at(edge.child_frame), edge.inverted, stamp, options.max_interpolation_gap_ns, selected);
    if (code == TfTimeCode::outside_range) {
      setTimeFailure(
        result, code, "Query is outside an edge's dynamic sample range", edge.child_frame);
      return result;
    }
    if (code == TfTimeCode::gap_too_large) {
      setTimeFailure(
        result, code, "Dynamic samples are too far apart for interpolation", edge.child_frame);
      return result;
    }
    result.edges.push_back(std::move(selected));
  }
  result.code = TfTimeCode::covered;
  return result;
}

/** 返回插入状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
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

/** 返回拓扑状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
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

/** 返回覆盖状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
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

/** 返回时刻检查状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
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
