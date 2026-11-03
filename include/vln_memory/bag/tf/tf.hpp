#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <vln_memory/bag/common/sensor_types.hpp>

/**
 * @file
 * 离线 TF 契约：先建立单段历史索引，再检查坐标链、共同覆盖和采样端点。
 * 报告不包含组合后的 map 位姿；数值插值/求逆/组合必须由后续求解步骤完成。
 */
namespace vln_memory
{

enum class TfKind {dynamic, static_transform};

/** @brief 一个索引只属于一个明确的时钟和数据段，ID 由 builder 管理，不从 stamp 猜测。 */
struct TfScope
{
  std::string clock_id;
  std::string segment_id;
};

/**
 * @brief 原始 T_parent_child：把 child 中的点变到 parent，单位米、四元数 xyzw。
 * @note 字符串/数组独立持有。保留原始 stamp 和近单位四元数，不归一化或改写证据。
 *       kind 必须由真实话题与显式配置确定，不能靠时间戳是否为零判断静态。
 */
struct TfTransform
{
  TfKind kind{TfKind::dynamic};
  SensorStamp stamp;
  std::string parent_frame;
  std::string child_frame;
  std::array<double, 3> translation_m{};
  std::array<double, 4> rotation_xyzw{0, 0, 0, 1};
  // 来源直接与样本并列；话题通过公共名称规则，ID/条目序号由 reader 提供。
  std::string source_topic;
  std::string source_message_id;
  std::size_t source_transform_index{0};
};

/** @brief 索引资源上限，重复广播不计入唯一样本数；不约束之前的 ROS 反序列化内存。 */
struct TfLimits
{
  std::size_t max_edges{4096};
  std::size_t max_samples{1000000};
};

enum class TfInsertCode
{
  inserted, duplicate, invalid_scope, invalid_limits, invalid_kind, invalid_frame,
  self_transform, invalid_stamp, invalid_translation, invalid_quaternion, missing_provenance,
  limit_exceeded, parent_conflict, kind_conflict, static_conflict, duplicate_stamp_conflict,
  cycle_detected, edge_blocked
};

struct TfInsertResult
{
  TfInsertCode code{TfInsertCode::invalid_scope};
  std::string message;
  /** @brief inserted/duplicate 均为接受；不表示整个数据段或坐标链有效。 */
  bool accepted() const noexcept
  {
    return code == TfInsertCode::inserted || code == TfInsertCode::duplicate;
  }
};

/** @brief 闭区间；单样本只覆盖这一个精确时刻，不能代表持续发布。 */
struct TfTimeInterval
{
  SensorStamp start;
  SensorStamp end;
};

struct TfCoverageOptions
{
  // 允许相邻动态样本间插值的最大间隔。0 只接受精确采样点，无隐含默认阈值。
  std::int64_t max_interpolation_gap_ns{0};
};

enum class TfChainCode
{
  connected, invalid_options, invalid_frame, unknown_frame, disconnected, topology_conflict
};
enum class TfCoverageStatus {not_evaluated, static_only, available, no_common_time};

/** @brief 路径中一条原始父子边；inverted=true 表示 source->target 方向需要求逆。 */
struct TfChainEdge
{
  std::string parent_frame;
  std::string child_frame;
  TfKind kind{TfKind::dynamic};
  bool inverted{false};
  std::size_t sample_count{0};
  // 静态边为空。动态首末包络可能有洞，真正覆盖必须看整条链的 intervals。
  std::optional<TfTimeInterval> dynamic_bounds;
};

struct TfChainReport
{
  TfChainCode code{TfChainCode::invalid_options};
  std::string message;
  std::string clock_id;
  std::string segment_id;
  std::string failed_frame;  // 缺失/无效/冲突 frame，方便 issues 报告定位。
  std::string target_frame;
  std::string source_frame;
  std::vector<std::string> frames;  // 从 source 到 target，含两端。
  std::vector<TfChainEdge> edges;
  TfCoverageStatus coverage_status{TfCoverageStatus::not_evaluated};
  std::vector<TfTimeInterval> intervals;  // 有序、不相交，取所有动态边的交集。
  /** @brief 只判断拓扑连通，时间可用性另查 coverage_status/intervals。 */
  bool chainExists() const noexcept {return code == TfChainCode::connected;}
};

struct TfTimeOptions
{
  // 查询图像的时钟/数据段声明，必须与索引一致；不从 stamp 推断。
  std::string clock_id;
  std::string segment_id;
  // 与 inspectChain 的覆盖策略含义相同，直接填写非负整数纳秒。
  std::int64_t max_interpolation_gap_ns{0};
};

enum class TfTimeCode
{
  covered, invalid_options, invalid_stamp, invalid_scope, clock_mismatch, segment_mismatch,
  chain_unavailable, outside_range, gap_too_large
};

/** @brief 原始精确/静态样本或插值的前后样本，为后续位姿求解保留完整来源。 */
struct TfSelectedEdge
{
  bool inverted{false};
  TfTransform before;
  std::optional<TfTransform> after;  // 仅需要插值时有值；本模块不计算插值后的位姿。
};

struct TfTimeCheck
{
  TfTimeCode code{TfTimeCode::invalid_options};
  std::string message;
  SensorStamp stamp;
  TfChainCode chain_code{TfChainCode::invalid_options};
  std::string failed_frame;
  std::vector<std::string> frames;
  std::vector<TfSelectedEdge> edges;  // 失败为空，不把部分链作为可用链输出。
  /** @brief 只说明端点可按策略使用，不表示已有插值/组合后的 map 位姿。 */
  bool covered() const noexcept {return code == TfTimeCode::covered;}
};

/** @brief 返回插入状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
const char * tfInsertCodeName(TfInsertCode code) noexcept;
/** @brief 返回拓扑状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
const char * tfChainCodeName(TfChainCode code) noexcept;
/** @brief 返回覆盖状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
const char * tfCoverageStatusName(TfCoverageStatus status) noexcept;
/** @brief 返回时刻检查状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
const char * tfTimeCodeName(TfTimeCode code) noexcept;

enum class TfPoseCode {ready, time_unavailable, invalid_limits, jump_too_large, numerical_failure};

/** @brief 只限制动态插值端点之间的变化，省略表示该项不设上限。 */
struct TfJumpLimits
{
  std::optional<double> translation_m;
  std::optional<double> rotation_rad;
};

/** @brief T_target_source 的数值，平移为米，旋转为单位四元数 xyzw。 */
struct TfPose
{
  std::array<double, 3> translation_m{};
  std::array<double, 4> rotation_xyzw{0, 0, 0, 1};
};

struct TfPoseResult
{
  TfPoseCode code{TfPoseCode::time_unavailable};
  TfTimeCheck time;
  std::optional<TfPose> pose;
  std::string failed_frame;  // 数值求解失败时的原始边 child。
  /** @brief 仅当整条链求解成功才返回 true。 */
  bool ok() const noexcept {return pose.has_value();}
};

/** @brief 返回静态状态名称，无需释放。 */
const char * tfPoseCodeName(TfPoseCode code) noexcept;

namespace detail
{

/** @brief 索引内部的一条边历史；样本按整数纳秒排序，至少包含一个已接受样本。 */
struct TfEdgeHistory
{
  std::string parent_frame;
  TfKind kind{TfKind::dynamic};
  std::map<std::int64_t, TfTransform> samples;
};

}  // namespace detail

/**
 * @brief 一个数据段的离线历史索引，保留所有接受样本，不使用短时 TF 缓存。
 * @note 先 insert，再只读查询；不得与 insert 并发访问。只读方法可并发调用。
 *       矛盾父子/值/类型或环会阻断相关节点，必须重新分段建立索引才能恢复。
 *       常见数值错误/资源上限失败不插入；系统分配异常继续通过 C++ 异常上报。
 */
class TfIndex
{
public:
  /** @brief 保存上下文/上限；无效配置由 insert/查询报告，不猜测默认域。 */
  explicit TfIndex(TfScope scope, TfLimits limits = {});
  /** @brief 返回索引持有的上下文引用，其生命周期随索引。 */
  const TfScope & scope() const noexcept {return scope_;}
  /** @brief 返回已接受的唯一 child 边数，重复广播不增加计数。 */
  std::size_t edgeCount() const noexcept {return edges_.size();}
  /** @brief 返回唯一原始样本数，不包含重复广播和被拒绝输入。 */
  std::size_t sampleCount() const noexcept {return sample_count_;}

  /** @brief 单条插入；重复几何保留首次来源，重复源消息本身由 builder 保存证据。 */
  TfInsertResult insert(TfTransform transform);

  /** @brief 检查拓扑及历史覆盖；链存在与时间有共同覆盖是两个独立结论。 */
  TfChainReport inspectChain(
    const std::string & target, const std::string & source, const TfCoverageOptions & options) const;

  /**
   * @brief 在图像采样时刻检查链，并选择有来源的精确样本或插值端点。
   * @note 无外推、无 latest、无最近样本替代；零时刻就是实际零时刻。
   *       不扫描整段区间，每条动态边用有序索引寻找相邻样本。
   */
  TfTimeCheck checkTime(
    const std::string & target, const std::string & source, const SensorStamp & stamp,
    const TfTimeOptions & options) const;

  /** @brief 复用 checkTime 的拥有型证据，返回 source 到 target 的完整数值变换。 */
  TfPoseResult lookupPose(
    const std::string & target, const std::string & source, const SensorStamp & stamp,
    const TfTimeOptions & options, const TfJumpLimits & jump_limits = {}) const;

  /**
   * @brief 对请求链中的动态边取两倍下中位间隔，返回各边估计的最大值。
   * @note 每边至少需三个样本。无足够样本返回 0；只在读完当前数据段后调用一次。
   */
  std::int64_t inferInterpolationGapNs(
    const std::string & target, const std::vector<std::string> & sources) const;

private:
  /** @brief 只检查拓扑并生成 source->target 路径，不计算覆盖或几何。 */
  TfChainReport findChain(const std::string & target, const std::string & source) const;
  /** @brief 收集到根的完整祖先，遇阻断节点写入 failed_frame；输出独立持有名称。 */
  bool collectAncestors(
    std::string frame, std::vector<std::string> & path, std::string & failed_frame) const;
  /** @brief 为已知 child 复制一条边摘要，保留动态首末包络和方向标志。 */
  TfChainEdge describeEdge(const std::string & child, bool inverted) const;
  /** @brief 检查新增 parent->child 是否闭环，不修改索引。 */
  bool wouldCreateCycle(const std::string & parent, const std::string & child) const;
  /** @brief 标记矛盾 child，保留旧样本并返回对应错误；后续相关查询被阻断。 */
  TfInsertResult blockFrame(const std::string & child, TfInsertCode code, const char * message);
  TfScope scope_;
  TfLimits limits_;
  std::map<std::string, detail::TfEdgeHistory> edges_;  // child -> 唯一 parent 与按 header 时间排序的样本。
  std::set<std::string> frames_;
  std::set<std::string> blocked_frames_;
  std::size_t sample_count_{0};
};

}  // namespace vln_memory
