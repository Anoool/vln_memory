#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "vln_memory/bag/common/sensor_types.hpp"

namespace vln_memory
{

enum class TfKind {dynamic, static_transform};

/** @brief 一个索引只属于一个明确的时钟和数据段，ID 由 builder 管理，不从 stamp 猜测。 */
struct TfScope
{
  std::string clock_id;
  std::string segment_id;
};

/** @brief 来源消息 ID 与消息内序号共同定位原始 TransformStamped。 */
struct TfSource
{
  std::string topic;
  std::string message_id;
  std::size_t transform_index{0};
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
  TfSource source;
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
  TfScope scope;
  std::string failed_frame;  // 缺失/无效/冲突 frame，方便 issues 报告定位。
  std::string target_frame;
  std::string source_frame;
  std::vector<std::string> frames;  // 从 source 到 target，含两端。
  std::vector<TfChainEdge> edges;
  TfCoverageStatus coverage_status{TfCoverageStatus::not_evaluated};
  std::vector<TfTimeInterval> intervals;  // 有序、不相交，取所有动态边的交集。
  bool chainExists() const noexcept {return code == TfChainCode::connected;}
};

struct TfTimeOptions
{
  TfScope scope;  // 查询图像的时钟/数据段声明，必须与索引一致。
  TfCoverageOptions coverage;
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
  bool covered() const noexcept {return code == TfTimeCode::covered;}
};

const char * tfInsertCodeName(TfInsertCode code) noexcept;
const char * tfChainCodeName(TfChainCode code) noexcept;
const char * tfCoverageStatusName(TfCoverageStatus status) noexcept;
const char * tfTimeCodeName(TfTimeCode code) noexcept;

/**
 * @brief 一个数据段的离线历史索引，保留所有接受样本，不使用短时 TF 缓存。
 * @note 先 insert，再只读查询；不得与 insert 并发访问。只读方法可并发调用。
 *       矛盾父子/值/类型或环会阻断相关节点，必须重新分段建立索引才能恢复。
 *       常见数值错误/资源上限失败不插入；系统分配异常继续通过 C++ 异常上报。
 */
class TfIndex
{
public:
  explicit TfIndex(TfScope scope, TfLimits limits = {});
  const TfScope & scope() const noexcept {return scope_;}
  std::size_t edgeCount() const noexcept {return edges_.size();}
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

private:
  struct Edge
  {
    std::string parent;
    TfKind kind{TfKind::dynamic};
    std::map<std::int64_t, TfTransform> samples;
  };

  TfChainReport findChain(const std::string & target, const std::string & source) const;
  TfScope scope_;
  TfLimits limits_;
  std::map<std::string, Edge> edges_;  // child -> 唯一 parent 与按 header 时间排序的样本。
  std::set<std::string> frames_;
  std::set<std::string> blocked_frames_;
  std::size_t sample_count_{0};
};

}  // namespace vln_memory
