#include <vln_memory/bag/tf/tf.hpp>

#include <algorithm>
#include <cmath>

namespace vln_memory
{
namespace
{
using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;

/** @brief 已校验时刻转整数纳秒，差值在 int64 范围内。 */
std::int64_t nanoseconds(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

/** @brief 对求解副本归一化，原始 TF 样本保留不变。 */
Quaternion unit(Quaternion q)
{
  const double length = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  for (auto & component : q) {component /= length;}
  return q;
}

/** @brief 四元数内积用于选择等价符号及计算旋转角。 */
double dot(const Quaternion & a, const Quaternion & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

/** @brief 先施加 b 旋转，再施加 a 旋转，分量顺序为 xyzw。 */
Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return {
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

/** @brief 单位四元数的逆旋转。 */
Quaternion conjugate(const Quaternion & q)
{
  return {-q[0], -q[1], -q[2], q[3]};
}

/** @brief 在给定旋转下变换向量，保留向量单位。 */
Vector3 rotate(const Quaternion & q, const Vector3 & v)
{
  const auto rotated = multiply(multiply(q, {v[0], v[1], v[2], 0}), conjugate(q));
  return {rotated[0], rotated[1], rotated[2]};
}

/** @brief 两个单位四元数沿短弧插值；接近同向时归一化线性插值。 */
Quaternion interpolateRotation(const Quaternion & before, Quaternion after, double ratio)
{
  double cosine = dot(before, after);
  if (cosine < 0.0) {
    for (auto & component : after) {component = -component;}
    cosine = -cosine;
  }
  double before_weight = 1.0 - ratio;
  double after_weight = ratio;
  if (cosine < 0.9995) {
    const double angle = std::acos(cosine);
    before_weight = std::sin((1.0 - ratio) * angle) / std::sin(angle);
    after_weight = std::sin(ratio * angle) / std::sin(angle);
  }
  Quaternion result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = before_weight * before[i] + after_weight * after[i];
  }
  return unit(result);
}

/** @brief 可省略的上限须为有限正值，旋转单位为弧度。 */
bool validLimit(const std::optional<double> & value)
{
  return !value || (std::isfinite(*value) && *value > 0.0);
}

/** @brief 在原父坐标系比较端点，不把四元数符号变化当成旋转跳变。 */
bool exceedsJump(const TfPose & before, const TfPose & after, const TfJumpLimits & limits)
{
  if (limits.translation_m) {
    const auto & a = before.translation_m;
    const auto & b = after.translation_m;
    if (std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) > *limits.translation_m) {
      return true;
    }
  }
  if (!limits.rotation_rad) {return false;}
  const auto relative = multiply(conjugate(before.rotation_xyzw), after.rotation_xyzw);
  const double angle = 2.0 * std::atan2(
    std::hypot(relative[0], relative[1], relative[2]), std::abs(relative[3]));
  return angle > *limits.rotation_rad;
}

/** @brief 在原父坐标系内求 T_parent_child，拒绝超过显式上限的插值。 */
std::optional<TfPose> parentPose(
  const TfSelectedEdge & edge, const SensorStamp & stamp, const TfJumpLimits & limits)
{
  TfPose result{edge.before.translation_m, unit(edge.before.rotation_xyzw)};
  if (!edge.after) {return result;}
  const TfPose after{edge.after->translation_m, unit(edge.after->rotation_xyzw)};
  if (exceedsJump(result, after, limits)) {return std::nullopt;}
  const auto before_ns = nanoseconds(edge.before.stamp);
  const double ratio = static_cast<double>(nanoseconds(stamp) - before_ns) /
    static_cast<double>(nanoseconds(edge.after->stamp) - before_ns);
  for (std::size_t i = 0; i < result.translation_m.size(); ++i) {
    result.translation_m[i] = (1.0 - ratio) * result.translation_m[i] +
      ratio * after.translation_m[i];
  }
  result.rotation_xyzw = interpolateRotation(result.rotation_xyzw, after.rotation_xyzw, ratio);
  return result;
}

/** @brief 反转坐标方向，逆平移为 -R^-1 t。 */
TfPose inverse(const TfPose & pose)
{
  const auto rotation = conjugate(pose.rotation_xyzw);
  auto translation = rotate(rotation, pose.translation_m);
  for (auto & value : translation) {value = -value;}
  return {translation, rotation};
}

/** @brief 将左边变换施加在累计变换之后，旋转累计平移再相加。 */
TfPose compose(const TfPose & left, const TfPose & right)
{
  auto translation = rotate(left.rotation_xyzw, right.translation_m);
  for (std::size_t i = 0; i < translation.size(); ++i) {
    translation[i] += left.translation_m[i];
  }
  return {translation, unit(multiply(left.rotation_xyzw, right.rotation_xyzw))};
}

/** @brief 非有限累计结果不得作为有效位姿导出。 */
bool finitePose(const TfPose & pose)
{
  const auto finite = [](double value) {return std::isfinite(value);};
  return std::all_of(pose.translation_m.begin(), pose.translation_m.end(), finite) &&
    std::all_of(pose.rotation_xyzw.begin(), pose.rotation_xyzw.end(), finite);
}

/** @brief 校验公开点变换使用的位姿，保持正逆变换的输入约束一致。 */
bool validRigidPose(const TfPose & pose)
{
  if (!finitePose(pose)) {return false;}
  const auto & q = pose.rotation_xyzw;
  const double length = std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  return std::abs(length - 1.0) <= 1e-9;
}

}  // namespace

/** @brief 复用位姿组合的旋转运算，校验公开输入与变换后的数值。 */
std::optional<std::array<double, 3>> transformTfPoint(
  const TfPose & pose, const std::array<double, 3> & point_source_m) noexcept
{
  if (!validRigidPose(pose)) {return std::nullopt;}

  auto point_target_m = rotate(pose.rotation_xyzw, point_source_m);
  for (std::size_t axis = 0; axis < point_target_m.size(); ++axis) {
    point_target_m[axis] += pose.translation_m[axis];
    if (!std::isfinite(point_target_m[axis])) {return std::nullopt;}
  }
  return point_target_m;
}

/** @brief 先扣除目标帧平移，再施加逆旋转，返回独立拥有的源帧坐标。 */
std::optional<std::array<double, 3>> inverseTransformTfPoint(
  const TfPose & pose, const std::array<double, 3> & point_target_m) noexcept
{
  if (!validRigidPose(pose)) {return std::nullopt;}
  auto offset_m = point_target_m;
  for (std::size_t axis = 0; axis < offset_m.size(); ++axis) {
    offset_m[axis] -= pose.translation_m[axis];
  }
  const auto point_source_m = rotate(conjugate(pose.rotation_xyzw), offset_m);
  if (!std::all_of(point_source_m.begin(), point_source_m.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return std::nullopt;
  }
  return point_source_m;
}

/** @brief 按 checkTime 的 source 到 target 顺序组合，结果独立拥有证据和数值。 */
TfPoseResult TfIndex::lookupPose(
  const std::string & target, const std::string & source, const SensorStamp & stamp,
  const TfTimeOptions & options, const TfJumpLimits & jump_limits) const
{
  TfPoseResult result;
  result.time = checkTime(target, source, stamp, options);
  if (!result.time.covered()) {return result;}
  if (!validLimit(jump_limits.translation_m) || !validLimit(jump_limits.rotation_rad)) {
    result.code = TfPoseCode::invalid_limits;
    return result;
  }
  TfPose accumulated;
  for (const auto & edge : result.time.edges) {
    auto pose = parentPose(edge, stamp, jump_limits);
    if (!pose) {
      result.code = TfPoseCode::jump_too_large;
      result.failed_frame = edge.before.child_frame;
      return result;
    }
    if (edge.inverted) {*pose = inverse(*pose);}
    accumulated = compose(*pose, accumulated);
    if (!finitePose(accumulated)) {
      result.code = TfPoseCode::numerical_failure;
      result.failed_frame = edge.before.child_frame;
      return result;
    }
  }
  result.code = TfPoseCode::ready;
  result.pose = accumulated;
  return result;
}

/** @brief 序列化用的稳定状态名称。 */
const char * tfPoseCodeName(TfPoseCode code) noexcept
{
  switch (code) {
    case TfPoseCode::ready: return "ready";
    case TfPoseCode::time_unavailable: return "time_unavailable";
    case TfPoseCode::invalid_limits: return "invalid_limits";
    case TfPoseCode::jump_too_large: return "jump_too_large";
    case TfPoseCode::numerical_failure: return "numerical_failure";
  }
  return "unknown";
}
}  // namespace vln_memory
