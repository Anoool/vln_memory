#include "vln_memory/bag/builder/camera_info_index.hpp"

#include <algorithm>
#include <cmath>

namespace vln_memory
{
namespace
{
/** @brief 合法 stamp 转整数纳秒，不经过浮点秒。 */
std::int64_t stampNs(const SensorStamp & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

/** @brief 比较实际几何/ROI，不比较广播时间与来源；非有限参数不会被当成一致标定。 */
bool sameModel(const CameraInfoData & left, const CameraInfoData & right)
{
  return left.frame_id == right.frame_id && left.width == right.width && left.height == right.height &&
         left.distortion_model == right.distortion_model && left.d == right.d && left.k == right.k &&
         left.r == right.r && left.p == right.p && left.binning_x == right.binning_x &&
         left.binning_y == right.binning_y && left.roi.x_offset == right.roi.x_offset &&
         left.roi.y_offset == right.roi.y_offset && left.roi.width == right.roi.width &&
         left.roi.height == right.roi.height && left.roi.do_rectify == right.roi.do_rectify;
}

/** @brief 分析一个话题的全部记录；无效时间使整组选择拒绝，避免未知版本边界。 */
void analyzeHistory(
  detail::CalibrationHistory & history, const std::vector<BagCameraInfoRecord> & records)
{
  auto & ordered = history.ordered;
  std::sort(ordered.begin(), ordered.end(), [&records](std::size_t left, std::size_t right) {
    const auto left_ns = stampNs(records[left].data.stamp);
    const auto right_ns = stampNs(records[right].data.stamp);
    if (left_ns != right_ns) {return left_ns < right_ns;}
    return records[left].source.message_order < records[right].source.message_order;
  });
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    const auto & record = records[ordered[index]];
    if (record.data.stamp.nanosec >= 1000000000U) {history.invalid_stamp = true;}
    if (!sameModel(records[ordered.front()].data, record.data)) {history.static_consistent = false;}
    if (index > 0) {
      const auto & previous = records[ordered[index - 1]];
      if (stampNs(previous.data.stamp) == stampNs(record.data.stamp) && !sameModel(previous.data, record.data)) {
        history.ambiguous_stamps.insert(stampNs(record.data.stamp));
      }
    }
  }
}

/** @brief 按有序候选查最近时刻，同刻选原始顺序最早者，等距选较早时刻。 */
std::size_t nearestRecord(
  const detail::CalibrationHistory & history, const std::vector<BagCameraInfoRecord> & records,
  const SensorStamp & stamp)
{
  const auto & ordered = history.ordered;
  const auto by_stamp = [&records](std::size_t index, std::int64_t value) {
    return stampNs(records[index].data.stamp) < value;
  };
  const auto query = stampNs(stamp);
  const auto next = std::lower_bound(ordered.begin(), ordered.end(), query, by_stamp);
  if (next == ordered.begin()) {return *next;}
  const auto previous_ns = stampNs(records[*std::prev(next)].data.stamp);
  const auto previous = std::lower_bound(ordered.begin(), next, previous_ns, by_stamp);
  if (next == ordered.end() || query - previous_ns <= stampNs(records[*next].data.stamp) - query) {
    return *previous;
  }
  return *next;
}

}  // namespace

/** @brief 借用记录，仅为索引保存整数位置；调用方在使用期间保持原容器不变。 */
CameraInfoIndex::CameraInfoIndex(const std::vector<BagCameraInfoRecord> & records)
: records_(records)
{
  for (std::size_t index = 0; index < records.size(); ++index) {
    histories_[records[index].data.topic].ordered.push_back(index);
  }
  for (auto & entry : histories_) {analyzeHistory(entry.second, records_);}
}

/** @brief 选择不跳过无效候选，静态模式不掩盖模型变化，最后复用实际相机匹配规则。 */
CameraInfoSelection CameraInfoIndex::select(
  const CameraImageReference & image, const CameraInfoMatchOptions & options) const
{
  CameraInfoSelection result;
  const auto found = histories_.find(options.camera_info_topic);
  if (found == histories_.end() || found->second.ordered.empty()) {return result;}
  const auto & history = found->second;
  if (history.invalid_stamp || image.stamp.nanosec >= 1000000000U) {
    result.code = "invalid_calibration_stamp";
    result.message = "Invalid stamp prevents safe calibration version selection";
    return result;
  }
  const auto selected = nearestRecord(history, records_, image.stamp);
  const auto & record = records_[selected];
  result.calibration_id = record.source.message_id;
  if (options.time_policy == CameraInfoTimePolicy::static_calibration && !history.static_consistent) {
    result.code = "static_calibration_changed";
    result.message = "Model/ROI/binning changed within the declared static segment";
    return result;
  }
  if (history.ambiguous_stamps.count(stampNs(record.data.stamp)) != 0) {
    result.code = "ambiguous_calibration";
    result.message = "Conflicting models share the selected timestamp";
    return result;
  }
  result.match = checkCameraInfoImage(record.data, image, options);
  result.code = cameraInfoMatchCodeName(result.match.code);
  result.message = result.match.message;
  return result;
}

}  // namespace vln_memory
