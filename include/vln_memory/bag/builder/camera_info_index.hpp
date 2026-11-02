#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include <vln_memory/bag/builder/bag_types.hpp>
#include <vln_memory/bag/camera_info/camera_info.hpp>

namespace vln_memory
{

/** @brief 原始标定拥有型记录；坏标定也保留原值，不能悄悄跳过后改选旧模型。 */
struct BagCameraInfoRecord
{
  BagMessageRef source;
  CameraInfoData data;
  CameraInfoErrorCode validation_code{CameraInfoErrorCode::none};
  std::string validation_message;
  std::optional<CameraInfoData> normalized;  // 独立拥有派生参数；原 data 永远是原值。
  CameraInfoErrorCode normalized_code{CameraInfoErrorCode::none};
  bool normalization_applied{false};
  CameraInfoCompatibility compatibility{CameraInfoCompatibility::strict};
};

/** @brief 候选选择与最终匹配分别报告；失败仍保留已选来源便于审阅。 */
struct CameraInfoSelection
{
  std::string code{"missing_calibration"};
  std::string message;
  std::string calibration_id;
  CameraInfoMatchResult match;
  // 仅 matched 时借用已选标定；records 在整个使用期必须不可变且存活。
  const CameraInfoData * info{nullptr};
  bool normalization_applied{false};
  /** @brief 只表示选择及既有匹配检查通过，不包含有效内参求解。 */
  bool matched() const noexcept {return code == "matched" && match.matched();}
};

namespace detail
{
/** @brief 内部按话题有序候选与一次性歧义分析，不保存像素。 */
struct CalibrationHistory
{
  std::vector<std::size_t> ordered;
  std::set<std::int64_t> ambiguous_stamps;
  bool static_consistent{true};
  bool invalid_stamp{false};
};
}

/** @brief 借用不可变记录，索引存活期间 records 不得修改/销毁；select 为只读。 */
class CameraInfoIndex
{
public:
  /** @brief 一次排序并分析模型变化/同刻矛盾；所有输入都属于 builder 的同一段。 */
  explicit CameraInfoIndex(const std::vector<BagCameraInfoRecord> & records);
  /** @brief 二分选最近候选或核对全段静态一致，再调用 checkCameraInfoImage。 */
  CameraInfoSelection select(
    const CameraImageReference & image, const CameraInfoMatchOptions & options) const;

private:
  const std::vector<BagCameraInfoRecord> & records_;
  std::map<std::string, detail::CalibrationHistory> histories_;
};

}  // namespace vln_memory
