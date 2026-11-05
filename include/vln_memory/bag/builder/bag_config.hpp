#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <vln_memory/bag/builder/bag_types.hpp>
#include <vln_memory/bag/camera_info/camera_info.hpp>
#include <vln_memory/bag/depth/depth.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>
#include <vln_memory/bag/tf/tf.hpp>

namespace vln_memory
{

enum class BagDepthStorage {metric_npy, source_png16};

/** @brief 返回写入 manifest 的静态存储策略名称。 */
const char * bagDepthStorageName(BagDepthStorage storage) noexcept;

/**
 * @brief 第一版只处理一个明确声明的时钟段；复用实际相机/深度契约，不多层包装。
 * @note 路径可由 CLI 覆盖；所有时间阈值是整数纳秒。既有核心上限不等于反序列化预算。
 */
struct BagBuildOptions
{
  std::string bag_uri;
  std::string output_directory;
  std::string storage_id{"sqlite3"};
  std::string clock_id;
  std::string clock_basis;
  std::string segment_id;
  std::string segment_basis;
  SensorClockRelation clock_relation{SensorClockRelation::unknown};
  std::string map_frame{"map"};
  std::string body_frame{"base_link"};
  // 非负整数纳秒；0 为严格匹配，正值为最近候选的最大允许时差，不改写原始时间。
  std::int64_t sync_tolerance_ns{0};
  // 省略时从本段所需 TF 链估计；显式 0 只允许精确动态样本。
  std::optional<std::int64_t> tf_max_gap_ns;
  TfJumpLimits tf_jump_limits;
  std::int64_t max_header_reorder_ns{0};
  CameraInfoMatchOptions rgb_camera;
  CameraInfoMatchOptions depth_camera;
  std::string tf_topic{"/tf"};
  std::string tf_static_topic{"/tf_static"};
  DepthOptions depth;
  // source_png16 只改变 16UC1 的磁盘格式，其他编码继续写米制 NPY。
  BagDepthStorage depth_storage{BagDepthStorage::metric_npy};
  RgbLimits rgb_limits;
  DepthLimits depth_limits;
  TfLimits tf_limits;
  double min_valid_depth_ratio{0.0};
  std::size_t max_messages{1000000};
  std::size_t max_frames{200000};
  std::size_t max_calibrations{100000};
  std::size_t max_tf_entries{1000000};
  std::size_t max_serialized_bytes{128U * 1024U * 1024U};
  std::uint64_t max_output_bytes{10ULL * 1024ULL * 1024ULL * 1024ULL};
  // 默认仅保留提取产物；true 时另外保存每条原始 CDR。
  bool save_raw_cdr{false};
};

/** @brief 读取严格 bag YAML；未知/重复字段、错误类型抛 BagBuildError，不启动处理。 */
BagBuildOptions loadBagBuildOptions(const std::string & yaml_path);
/** @brief 返回全部配置诊断；纯检查，不访问 bag、不改动文件。 */
std::vector<std::string> validateBagBuildOptions(const BagBuildOptions & options);

}  // namespace vln_memory
