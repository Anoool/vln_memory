#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vln_memory/bag/builder/bag_types.hpp"
#include "vln_memory/bag/camera_info/camera_info.hpp"
#include "vln_memory/bag/depth/depth.hpp"
#include "vln_memory/bag/rgb/rgb.hpp"
#include "vln_memory/bag/tf/tf.hpp"

namespace vln_memory
{

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
  std::int64_t sync_tolerance_ns{0};
  std::int64_t tf_max_gap_ns{0};
  std::int64_t max_header_reorder_ns{0};
  CameraInfoMatchOptions rgb_camera;
  CameraInfoMatchOptions depth_camera;
  std::string tf_topic{"/tf"};
  std::string tf_static_topic{"/tf_static"};
  DepthOptions depth;
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
};

/** @brief 读取严格 bag YAML；未知/重复字段、错误类型抛 BagBuildError，不启动处理。 */
BagBuildOptions loadBagBuildOptions(const std::string & yaml_path);
/** @brief 返回全部配置诊断；纯检查，不访问 bag、不改动文件。 */
std::vector<std::string> validateBagBuildOptions(const BagBuildOptions & options);

}  // namespace vln_memory
