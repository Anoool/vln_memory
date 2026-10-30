#pragma once

#include <functional>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>

#include "vln_memory/bag/builder/bag_config.hpp"
#include "vln_memory/bag/builder/camera_info_index.hpp"
#include "vln_memory/bag/builder/media_writer.hpp"

namespace vln_memory
{

/** @brief 完整第一遍索引，拥有元数据和 TF；无整包像素/序列化缓冲。 */
struct BagScan
{
  std::string bag_id;
  std::string input_scope;
  std::string metadata_sha256;
  std::vector<BagSourceFile> source_files;
  std::vector<BagTopicRecord> topics;
  std::vector<BagMessageRef> messages;
  std::vector<BagImageRecord> images;
  std::vector<BagCameraInfoRecord> calibrations;
  std::vector<BagIssue> issues;
  TfIndex tf_index{TfScope{}};
  std::size_t tf_entries{0};
  std::size_t tf_inserted{0};
  std::size_t tf_duplicates{0};
};

/** @brief 同步借用 Image 仅在回调期间有效；nullptr 表示首遍反序列化失败。 */
using BagImageConsumer = std::function<void(const BagImageRecord &, const sensor_msgs::msg::Image *)>;

/** @brief 扫描所有配置消息、保存原始 CDR/标定/TF，返回拥有型索引；失败抛 BagBuildError。 */
BagScan scanBag(const BagBuildOptions & options, OutputFiles & output);
/** @brief 同约定重扫并逐条核对来源/摘要，仅图像调用 consumer，不保留借用引用。 */
void readBagImages(const BagBuildOptions & options, const BagScan & scan, const BagImageConsumer & consumer);
/** @brief 复核全部输入分卷及 metadata 指纹；改变/活动 WAL 拒绝发布。 */
void verifyBagSources(const BagBuildOptions & options, const BagScan & scan);

}  // namespace vln_memory
