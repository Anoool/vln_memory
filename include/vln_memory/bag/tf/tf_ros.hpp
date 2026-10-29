#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <tf2_msgs/msg/tf_message.hpp>

#include "vln_memory/bag/tf/tf.hpp"

namespace vln_memory
{

/** @brief 实际话题到静态/动态语义的显式映射，支持离线 bag 中的 remap。 */
struct TfTopicBinding
{
  std::string dynamic_topic{"/tf"};
  std::string static_topic{"/tf_static"};
};

enum class TfMessageCode {processed, invalid_binding, unexpected_topic, missing_message_id};

struct TfMessageIssue
{
  std::size_t transform_index{0};
  TfInsertCode code{TfInsertCode::invalid_frame};
  std::string message;
};

/**
 * @brief TFMessage 可以部分成功；ok() 要求话题接收成功且所有条目通过。
 * @note inserted/duplicates 只统计本条消息；空消息是合法的零条处理，不产生坐标边。
 */
struct TfMessageReport
{
  TfMessageCode code{TfMessageCode::unexpected_topic};
  std::string message;
  std::size_t inserted{0};
  std::size_t duplicates{0};
  std::vector<TfMessageIssue> issues;
  bool ok() const noexcept {return code == TfMessageCode::processed && issues.empty();}
};

const char * tfMessageCodeName(TfMessageCode code) noexcept;

/**
 * @brief 把真实 TFMessage 逐条映射到核心索引，保留 TransformStamped 自己的 header 时间。
 * @param message_id 由 builder 提供的稳定来源消息 ID，不是 ROS 消息自带字段。
 * @note 调用前应验证 bag 话题类型为 tf2_msgs/msg/TFMessage；本函数不反序列化或订阅。
 *       topic/message_id/消息所有字段均被复制，返回后不依赖输入生命周期。
 *       每条变换独立处理，失败详情保留 transform_index；调用方必须检查整个报告。
 */
TfMessageReport ingestRosTf(
  TfIndex & index, const tf2_msgs::msg::TFMessage & message, std::string_view topic,
  std::string_view message_id, const TfTopicBinding & binding = {});

}  // namespace vln_memory
