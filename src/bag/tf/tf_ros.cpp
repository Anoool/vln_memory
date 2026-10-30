#include "vln_memory/bag/tf/tf_ros.hpp"

#include <utility>

#include "vln_memory/bag/common/sensor_names.hpp"

/**
 * @file
 * 实际 topic 经过公共规则和绑定核对后，逐条映射到 TfIndex::insert。
 * 不保留消息引用；每条失败返回自己的原始序号，有效条目不会被整条回滚。
 */
namespace vln_memory
{

/** 按显式话题绑定逐条复制真实消息；条目失败不回滚其他条目，完整来源由索引拥有。 */
TfMessageReport ingestRosTf(
  TfIndex & index, const tf2_msgs::msg::TFMessage & message, std::string_view topic,
  std::string_view message_id, const TfTopicBinding & binding)
{
  TfMessageReport report;
  if (!isResolvedTopicName(binding.dynamic_topic) || !isResolvedTopicName(binding.static_topic) ||
    binding.dynamic_topic == binding.static_topic)
  {
    report.code = TfMessageCode::invalid_binding;
    report.message = "Distinct resolved dynamic/static topics are required";
    return report;
  }
  TfKind kind;
  if (topic == binding.dynamic_topic) {
    kind = TfKind::dynamic;
  } else if (topic == binding.static_topic) {
    kind = TfKind::static_transform;
  } else {
    report.code = TfMessageCode::unexpected_topic;
    report.message = "Actual TF topic is absent from the configured binding";
    return report;
  }
  if (message_id.empty()) {
    report.code = TfMessageCode::missing_message_id;
    report.message = "A stable source message ID is required";
    return report;
  }
  // processed 表示允许逐条处理；最终是否全成功还要看 issues，见 report.ok()。
  report.code = TfMessageCode::processed;
  for (std::size_t transform_index = 0; transform_index < message.transforms.size(); ++transform_index) {
    const auto & source = message.transforms[transform_index];
    TfTransform input;
    // 每个 TransformStamped 有自己的 stamp，不能用整个 TFMessage 的接收时间覆盖。
    input.kind = kind;
    input.stamp = {source.header.stamp.sec, source.header.stamp.nanosec};
    input.parent_frame = source.header.frame_id;
    input.child_frame = source.child_frame_id;
    input.translation_m = {source.transform.translation.x, source.transform.translation.y,
      source.transform.translation.z};
    input.rotation_xyzw = {source.transform.rotation.x, source.transform.rotation.y,
      source.transform.rotation.z, source.transform.rotation.w};
    // 消息内序号不随时间排序改变，保留它才能回查同一 TFMessage 的具体条目。
    input.source_topic = topic;
    input.source_message_id = message_id;
    input.source_transform_index = transform_index;
    const auto inserted = index.insert(std::move(input));
    if (inserted.code == TfInsertCode::inserted) {
      ++report.inserted;
    } else if (inserted.code == TfInsertCode::duplicate) {
      ++report.duplicates;
    } else {
      report.issues.push_back({transform_index, inserted.code, inserted.message});
    }
  }
  return report;
}

/** 返回消息接入状态的稳定静态名称；未知枚举返回 unknown，无需释放。 */
const char * tfMessageCodeName(TfMessageCode code) noexcept
{
  switch (code) {
    case TfMessageCode::processed: return "processed";
    case TfMessageCode::invalid_binding: return "invalid_binding";
    case TfMessageCode::unexpected_topic: return "unexpected_topic";
    case TfMessageCode::missing_message_id: return "missing_message_id";
  }
  return "unknown";
}

}  // namespace vln_memory
