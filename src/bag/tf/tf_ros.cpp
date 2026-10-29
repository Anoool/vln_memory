#include "vln_memory/bag/tf/tf_ros.hpp"

#include <utility>

namespace vln_memory
{
namespace
{
bool letter(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

/** 只检查已解析的绝对话题；不执行 remap 或从 /tf 名称后缀推断静态语义。 */
bool validTopic(const std::string & topic)
{
  if (topic.size() < 2 || topic.front() != '/' || topic.back() == '/') {return false;}
  bool first = true;
  for (std::size_t i = 1; i < topic.size(); ++i) {
    const char c = topic[i];
    if (c == '/') {
      if (first) {return false;}
      first = true;
    } else {
      if (!(letter(c) || (!first && c >= '0' && c <= '9'))) {return false;}
      first = false;
    }
  }
  return !first;
}
}  // namespace

TfMessageReport ingestRosTf(
  TfIndex & index, const tf2_msgs::msg::TFMessage & message, std::string_view topic,
  std::string_view message_id, const TfTopicBinding & binding)
{
  TfMessageReport report;
  if (!validTopic(binding.dynamic_topic) || !validTopic(binding.static_topic) ||
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
  report.code = TfMessageCode::processed;
  for (std::size_t i = 0; i < message.transforms.size(); ++i) {
    const auto & source = message.transforms[i];
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
    input.source = {std::string(topic), std::string(message_id), i};
    const auto inserted = index.insert(std::move(input));
    if (inserted.code == TfInsertCode::inserted) {
      ++report.inserted;
    } else if (inserted.code == TfInsertCode::duplicate) {
      ++report.duplicates;
    } else {
      report.issues.push_back({i, inserted.code, inserted.message});
    }
  }
  return report;
}

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
