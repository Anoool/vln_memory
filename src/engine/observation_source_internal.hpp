#pragma once

#include <map>

#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/memory/observation.hpp>

namespace vln_memory::detail
{

/** @brief 比较原始 sec/nanosec，保留纳秒精度。 */
inline bool sameStamp(const SensorStamp & a, const SensorStamp & b)
{
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

/** @brief 核对配对、两路消息、地图坐标系和时间段的共同来源。 */
inline bool matchesPreparationIdentity(const PreparedPair & pair, const Observation & observation)
{
  return pair.source_manifest_sha256 == observation.source_manifest_sha256 && pair.bag_id == observation.bag_id &&
    pair.pair_id == observation.pair_id && pair.rgb_message_id == observation.rgb_message_id &&
    pair.depth_message_id == observation.depth_message_id && pair.time_scope.clock_id == observation.clock_id &&
    pair.time_scope.segment_id == observation.segment_id && pair.map_frame == observation.map_frame;
}

/** @brief 比较原图来源、坐标网格和原始采样时刻。 */
inline bool sameImageReference(const CameraImageReference & a, const CameraImageReference & b)
{
  return a.topic == b.topic && a.frame_id == b.frame_id && a.width == b.width &&
    a.height == b.height && sameStamp(a.stamp, b.stamp);
}

/** @brief 比较两条来自同一消息和变换索引的原始 TF 端点及数值。 */
inline bool sameEndpoint(const TfTransform & a, const TfTransform & b)
{
  return a.kind == b.kind && sameStamp(a.stamp, b.stamp) && a.parent_frame == b.parent_frame &&
    a.child_frame == b.child_frame && a.translation_m == b.translation_m && a.rotation_xyzw == b.rotation_xyzw &&
    a.source_topic == b.source_topic && a.source_message_id == b.source_message_id &&
    a.source_transform_index == b.source_transform_index;
}

/** @brief 核对历史时刻、变换链及原始插值端点，拒绝改写的轨迹。 */
inline bool sameRecordedPose(const TfPoseResult & a, const TfPoseResult & b)
{
  if (!a.ok() || !b.ok() || !sameStamp(a.time.stamp, b.time.stamp) || a.time.frames != b.time.frames ||
    a.pose->translation_m != b.pose->translation_m || a.pose->rotation_xyzw != b.pose->rotation_xyzw ||
    a.time.edges.size() != b.time.edges.size()) {return false;}
  for (std::size_t i = 0; i < a.time.edges.size(); ++i) {
    const auto & x = a.time.edges[i];
    const auto & y = b.time.edges[i];
    if (x.inverted != y.inverted || !sameEndpoint(x.before, y.before) ||
      x.after.has_value() != y.after.has_value() || (x.after && !sameEndpoint(*x.after, *y.after))) {return false;}
  }
  return true;
}

/** @brief 比较来源摘要及可选原始文件定位，观测中的顺序不影响核对。 */
inline bool sameObservationEvidence(const PreparedPair & pair, const Observation & observation)
{
  if (pair.message_summaries.size() != observation.evidence.size()) {return false;}
  std::map<std::string_view, const ObservationEvidence *> by_id;
  for (const auto & entry : observation.evidence) {by_id.emplace(entry.message_id, &entry);}
  for (const auto & message : pair.message_summaries) {
    const auto found = by_id.find(message.message_id);
    if (found == by_id.end()) {return false;}
    const auto & entry = *found->second;
    if (entry.topic != message.topic || entry.payload_sha256 != message.payload_sha256 ||
      entry.serialized_bytes != message.serialized_bytes || entry.file_index != message.file_index ||
      entry.topic_ordinal != message.topic_ordinal || entry.message_order != message.message_order ||
      entry.bag_record_time_ns != message.bag_record_time_ns ||
      entry.raw_path.has_value() != message.raw_path.has_value() ||
      (message.raw_path && *entry.raw_path != message.raw_path->generic_string())) {return false;}
  }
  return true;
}

}  // namespace vln_memory::detail
