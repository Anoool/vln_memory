#include <vln_memory/bag/tf/tf.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>
#include <vln_memory/bag/depth/depth.hpp>

#include <iostream>

/** 只使用安装后的公开目标验证扁平 TF 字段和图像元数据/真实消息交接。 */
int main()
{
  using namespace vln_memory;
  const TfScope scope{"camera_clock_epoch_1", "map_segment_1"};
  TfIndex index(scope);
  TfTransform fixed;
  fixed.kind = TfKind::static_transform;
  fixed.parent_frame = "base";
  fixed.child_frame = "rgb_optical";
  fixed.source_topic = "/tf_static";
  fixed.source_message_id = "bag:static:0";
  fixed.source_transform_index = 0;
  if (!index.insert(fixed).accepted()) {return 1;}
  TfTransform moving;
  moving.parent_frame = "map";
  moving.child_frame = "base";
  moving.source_topic = "/tf";
  moving.source_message_id = "bag:dynamic:0";
  moving.source_transform_index = 0;
  moving.stamp = {10, 0};
  if (!index.insert(moving).accepted()) {return 1;}
  moving.stamp = {10, 20000000};
  moving.translation_m[0] = 4.0;
  moving.source_message_id = "bag:dynamic:1";
  if (!index.insert(moving).accepted()) {return 1;}

  // 示意 builder 已解码帧的元数据交接；TF 只读 frame/stamp，不需要图像像素。
  RgbFrame rgb;
  rgb.frame_id = "rgb_optical";
  rgb.stamp = {10, 0};
  DepthFrame depth;
  depth.frame_id = "rgb_optical";  // 已声明注册到此光学帧的深度流。
  depth.stamp = {10, 5000000};
  TfTimeOptions query;
  query.clock_id = scope.clock_id;
  query.segment_id = scope.segment_id;
  query.max_interpolation_gap_ns = 20000000;
  const auto rgb_tf = index.checkTime("map", rgb.frame_id, rgb.stamp, query);
  const auto depth_tf = index.checkTime("map", depth.frame_id, depth.stamp, query);
  if (!rgb_tf.covered() || !depth_tf.covered() || rgb_tf.edges[1].after || !depth_tf.edges[1].after) {
    return 1;
  }
  const auto rgb_pose = index.lookupPose("map", rgb.frame_id, rgb.stamp, query);
  const auto depth_pose = index.lookupPose("map", depth.frame_id, depth.stamp, query);
  if (!rgb_pose.ok() || !depth_pose.ok() ||
    rgb_pose.pose->translation_m[0] != 0.0 || depth_pose.pose->translation_m[0] != 1.0)
  {
    return 1;
  }
  const auto report = index.inspectChain("map", depth.frame_id, TfCoverageOptions{query.max_interpolation_gap_ns});
  if (!report.chainExists() || report.intervals.size() != 1) {return 1;}
  std::cout << "Installed TF/RGB/Depth metadata interfaces: passed\n";
  return 0;
}
