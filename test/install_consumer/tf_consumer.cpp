#include <vln_memory/bag/tf/tf.hpp>
#include <vln_memory/bag/rgb/rgb.hpp>
#include <vln_memory/bag/depth/depth.hpp>

#include <iostream>

int main()
{
  using namespace vln_memory;
  const TfScope scope{"camera_clock_epoch_1", "map_segment_1"};
  TfIndex index(scope);
  TfTransform fixed;
  fixed.kind = TfKind::static_transform;
  fixed.parent_frame = "base";
  fixed.child_frame = "rgb_optical";
  fixed.source = {"/tf_static", "bag:static:0", 0};
  if (!index.insert(fixed).accepted()) {return 1;}
  TfTransform moving;
  moving.parent_frame = "map";
  moving.child_frame = "base";
  moving.source = {"/tf", "bag:dynamic:0", 0};
  moving.stamp = {10, 0};
  if (!index.insert(moving).accepted()) {return 1;}
  moving.stamp = {10, 20000000};
  moving.source.message_id = "bag:dynamic:1";
  if (!index.insert(moving).accepted()) {return 1;}

  // 示意 builder 已解码帧的元数据交接；TF 只读 frame/stamp，不需要图像像素。
  RgbFrame rgb;
  rgb.frame_id = "rgb_optical";
  rgb.stamp = {10, 0};
  DepthFrame depth;
  depth.frame_id = "rgb_optical";  // 已声明注册到此光学帧的深度流。
  depth.stamp = {10, 5000000};
  const TfTimeOptions query{scope, {20000000}};
  const auto rgb_tf = index.checkTime("map", rgb.frame_id, rgb.stamp, query);
  const auto depth_tf = index.checkTime("map", depth.frame_id, depth.stamp, query);
  if (!rgb_tf.covered() || !depth_tf.covered() || rgb_tf.edges[1].after || !depth_tf.edges[1].after) {
    return 1;
  }
  const auto report = index.inspectChain("map", depth.frame_id, query.coverage);
  if (!report.chainExists() || report.intervals.size() != 1) {return 1;}
  std::cout << "Installed TF/RGB/Depth metadata interfaces: passed\n";
  return 0;
}
