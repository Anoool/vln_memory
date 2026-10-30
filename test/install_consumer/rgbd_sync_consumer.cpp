#include "vln_memory/bag/builder/rgbd_sync.hpp"

/** @brief 只通过安装后的接口验证配对目标和时间依赖传递。 */
int main()
{
  vln_memory::RgbdStreamIndex rgb;
  rgb.topic = "/rgb";
  rgb.clock_id = "device";
  rgb.segment_id = "segment";
  rgb.frames.push_back({"rgb_0", 0, {1, 0}});
  auto depth = rgb;
  depth.topic = "/depth";
  depth.frames[0].message_id = "depth_0";
  vln_memory::RgbdSyncOptions options;
  options.clock_relation = vln_memory::SensorClockRelation::same_clock;
  options.clock_basis = "fixture";
  const auto result = vln_memory::pairRgbDepth(rgb, depth, options);
  if (!result.ok() || result.report->exact_count != 1) {return 1;}
  return 0;
}
