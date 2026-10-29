#include <vln_memory/bag/tf/tf_ros.hpp>

#include <iostream>

int main()
{
  using namespace vln_memory;
  const TfScope scope{"ros_clock", "bag_segment"};
  TfIndex index(scope);
  tf2_msgs::msg::TFMessage message;
  geometry_msgs::msg::TransformStamped entry;
  entry.header.frame_id = "base";
  entry.child_frame_id = "camera_optical";
  entry.transform.rotation.w = 1;
  message.transforms.push_back(entry);
  const auto ingested = ingestRosTf(index, message, "/tf_static", "bag:message:1");
  const auto chain = index.inspectChain("base", "camera_optical", {});
  const auto checked = index.checkTime("base", "camera_optical", {20, 0}, {scope, {0}});
  if (!ingested.ok() || !chain.chainExists() || !checked.covered()) {return 1;}
  std::cout << "Installed ROS TF target and message dependencies: passed\n";
  return 0;
}
