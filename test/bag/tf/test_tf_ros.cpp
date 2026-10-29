#include "vln_memory/bag/tf/tf_ros.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void expect(bool value, const std::string & reason)
{
  if (!value) {throw std::runtime_error(reason);}
}

geometry_msgs::msg::TransformStamped transform(
  const std::string & parent, const std::string & child, std::int32_t sec)
{
  geometry_msgs::msg::TransformStamped result;
  result.header.frame_id = parent;
  result.child_frame_id = child;
  result.header.stamp.sec = sec;
  result.header.stamp.nanosec = 123;
  result.transform.translation.x = 0.1;
  result.transform.translation.y = 0.2;
  result.transform.translation.z = 0.3;
  result.transform.rotation.z = 0.6;
  result.transform.rotation.w = 0.8;
  return result;
}
}

int main()
{
  using namespace vln_memory;
  try {
    const TfScope scope{"driver_clock_1", "session_1"};
    TfIndex index(scope);
    tf2_msgs::msg::TFMessage fixed;
    fixed.transforms.push_back(transform("base", "camera_optical", 99));
    auto report = ingestRosTf(index, fixed, "/tf_static", "bag_1:message_7");
    expect(report.ok() && report.inserted == 1, "Static TFMessage was not accepted");
    tf2_msgs::msg::TFMessage dynamic;
    dynamic.transforms.push_back(transform("map", "base", 3));
    dynamic.transforms.push_back(transform("map", "base", 1));
    dynamic.transforms[0].transform.translation.x = 3.0;
    dynamic.transforms[1].transform.translation.x = 1.0;
    report = ingestRosTf(index, dynamic, "/tf", "bag_1:message_8");
    expect(report.ok() && report.inserted == 2, "Out-of-order dynamic TFMessage was not indexed");
    const auto selected = index.checkTime("map", "camera_optical", {2, 0}, {scope, {2000000000LL}});
    expect(selected.covered() && selected.edges.size() == 2 && selected.edges[1].after,
      "ROS TF chain time selection failed");
    const auto & saved = selected.edges[0].before;
    expect(saved.kind == TfKind::static_transform && saved.stamp.sec == 99 && saved.stamp.nanosec == 123 &&
      saved.parent_frame == "base" && saved.child_frame == "camera_optical" &&
      saved.translation_m == std::array<double, 3>{0.1, 0.2, 0.3} &&
      saved.rotation_xyzw == std::array<double, 4>{0, 0, 0.6, 0.8} &&
      saved.source.topic == "/tf_static" && saved.source.message_id == "bag_1:message_7" &&
      saved.source.transform_index == 0, "TFMessage fields/source were not preserved");
    expect(selected.edges[1].before.source.transform_index == 1 &&
      selected.edges[1].after->source.transform_index == 0 &&
      selected.edges[1].before.stamp.sec == 1 && selected.edges[1].after->stamp.sec == 3,
      "Header time order or message-local source indices were lost");

    // 对照成熟 tf2 的基本连通/时间语义；生产索引并不依赖其短时缓存。
    tf2::BufferCore reference(tf2::durationFromSec(100.0));
    expect(reference.setTransform(fixed.transforms[0], "test", true), "tf2 rejected static fixture");
    for (const auto & entry : dynamic.transforms) {
      expect(reference.setTransform(entry, "test", false), "tf2 rejected dynamic fixture");
    }
    std::string error;
    expect(reference.canTransform("map", "camera_optical", tf2::TimePoint(std::chrono::seconds(2)), &error),
      "Native tf2 disagreed with covered chain: " + error);
    expect(!reference.canTransform("map", "camera_optical", tf2::TimePoint(std::chrono::seconds(4)), &error),
      "Native tf2 unexpectedly allowed extrapolation");
    report = ingestRosTf(index, fixed, "/tf_static", "bag_1:message_9");
    expect(report.ok() && report.duplicates == 1 && index.sampleCount() == 3,
      "Repeated static broadcast expanded history");
    fixed.transforms[0].header.frame_id = "changed";
    fixed.transforms[0].transform.translation.x = 999;
    const auto owned = index.checkTime("map", "camera_optical", {2, 0}, {scope, {2000000000LL}});
    expect(owned.covered() && owned.edges[0].before.parent_frame == "base" &&
      owned.edges[0].before.translation_m[0] == 0.1, "ROS source message was retained by reference");

    TfIndex partial(scope);
    tf2_msgs::msg::TFMessage mixed;
    mixed.transforms = {transform("map", "", 1), transform("map", "base", 1)};
    const auto mixed_report = ingestRosTf(partial, mixed, "/tf", "bag_2:message_1");
    expect(!mixed_report.ok() && mixed_report.inserted == 1 && mixed_report.issues.size() == 1 &&
      mixed_report.issues[0].transform_index == 0 && mixed_report.issues[0].code == TfInsertCode::invalid_frame,
      "Partial TFMessage failure was not reported per transform");
    TfIndex remapped(scope);
    const TfTopicBinding binding{"/robot/tf", "/robot/tf_static"};
    const auto remap = ingestRosTf(remapped, dynamic, "/robot/tf", "bag_3:message_1", binding);
    expect(remap.ok() && remap.inserted == 2, "Explicit remapped TF topic was rejected");
    expect(ingestRosTf(remapped, dynamic, "/tf", "bad_topic", binding).code == TfMessageCode::unexpected_topic &&
      remapped.sampleCount() == 2, "Unexpected topic mutated the index");
    expect(ingestRosTf(remapped, dynamic, "/robot/tf", "", binding).code == TfMessageCode::missing_message_id,
      "Missing source identity was accepted");
    expect(ingestRosTf(remapped, dynamic, "/tf", "id", {"/tf", "/tf"}).code == TfMessageCode::invalid_binding,
      "Ambiguous static/dynamic binding was accepted");
    const tf2_msgs::msg::TFMessage empty_message;
    expect(ingestRosTf(remapped, empty_message, "/robot/tf", "empty", binding).ok(),
      "Empty TFMessage was not a no-op");

    // tf2 的 time=0 是 latest；本索引的零时刻必须仍选择实际的零时间样本。
    TfIndex zero_index(scope);
    tf2::BufferCore zero_reference(tf2::durationFromSec(100.0));
    for (const auto sec : {0, 20}) {
      auto entry = transform("map", "base", sec);
      entry.header.stamp.nanosec = 0;
      entry.transform.translation.x = static_cast<double>(sec);
      tf2_msgs::msg::TFMessage message;
      message.transforms.push_back(entry);
      expect(ingestRosTf(zero_index, message, "/tf", "zero:" + std::to_string(sec)).ok(),
        "Zero-time fixture rejected");
      expect(zero_reference.setTransform(entry, "test", false), "tf2 rejected zero-time fixture");
    }
    const auto actual_zero = zero_index.checkTime("map", "base", {}, {scope, {0}});
    expect(actual_zero.covered() && actual_zero.edges[0].before.translation_m[0] == 0.0,
      "Actual zero time selected the latest transform");
    expect(zero_reference.lookupTransform("map", "base", tf2::TimePointZero).transform.translation.x == 20.0,
      "Native tf2 latest sentinel fixture changed");
    std::cout << "ROS TFMessage mapping, reports, remap and native tf2 comparison: passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
