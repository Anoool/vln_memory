# tf_ros.hpp / tf_ros.cpp：/tf 与 /tf_static 离线接入

## 1. 编码前规划与自审

使用真实 tf2_msgs::msg::TFMessage，保持“ROS 只映射，核心判断”的分层。话题、来源 ID 由 bag reader 提供；每个 TransformStamped 的时间独立保留，不用 bag 接收时刻代替。按条返回失败以便报告部分成功，不建立在线订阅或短时 Buffer。

头文件 include/vln_memory/bag/tf/tf_ros.hpp，实现 src/bag/tf/tf_ros.cpp；对应目标 vln_memory::vln_memory_tf_ros。PUBLIC 传递核心、tf2_msgs、geometry_msgs 依赖，核心仍可无 ROS 构建。

## 2. 接口

```cpp
TfMessageReport ingestRosTf(
  TfIndex & index, const tf2_msgs::msg::TFMessage & message,
  std::string_view topic, std::string_view message_id,
  const TfTopicBinding & binding = {});
```

默认 binding.dynamic_topic=/tf、static_topic=/tf_static；可以显式改为 /robot/tf 等。两个配置话题必须是不同的绝对 ROS 名称。未匹配话题、歧义配置、缺少消息 ID 均在条目插入前拒绝，保持索引不变。

真实 topic 决定 TfKind，不从 stamp==0、发布频率或 frame 名称推断静态。映射 parent=header.frame_id、child=child_frame_id、header.stamp、translation.xyz、rotation.xyzw，并写入来源 topic/message_id/消息内 transform_index。

## 3. 返回与错误

TfMessageReport 含 code/message、inserted、duplicates、issues。每个 issue 保留 transform_index、TfInsertCode 和原因；tfMessageCodeName 提供稳定名称。ok() 要求 code=processed 且 issues 为空，不能只看 processed。

TFMessage 中有效条目可插入，其他条目逐个报告；不是整条消息事务回滚。空 TFMessage 合法且不产生 frame。重复广播只计 duplicates，唯一索引不增长。调用后源消息和字符串可以销毁；所有被索引数据独立持有。

bag reader 必须先核对实际话题类型 tf2_msgs/msg/TFMessage，再反序列化。source_message_id 应由 bag ID 与稳定消息序号生成；相同消息内的变换用 transform_index 区分。适配层不生成依赖执行顺序的随机 ID。

## 4. 完整使用示例

```cpp
#include <vln_memory/bag/tf/tf_ros.hpp>
#include <iostream>

int main()
{
  using namespace vln_memory;
  const TfScope scope{"driver_clock_1", "map_segment_1"};
  TfIndex index(scope);
  tf2_msgs::msg::TFMessage message;
  geometry_msgs::msg::TransformStamped entry;
  entry.header.frame_id = "base";
  entry.child_frame_id = "camera_optical";
  entry.transform.rotation.w = 1;
  message.transforms.push_back(entry);
  const auto report = ingestRosTf(index, message, "/tf_static", "bag:message:1");
  if (!report.ok()) {return 1;}
  TfTimeOptions options;
  options.clock_id = scope.clock_id;
  options.segment_id = scope.segment_id;
  const auto checked = index.checkTime("base", "camera_optical", {10, 0}, options);
  if (!checked.covered()) {return 1;}
  std::cout << checked.edges[0].before.source_topic << '\n';
  return 0;
}
```

链接 vln_memory::vln_memory_tf_ros 即可得到全部公开头与消息依赖。无 ROS 模式不构建/安装此适配目标和头文件。

## 5. 与离线 bag 的关系

读取包含多个发布者的 /tf_static 时要遍历全部消息，按 child 汇总并检查是否一致，不能只读 bag 中“最后一条 /tf_static”。离线文件能提供的静态关系取决于当时记录是否完整；本模块不能补回未录到的外参。

builder 第一遍完成该段数据的收集后再对图像做 checkTime；缺链、冲突、超范围和时间空洞应保留为 pending/issues。不会自动启动在线 TF Listener 来填补离线证据。

数值、方向、冲突、时钟/段范围见 [核心说明](tf.md)。真实消息及原生 tf2 对照见 [test_tf_ros](tests/test_tf_ros.md)。


## 2026-10-29 修复与注释复核

删除重复名称解析器，改用公共规则；注释说明 processed/ok、条目来源序号与部分成功。 话题格式统一最长 247 字节。具体修复与验证记录见 [本次实施记录](../../project/sensor_validation_comments_plan.md)。

## 2026-10-30 可读性整理

接入函数逐字段写入 source_topic/source_message_id/source_transform_index，消息内循环使用 transform_index；查询示例使用扁平 TfTimeOptions。真实 ROS header 时间、条目顺序、部分成功和输出所有权保持。字段迁移见 [TF 核心第 12 节](tf.md#12-2026-10-30-可读性重构与字段迁移)。
