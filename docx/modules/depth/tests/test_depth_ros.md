# test_depth_ros.cpp：真实 Image 适配测试

源码：[test_depth_ros.cpp](../../../../test/bag/depth/test_depth_ros.cpp)。

## 规划与用途

使用实际 sensor_msgs::msg::Image 构造 big-endian 16UC1，含行填充和零值。检查时间、帧名、布局、注册声明与归一化结果，随后修改源消息和 options，验证输出独立拥有数据。

另以 mono16 验证适配层不能绕过核心编码检查。无 ROS 模式不构建本测试；真实 Humble/ament 下 CTest 名称为 depth_ros_adapter。

本轮真实 ROS 测试已通过，未使用仿造消息类型。运行命令见 [项目 README](../../../../README.md)，结果见 [Depth 验证](../depth.md#验证结果)。
