# test_rgb_ros.cpp：真实 ROS Image 接口验证

## 规划与自审

直接构造真实 sensor_msgs::msg::Image，经 decodeRosRgb 调用核心；不创建 ROS Node 或模拟消息定义。只有启用 ROS 的构建才加入本测试。

## 验证内容

- header 时间、frame_id、行跨度、编码和大小端标记是否正确映射。
- BGR 像素与行填充是否通过核心正确转换。
- 修改源消息后输出是否独立有效。
- 未支持编码是否仍被核心拒绝，适配器没有绕开校验。

## 执行

加载 ROS 2 Humble 后，按 [项目 README](../../../../README.md) 运行 colcon build/test；CTest 名称为 rgb_ros_adapter。实际执行状态见 [RGB 模块验证记录](../rgb.md#实施与验证记录)，不能将纯核心测试通过等同于本测试通过。
