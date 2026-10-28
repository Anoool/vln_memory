# depth_ros_consumer.cpp：安装后的 ROS Depth 接口

源码：[depth_ros_consumer.cpp](../../../../test/install_consumer/depth_ros_consumer.cpp)。

## 规划与用途

独立下游链接 vln_memory::vln_memory_depth_ros，依靠目标自动传递 sensor_msgs 头文件和核心库。用真实 32FC1/米消息验证不会重复乘毫米比例。

仅在安装包提供 ROS 目标时构建，不在无 ROS 环境模拟消息。source 真实 Humble 与安装空间后，配置 test/install_consumer，构建并执行 depth_ros_consumer。

本轮已实际编译运行通过，见 [Depth 验证](../depth.md#验证结果)。
