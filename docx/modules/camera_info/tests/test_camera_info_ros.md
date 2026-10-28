# test_camera_info_ros.cpp：真实 CameraInfo 映射与安装接入

源码 test/bag/camera_info/test_camera_info_ros.cpp；ROS 测试目标 test_camera_info_ros，CTest 名 camera_info_ros_adapter。

用真实 Humble sensor_msgs::msg::CameraInfo，填写非零畸变、与 K 不同的 P、双目平移、ROI 全字段、binning 和精确采样时间；逐字段核对公开输出，再修改源消息和 topic，确认结果拥有数据；K[0]=0 必须由核心报告 uncalibrated。

安装消费者 CMake 复用这份消息检查源码构建 camera_info_ros_consumer，仅链接安装后的 vln_memory::vln_memory_camera_info_ros，用于验证公开头路径、C++17、sensor_msgs 和公共时间依赖的传递；不借用包源码的 include 路径。

运行：加载 /opt/ros/humble/setup.bash 后正常 colcon build/test，或执行安装消费者构建目录中的 camera_info_ros_consumer。未安装 ROS 时不会伪造消息执行。适配接口见 [camera_info_ros](../camera_info_ros.md)。
