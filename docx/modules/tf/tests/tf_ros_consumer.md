# tf_ros_consumer.cpp：ROS 安装目标与传递依赖

源码 test/install_consumer/tf_ros_consumer.cpp，独立下游只链接 vln_memory::vln_memory_tf_ros；不手工注入源码 include 路径，也不直接链接 tf2。

构造真实 TFMessage/TransformStamped，插入 /tf_static 后检查 base<-camera_optical 链以及 20 秒查询；验证 tf2_msgs、geometry_msgs、C++17、TF 核心头和库都从公开目标传递。

加载 build/tf-ros/install/setup.bash，构建 test/install_consumer 后运行 build/tf-ros-consumer/tf_ros_consumer。完整适配用法见 [tf_ros](../tf_ros.md)，更全面的字段/错误验证见 [test_tf_ros](test_tf_ros.md)。
