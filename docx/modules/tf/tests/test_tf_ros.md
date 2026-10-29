# test_tf_ros.cpp：真实 TFMessage 与原生 tf2 对照

源码 test/bag/tf/test_tf_ros.cpp；CTest 名 tf_ros_adapter。它链接 vln_memory_tf_ros，并以 tf2 为测试依赖；生产索引/适配库不依赖 tf2 缓存。

先验证全部父子 frame、各条消息独立时间、平移/四元数、实际话题、消息 ID、消息内索引和所有权；再验证乱序动态样本、静态重复、逐条失败、空消息、remap 及错误话题/配置拒绝。

原生 tf2::BufferCore 对照验证同一 fixture 在 2 秒的链可用、4 秒外推失败；另用零和 20 秒样本验证 tf2 的 time=0/latest 哨兵与本索引“真实零时刻”检查的区别。此对照不声称本索引已实现 tf2 全部几何或动态换 parent 行为。

运行：加载 /opt/ros/humble/setup.bash 后 colcon build/test，或 ctest --test-dir build/tf-ros/build/vln_memory -R tf_ros_adapter --output-on-failure。真实 Humble 回归已包含本项，没有伪造 ROS 类型。适配入口见 [tf_ros](../tf_ros.md)。
