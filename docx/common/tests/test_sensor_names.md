# test_sensor_names.cpp：公共话题规则回归

源码 test/bag/common/test_sensor_names.cpp；CTest 名 sensor_names_core。只使用安装规则相同的公开头，不链接 ROS；C++17。

采用明确的有效/无效样本表：绝对名称、命名空间、数字后缀、隐藏 token、双下划线、247 字节边界；拒绝相对名、空白、空 token、尾斜线、数字开头 token、替换/URL、非 ASCII、嵌入 NUL 和 248 字节名称。

TF 核心测试补充这些坏来源被拒绝且不增加索引计数；CameraInfo 测试补充超长来源拒绝。真实 test_tf_ros 使用 rmw_validate_full_topic_name_with_size 对照关键用例，避免测试与实现重复同一种判断算法。

运行：ctest --test-dir build/sensor-fix-core -R sensor_names_core --output-on-failure。完整规则与接口见 [sensor_names](../sensor_names.md)。
