# tf_consumer.cpp：安装后的 TF 与图像元数据交接

源码 test/install_consumer/tf_consumer.cpp，是独立 CMake 下游项目，只查找安装后的 vln_memory 配置和头文件。

规划：建立 map→base 动态、base→rgb_optical 静态索引；从 RGB/Depth 公开帧类型取 frame_id/stamp，分别检查 10 秒 RGB 精确样本与晚 5ms 的 Depth 插值端点。只构造元数据，不需要像素来检查 TF；已注册深度使用 RGB 光学 frame 是明确的示例声明。

该程序仅验证公开头文件、目标、C++17、范围查询和元数据交接；图像本身的解码与标定检查已由各自模块和 camera_info_consumer 验证。不会输出假 map 坐标。

```bash
cmake -S test/install_consumer -B build/tf-core-consumer \
  -DCMAKE_PREFIX_PATH=/home/dev/agent/vln_memory/build/tf-core-install
cmake --build build/tf-core-consumer
build/tf-core-consumer/tf_consumer
```

ROS 安装后也编译运行同一消费者。接口和完整示例见 [TF 核心](../tf.md)。
