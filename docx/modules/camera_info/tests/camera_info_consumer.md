# camera_info_consumer.cpp：三个模块的公开接口协作

源码 test/install_consumer/camera_info_consumer.cpp；它是独立 CMake 下游示例，不使用源码 include 路径。

先解码标准 RGB 和已声明注册 Depth；用 ImageGeometry 核对注册元数据；再用 checkSensorTimes 验证 Depth 比 RGB 晚 5ms，容差演示值 10ms；最后解码完整 CameraInfo，把标定 4x2、binning 2x2 与实际 2x1 图像匹配。

RGB 和已注册 Depth 分别使用明确的 image_topic 绑定，复用同一 CameraInfo 的目标网格。这里模拟一个明确声明此关系的设备配置，不表示任意原生深度都能使用 RGB 内参。示例只验证元数据；不做几何投影、文件导出或物理配准。

核心安装路径示例：

```bash
cmake -S test/install_consumer -B build/camera-core-consumer \
  -DCMAKE_PREFIX_PATH=/home/dev/agent/vln_memory/build/camera-core-install
cmake --build build/camera-core-consumer
build/camera-core-consumer/camera_info_consumer
```

ROS 安装后也构建相同消费者，并额外构建真实消息适配消费者。严格核心编译和真实 Humble 两种路径已运行通过。说明见 [CameraInfo 核心](../camera_info.md) 与 [公共时间约定](../../../common/sensor_time.md)。

2026-10-29：CameraImageReference 逐项填写 topic/width/height/frame_id/stamp；CameraInfoMatchOptions 直接填写绑定、时钟关系及纳秒容差，不再经 binding/time 包装。保留协作中的 Depth 扁平接口修改，仍分别验证 RGB 与注册 Depth 的显式绑定。
