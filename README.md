# vln_memory

ROS 2 Humble / C++17 离线观测处理包。当前实现第一个 RGB 模块。

RGB、Depth、CameraInfo、TF、bag 五模块的职责和实施顺序见 [整体架构](docx/project/architecture.md)。当前 RGB 模块输出标准内存帧；PNG、跨消息同步和 Observation JSONL 在后续 bag 模块统一实现。

## 当前接口

- `decodeRgb(RgbImageView, RgbLimits)`：支持 rgb8、bgr8、mono8，输出独立持有的紧密 RGB 数据。
- `decodeRosRgb(sensor_msgs::msg::Image, RgbLimits)`：ROS 消息薄适配，复用核心校验。
- `rgbErrorCodeName(code)`：提供稳定错误名称，便于后续报告使用。

全部说明从 [文档导航](docx/README.md) 进入。RGB 接口见 [核心说明](docx/modules/rgb/rgb.md)，ROS 接入见 [适配说明](docx/modules/rgb/rgb_ros.md)。

## ROS 2 构建

在已经加载 ROS 2 Humble 的工作区中：

```bash
colcon build --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select vln_memory
colcon test-result --verbose
```

下游使用 `find_package(vln_memory REQUIRED)`，链接 `vln_memory::vln_memory_rgb_ros`。只使用无 ROS 核心时链接 `vln_memory::vln_memory_rgb`。

## 无 ROS 验证

```bash
cmake -S . -B build/standalone -DVLN_MEMORY_ENABLE_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/standalone
ctest --test-dir build/standalone --output-on-failure
```

仅有 g++ 时也可以验证核心：

```bash
mkdir -p build/standalone
g++ -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror \
  -Iinclude src/rgb.cpp test/test_rgb.cpp -o build/standalone/test_rgb
./build/standalone/test_rgb
```

像素始终按 R,G,B 排列。未来若调用 OpenCV 的 imwrite，必须先按其接口需要转换到 BGR，不能直接把本模块结果误当 BGR。
