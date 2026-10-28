# vln_memory

ROS 2 Humble / C++17 空间语义记忆功能包。**当前正在实现观测准备层 bag，已完成该层的 RGB 与 Depth 子模块。**

整个系统构想、各层职责与开发顺序见 [系统整体规划](docx/project/system_plan.md)。

RGB、Depth、CameraInfo、TF、bag 五模块的职责和实施顺序见 [观测准备层与源码组织](docx/project/architecture.md)。RGB/Depth 模块输出标准内存帧；PNG、跨消息同步和 Observation JSONL 在后续 bag 模块统一实现。

## 当前源码布局

头文件和实现分别位于 include/vln_memory/bag/ 与 src/bag/，按 common、rgb、depth 分类；模块测试位于 test/bag/。安装后的头文件保留同样结构，例如：

```cpp
#include <vln_memory/bag/rgb/rgb_ros.hpp>
#include <vln_memory/bag/depth/depth_ros.hpp>
```

后续 CameraInfo、TF、bag 读取组装分别加入本层 camera_info、tf、builder 模块。其他系统层暂不生成空目录。

## 当前接口

- `decodeRgb(RgbImageView, RgbLimits)`：支持 rgb8、bgr8、mono8，输出独立持有的紧密 RGB 数据。
- `decodeRosRgb(sensor_msgs::msg::Image, RgbLimits)`：ROS 消息薄适配，复用核心校验。
- `rgbErrorCodeName(code)`：提供稳定错误名称，便于后续报告使用。
- `decodeDepth(DepthImageView, DepthOptions, DepthLimits)`：16UC1/32FC1 按显式米/毫米单位转换为 float32 米制 optical-z。
- `decodeRosDepth(sensor_msgs::msg::Image, DepthOptions, DepthLimits)`：真实 ROS 深度消息适配。
- `checkDepthRgbAlignment(DepthFrame, ImageGeometry)`：检查上游注册声明与实际 RGB 网格的一致性。

DepthOptions 必须明确 input_unit 与 convention。相同尺寸不能证明对齐；未知/未对齐仍可解码，但不会被标记为已注册。

全部说明从 [文档导航](docx/README.md) 进入。RGB 接口见 [核心说明](docx/modules/rgb/rgb.md)，ROS 接入见 [适配说明](docx/modules/rgb/rgb_ros.md)。Depth 见 [核心说明](docx/modules/depth/depth.md) 与 [ROS 适配](docx/modules/depth/depth_ros.md)。

## ROS 2 构建

在已经加载 ROS 2 Humble 的工作区中：

```bash
colcon build --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select vln_memory
colcon test-result --verbose
```

下游使用 `find_package(vln_memory REQUIRED)`，链接 `vln_memory::vln_memory_rgb_ros`。Depth 适配链接 `vln_memory::vln_memory_depth_ros`。只使用核心时分别链接 `vln_memory::vln_memory_rgb`、`vln_memory::vln_memory_depth`。

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
  -Iinclude src/bag/rgb/rgb.cpp test/bag/rgb/test_rgb.cpp -o build/standalone/test_rgb
./build/standalone/test_rgb
```

像素始终按 R,G,B 排列。未来若调用 OpenCV 的 imwrite，必须先按其接口需要转换到 BGR，不能直接把本模块结果误当 BGR。

## Git 审阅

本包是独立本地仓库。现有 RGB 已建立 main 基线；Depth 及配套修改位于 codex/depth-module。模块完成后执行本地 git add，保留暂存差异供审阅，合并由用户确认后执行。

```bash
git status --short --branch
git diff --cached --stat
git diff --cached
```
