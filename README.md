# vln_memory

ROS 2 Humble / C++17 空间语义记忆功能包。**当前正在实现观测准备层 bag，已完成该层的 RGB、Depth、CameraInfo，以及 TF 链/覆盖检查和公共时间检查。**

当前源码的逐项验收见 [核查报告](docx/reviews/observation_preparation_audit.md)，集中参数/返回值及提供文件见 [接口手册](docx/interfaces/observation_preparation_api.md)。现阶段仍无 bag reader、自动配对、组合位姿、关键帧与 Observation 导出入口。

整个系统构想、各层职责与开发顺序见 [系统整体规划](docx/project/system_plan.md)。

RGB、Depth、CameraInfo、TF、bag 五模块的职责和实施顺序见 [观测准备层与源码组织](docx/project/architecture.md)。RGB/Depth 模块输出标准内存帧；PNG、跨消息同步和 Observation JSONL 在后续 bag 模块统一实现。

## 当前源码布局

头文件和实现分别位于 include/vln_memory/bag/ 与 src/bag/，按 common、rgb、depth、camera_info、tf 分类；模块测试位于 test/bag/。安装后的头文件保留同样结构，例如：

```cpp
#include <vln_memory/bag/rgb/rgb_ros.hpp>
#include <vln_memory/bag/depth/depth_ros.hpp>
#include <vln_memory/bag/camera_info/camera_info_ros.hpp>
#include <vln_memory/bag/tf/tf_ros.hpp>
```

TF 处理位于本层 tf；后续 bag 读取组装加入 builder 模块。其他系统层暂不生成空目录。

## 当前接口

- `decodeRgb(RgbImageView, RgbLimits)`：支持 rgb8、bgr8、mono8，输出独立持有的紧密 RGB 数据。
- `decodeRosRgb(sensor_msgs::msg::Image, RgbLimits)`：ROS 消息薄适配，复用核心校验。
- `rgbErrorCodeName(code)`：提供稳定错误名称，便于后续报告使用。
- `decodeDepth(DepthImageView, DepthOptions, DepthLimits)`：16UC1/32FC1 按显式米/毫米单位转换为 float32 米制 optical-z。
- `decodeRosDepth(sensor_msgs::msg::Image, DepthOptions, DepthLimits)`：真实 ROS 深度消息适配。
- `checkDepthRgbAlignment(DepthFrame, ImageGeometry)`：检查上游注册声明与实际 RGB 网格的一致性。

DepthOptions 必须明确 input_unit 与 convention。相同尺寸不能证明对齐；未知/未对齐仍可解码，但不会被标记为已注册。

- `decodeCameraInfo(CameraInfoData)`：校验话题、光学 frame、标定及 ROI/binning，保留原始 K/D/R/P。
- `decodeRosCameraInfo(sensor_msgs::msg::CameraInfo, topic)`：真实 ROS 消息及来源话题适配。
- `checkCameraInfoImage(info, image, options)`：显式流绑定、光学帧、实际图像尺寸与标定时间策略检查。
- `checkSensorTimes(reference, candidate, options)`：明确同一时钟、整数纳秒容差检查；返回 candidate-reference，不修改时间。

- `TfIndex::insert(TfTransform)`：按 child/header 时间索引动态/静态变换，保存来源并检查冲突。
- `TfIndex::inspectChain(target, source, options)`：返回坐标链、各边首末时间和共同覆盖区间，保留时间空洞。
- `TfIndex::checkTime(target, source, stamp, options)`：核验指定采样时刻，返回精确/静态样本或插值端点，不输出尚未计算的位姿。
- `ingestRosTf(index, TFMessage, topic, message_id, binding)`：接入 /tf、/tf_static，逐条报告插入/重复/失败。

TF 规划、接口、完整示例见 [TF 核心](docx/modules/tf/tf.md) 与 [ROS 适配](docx/modules/tf/tf_ros.md)。显式 clock/segment 防止跨数据段混用；默认插值间隔 0 表示只接受精确采样点。链连通不代表时间有共同覆盖。

CameraInfo 的分步规划、接口、边界和完整示例见 [CameraInfo 核心](docx/modules/camera_info/camera_info.md)。各模块时间不能自动视为同步：Observation 索引采用 RGB 时间，深度点使用 Depth 时刻的 TF，原始时间分别保留；固定标定只能显式复用。写入与在线控制规则见 [公共时间约定](docx/common/sensor_time.md)。

全部说明从 [文档导航](docx/README.md) 进入。RGB 接口见 [核心说明](docx/modules/rgb/rgb.md)，ROS 接入见 [适配说明](docx/modules/rgb/rgb_ros.md)。Depth 见 [核心说明](docx/modules/depth/depth.md) 与 [ROS 适配](docx/modules/depth/depth_ros.md)。

## ROS 2 构建

在已经加载 ROS 2 Humble 的工作区中：

```bash
colcon build --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select vln_memory
colcon test-result --verbose
```

下游使用 `find_package(vln_memory REQUIRED)`，链接 `vln_memory::vln_memory_rgb_ros`。Depth 适配链接 `vln_memory::vln_memory_depth_ros`。CameraInfo 适配链接 `vln_memory::vln_memory_camera_info_ros`，其核心为 `vln_memory::vln_memory_camera_info`，自动传递公共时间目标。只使用核心时分别链接 `vln_memory::vln_memory_rgb`、`vln_memory::vln_memory_depth`。

TF ROS 入口链接 `vln_memory::vln_memory_tf_ros`，公开传递 `tf2_msgs`、`geometry_msgs` 与核心；只检查索引时链接 `vln_memory::vln_memory_tf`。原生 `tf2` 仅用于当前对照测试，不是索引/适配库的运行依赖。

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

本包是独立本地仓库。现有 RGB 已建立 main 基线；Depth 已在 codex/depth-module 本地提交；CameraInfo 已本地提交 d9b030b 于 codex/camera-info-module；当前 TF 位于基于该提交的 codex/tf-module。模块完成后执行本地 git add，保留暂存差异供审阅，合并由用户确认后执行。

```bash
git status --short --branch
git diff --cached --stat
git diff --cached
```
