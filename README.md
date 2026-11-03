# vln_memory

ROS 2 Humble / C++17 空间语义记忆功能包。**当前交付目标限定为原设计第 1、2 部分：核心数据规范及离线构建、工具、纠错、标注、发布与 Viewer；完整在线查询/导航接入暂不开发。**

逐条完成情况见 [原设计第 1、2 部分范围对照](docx/project/sections_1_2_scope.md)。当前两个原文部分均未完成。

**当前已实现观测准备层的读取/解码/配对、D8 受控兼容、raw/rectified 有效相机模型及 preparation 导出；TF 数值位姿已接入导出，下一步是深度几何。preparation 的读取/引用/筛选放到后续 API 输入组织时处理，不先新增最小 Observation。**

传感器基础的历史核查见 [核查报告](docx/reviews/observation_preparation_audit.md)，集中参数/返回值及提供文件见 [接口手册](docx/interfaces/observation_preparation_api.md)。当前已实现 BagBuilder 第一版：SQLite rosbag 两遍读取、自动配对、标定选择/TF 历史检查、原始 CDR、PNG/米制 NPY 和 JSONL 导出。有效相机模型与数值位姿已接入逐帧输出，关键帧与后续记忆业务待实现，详见 [BagBuilder 接口](docx/interfaces/bag_builder_api.md)。

收敛后的目标、各层职责与开发顺序见 [系统整体规划](docx/project/system_plan.md)。

RGB、Depth、CameraInfo、TF、bag 五模块的职责和实施顺序见 [观测准备层与源码组织](docx/project/architecture.md)。RGB/Depth 输出标准内存帧；builder 已保存媒体并进行时间配对，正式 Observation 在有效模型/位姿补齐后输出。

## 当前源码布局

头文件和实现分别位于 include/vln_memory/bag/ 与 src/bag/，按 common、rgb、depth、camera_info、tf、builder 分类；模块测试位于 test/bag/。安装后的头文件保留同样结构，例如：

```cpp
#include <vln_memory/bag/rgb/rgb_ros.hpp>
#include <vln_memory/bag/depth/depth_ros.hpp>
#include <vln_memory/bag/camera_info/camera_info_ros.hpp>
#include <vln_memory/bag/tf/tf_ros.hpp>
```

TF 位于本层 tf，bag 读取与准备组装位于 builder，薄入口位于 src/apps。其他系统层暂不生成空目录。

公共头的 `vln_memory/bag/...` 相对编译器的 `include/` 搜索根，实际位于 `include/vln_memory/bag/...`，不表示头文件位于 src/bag。所有库直接声明源码/安装后的搜索根，解释与验证见 [头文件路径说明](docx/project/header_include_paths.md)。

## BagBuilder 运行（严格或最近时间配对）

先修改 [config/vln_memory.yaml](config/vln_memory.yaml) 的真实话题、光学 frame、深度单位/注册及同钟分段依据。第一版只支持一个明确段的未压缩 SQLite ROS 2 bag；目录读取 metadata 的全部分卷，单独 .db3 明确标记单文件范围。

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run vln_memory bag_builder --config config/vln_memory.yaml \
  --bag /absolute/path/recording --output /absolute/path/new_dataset
```

RGB-D 的 sync_tolerance_ns 支持非负整数纳秒：0 严格，正值按 RGB 基准单调一对一选择最近 Depth。示例显式 30000000（30 ms 试用值），API/省略默认仍为 0。保留原始 stamp 和 delta_ns，exact 按实际差值填写。支持原 Image、JPEG RGB8/BGR8 和 16UC1 compressedDepth PNG，其他压缩编码明确报告未支持。输出必须是新目录。返回 completed_with_issues 表示数据集完整写出但有坏帧/未配对/缺几何依据，应检查 reports 和 prepared_pairs。完整文件树及包外调用见 [接口手册](docx/interfaces/bag_builder_api.md)，每个文件职责见 [BagBuilder](docx/modules/bag/bag_builder.md)。样例配置仅为格式示例，不能证明设备单位或硬件同步。零门限无对为 no_exact_pairs，正门限无对为 no_pairs；当前全包时间戳与短片段验证见 [最近配对验收](docx/reviews/rgbd_nearest_pairing_verification.md)。

## 当前验证与下一步

截至 2026-11-03，ROS CTest 16/16、无 ROS 核心 9/9 通过。实际模型接口见 [CameraModel](docx/modules/camera_info/camera_model.md)，当前验证边界见 [相机模型验收](docx/reviews/camera_model_verification.md)；之前的 [最近配对验收](docx/reviews/rgbd_nearest_pairing_verification.md) 保留历史结果。

完整真实 bag 已提取：21441 条消息，3474 RGB + 3415 Depth 全部解码并生成 6889 个相机模型；7268 条原始 D8 保留、显式兼容生成 D5。30 ms 下 2671 对全部具备两图模型，prepared_pairs=0 仍由动态 TF 精确时间/注册依据阻断。当前使用显式固定标定复用，索引复核全段一致；物理同步、尺度及注册精度仍待验收。

当前产物为 bag_preparation/1，builder_version=5。每帧/配对的 effective_model_computed 记录实际成功；全局标志表示至少一帧成功，数量看 camera_models。可用的 map 数值位姿与样本证据一起写入逐帧记录。关键帧待实现。完整媒体、临时输入/子集及重复测试产物已按用户要求于 2026-11-03 清理；小型验收摘要/日志/元数据快照仍在 build/camera-model-20261103，当前构建与安装保留。需要完整 preparation 时从原始 ZIP 重建，快照不能直接消费。见 [清理记录](docx/reviews/build_cleanup_20261103.md)。

本录包话题/frame/兼容策略见 [camera_record.yaml](config/camera_record.yaml)，修改路径后可用原 CLI 运行。完整验收及独立回查命令见上述报告。

下一步按 [系统规划第 10 节](docx/project/system_plan.md#10-当前最近行动) 完善深度几何；preparation 输入筛选在调用 API 时实现。S3–S6 工作库、工具/模型构建、纠错发布和 Viewer 尚未实现。

## 当前接口

- `buildBag(BagBuildOptions)`：返回结构化结果和完整 preparation 数据集路径；不含虚构位姿。
- `pairRgbDepth(...)` / `CameraInfoIndex::select(...)`：独立时间配对与标定候选选择。

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

本包是独立仓库，公开远程为 [Anoool/vln_memory](https://github.com/Anoool/vln_memory)。main 保留 RGB 基线；此前能力基线为 24af181，进度文档基线为 c1c7ab3。本轮计划、实现和验收位于 codex/camera-model-compatibility，分步提交并推送供审阅，不自动合并 main。实时分支与暂存状态以下列命令为准。

```bash
git status --short --branch
git diff --cached --stat
git diff --cached
```


当前来源话题校验差异已统一修复，各模块注释也已补充。修复范围、输入约束收紧和验证见 [实施记录](docx/project/sensor_validation_comments_plan.md)。
