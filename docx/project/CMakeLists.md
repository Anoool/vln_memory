# CMakeLists.txt：观测准备层构建与导出

## 实施前规划与自审

TF、builder 尚未生成空目标。当前建立独立 RGB/Depth/CameraInfo 核心、公共时间检查及各自 ROS 适配。两条构建路径复用同一核心源文件和测试，避免出现测试版本与正式版本行为不同。

## 目标与选项

| 名称 | 用途 |
| --- | --- |
| vln_memory_rgb | 无 ROS 依赖的共享库，公开 C++17 和 include 路径 |
| vln_memory_rgb_ros | 启用 ROS 时构建，薄适配真实 sensor_msgs::msg::Image |
| vln_memory_depth | 独立 Depth 核心，不链接 RGB 实现 |
| vln_memory_depth_ros | ROS Image 到 Depth 的薄适配，传递 sensor_msgs 依赖 |
| vln_memory_sensor_time | 无 ROS 的整数纳秒时差检查，可独立给 builder/TF 使用 |
| vln_memory_camera_info | 标定与图像关联；PUBLIC 链接 sensor_time，不链接 RGB/Depth |
| vln_memory_camera_info_ros | ROS CameraInfo 到核心的薄适配，PUBLIC 传递 sensor_msgs |
| test_sensor_time | 公共时间边界/时钟关系测试 |
| test_camera_info | 标定、ROI/binning、流绑定、时间策略测试 |
| test_camera_info_ros | 真实 ROS CameraInfo 映射与所有权测试 |
| test_rgb | 始终可构建的核心行为测试 |
| test_depth | 单位、数值类型、大小端、统计和注册声明测试 |
| test_depth_ros | 真实 Image 深度适配测试 |
| test_rgb_ros | 启用 ROS 时构建的真实消息适配测试 |
| VLN_MEMORY_ENABLE_ROS | 默认 ON，生成 ament 包；OFF 用于无 ROS 核心验证 |
| BUILD_TESTING | 控制测试构建 |

安装后导出核心目标 `vln_memory::vln_memory_rgb`、`vln_memory::vln_memory_depth`，以及启用 ROS 时的 `vln_memory::vln_memory_rgb_ros`、`vln_memory::vln_memory_depth_ros`。无 ROS 路径使用 `cmake/vln_memoryConfig.cmake.in` 生成包配置供 find_package 使用。ROS 路径由 ament 生成配置，公开 sensor_msgs 依赖。

`package.xml` 声明 ament_cmake 和 sensor_msgs；未引入 vln_agent、模型、TF 或数据库依赖。README 和 docx 说明随包安装。

操作命令见 [项目 README](../../README.md)。RGB 的源码、测试、安装与下游消费验证情况见 [RGB 验证记录](../modules/rgb/rgb.md#实施与验证记录)。

Depth 和真实 Humble 安装后的下游验证见 [Depth 验证记录](../modules/depth/depth.md#验证结果)。

## 按层与模块组织源码

库目标不因目录整理改名。RGB 源码位于 src/bag/rgb，Depth 位于 src/bag/depth；测试分别位于 test/bag/rgb、test/bag/depth。公开头文件的安装位置保留 include/vln_memory/bag/common、bag/rgb、bag/depth，消费者按该完整路径 include。

目录与整体系统位置见 [当前架构](architecture.md) 和 [系统规划](system_plan.md)。

## CameraInfo 新增目标与验证

新增目标统一导出为 vln_memory::vln_memory_sensor_time、vln_memory::vln_memory_camera_info、vln_memory::vln_memory_camera_info_ros；相机核心经 PUBLIC 链接继承公共头文件和 C++17，适配经 PUBLIC 传递核心及 sensor_msgs。

新增安装路径为 include/vln_memory/bag/common/sensor_time.hpp、bag/camera_info/camera_info.hpp；ROS 模式额外安装 camera_info_ros.hpp。核心测试总数 4，真实 ROS 模式总数 7。独立安装项目编译运行 camera_info_consumer，ROS 模式额外以同一真实消息测试源码构建 camera_info_ros_consumer。

没有引入新第三方依赖。详细使用见 [CameraInfo 核心](../modules/camera_info/camera_info.md) 与 [公共时间](../common/sensor_time.md)。
