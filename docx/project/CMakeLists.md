# CMakeLists.txt：RGB 模块构建与导出

## 实施前规划与自审

四个后续模块不生成空目标。本轮只建立 RGB 核心和可选 ROS 适配。两条构建路径复用同一核心源文件和测试，避免出现测试版本与正式版本行为不同。

## 目标与选项

| 名称 | 用途 |
| --- | --- |
| vln_memory_rgb | 无 ROS 依赖的共享库，公开 C++17 和 include 路径 |
| vln_memory_rgb_ros | 启用 ROS 时构建，薄适配真实 sensor_msgs::msg::Image |
| test_rgb | 始终可构建的核心行为测试 |
| test_rgb_ros | 启用 ROS 时构建的真实消息适配测试 |
| VLN_MEMORY_ENABLE_ROS | 默认 ON，生成 ament 包；OFF 用于无 ROS 核心验证 |
| BUILD_TESTING | 控制测试构建 |

安装后导出 `vln_memory::vln_memory_rgb` 和（启用 ROS 时）`vln_memory::vln_memory_rgb_ros`。无 ROS 路径使用 `cmake/vln_memoryConfig.cmake.in` 生成包配置供 find_package 使用。ROS 路径由 ament 生成配置，公开 sensor_msgs 依赖。

`package.xml` 声明 ament_cmake 和 sensor_msgs；未引入 vln_agent、模型、TF、Depth 或数据库依赖。README 和 docx 说明随包安装。

操作命令见 [项目 README](../../README.md)。RGB 的源码、测试、安装与下游消费验证情况见 [RGB 验证记录](../modules/rgb/rgb.md#实施与验证记录)。
