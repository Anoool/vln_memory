# CMakeLists.txt：观测准备层构建与导出

当前公共头路径配置以文末“2026-10-31 统一公共头搜索根”小节为准；前面保留各模块引入时的阶段记录。
## 实施前规划与自审

builder 尚未生成空目标。当前建立独立 RGB/Depth/CameraInfo/TF 核心、公共时间检查及各自 ROS 适配。两条构建路径复用同一核心源文件和测试，避免出现测试版本与正式版本行为不同。

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
| vln_memory_tf | 无 ROS 的历史索引、坐标链和时间覆盖检查 |
| vln_memory_tf_ros | 真实 TFMessage 接入，传递 tf2_msgs/geometry_msgs |
| test_tf | 拓扑、区间交集、冲突及生成用例 |
| test_tf_ros | 真实消息字段/来源/话题，原生 tf2 语义对照 |
| test_rgb | 始终可构建的核心行为测试 |
| test_depth | 单位、数值类型、大小端、统计和注册声明测试 |
| test_depth_ros | 真实 Image 深度适配测试 |
| test_rgb_ros | 启用 ROS 时构建的真实消息适配测试 |
| VLN_MEMORY_ENABLE_ROS | 默认 ON，生成 ament 包；OFF 用于无 ROS 核心验证 |
| BUILD_TESTING | 控制测试构建 |

安装后导出核心目标 `vln_memory::vln_memory_rgb`、`vln_memory::vln_memory_depth`，以及启用 ROS 时的 `vln_memory::vln_memory_rgb_ros`、`vln_memory::vln_memory_depth_ros`。无 ROS 路径使用 `cmake/vln_memoryConfig.cmake.in` 生成包配置供 find_package 使用。ROS 路径由 ament 生成配置，公开 sensor_msgs 依赖。

`package.xml` 声明 ament_cmake、sensor_msgs、tf2_msgs、geometry_msgs，tf2 为测试依赖；未引入 vln_agent、模型或数据库依赖。README 和 docx 说明随包安装。

操作命令见 [项目 README](../../README.md)。RGB 的源码、测试、安装与下游消费验证情况见 [RGB 验证记录](../modules/rgb/rgb.md#实施与验证记录)。

Depth 和真实 Humble 安装后的下游验证见 [Depth 验证记录](../modules/depth/depth.md#验证结果)。

## 按层与模块组织源码

库目标不因目录整理改名。RGB 源码位于 src/bag/rgb，Depth 位于 src/bag/depth；测试分别位于 test/bag/rgb、test/bag/depth。公开头文件的安装位置保留 include/vln_memory/bag/common、bag/rgb、bag/depth，消费者按该完整路径 include。

目录与整体系统位置见 [当前架构](architecture.md) 和 [系统规划](system_plan.md)。

## CameraInfo 新增目标与验证

新增目标统一导出为 vln_memory::vln_memory_sensor_time、vln_memory::vln_memory_camera_info、vln_memory::vln_memory_camera_info_ros；相机核心经 PUBLIC 链接继承公共头文件和 C++17，适配经 PUBLIC 传递核心及 sensor_msgs。

新增安装路径为 include/vln_memory/bag/common/sensor_time.hpp、bag/camera_info/camera_info.hpp；ROS 模式额外安装 camera_info_ros.hpp。核心测试总数 4，真实 ROS 模式总数 7。独立安装项目编译运行 camera_info_consumer，ROS 模式额外以同一真实消息测试源码构建 camera_info_ros_consumer。

没有引入新第三方依赖。详细使用见 [CameraInfo 核心](../modules/camera_info/camera_info.md) 与 [公共时间](../common/sensor_time.md)。

## TF 新增目标与导出

vln_memory::vln_memory_tf 显式公开 C++17/include，不依赖 ROS；vln_memory::vln_memory_tf_ros PUBLIC 链接核心，并传递 tf2_msgs、geometry_msgs。tf2 仅在 ROS+BUILD_TESTING 时查找用于对照测试，关闭测试的生产包不需要 tf2 库。

公开头按 bag/tf/tf.hpp 和 tf_ros.hpp 安装；无 ROS 模式不安装适配头。完整核心测试总数 5，真实 ROS 模式总数 9；新增独立下游 tf_consumer、tf_ros_consumer。详情见 [TF 核心](../modules/tf/tf.md) 和 [ROS 适配](../modules/tf/tf_ros.md)。


## 统一名称检查修复

新增 header-only 的 common/sensor_names.hpp 并随公共头安装，无新增运行库。sensor_names_core 独立无 ROS 测试使核心数量变为 6；真实 ROS 总数为 10。rmw 仅作为 test_tf_ros 对照测试依赖，运行目标不依赖 rmw。当前编辑器格式约定见根目录 .editorconfig，修复说明见 [实施记录](sensor_validation_comments_plan.md)。

## BagBuilder 第一版新增目标与安装验证

- vln_memory::vln_memory_rgbd_sync：无 ROS，公开传递 sensor_time。
- vln_memory::vln_memory_camera_info_index：无 ROS，公开传递 CameraInfo。
- vln_memory::vln_memory_bag_builder：仅 ROS 构建，集成配置/官方 reader/媒体/JSONL/调度。公开导出 rosbag2_cpp、rosbag2_storage、rclcpp、rcutils、消息/JSON 头依赖；YAML、OpenSSL、OpenCV 是实现依赖。
- bag_builder 可执行文件安装到 lib/vln_memory，config 安装到 share/vln_memory/config。
- Linux 安装共享库设置 $ORIGIN，以解析同目录的传递依赖。

现有核心仍可 VLN_MEMORY_ENABLE_ROS=OFF 构建；第一版共有 8 个核心、14 个 ROS 构建 CTest。test/install_consumer 只查安装包：核心 6 个消费者、ROS 10 个；完整 builder 消费者还可传 YAML 与新输出目录实际处理 bag。

实际接口和参数见 [BagBuilder](../interfaces/bag_builder_api.md)，验证范围见 [验收记录](../reviews/bag_builder_v1_verification.md)。

## 零容差压缩输入接入

ROS builder 新增 image_decoder_ros.cpp，使用 cv_bridge 和 compressed_depth_image_transport。Humble 2.5.5 未导出 codec 链接 target，从其已找到的包前缀定位头/库，不写死 /opt/ros/humble。新增 image_decoder_ros 测试，公开新头并验证安装后消费者；无 ROS 核心目标不增加这些依赖。

## 2026-10-31 统一公共头搜索根

所有实际库在 memory_export_targets 清单内逐一直接声明以下属性，不再仅依赖另一个库间接提供本包头文件路径：

```cmake
foreach(memory_target IN LISTS memory_export_targets)
  target_compile_features(${memory_target} PUBLIC cxx_std_17)
  target_include_directories(${memory_target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()
```

例如 `<vln_memory/bag/builder/bag_builder.hpp>` 实际查找 `<仓库>/include/vln_memory/bag/builder/bag_builder.hpp`，安装后查找 `<prefix>/include/vln_memory/bag/builder/bag_builder.hpp`。cpp 所在 src/bag 不是头文件搜索根。

PUBLIC 同时供本库编译和下游使用；target_link_libraries 继续保留真实实现依赖。没有把头复制到 src，也没有将引用改成 include/vln_memory 或跨目录 ../。

详细路径对应、人工编译和验证见 [头文件搜索说明](header_include_paths.md)。导出的函数、库目标、安装结构和零容差行为不变。
