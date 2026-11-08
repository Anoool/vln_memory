# CMakeLists.txt 构建与导出

源码构建入口为根目录 [CMakeLists.txt](../../CMakeLists.txt)。所有库使用 C++17，公开头按 `include/vln_memory/{bag,memory,engine}/` 安装。每个导出库直接声明头搜索根和语言版本，目标之间通过链接表达实际依赖。详见 [头文件搜索说明](header_include_paths.md)。

## 构建模式

| 设置 | 可用能力 |
| --- | --- |
| `VLN_MEMORY_ENABLE_ROS=ON`（默认） | 构建核心能力、四个 ROS 消息适配库及 `vln_memory_bag_builder`。提供 ament 包和 `bag_builder` 命令。 |
| `VLN_MEMORY_ENABLE_ROS=OFF` | 使用普通 CMake 生成核心包。可使用 preparation 回读与精简转换、选帧、观测数据集发布、点和区域投影、物品与空间创建。 |
| `BUILD_TESTING=ON` | 启用核心测试。ROS 构建额外执行消息适配及从构造录包到正式观测、实体的链路回归。 |

两种模式均使用 OpenCV、nlohmann/json、OpenSSL Crypto 和 SQLite3。ROS 录包读取另依赖 rosbag2、YAML 和压缩图像相关组件。安装命令和系统包列表见 [根 README](../../README.md)。

## 主要公开目标

所有目标均以 `vln_memory::` 命名空间导出。

| 目标 | 用途 |
| --- | --- |
| `vln_memory_rgb`、`vln_memory_depth`、`vln_memory_sensor_time`、`vln_memory_camera_info`、`vln_memory_tf` | 传感器原语、时间检查和历史位姿。 |
| `vln_memory_rgbd_sync`、`vln_memory_camera_info_index`、`vln_memory_camera_model` | 时间配对、标定索引及有效模型。 |
| `vln_memory_depth_projection`、`vln_memory_preparation_reader` | 深度单点和区域几何；preparation 索引回读及精简转换。 |
| `vln_memory_observation`、`vln_memory_entity` | 正式观测与实体的数据接口。 |
| `vln_memory_build_input`、`vln_memory_observation_builder`、`vln_memory_observation_dataset` | 输入选择、正式观测组装、数据集发布及索引回读。 |
| `vln_memory_observation_depth_region`、`vln_memory_entity_builder` | 观测区域回读投影及实体创建。 |
| `vln_memory_entity_store`、`vln_memory_observed_entity_store` | SQLite 工作库及正式观测集写入接入。 |

ROS 模式还导出 `vln_memory_rgb_ros`、`vln_memory_depth_ros`、`vln_memory_camera_info_ros`、`vln_memory_tf_ros` 及 `vln_memory_bag_builder`。两种模式都安装 `preparation_compact`、`memory_build_input` 和 `memory_observation_dataset` 至 `lib/vln_memory/`。ROS 模式还在该目录安装 `bag_builder`，并将录包配置安装到 `share/vln_memory/config/`。文档随两种模式安装。

## 下游使用

```cmake
find_package(vln_memory CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE vln_memory::vln_memory_entity_builder)
```

代码可直接使用 `#include <vln_memory/engine/entity_builder.hpp>`。无 ROS 包配置位于安装前缀下的 `lib/cmake/vln_memory/`，ROS 构建使用 ament 生成的配置。Linux 安装设置库和命令的相对运行时库路径。

## 验证

工作库基线的 CTest 记录为无 ROS **16/16**、ROS **29/29**。核心测试覆盖数据契约、数值行为和 SQLite 回滚，ROS 录包夹具额外覆盖正式观测发布、区域投影、实体创建与 SQLite 回读。可用 `entity_store_consumer` 验证安装后的工作库入口。

完成 [无 ROS 安装](../../README.md#无-ros-构建) 后，可从源码根目录验证安装目标：

```bash
cmake -S test/install_consumer -B build/standalone-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/build/standalone-install"
cmake --build build/standalone-consumer --target entity_consumer
build/standalone-consumer/entity_consumer
```

该调用执行核心 ID 检查。带录包夹具目录或既有短包产物时，可使用 `--fixture-pointer` 或 `--published` 运行实体链路检查，参数见 [实体创建说明](../modules/engine/entity_builder.md)。物理标定、注册和曝光同步单独验收。
