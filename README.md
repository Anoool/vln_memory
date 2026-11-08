# vln_memory

`vln_memory` 是面向视觉语言导航（VLN）的空间语义记忆项目，使用 ROS 2 Humble 和 C++17。当前已跑通 **ROS bag → preparation → 正式观测 → 物品与空间实体创建 → SQLite 工作库**，处于 S3 几何与工作库阶段。已发布的 preparation 和正式观测可在无 ROS 环境中使用。

| 入口 | 作用 |
| --- | --- |
| `bag_builder` | 读取 RGB-D 录包，提取媒体、标定和历史 TF，保存配对、来源及待处理原因。 |
| `memory_build_input` | 单独生成用途筛选、关键帧和分批清单，适合检查选帧结果。 |
| `memory_observation_dataset` | 直接从 preparation 筛选并发布正式观测、批次及文件摘要。 |
| `createObjectEntity` / `createRegionEntity` | C++ 接口。用观测和深度选区或空间内部确认创建拥有型实体记录。 |
| `createObservedEntityStore` / `insertObservedEntity` / `appendObservedEvidence` / `resolveEntityLocation` | C++ 接口。绑定正式观测集，事务写入、追加观测及选用已有证据补定位。 |

## 安装

### ROS 2 构建

以下命令适用于已配置 ROS 2 软件源并安装 Humble 的 Ubuntu 22.04。首次安装可参照 [ROS 2 Humble 官方安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

```bash
sudo apt update
sudo apt install git build-essential cmake python3-colcon-common-extensions python3-numpy \
  libopencv-dev nlohmann-json3-dev libyaml-cpp-dev libssl-dev libsqlite3-dev \
  ros-humble-ament-cmake ros-humble-sensor-msgs ros-humble-geometry-msgs \
  ros-humble-tf2-msgs ros-humble-tf2 ros-humble-rmw ros-humble-rclcpp ros-humble-rcutils \
  ros-humble-rosbag2-cpp ros-humble-rosbag2-storage ros-humble-rosbag2-storage-default-plugins \
  ros-humble-cv-bridge ros-humble-compressed-depth-image-transport

git clone https://github.com/Anoool/vln_memory.git
cd vln_memory
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run vln_memory bag_builder --help
```

构建完成后运行 ROS 和录包回归测试。

```bash
colcon test --packages-select vln_memory --return-code-on-test-failure
colcon test-result --verbose
```

### 无 ROS 构建

已经有 preparation 数据时，可用无 ROS 构建完成回读、观测发布、投影和实体创建。首次从 ROS bag 提取数据使用上面的 ROS 构建。

```bash
sudo apt install build-essential cmake libopencv-dev nlohmann-json3-dev libssl-dev libsqlite3-dev
cmake -S . -B build/standalone -DVLN_MEMORY_ENABLE_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/standalone --parallel 2
ctest --test-dir build/standalone --output-on-failure
cmake --install build/standalone --prefix "$PWD/build/standalone-install"
```

## 快速使用

### 1. 从录包导出 preparation

输入可为带 `metadata.yaml` 的未压缩 SQLite 录包目录，或其中一个 `.db3` 分卷。复制配置后按采集条件填写话题、坐标系、深度单位、注册状态及共同时间基准的依据。输出目录应尚未存在。详见 [录包配置](docx/config/vln_memory.md)。

```bash
cp config/vln_memory.yaml /tmp/vln_memory-local.yaml
# 按录包实际内容编辑配置，并替换下方录包路径
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config /tmp/vln_memory-local.yaml \
  --bag /absolute/path/to/recording --output "$PWD/build/my_preparation" --sync-tolerance-ms 30
```

`--sync-tolerance-ms 30` 仅覆盖本次运行的时间配对门限，等价于 `bag.sync_tolerance_ns=30000000`。门限为 0 时仅做精确配对，正值使用门限内一对一最近配对。`pairs` 是时间配对数，`prepared_pairs` 还需通过标定、位姿、深度质量和 RGB/Depth 注册检查。`bag.save_raw_cdr` 默认省略 CDR 副本并保留消息摘要，显式设为 `true` 可保留原始消息。新导出绑定 PNG/NPY 内容摘要，历史数据可用 [preparation_compact](docx/modules/bag/preparation_compactor.md) 生成带摘要的新副本。

`bag_manifest.json` 描述来源与有效配置，`frames.jsonl`、`rgbd_pairs.jsonl` 记录帧和配对，媒体位于 `evidence/`。在 `reports/` 查看统计、错误及待处理原因。接口和回读方式见 [BagBuilder](docx/interfaces/bag_builder_api.md) 与 [PreparationReader](docx/modules/bag/preparation_reader.md)。

### 2. 发布正式观测

从上一步的 preparation 直接发布。将示例地图 ID、轨迹版本、frame 和依据换成这批数据的实际上下文，其中 `--map-frame` 必须与 preparation 一致。`--purpose rgb` 选择有 RGB 媒体的可读配对，`depth_map` 要求深度模型与历史位姿，`prepared_rgbd` 还要求来源配对达到完整 RGB-D 几何准备状态。

```bash
ros2 run vln_memory memory_observation_dataset \
  --input "$PWD/build/my_preparation" --output "$PWD/build/my_observations" \
  --purpose depth_map --batch-size 4 --preparation-mode bundle \
  --map-id sample_map --geometry-revision trajectory-v1 \
  --map-frame map --map-basis "Recorded map and TF for this acquisition"
```

无 ROS 安装使用 `build/standalone-install/lib/vln_memory/memory_observation_dataset`，参数相同。产物包括 `observation_manifest.json`、`observations.jsonl`、`batches.jsonl`、选帧清单和问题报告。示例的 `bundle` 模式把精简 preparation 一并发布到输出的 `preparation/` 子目录。移动整个观测目录后仍可读取媒体和投影。默认 `reference` 模式共用输入 preparation，适合从同一套媒体派生多份观测。只需预览选帧与分批时，可独立运行 [memory_build_input](docx/modules/engine/build_input.md)。完整输出与错误语义见 [观测数据集发布](docx/modules/engine/observation_dataset.md)。

### 3. 使用观测创建实体

C++ 下游通过 `find_package(vln_memory CONFIG REQUIRED)` 链接 `vln_memory::vln_memory_observation_dataset` 与 `vln_memory::vln_memory_entity_builder`。用 `openObservationDataset` 打开观测目录，再按 ID 或批次取回记录。需要媒体时使用 `openSourcePreparation` 核对来源，便携包自动打开内置来源，引用模式迁移后可显式传入实际路径。详见 [数据集读取接口](docx/modules/engine/observation_dataset_reader.md)。随后选择实际业务入口：

- `createObjectEntity` 接收深度原图的 `bbox` 或 `polygon`，用该深度帧的历史位姿取得实测表面代表点。
- `createRegionEntity` 接收上游给出的区域内部确认，在 RGB 参考时刻从历史底盘位姿取代表点。
- 缺定位条件时保留 `pending_location`、观测引用和原因。调用方可将记录写入与观测集绑定的 SQLite 工作库。

详见 [实体契约](docx/modules/memory/entity.md)、[创建接口](docx/modules/engine/entity_builder.md) 、[SQLite 写入与回读](docx/modules/engine/observed_entity_store.md)和[安装导出](docx/project/CMakeLists.md)。

## 项目进度

| 阶段 | 当前状态 |
| --- | --- |
| S0–S1 数据契约、读取与配对 | 传感器时间、来源、录包配置、读取配对及 preparation 导出已实现。完整构建系统配置随业务模块补齐。 |
| S2 离线观测 | 相机模型、历史 TF、索引回读、用途筛选、关键帧、正式观测发布和按 ID/批次回读已实现。 |
| S3 几何与工作库 | 已实现单点和深度原图区域投影、物品及空间创建、SQLite 事务写入、多观测追加、补定位和按 ID 回读。关系和查询索引待实现。 |
| S4–S6 构建、发布与查看 | 八项构建工具、模型批次、纠错标注、完整记忆版本和 MemoryViewer 待实现。 |

真实短包已验证媒体回读、历史位姿、数值投影和物品创建。空间定位与异常场景使用带明确确认的构造录包验证。深度物理尺度、RGB/Depth 注册精度和曝光同步仍需独立验收。

**下一步**接入 `inside` 空间归属和 `near` 距离关系。完整顺序见 [系统规划](docx/project/system_plan.md#102-后续任务与依赖)，原设计逐条状态见 [范围与验收](docx/project/sections_1_2_scope.md)。

## 目录结构

```text
include/vln_memory/{bag,memory,engine}/  # 按层公开的 C++ 接口
src/bag/                                # 传感器、几何和 preparation
src/memory/                             # Observation 与 Entity 记录
src/engine/                             # 选帧、观测发布、回读投影和实体创建
src/apps/                               # 命令行入口
test/{bag,memory,engine}/                # 模块与链路验证
test/install_consumer/                  # 安装包下游验证
config/                                 # 录包配置及采集示例
docx/                                   # 架构、模块说明、规划及验收记录
```

从 [文档导航](docx/README.md) 查看各模块，依赖和数据边界见 [当前架构](docx/project/architecture.md)。

## 贡献指南

1. 阅读 [仓库开发约定](AGENTS.md) 和 [系统规划](docx/project/system_plan.md)。在 `codex/` 分支上按可独立审阅的改动提交。
2. 修改接口或行为时同步更新对应模块文档和 [文档导航](docx/README.md)。保留原始时间戳、单位和数据来源。
3. 运行受影响的测试并提交 Pull Request，说明改动范围、接口影响、验证方式与已知限制。录包、生成数据和凭据请保留在 Git 之外。

## 许可证

项目采用 [Apache License 2.0](LICENSE)，与 [ROS 包清单](package.xml) 中的 `Apache-2.0` 声明一致。
