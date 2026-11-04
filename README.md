# vln_memory

`vln_memory` 是面向视觉语言导航（VLN）的空间语义记忆项目，使用 ROS 2 Humble 和 C++17。当前可用的 `bag_builder` 将 ROS 2 RGB-D 录包转换为离线观测准备数据，保留图像、标定、历史位姿、原始消息与质量报告，供后续构建空间记忆。

## 安装

### ROS 2 构建

以下命令适用于已配置 ROS 2 软件源并安装 Humble 的 Ubuntu 22.04。首次安装可参照 [ROS 2 Humble 官方安装指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

```bash
sudo apt update
sudo apt install git build-essential cmake python3-colcon-common-extensions python3-numpy \
  libopencv-dev nlohmann-json3-dev libyaml-cpp-dev libssl-dev \
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

仅使用核心库时，在仓库根目录执行以下命令。此模式提供标定、配对、TF、单点投影和 preparation 读取接口。录包 CLI 由 ROS 构建提供。

```bash
sudo apt install build-essential cmake libopencv-dev nlohmann-json3-dev
cmake -S . -B build/standalone -DVLN_MEMORY_ENABLE_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/standalone --parallel 2
ctest --test-dir build/standalone --output-on-failure
cmake --install build/standalone --prefix "$PWD/build/standalone-install"
```

## 使用示例

输入是单一时钟段的未压缩 SQLite rosbag。可传入含 `metadata.yaml` 的录包目录或指定一个 `.db3` 分卷。先复制配置，根据设备和录制方式填写 RGB、Depth、CameraInfo 与 TF 话题、坐标系、深度单位、注册状态及共同时间基准的依据。详见 [配置说明](docx/config/vln_memory.md)。

```bash
cp config/vln_memory.yaml /tmp/vln_memory-local.yaml
# 编辑 /tmp/vln_memory-local.yaml，并替换下面的录包路径
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config /tmp/vln_memory-local.yaml \
  --bag /absolute/path/to/recording --output "$PWD/build/my_preparation"
```

输出目录必须尚未存在。配置项 `bag.sync_tolerance_ns` 为 `0` 时按原始时间戳精确配对，正值启用容差内的一对一最近配对。样例中的 30 ms 是试用值，请按录制条件设定。配对会保留两路原始时间与实际时差，物理同步与图像注册需要独立验证。

- `bag_manifest.json` 记录数据集版本、来源和有效配置。
- `frames.jsonl` 和 `rgbd_pairs.jsonl` 记录帧、配对、模型、位姿和状态，`evidence/` 保存原始消息与媒体。
- `reports/summary.json`、`reports/issues.jsonl` 和 `reports/pending.jsonl` 记录统计、问题与待处理原因。`completed_with_issues` 表示数据集已写出，需结合 `prepared_pairs` 和报告判断可用性。

C++ 下游可用 `find_package(vln_memory CONFIG REQUIRED)`，链接 `vln_memory::vln_memory_preparation_reader`，调用 `loadPreparedPair(root, pair_id)` 读取已保存的 v5/v6 配对。完整字段及调用方式见 [BagBuilder 接口](docx/interfaces/bag_builder_api.md) 和 [Preparation 读取接口](docx/modules/bag/preparation_reader.md)。

## 项目进度与下一步

当前处于 **S2 离线观测准备**。原设计第 1、2 部分仍在实施中。

| 阶段 | 当前状态 |
| --- | --- |
| S0 数据契约与配置 | 传感器时间、来源和准备数据契约以及录包配置已落地。正式观测、地图及版本契约待完善。 |
| S1 读取与配对 | 已支持 SQLite 录包读取、原始图像及指定压缩格式解码、精确配对与容差内最近配对、原始消息与媒体导出。 |
| S2 完整离线观测 | 已支持有效相机模型、历史 TF 位姿、单点 Depth→map 与 Depth→RGB 候选投影、米制 NPY 与可选源 16UC1 无损 PNG，以及按 ID 读取单对数据。多对筛选、关键帧、批次及正式观测待完成。 |
| S3–S6 记忆构建与交付 | 待实现区域投影、实体与关系、SQLite 工作库、构建工具与模型批次、纠错与标注、版本发布及 MemoryViewer。 |

固定真实短包已通过 PNG/NPY 回读、TF 端点复核和三个投影点的数值对照。该样本的 10 对数据均保留注册待核实状态。实际深度尺度、注册精度和曝光同步仍需独立实测。

**下一步**是在离线构建入口复用 `loadPreparedPair`，按用途筛选多对数据，保留来源和 pending 原因。基于有效底盘位姿与显式采样规则选择关键帧并组成批次，再随构建流程落实正式观测及其地图、版本和证据引用。验收顺序见 [系统规划第 10 节](docx/project/system_plan.md#102-后续任务与依赖)。

## 目录结构

```text
include/vln_memory/bag/  # 公共头文件
src/bag/                 # common、rgb、depth、camera_info、tf、builder 实现
src/apps/                # bag_builder 命令行入口
config/                  # 录包配置及采集示例
test/bag/                # 核心、ROS 适配和录包回归
test/install_consumer/   # 安装后接口验证
docx/                    # 系统规划、接口文档和验收记录
```

更多资料见 [文档导航](docx/README.md)、[源码架构](docx/project/architecture.md) 和 [设计范围与验收](docx/project/sections_1_2_scope.md)。

## 贡献指南

1. 阅读 [仓库开发约定](AGENTS.md) 和 [系统规划](docx/project/system_plan.md)。在 `codex/` 分支上按可独立审阅的改动提交。
2. 修改接口或行为时同步更新对应模块文档和 [文档导航](docx/README.md)。保留原始时间戳、单位和数据来源。
3. 运行受影响的测试并提交 Pull Request，说明改动范围、接口影响、验证方式与已知限制。录包、生成数据和凭据请保留在 Git 之外。

## 许可证

项目采用 [Apache License 2.0](LICENSE)，与 [ROS 包清单](package.xml) 中的 `Apache-2.0` 声明一致。
