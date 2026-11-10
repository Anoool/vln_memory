# vln_memory

`vln_memory` 是用于视觉语言导航的 ROS 2 Humble / C++17 空间语义记忆包。BagBuilder 从 RGB-D 录包发布可回读的正式观测；MemoryEngine 读取观测、计算地图坐标并把实体和关系写入 SQLite。MemoryViewer 处于规划阶段。

| 模块 | 入口与文档 |
| --- | --- |
| BagBuilder | `bag_builder`，[录包配置、配对与产物](docx/modules/bag/bag_builder.md) |
| MemoryEngine | `memory_engine`，[JSONL 工具、模型批次与恢复](docx/modules/engine/memory_engine.md) |
| MemoryViewer | [地图检查与人工标注（规划）](docx/modules/viewer/memory_viewer.md) |

基础模块、系统架构及验收记录见[文档导航](docx/README.md)。

## 构建与测试

在已配置 ROS 2 Humble 软件源的 Ubuntu 22.04 / WSL 中，安装依赖后从仓库根目录构建。

~~~bash
sudo apt update
sudo apt install git build-essential cmake python3-colcon-common-extensions python3-numpy \
  libopencv-dev nlohmann-json3-dev libyaml-cpp-dev libssl-dev libsqlite3-dev libcurl4-openssl-dev \
  ros-humble-ament-cmake ros-humble-sensor-msgs ros-humble-geometry-msgs \
  ros-humble-tf2-msgs ros-humble-tf2 ros-humble-rmw ros-humble-rclcpp ros-humble-rcutils \
  ros-humble-rosbag2-cpp ros-humble-rosbag2-storage ros-humble-rosbag2-storage-default-plugins \
  ros-humble-cv-bridge ros-humble-compressed-depth-image-transport
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select vln_memory --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vln_memory --return-code-on-test-failure
colcon test-result --verbose
~~~

不读取 ROS bag 时可单独构建 C++ 核心和非 ROS 工具。需要上述 OpenCV、JSON、YAML、OpenSSL、SQLite 和 libcurl 开发库。

~~~bash
cmake -S . -B build/standalone -DVLN_MEMORY_ENABLE_ROS=OFF -DBUILD_TESTING=ON
cmake --build build/standalone --parallel 2
ctest --test-dir build/standalone --output-on-failure
cmake --install build/standalone --prefix "$PWD/build/standalone-install"
~~~

C++ 调用方用 `find_package(vln_memory CONFIG REQUIRED)` 和 `vln_memory::` 命名空间下的导出目标。公开头从 `<vln_memory/...>` 引入。各模块文档列出对应头文件、目标及示例。

## 快速开始

在 [config/vln_memory.yaml](config/vln_memory.yaml) 填录包、尚不存在的输出目录和地图信息。先生成观测，再按需启动模型构建。

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config config/vln_memory.yaml
ros2 run vln_memory memory_engine --config config/vln_memory.yaml --build
~~~

第二条命令需要配置兼容 Responses 的视觉模型。服务密钥填同目录的本地 `config/api_key.yaml`。BagBuilder 输出 `observations.jsonl` 和 `evidence/`，MemoryEngine 在同一目录写 `entities.sqlite3`。单条工具请求和旧入口见 [MemoryEngine 文档](docx/modules/engine/memory_engine.md#快速开始与典型示例)。

开发约定见 [AGENTS.md](AGENTS.md)。项目采用 [Apache License 2.0](LICENSE)。
