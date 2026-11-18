# vln_memory_RAG

`vln_memory` 是用于视觉语言导航的 ROS 2 Humble / C++17 空间语义记忆包。BagBuilder 从 RGB-D 录包发布可回读的正式观测；MemoryEngine 读取观测、计算地图坐标并把实体和关系写入 SQLite。本仓库在该离线构建基线上规划 RAG 查询。ROS 包名和现有命令仍为 `vln_memory`。MemoryViewer 处于规划阶段。

| 模块 | 入口与文档 |
| --- | --- |
| BagBuilder | `bag_builder`，[录包配置、配对与产物](docx/modules/bag/bag_builder.md) |
| MemoryEngine | `memory_engine`，[JSONL 工具、模型批次与恢复](docx/modules/engine/memory_engine.md) |
| MemoryViewer | [地图检查与人工标注（规划）](docx/modules/viewer/memory_viewer.md) |
| RAG 查询 | [本地建索引与查询](docx/modules/engine/memory_rag.md) · [实施规划](docx/project/rag_plan.md) |

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

工作库可用 `ros2 run vln_memory memory_engine --config config/vln_memory.yaml --publish OUTPUT_DIR` 发布为独立的记忆版本，参见[记忆版本发布](docx/modules/engine/memory_publication.md)。

## RAG 查询

RAG 从 `memory_engine --publish` 生成的记忆版本读取实体、标注、关系和观测证据。第一版在本地运行 `intfloat/multilingual-e5-small`。向量按实体编码，与实体 ID 一起存到发布目录旁的可重建索引。查询使用同一模型编码中文或英文短语，再按实体 ID 回查坐标、关系和原图引用。SQLite 继续管理离线构建、标注和纠错。

现有 Python CLI 提供本地建索引、实体检索和关系与观测回查。发布目录、索引目录、模型目录和候选数量集中在 [config/memory_rag.yaml](config/memory_rag.yaml)，参见[安装与调用方式](docx/modules/engine/memory_rag.md)。`vln_agent` 的只读 `search_memory` 工具通过 ROS 2 调用本地常驻检索服务，由现有会话模型根据检索结果回答。当前范围与实施顺序见 [RAG 实施方案](docx/project/rag_plan.md) 和 [系统规划](docx/project/system_plan.md)。

开发约定见 [AGENTS.md](AGENTS.md)。项目采用 [Apache License 2.0](LICENSE)。
