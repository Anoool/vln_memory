# vln_memory_RAG

`vln_memory` 是用于视觉语言导航的 ROS 2 Humble / C++17 空间语义记忆包。BagBuilder 从 RGB-D 录包发布可回读的正式观测；MemoryEngine 读取观测、计算地图坐标并把实体和关系写入 SQLite。记忆发布后，由 RAG 检索实体与关系，并向 Agent 提供坐标和观测证据。ROS 包名和现有命令仍为 `vln_memory`。MemoryViewer 处于规划阶段。

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

[config/memory_rag.yaml](config/memory_rag.yaml) 当前指向 `site_s4-bag217-deepseek-v1` 正式包及其共享索引，包含 217 条观测、53 个已定位实体和 27 条关系。编码使用本地 E5-small INT8 ONNX。

从仓库根目录，直接用现有索引查询：

~~~bash
build/rag/venv/bin/python tools/memory_rag.py query \
  --text "black backpack hung on chair backrest" --relation hanging_on
~~~

当前环境为 `desktop`。部署到 Jetson Nano 时，将同一份 YAML 的 `environment` 改为 `jetson`，即可用该环境的批大小和 CPU 线程数读取现有索引。发布版本或编码配置变更时，再按[建索引与部署说明](docx/modules/engine/memory_rag.md#建索引和查询)构建索引。

`vln_agent` 通过同一个检索进程使用两条只读路径。模型可主动调用 `search_memory` 查证实体或关系，Agent 也可在发送用户指令前通过 `retrieve_context` 附上相似候选。参见 [Agent 接入](docx/modules/engine/memory_rag.md#接入-vln_agent)和 [217 条观测验收](docx/reviews/verification.md#217-条观测与共享索引验收)。

开发约定见 [AGENTS.md](AGENTS.md)。项目采用 [Apache License 2.0](LICENSE)。
