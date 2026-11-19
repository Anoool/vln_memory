# vln_memory_RAG

这是用于视觉语言导航的 ROS 2 Humble / C++17 空间语义记忆项目，ROS 包名仍为 `vln_memory`。离线流程是 **RGB-D 录包 → BagBuilder 导出观测 → MemoryEngine 识别和定位实体 → 发布记忆包**。RAG 从发布包建立本地向量索引，供 `vln_agent` 查询实体、地图坐标、关系和观测证据。SQLite 用于离线建库和纠错，RAG 查询读取发布文件和向量索引。

| 模块 | 入口与文档 |
| --- | --- |
| BagBuilder | `bag_builder`，[录包配置、配对与产物](docx/modules/bag/bag_builder.md) |
| MemoryEngine | `memory_engine`，[JSONL 工具、模型批次与恢复](docx/modules/engine/memory_engine.md) |
| MemoryViewer | [启动地图、图库和人工标注](docx/modules/viewer/memory_viewer.md) |
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

`bag_builder` 生成 `observations.jsonl` 和 `evidence/`。`memory_engine --build` 调用配置中的视觉 API（217 条观测验收使用 DeepSeek），将实体和关系写入同目录的 `entities.sqlite3`；服务密钥放在本地 `config/api_key.yaml`。RAG 的文本编码使用本地 ONNX 模型，无需调用视觉 API。更多操作见 [MemoryEngine 文档](docx/modules/engine/memory_engine.md#快速开始与典型示例)。

建库完成后，用新目录发布记忆：

~~~bash
ros2 run vln_memory memory_engine --config config/vln_memory.yaml --publish OUTPUT_DIR
~~~

发布目录包含 `manifest.json`、`entities.jsonl`、`relations.jsonl`、`observations.jsonl` 和 `evidence/`，可独立回读实体、关系及 RGB-D 证据。完整结构见[记忆版本发布](docx/modules/engine/memory_publication.md)。

## RAG 查询

[config/memory_rag.yaml](config/memory_rag.yaml) 统一指定发布目录 `release`、索引目录 `index_dir`、模型 `model_path` 和分词器 `tokenizer_path`。当前配置指向正式包 `site_s4-bag217-deepseek-v1` 和同级的 `index/`，默认使用 E5-small INT8 ONNX 编码为 384 维向量。发布包、模型权重和索引属于本地产物，新检出仓库后需准备这些文件并配置路径。首次建索引或更换发布包、模型、分词器、编码设置时，运行 `build/rag/venv/bin/python tools/memory_rag.py build-index`。安装与建索引步骤见 [RAG 使用说明](docx/modules/engine/memory_rag.md#安装和模型)。

已有索引时，从项目根目录查询：

~~~bash
# 主动查证：查询黑色背包及挂在椅背上的关系
build/rag/venv/bin/python tools/memory_rag.py query \
  --text "black backpack hung on chair backrest" --relation hanging_on

# 自动召回：查看发给模型前会附加的候选
build/rag/venv/bin/python tools/memory_rag.py query --context --text "实验室里的绿帘"
~~~

Agent 通过同一个 RAG 进程使用两条路径：模型主动调用 `search_memory` 查实体或关系；Agent 在发出用户消息前调用 `retrieve_context`，先用目标名称和词表筛选，再对原句编码排序。自动入口没有候选、超时或出错时，Agent 继续处理原消息。服务启动和配置见 [Agent 接入](docx/modules/engine/memory_rag.md#接入-vln_agent)。

在 YAML 中设置 `environment: desktop` 或 `environment: jetson` 即可切换批大小和 CPU 线程数。两种环境复用同一份 `vectors.npy`、`entity_ids.json` 和 `index.json`，切换环境无需重建索引。目标机还需准备相同的发布包、模型和分词器。

## 当前验收与待完善

真实录包的 217 条观测已完成 DeepSeek 建库，发布包包含 **53 个已定位实体、27 条关系和 11 条待定位记录**。在 x86_64 WSL 上，两个环境配置读取同一份索引；本地模拟 Responses 会话验证了 Agent 的自动召回和主动查证。优化后，同一录包、同组选帧设置的导出耗时约从 158 秒降到 59 秒，217 条观测记录及引用图像摘要一致。四帧本地回环测试中，同尺寸 JPEG 请求体减少约 90%；真实 API 效果仍需对照。详见 [217 条观测与共享索引验收](docx/reviews/verification.md#217-条观测与共享索引验收)。

自动召回已支持「座椅」等常用别名，并在常见空间句式中按目标筛选。例如问「白色设备柜旁边的紫色直升机」时，参照物「设备柜」不会单独成为自动候选。

- **Jetson Nano 与会话：** 上板验证模型可加载，测首次查询、后续查询延迟及内存，再用真实会话模型验证两条检索路径和超时回退。
- **视觉请求压缩：** 用同一批帧对照 PNG 和 JPEG 的 DeepSeek 结果，比较标签、定位选区、关系和整轮耗时。
- **坐标可靠性：** 当前观测的物理同步与配准仍标为 `unknown`。把历史坐标用于实车导航前，需要用受控实拍验证坐标误差。

开发约定见 [AGENTS.md](AGENTS.md)。项目采用 [Apache License 2.0](LICENSE)。
