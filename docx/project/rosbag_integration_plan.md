# BagBuilder：rosbag 读取与现有模块整合方案（编码前设计记录）

实施更新：2026-10-30 第一版已落地，实际接口/支持边界见 [BagBuilder 手册](../interfaces/bag_builder_api.md) 和 [实施记录](bag_builder_v1_implementation.md)。以下保留原规划，不作为当前全部已实现能力声明。
日期：2026-10-30。本文基于当前工作区的实际头文件、实现、ROS 适配、测试、安装目标及系统规划编写。本轮只形成方案，不新增功能代码；下文的新文件、类型、命令和产物均为计划。

相关约束：[Repository Guidelines](../../AGENTS.md)、[系统规划](system_plan.md)、[第 1、2 部分范围](sections_1_2_scope.md)、[既有 RGB-D 配对方案](rgbd_sync_plan.md)、[已实现接口手册](../interfaces/observation_preparation_api.md)。

## 1. 阅读结果与本轮目标

实际源码仍没有 builder、bag reader、配对器、媒体保存或 JSONL 写入入口。CMake/package.xml 当前只依赖传感器消息及对应处理库，没有 rosbag2 读取依赖。不能直接把 bag 路径传给已有四个模块。

| 已实现模块 | 当前真实入口 | 整合时负责什么 | 还需要上层补什么 |
| --- | --- | --- | --- |
| RGB | decodeRosRgb → decodeRgb → RgbFrame | rgb8/bgr8/mono8 转紧密 RGB8，保留 stamp/frame/源布局 | 话题身份、反序列化、来源、存图 |
| Depth | decodeRosDepth、checkDepthRgbAlignment | 16UC1/32FC1；显式米/毫米、optical_z；质量统计和注册声明核对 | 设备语义配置、实际配对 RGB 网格、NPY 保存 |
| CameraInfo | decodeRosCameraInfo、checkCameraInfoImage | 完整模型校验；话题/frame/ROI/binning/尺寸/时间检查 | 标定候选选择、版本/有效段、有效投影模型 |
| TF | ingestRosTf、TfIndex::inspectChain/checkTime | 动静态历史索引、冲突阻断、覆盖及有来源的端点 | 来源 ID、clock/segment；后续插值/组合的数值位姿 |
| common | checkSensorTimes、isResolvedTopicName | 时间差和统一话题规则 | 流绑定、时钟依据、自动选帧 |

当前 DepthOptions/DepthFrame、CameraInfoMatchOptions/CameraImageReference 和 TF 来源/查询字段已经扁平化。实现时按现有公开字段填写，不恢复旧的 DepthAlignmentInfo、CameraInfoBinding 或 TfSource 包装。

本轮优先交付“读取与转换闭环”：**bag 路径 + 明确配置 → 五类消息提取/转换 → RGB-D 配对和标定/TF 检查 → 图像证据、来源索引及 JSONL 报告**。完整 Observation 所需的有效相机模型、数值 map 位姿和关键帧随后单独收口，仍属于系统规划 S2。二者分别验收。

## 2. 输入范围与技术选择

### 2.1 第一版支持范围

- ROS 2 Humble，C++17。优先支持带 metadata.yaml 的未压缩 sqlite3 bag 目录，包含其全部分卷文件。
- 可接收独立 .db3，但 manifest 必须标记 single_storage_file；不能声称读取了原本可能存在的其他分卷。
- 输入图像为 sensor_msgs/msg/Image；CameraInfo 为 sensor_msgs/msg/CameraInfo；/tf 与 /tf_static 为 tf2_msgs/msg/TFMessage。话题名由配置指定，支持 remap。
- 清点包内所有话题；处理配置选中的上述流。未选话题列入 inventory，不任意反序列化，也不把五类信息提取称为所有 ROS 消息类型转换。
- CompressedImage、ROS 1 .bag、压缩存储、未验证的 MCAP 输入返回明确的不支持原因。扩展时使用对应标准插件/适配，不能假装按 Image 或 sqlite3 读取。
- 当前本机 rosbag2_cpp/storage/default plugins 为 0.15.17，MCAP 插件未查到已安装；不能在文档中将其写成已验证能力。

### 2.2 使用官方 reader，不直接解析数据库

复用 rosbag2_cpp::readers::SequentialReader 的 open、get_metadata、get_all_topics_and_types、set_filter、has_next/read_next 和 get_current_file；使用 rclcpp::Serialization 对三种已核对类型反序列化。不创建在线订阅、ros2 bag play 或 TransformListener。

Humble 的 SerializedBagMessage 提供序列化数据、topic_name 和 time_stamp，没有通用源记录 ID。seek 按记录时间定位，不能替代按消息 ID 随机取回。SequentialReader 会按 metadata 的文件列表切换分卷，分卷切换后的记录时间不保证全局递增。这些约束直接决定下文的来源设计和两遍顺序读取。

依据：[Humble SequentialReader 源码](https://github.com/ros2/rosbag2/blob/humble/rosbag2_cpp/src/rosbag2_cpp/readers/sequential_reader.cpp)、[读取接口](https://github.com/ros2/rosbag2/blob/humble/rosbag2_cpp/include/rosbag2_cpp/readers/sequential_reader.hpp)、[SerializedBagMessage](https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage/include/rosbag2_storage/serialized_bag_message.hpp)。

## 3. 整体调用路径

```text
配置/输入检查
  → 话题与分卷清单、bag 身份、clock/segment 声明
  → 第一遍顺序读取：图像元数据 + CameraInfo 索引 + 全段 TF 索引
  → RGB-D 时间配对 + 两张图各自的标定选择 + TF 链/覆盖检查
  → 第二遍顺序读取：图像解码、原始消息证据、PNG/NPY 立即落盘
  → 组合提取记录、未配对/坏帧/缺标定/缺 TF 报告
  → 校验数量与文件引用，发布 preparation 数据集
  → 后续 S2：有效模型 + 数值位姿 + 关键帧 → 正式 observations.jsonl
```

builder 负责调度；前四个模块仍不反向依赖 builder。第一版串行执行，输入不变时结果可复现；不先引入线程池、消息总线、工厂或插件框架。

### 为什么采用两遍读取

第一遍结束后才能知道完整的 TF 覆盖、后到的静态 TF、标定变化及配对结果。第二遍按稳定来源引用处理图像，处理完一帧就写文件并释放像素；组装阶段只保存路径和小型元数据，因此不依赖图像到达顺序，也不积压整包图像。

第一版导出配置图像流的全部可解码帧，包含未配对帧；未配对不等于坏图像。后续若只导出关键帧，应新增明确策略与统计，不静默丢弃。

代价：图像会被顺序读取两次。第一遍虽只保留 header/布局，标准 Image 反序列化仍会临时分配完整 data；这不是“只读取 header”的零拷贝优化。第二遍才调用 RGB/Depth 核心做数值转换。先接受可检查的两遍流程，后续依据真实 bag 性能再优化，不能假定 reader 有随机按 ID 读取能力。

## 4. 分步骤实现与接口责任

### 4.1 配置、来源与运行状态

BagBuildOptions 持有明确配置；BagMessageRef 为拥有型、扁平来源记录。保留 SensorStamp 等有实际含义的公共类型，不把字段机械拆平或套多层配置包装。

| 契约 | 必需内容 |
| --- | --- |
| 输入/输出 | bag_uri、output_directory、storage_id；输入只读，输出不覆盖已有完整数据集 |
| 流绑定 | RGB/Depth/各自 CameraInfo 的真实话题，动态/静态 TF 绑定；相机光学 frame 和绑定依据 |
| 深度语义 | input_unit、optical_z、有效范围、注册状态及依据；仅在有证明时声明 aligned_to_rgb |
| 时间 | clock_id、clock_basis、segment_id/边界；RGB-D 容差、CameraInfo 容差、TF 最大插值间隔分别配置 |
| 地图 | 目标 map/body frame；map_id、geometry_revision 在正式 map 观测收口时必须明确 |
| 资源 | 序列化消息字节数、图像核心上限、索引条数、TF 样本/边数、输出预算 |

source_message_id 由不可变 bag 身份、分卷序号、实际 topic 及该分卷内该 topic 的读取序号生成；同刻重复消息仍有不同 ID。另存配置流内的稳定扫描顺序，作为同刻候选平局依据。序号不是原设备发布顺序，也不是数据库通用行号。

两遍使用相同的已验证输入和扫描约定；记录文件清单/内容指纹，并核对消息来源、时间、布局及必要的内容摘要。输入变化或扫描对应不一致终止构建，不按接近时间猜回原消息。SQLite 同时间记录按 timestamp/id 排序是该插件的行为；不得泛化为所有插件保证，需用实际分卷/同刻用例验证。[Humble SQLite 实现](https://github.com/ros2/rosbag2/blob/humble/rosbag2_storage_default_plugins/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp)

时钟关系和分段由采集配置/可信依据声明。时间回退只作为诊断信号：可能是乱序、分卷切换或时钟重启。不能见回退就任意切段，更不能全局排序后跨 epoch 配对。无法明确时钟段时可保留原始提取信息，但停止对应的同步/TF 关联并报告原因。静态 TF 按声明的数据段归属，不以零时间判断其归属。

### 4.2 预检查与第一遍扫描

1. 检查输入存在、metadata 所列分卷完整、输出路径可用且不与输入冲突；读取 inventory。
2. 核对所有绑定话题的名称、类型和序列化格式；缺少必需图像/标定话题或类型错误提前返回。TF 两话题是否必需按所需链判断，不能要求每个 bag 同时包含两者才能处理。
3. 保留 metadata 中的声明计数与扫描实际计数，两者分开，不能把未读取话题计成已提取。
4. 逐条分配来源 ID；先检查 serialized_data 长度和运行预算，再按已验证类型反序列化。reader 的内部缓冲区分配早于本层检查，现有图像限制也不能当作反序列化阶段的绝对内存保障。单条反序列化失败保留原始 CDR/来源并报告；存储损坏致无法继续扫描或资源失败则终止。
5. Image：只长期保留 stamp、frame、尺寸、encoding、step、大小端、来源和记录时间，不保留 data。非法 stamp 不进入有效时间索引；编码/布局记录原值，最终合法性由第二遍的图像核心判断，避免复制一套解码规则。
6. CameraInfo：保存原始字段/来源及 decodeRosCameraInfo 的结果，建立按 topic/segment/header 时间的候选索引。坏标定仍留下来源和诊断。
7. TFMessage：调用 ingestRosTf，保留每条 TransformStamped 的 header 时间及原消息内序号，处理 inserted/duplicates/issues。原始消息条目另行导出，不能因为索引去重而丢掉重复广播证据。
8. 每个明确 clock/segment 建立独立 TfIndex。全段建立结束后再检查所需链，避免因为 TF 后到而误判缺失。

注意本机 Humble SequentialReader 内部 seek_time_ 初始为 0。全范围提取须在 open 后显式 seek 到记录时间类型的最小值，两遍采用相同规则，避免负 bag 记录时间被默认起点漏掉；这与合法的负 header 秒数是两回事。has_next 可能触发分卷切换，取当前分卷来源应在 has_next 成功后、read_next 前进行。

### 4.3 RGB-D 配对

落实已有 rgbd_sync_plan，不另写第二套时间规则：

- 同一明确时钟段内，按 header 时间及稳定来源顺序排序。
- RGB 为基准；一对一、Depth 选择顺序单调；在允许后缀中选容差内最近 Depth。
- 等距选较早时间，同刻按稳定来源顺序；Depth 不重复复用。
- 使用 checkSensorTimes 复核，delta_ns = depth - rgb；0 容差表示严格相等。
- 保存两份原始时间、精确差值、策略、未配对 RGB 和未使用 Depth。
- 贪心策略不保证全局最大配对数，保持既有方案的已公开取舍。
- 图像随后解码失败不重新配别的帧，防止不同执行路径产生不同配对。

这是时间关联，不执行像素注册，不校正设备时钟。相同 stamp 也可能来自上游重写，不能声称物理曝光同步。

### 4.4 CameraInfo 选择与绑定

为 RGB、Depth 分别选标定，不能默认两者共用 RGB CameraInfo。只有已注册深度及明确配置证明时，才能引用同一有效模型。

- synchronized：同一流/时钟段按 header 时间二分定位最近候选，使用独立的 CameraInfo 容差；等距优先较早。允许同一标定消息为多个图像提供依据，不采用 RGB-D 的一对一限制。
- 同刻存在不同模型、候选跨越未明确的版本边界或最近候选本身无效时，返回歧义/坏标定，不静默换成另一条“能通过”的旧标定。
- static_calibration：配置明确证明该段模型、ROI/binning 不变；扫描核对几何字段是否发生变化。发生变化则要求明确有效区间或改用逐帧策略，不取第一条/最后一条覆盖整段。
- 选定后构造当前扁平 CameraImageReference 和 CameraInfoMatchOptions，调用 checkCameraInfoImage；结果保留所选 calibration_id、原始 stamp、匹配状态及依据。
- 保持已有严格规则：完整 R/P、受支持畸变模型、ROI/binning 整除、rectified + roi.do_rectify 的显式拒绝。reader 不补单位 R/P，不用相同尺寸证明标定有效。

### 4.5 TF 查询与历史时间

段级 inspectChain 生成链/共同覆盖报告；逐图 checkTime 使用原始图像采样时间，不重复扫描整段覆盖：

| 查询 | 时间 |
| --- | --- |
| map ← RGB optical | RGB header.stamp |
| map ← Depth optical | Depth header.stamp |
| map ← body（RGB 索引） | RGB header.stamp |

checkTime 返回的 before/after 和 inverted 是求解依据，第一阶段写入 tf_status/所用样本引用，不把它填成 camera_pose_map。静态 TF 不受动态时间区间限制，但受 segment 及拓扑冲突约束；动态 TF 不外推，不以 latest、最近样本或 bag 记录时间补齐。

bag time_stamp 只记录为 bag_record_time_ns；当前 API 并不提供独立的发布/曝光时间，不能将它自行命名为精确接收时间。处理时间也单列，不参与传感器对齐。

### 4.6 第二遍解码、注册核对与媒体保存

重开同一 bag，按稳定来源找回图像；不按 header 时间 seek 后假定唯一。逐帧调用 decodeRosRgb/decodeRosDepth，保存结果后释放源消息和像素。

- RGB 保存 PNG；RgbFrame 是 RGB 通道，若使用 OpenCV 写入须正确转换其所需通道顺序，做红/蓝像素回读测试。
- Depth 保存标准 NPY：二维 height×width、C 顺序、float32 米制 optical-z，显式字节序。NaN 留在二进制数组，不能转成有损预览或冒充 uint16 毫米图。NPY 的 shape/dtype 设计依据 [NumPy 格式说明](https://numpy.org/doc/stable/reference/generated/numpy.lib.format.html)。
- 取真正配对 RGB 的 ImageGeometry，调用 checkDepthRgbAlignment；注册配置必须有独立依据，不能从这张 RGB 的尺寸反推“已注册”。未配对 Depth 仍可解码/保存，不能声称已核对实际 RGB 网格。
- 第一版将配置图像消息的原始 CDR 落盘，同时保留 bag 来源引用；成功帧另存标准文件，坏帧保留读到的原始字节及诊断。保存格式、消息类型/序列化格式均由来源记录说明。
- CameraInfo 和 TF 的原始字段及所有来源条目写 JSONL；遇非有限原值，用原始 CDR 保证可回查，JSON 字段用 null 加异常标记，不输出 NaN/Inf 字面量。
- 深度全部无效仍可能 decode 成功；组装层根据 valid_ratio/配置质量规则进入 pending，不只检查 ok()。
- 文件写失败、预算耗尽属于运行失败；不伪装成普通坏帧继续发布。

## 5. 文件组织与每个文件的实际职责（计划）

保持第五个模块 builder，不新增第六个顶层传感器模块。以下文件随相应步骤落地，不一次生成空壳。

```text
include/vln_memory/bag/builder/
├── bag_types.hpp
├── bag_config.hpp
├── bag_reader_ros.hpp
├── rgbd_sync.hpp
├── camera_info_index.hpp
├── media_writer.hpp
├── jsonl_writer.hpp
└── bag_builder.hpp
src/bag/builder/
├── bag_config.cpp
├── bag_reader_ros.cpp
├── rgbd_sync.cpp
├── camera_info_index.cpp
├── media_writer.cpp
├── jsonl_writer.cpp
└── bag_builder.cpp
src/apps/bag_builder_main.cpp
config/vln_memory.yaml
test/bag/builder/test_<小模块>.cpp
test/install_consumer/bag_builder_consumer.cpp
docx/modules/bag/<同名文件>.md
docx/modules/bag/tests/test_<小模块>.md
```

| 文件（头/源同名） | 输入 → 输出 | 只负责的工作 |
| --- | --- | --- |
| bag_types.hpp | 来源/图像元数据/报告字段 | 少量共享拥有型契约，不成为所有业务类型的集中大头文件；各模块专有类型留在其头文件 |
| bag_config | YAML 路径 → BagBuildOptions 或配置错误 | 加载、字段/单位/绑定/资源约束；解释实际支持字段，不提前创建模型/Viewer 配置桩 |
| bag_reader_ros | options → BagScanResult；第二遍向受控同步消费者提供消息 | 打开/清点/分卷/来源/反序列化，建立轻量索引；不自行实现 RGB/Depth 转换 |
| rgbd_sync | 两个流的时间引用 → 配对/未配对/统计 | 纯 C++ 时间选择，链接现有 sensor_time |
| camera_info_index | 标定记录、图像引用、匹配配置 → 标定 ID/诊断 | 候选检索和版本边界，复用已有 CameraInfo 校验 |
| media_writer | 成功 RgbFrame/DepthFrame 或原始 CDR、目标路径 → MediaWriteResult | 文件编码、原子单文件写入、返回相对路径/大小；不保留像素 |
| jsonl_writer | 明确类型的记录 → JSONL/summary/preparation manifest | schema/精确整数/有限值/引用序列化；不选择图像或求 TF |
| bag_builder | BagBuildOptions → BagBuildResult | 串联步骤、累计状态/计数、质量门控、发布结果；不堆入底层转换循环 |
| bag_builder_main.cpp | CLI 参数 → 调用 buildBag、退出码 | 薄入口，不承担算法 |

关键公开入口暂定 buildBag(const BagBuildOptions&)；返回 BagBuildResult，只包含状态、计数和产物路径，不返回整包像素。辅助 scanBag、pairRgbDepth、selectCameraInfo、writeRgb/writeDepth 的具体签名在各小模块编码前定稿，分别说明输入格式、所有权、失败与输出生命周期。

计划 CMake 导出独立 rgbd_sync 核心目标和 ROS bag builder 目标。rosbag2_cpp、rosbag2_storage、rclcpp 只进入 ROS 构建分支；yaml-cpp/JSON/图像编码依赖按目标引入，不传播到现有纯传感器核心。继续使用项目已有 CTest 和安装消费者，不另建测试框架。

vln_agent 已使用 nlohmann_json、yaml-cpp、OpenCV；可沿用库选择并在 vln_memory 中显式声明依赖，不链接 vln_agent 来间接获得这些库。依赖实际可用性及 NPY 写入实现的选择在编码前验证，不自行实现通用 JSON/YAML 解析器。

## 6. 对其他模块交付的文件契约（计划）

```text
output/<dataset_id>/
├── bag_manifest.json
├── frames.jsonl
├── rgbd_pairs.jsonl
├── camera_info.jsonl
├── tf.jsonl
├── evidence/
│   ├── rgb/<message_id>.png
│   ├── depth/<message_id>.npy
│   └── raw/<message_id>.cdr
└── reports/
    ├── issues.jsonl
    ├── pending.jsonl
    └── summary.json
```

- bag_manifest：schema_version、artifact_kind=bag_preparation、来源指纹/范围/分卷/话题清单、配置与处理版本、clock/segment、文件集合和完成状态。它不是原文 1.6 的完整 memory manifest。
- frames：每个选定图像消息一条身份/采样时间/记录时间/布局/解码状态/证据路径/质量记录，坏帧和未配对帧同样可查。
- rgbd_pairs：两侧 message_id、各自采样时间、有符号时差、配对策略、各自 calibration_id、注册/TF 检查状态和诊断引用。不含虚构的 map 位姿。
- camera_info：原始 K/D/R/P/ROI/binning、topic/frame/stamp、calibration_id、来源、校验结果；几何版本与重复消息来源分开表示。
- tf：每个源消息的每个 TransformStamped 一条，保留动静态类别、父子、原时间、米制平移、xyzw、源消息和 transform_index，以及索引接受/重复/拒绝状态。
- issues：stage、稳定错误码、message_id/pair_id、原因及上游模块错误码；不要求调用者解析自由文本。
- pending：不能进入后续几何处理的候选引用及原因，可有多个原因；unused Depth 单独统计，不自动当坏数据。
- summary：声明/实际读取/解码/配对/未配对/坏标定/TF 缺失/重复/质量门控等计数，校验流内守恒关系。

时间规则：原始 stamp 用 sec/nanosec；int64 记录时间、时差及可能超出 JavaScript 安全整数范围的序号用十进制字符串，schema 明确解析规则，不经过 double 秒。JSONL 不嵌入大像素数组，文件路径均相对数据集根目录，路径片段由安全 ID 生成而非直接拼 topic。

所有记录必须有 schema_version 和稳定引用。标准图像可以直接被后续模块读取；几何模块还须检查有效模型、时间、注册和位姿状态。早期产物使用上述 preparation 命名，不能提前生成含空/单位位姿的正式 observations.jsonl。

工作目录使用同一输出文件系统上的临时目录；单文件先临时写再改名，关闭并检查写入后做数量/引用验证，最后发布 bag_manifest。状态区分 completed、completed_with_issues、failed；正常扫描结束不等于所有候选可用。部分产物/中断目录不作为可消费的完整数据集，已有完整输出不自动覆盖。本阶段不承诺断点恢复；需要时另加有版本的进度契约。

## 7. 完整观测阶段还要补什么

读取与转换闭环完成后，不改写其原始证据，继续按 S2 增加：

1. TF 数值求解：复用 checkTime 选出的端点，做平移插值、四元数插值、求逆和链组合；明确 T_target_source 方向、四元数规范化与原始证据。与 tf2 进行已知答案对照，不另建冲突规则。
2. 有效相机模型：区分 raw K/D 与 rectified P，处理受支持 ROI/binning；未支持情况显式失败。未注册深度需要自己的模型和外参/重投影，不能仅改 frame 名。
3. Observation：RGB 参考时刻的 camera/body map 位姿与 Depth 自己时刻的位姿分开；明确 map_id/geometry_revision、两图来源、标定和质量。RGB-D 非同刻、运动较快时，记录运动影响并门控；仅同 frame 不代表相同采样位姿。
4. 关键帧：基于有效底盘位姿的距离/转角及必要采样规则，处理首帧、静止、缺姿态及定位跳变；保留完整提取索引，关键帧仅作为引用子集。
5. 验证通过后，正式导出 observations.jsonl、evidence 与关键帧/质量报告，接入后续 MemoryEngine。

这部分预计在现有 tf/camera_info 模块中新增相应求解文件，并在 builder 中新增 observation/关键帧组装文件；实施时逐个规划，不在本轮建立占位目录。MemoryEngine、实体/关系及 Viewer 按总规划继续，不能由本阶段报告替代。

## 8. 与 vln_agent 的结合位置

BagBuilder 是确定性的离线数据准备功能，当前不调用模型、不启动 SessionManager，不经过在线 ToolManager。标准文件及稳定引用构成与后续 MemoryEngine 的边界。

后续 MemoryEngine 的 get_observation/project_depth 读取完成的 Observation 和证据，校验相机模型、两图时间/位姿与 map 版本；再通过适配层复用 vln_agent 的 SessionManager。现有 EnvironmentMemory 保存历史观测，不做跨观测实体关联，不能把 preparation 记录直接写入它后就宣称永久空间记忆已完成。

这个方向使 bag 层可独立运行/测试，也避免 vln_agent 与 vln_memory 核心形成循环依赖。

## 9. 分次实施、复杂度和验收

| 顺序 | 本次小模块交付 | 通过条件 |
| --- | --- | --- |
| P0 | 最小来源/配置/结果契约及配置校验 | 人工能读懂所有字段、时间单位、所有权和失败；不为后续所有层建类型 |
| P1 | rgbd_sync | 固定配对答案、同刻/平局/一对一/段边界/空流/溢出/数量守恒 |
| P2 | reader 第一遍 + 来源索引 | 用 rosbag2 Writer 生成真实序列化测试 bag，读回三种消息类型及分卷/重复 stamp/零与负记录时间；同时区别于真实采集验收 |
| P3 | CameraInfo 索引选择 + TF 联动检查 | 双相机绑定、静态模型变化、坏候选、迟到 TF、内部缺口/冲突可追溯；尚不生成数值位姿 |
| P4 | 第二遍解码 + 媒体/JSONL + buildBag/CLI | PNG 色彩回读、NPY 独立读取米/NaN/shape、原始证据、坏帧、磁盘失败、引用完整性、输入变化不误关联 |
| P5 | 安装消费与短真实采集 bag 验收 | 包外程序只通过导出目标调用；人工核对消息数/两图/时间差/标定/TF/未配对记录；同输入配置重跑结果一致 |
| S2 后续 | 数值几何、Observation、关键帧 | 已知位姿/投影答案及真实 bag 验证后，才标记完整观测准备完成 |

复杂度约定：

- 两遍读取是固定次数的顺序访问；图像转换时间为实际处理的总像素数 O(P)，不能藏入“常数时间读取”。
- RGB-D 索引排序/候选查找沿用 O(R log R + D log D + R log D)，不逐张 RGB 全扫 Depth；元数据空间 O(R+D)。
- CameraInfo 按流/段排序索引，候选查找二分；固定模型变化检查一次完成，不为每张图扫描全部标定。
- TF 保持已有 child/时间有序索引，逐图使用 checkTime；inspectChain 的历史区间扫描按链/段执行，避免每帧重扫全部 TF。调用成本仍包含路径查找和逐边二分，不能称 O(1)。
- 峰值内存包含全部轻量索引/TF 历史、单条序列化消息、临时 ROS Image、标准输出及编码器工作区；不承诺仅一张图大小，也不长期保留全包像素。超限明确失败，不静默截断。源指纹和 CDR 证据的 I/O 成本单列。

测试沿用 CTest；核心必须继续无 ROS 构建，ROS reader 测试使用实际 rosbag2 API，安装消费者校验依赖传递。运行现有全部受影响测试后，检查路径/错误/所有权及框架调用关系；不会用构造 bag 测试替代真实设备录包的物理对齐验证。

每个小模块编码前先审接口，再写实现与必要验证。生产函数和辅助函数均补函数级注释；短函数按职责分拆，变量显式体现角色/单位，无嵌套三元表达式，无过度嵌套结构体/生成器。完成后通读全链，更新同名分类文档、接口手册、CMake/导航，在 codex 分支暂存，合并由仓库所有者审阅。

## 10. 本轮自审与结论

- 以实际扁平接口设计接入；不依赖已移除包装或不存在的 reader/Observation。
- 复用官方读取/反序列化与现有四模块，不自行解析 .db3，不生成在线回放链。
- 采样/记录/处理时间、时间配对/空间注册、TF 端点/数值位姿分别有明确含义。
- 来源、分卷、重复 stamp、时钟段、标定变化、部分失败、图像全无效和输出发布都有处理路径。
- 标明两遍扫描的实际成本与输入范围，保留未配对图像，不把缺几何的准备记录称为正式 Observation。
- 本轮只新增这份方案和导航入口，不修改生产代码、测试、依赖或原系统规划。后续从 P0/P1 开始逐个小模块实施。
