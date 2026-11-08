# vln_memory 当前架构

当前交付范围为原设计第 1、2 部分，项目推进到 **S3 几何与工作库**。已实现录包准备、正式观测发布、地图几何投影以及物品与空间创建。已定位和待定位实体可写入 SQLite 工作库并按原 ID 回读。当前进度和验收条件见 [范围对照](sections_1_2_scope.md) 与 [系统规划](system_plan.md#102-后续任务与依赖)。

## 数据流与边界

```mermaid
flowchart LR
    Bag["ROS 2 RGB-D bag"] --> Builder["bag_builder"]
    Builder --> Preparation["preparation<br/>媒体、配对与来源"]
    Preparation --> Reader["PreparationReader"]
    Reader --> Select["build_input<br/>用途筛选、关键帧与批次"]
    Select --> Dataset["observation_dataset<br/>正式观测与批次"]
    Dataset --> DatasetReader["observation_dataset_reader<br/>按 ID 或批次回读"]
    Reader --> Dataset
    DatasetReader --> Geometry["observation_depth_region<br/>深度原图投影"]
    Reader --> Geometry
    Geometry --> Object["createObjectEntity<br/>实测表面代表点"]
    DatasetReader --> Region["createRegionEntity<br/>历史底盘锚点"]
    Reader --> Region
    Confirmation["上游空间内部确认"] --> Region
    Object --> Store["SQLite 工作库"]
    Region --> Store
    DatasetReader --> Store
```

`memory_build_input` 可单独输出选帧与分批清单。`memory_observation_dataset` 内部复用同一选帧逻辑，直接接收 preparation。图中的实体创建为 C++ 调用，由调用方选择观测和证据。

| 层 | 职责与依赖 |
| --- | --- |
| `bag/common`、`bag/rgb`、`bag/depth`、`bag/camera_info`、`bag/tf` | 定义传感器基础类型，完成图像解码、标定匹配、历史 TF 查询和数值投影。核心库使用 C++17，ROS 适配映射消息字段。 |
| `bag/builder` | `BagBuilder` 读取录包、配对和导出，`PreparationReader` 建立配对索引并核对来源，`preparation_compact` 生成可回读的精简副本。 |
| `memory` | `Observation` 保存正式观测、证据与版本并提供校验和 JSON 编解码。`Entity` 保存永久 ID、修订、状态及物品或空间证据。两者可作为独立的 C++ 记录使用。 |
| `engine` | 将已核对的配对转为正式观测，组织批次并发布；回读观测源验证投影和实体创建。该层单向消费准备层和记录层。 |
| `apps` | 命令行解析及结果报告。当前入口为 `bag_builder`、`preparation_compact`、`memory_build_input` 和 `memory_observation_dataset`。 |

## 三类产物

| 产物 | 入口与作用 |
| --- | --- |
| preparation | `bag_preparation/1`。保存媒体、源消息摘要、配对、相机模型、历史 TF 和待处理报告。新导出的 v7 manifest 绑定媒体 SHA-256 索引。 |
| 正式观测数据集 | `observation_dataset/2` 的 manifest、`observation/1` 的逐条记录、批次映射及选择和问题报告。读取器兼容旧 v1，按 ID 或批次回读，并通过来源指纹回查 preparation。便携发布可在同一目录内附带精简 preparation。 |
| 实体记录 | 拥有型 C++ `Entity`，状态为 `located` 或 `pending_location`。物品保存深度选区和表面代表点，空间保存内部确认与底盘锚点。SQLite 工作库绑定正式观测集，提供事务写入和按 ID 回读。完整记忆版本待实现。 |

## 时间、几何与证据

- 正式观测以 RGB 原始采样时间为参考，保留两路原始时间、实际时差、时钟与数据段。RGB 相机及底盘位姿查询用 RGB 参考时刻，深度相机位姿使用 Depth 时刻。详见 [时间契约](observation_time_contract.md)。
- 深度到地图使用米制 optical-z、该图的有效模型和历史深度位姿。物品代表点来自深度选区中可信簇的实测像素。深度到 RGB 候选像素仍依赖设备标定、外参和实际时间关系。
- 空间代表点取自经上游确认在区域内的 `body_pose_map` 平移。确认绑定观测修订、来源指纹及地图几何版本，创建器会回读源配对核对位姿。底盘朝向仍属观测。详见 [实体创建](../modules/engine/entity_builder.md)。
- Reader 的元数据路径用于组装和空间定位，完整回读用于恢复像素、核对文件摘要和实际投影。结构合法但缺几何的记录可保留待定位原因。详见 [PreparationReader](../modules/bag/preparation_reader.md)。

## 源码、构建和验证

```text
include/vln_memory/
├── bag/                 # common、rgb、depth、camera_info、tf、builder
├── memory/              # observation、entity
└── engine/              # 选帧、观测组装/发布、回读投影、实体创建
src/
├── bag/                 # 传感器、几何和 preparation 实现
├── memory/              # 业务记录的实现
├── engine/              # 离线构建流程
└── apps/                # 命令行入口
test/
├── bag/                 # 核心行为、ROS 适配和录包回归
├── memory/              # 观测与实体
├── engine/              # 选帧、观测组装/发布和区域回读
└── install_consumer/    # 独立安装包消费者
```

公开头从 `include/` 搜索，安装后保留 `vln_memory/{bag,memory,engine}/` 路径；实现目录中的内部头只供包内使用。核心构建消费已有 preparation。ROS 构建额外提供传感器适配与录包入口。目标、运行命令和验证边界见 [构建与导出](CMakeLists.md)。

项目开发约定以 [AGENTS.md](../../AGENTS.md) 为准。模块文档位于 [文档导航](../README.md)，各阶段历史数据和测试结论保留于 [验收记录](../reviews/) 及 [范围对照的历史记录](sections_1_2_scope.md#历史实施记录)。
