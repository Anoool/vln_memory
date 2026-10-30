# vln_memory 文档导航

## 从哪里开始

| 内容 | 入口 |
| --- | --- |
| 当前只交付原设计第 1、2 部分：逐条完成情况与验收 | [范围与进度对照](project/sections_1_2_scope.md) |
| BagBuilder 第一版实际接口、参数与文件格式 | [BagBuilder 接口](interfaces/bag_builder_api.md) |
| BagBuilder 第一版实施与验证记录 | [实施记录](project/bag_builder_v1_implementation.md) |
| CLI 入口与运行 | [bag_builder_main](modules/bag/bag_builder_main.md) |
| 实际 YAML 配置/单位/独立时间阈值 | [vln_memory](config/vln_memory.md) |
| RGB-D 时间配对核心与真实接口 | [rgbd_sync](modules/bag/rgbd_sync.md) |
| BagBuilder：rosbag 读取、五模块整合的设计依据 | [整合方案](project/rosbag_integration_plan.md) |
| RGB-D 自动时间配对的分步方案与实施依据 | [配对方案](project/rgbd_sync_plan.md) |
| 当前问题修复、注释改进与验证 | [修复实施记录](project/sensor_validation_comments_plan.md) |
| 统一的已解析话题名称规则 | [sensor_names](common/sensor_names.md) |
| 当前真实能力、缺口与逐模块核查结果 | [源码核查报告](reviews/observation_preparation_audit.md) |
| 公开接口、提供文件、参数格式、返回值与作用 | [观测准备层接口手册](interfaces/observation_preparation_api.md) |
| 原设计第 1、2 部分的收敛实施规划 | [系统整体规划](project/system_plan.md) |
| 当前观测准备层、源码分类与开发/Git 约定 | [当前架构](project/architecture.md) |
| 构建目标、安装与导出 | [CMakeLists 说明](project/CMakeLists.md) |
| 采样时间与图像网格 | [共享数据类型](common/sensor_types.md) |
| RGB 输入输出、错误码、分步审阅、示例与验证 | [RGB 核心](modules/rgb/rgb.md) |
| ROS Image 怎样接入 RGB | [RGB ROS 适配](modules/rgb/rgb_ros.md) |
| Depth 扁平字段、单位、质量、对齐检查与可读性重构 | [Depth 核心](modules/depth/depth.md) |
| ROS Image 怎样接入 Depth | [Depth ROS 适配](modules/depth/depth_ros.md) |
| CameraInfo 扁平字段、标定检查、图像关联与迁移 | [CameraInfo 核心](modules/camera_info/camera_info.md) |
| CameraInfo 可读性重构的计划、边界与验证 | [重构记录](project/camera_info_readability_plan.md) |
| ROS CameraInfo 怎样接入 | [CameraInfo ROS 适配](modules/camera_info/camera_info_ros.md) |
| TF 可读性重构、扁平字段与验证 | [实施记录](project/tf_readability_plan.md) |
| TF 坐标链、静态/动态覆盖与历史样本 | [TF 核心](modules/tf/tf.md) |
| /tf、/tf_static 与逐条来源报告 | [TF ROS 适配](modules/tf/tf_ros.md) |
| 各模块时间、TF/入库/在线控制使用哪个时刻 | [sensor_time](common/sensor_time.md) |
| 构建和运行命令 | [项目 README](../README.md) |

## 当前目录

```text
docx/
├── README.md
├── interfaces/observation_preparation_api.md
├── reviews/observation_preparation_audit.md
├── project/{system_plan,architecture,CMakeLists}.md
├── common/
│   ├── sensor_names.md
│   ├── sensor_types.md
│   ├── sensor_time.md
│   └── tests/{test_sensor_names,test_sensor_time}.md
└── modules/
    ├── rgb/{rgb,rgb_ros}.md 与 tests/
    ├── depth/{depth,depth_ros}.md 与 tests/
    ├── camera_info/
    │   ├── camera_info.md
    │   ├── camera_info_ros.md
    │   └── tests/{test_camera_info,test_camera_info_ros,camera_info_consumer}.md
    └── tf/
        ├── tf.md
        ├── tf_ros.md
        └── tests/{test_tf,test_tf_ros,tf_consumer,tf_ros_consumer}.md
```

## 测试说明

| 模块 | 核心 | ROS 适配 | 安装后接口 |
| --- | --- | --- | --- |
| RGB | [行为测试](modules/rgb/tests/test_rgb.md) | [消息适配](modules/rgb/tests/test_rgb_ros.md) | [消费者](modules/rgb/tests/rgb_consumer.md) |
| Depth | [行为测试](modules/depth/tests/test_depth.md) | [消息适配](modules/depth/tests/test_depth_ros.md) | [RGB/Depth 协作](modules/depth/tests/depth_consumer.md)、[ROS 消费者](modules/depth/tests/depth_ros_consumer.md) |
| CameraInfo | [行为测试](modules/camera_info/tests/test_camera_info.md) | [消息适配及 ROS 安装](modules/camera_info/tests/test_camera_info_ros.md) | [三模块协作](modules/camera_info/tests/camera_info_consumer.md) |
| TF | [行为测试](modules/tf/tests/test_tf.md) | [消息/tf2 对照](modules/tf/tests/test_tf_ros.md) | [核心](modules/tf/tests/tf_consumer.md)、[ROS](modules/tf/tests/tf_ros_consumer.md) |
| 公共时间 | [边界测试](common/tests/test_sensor_time.md) | 随核心运行 | 由 CameraInfo 消费者验证传递依赖 |

## 后续模块位置

| 模块 | 文档目录 | 状态 |
| --- | --- | --- |
| RGB | modules/rgb/ | 已实现 |
| Depth | modules/depth/ | 已实现 |
| CameraInfo | modules/camera_info/ | 已实现 |
| TF | modules/tf/ | 链/覆盖已实现；位姿求解后续 |
| bag | modules/bag/ | 第一版读取/转换/配对/准备导出已实现；完整 Observation 后续 |

源码说明保留源文件名，模块测试放各自 tests/。共享约定进入 common/，项目约定进入 project/；模块的详细验证记录随模块保存。

## 当前开发位置

当前实现的是系统整体规划中的观测准备层 bag；这里的 RGB、Depth、CameraInfo、TF 和 builder 均属于该层。源码采用层/模块目录，现有同名文档按 modules/rgb、modules/depth、modules/camera_info、modules/tf 及 common 分类，不为尚未实现的其他层建立空说明。

## BagBuilder 文件导航

| 文件 | 说明 |
| --- | --- |
| 共享身份与运行结果 | [bag_types](modules/bag/bag_types.md) |
| 配置加载和校验 | [bag_config](modules/bag/bag_config.md) |
| 官方 reader、分卷及跨遍来源 | [bag_reader_ros](modules/bag/bag_reader_ros.md) |
| 标定检索/版本歧义 | [camera_info_index](modules/bag/camera_info_index.md) |
| 媒体与字节预算 | [media_writer](modules/bag/media_writer.md) |
| JSONL 数值/引用契约 | [jsonl_writer](modules/bag/jsonl_writer.md) |
| 完整调度与发布 | [bag_builder](modules/bag/bag_builder.md) |
| 磁盘集成/特殊场景回归 | [test_bag_builder](modules/bag/tests/test_bag_builder.md) |
| 独立 NumPy/JSON/摘要验证 | [test_bag_output](modules/bag/tests/test_bag_output.md) |
| 标定索引核心测试 | [test_camera_info_index](modules/bag/tests/test_camera_info_index.md) |
| 安装后接口调用 | [bag_builder_consumer](modules/bag/tests/bag_builder_consumer.md) |
