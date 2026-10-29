# vln_memory 文档导航

## 从哪里开始

| 内容 | 入口 |
| --- | --- |
| 当前真实能力、缺口与逐模块核查结果 | [源码核查报告](reviews/observation_preparation_audit.md) |
| 公开接口、提供文件、参数格式、返回值与作用 | [观测准备层接口手册](interfaces/observation_preparation_api.md) |
| 整个系统目标、各层职责与实施顺序 | [系统整体规划](project/system_plan.md) |
| 当前观测准备层、源码分类与开发/Git 约定 | [当前架构](project/architecture.md) |
| 构建目标、安装与导出 | [CMakeLists 说明](project/CMakeLists.md) |
| 采样时间与图像网格 | [共享数据类型](common/sensor_types.md) |
| RGB 输入输出、错误码、示例与验证 | [RGB 核心](modules/rgb/rgb.md) |
| ROS Image 怎样接入 RGB | [RGB ROS 适配](modules/rgb/rgb_ros.md) |
| Depth 类型、单位、质量与对齐检查 | [Depth 核心](modules/depth/depth.md) |
| ROS Image 怎样接入 Depth | [Depth ROS 适配](modules/depth/depth_ros.md) |
| CameraInfo 校验、图像话题/frame/尺寸关联 | [CameraInfo 核心](modules/camera_info/camera_info.md) |
| ROS CameraInfo 怎样接入 | [CameraInfo ROS 适配](modules/camera_info/camera_info_ros.md) |
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
│   ├── sensor_types.md
│   ├── sensor_time.md
│   └── tests/test_sensor_time.md
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
| bag | modules/bag/ | 实现时建立 |

源码说明保留源文件名，模块测试放各自 tests/。共享约定进入 common/，项目约定进入 project/；模块的详细验证记录随模块保存。

## 当前开发位置

当前实现的是系统整体规划中的观测准备层 bag；这里的 RGB、Depth、CameraInfo、TF 和 builder 均属于该层。源码采用层/模块目录，现有同名文档按 modules/rgb、modules/depth、modules/camera_info、modules/tf 及 common 分类，不为尚未实现的其他层建立空说明。
