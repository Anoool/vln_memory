# vln_memory 文档导航

## 从哪里开始

| 内容 | 入口 |
| --- | --- |
| 整个系统目标、各层职责与实施顺序 | [系统整体规划](project/system_plan.md) |
| 当前观测准备层、源码分类与开发/Git 约定 | [当前架构](project/architecture.md) |
| 构建目标、安装与导出 | [CMakeLists 说明](project/CMakeLists.md) |
| 采样时间与图像网格 | [共享数据类型](common/sensor_types.md) |
| RGB 输入输出、错误码、示例与验证 | [RGB 核心](modules/rgb/rgb.md) |
| ROS Image 怎样接入 RGB | [RGB ROS 适配](modules/rgb/rgb_ros.md) |
| Depth 类型、单位、质量与对齐检查 | [Depth 核心](modules/depth/depth.md) |
| ROS Image 怎样接入 Depth | [Depth ROS 适配](modules/depth/depth_ros.md) |
| 构建和运行命令 | [项目 README](../README.md) |

## 当前目录

```text
docx/
├── README.md
├── project/
│   ├── system_plan.md
│   ├── architecture.md
│   └── CMakeLists.md
├── common/
│   └── sensor_types.md
└── modules/
    ├── rgb/
    │   ├── rgb.md
    │   ├── rgb_ros.md
    │   └── tests/
    │       ├── test_rgb.md
    │       ├── test_rgb_ros.md
    │       └── rgb_consumer.md
    └── depth/
        ├── depth.md
        ├── depth_ros.md
        └── tests/
            ├── test_depth.md
            ├── test_depth_ros.md
            ├── depth_consumer.md
            └── depth_ros_consumer.md
```

## 测试说明

| 模块 | 核心 | ROS 适配 | 安装后接口 |
| --- | --- | --- | --- |
| RGB | [行为测试](modules/rgb/tests/test_rgb.md) | [消息适配](modules/rgb/tests/test_rgb_ros.md) | [消费者](modules/rgb/tests/rgb_consumer.md) |
| Depth | [行为测试](modules/depth/tests/test_depth.md) | [消息适配](modules/depth/tests/test_depth_ros.md) | [RGB/Depth 协作](modules/depth/tests/depth_consumer.md)、[ROS 消费者](modules/depth/tests/depth_ros_consumer.md) |

## 后续模块位置

| 模块 | 文档目录 | 状态 |
| --- | --- | --- |
| RGB | modules/rgb/ | 已实现 |
| Depth | modules/depth/ | 已实现 |
| CameraInfo | modules/camera_info/ | 实现时建立 |
| TF | modules/tf/ | 实现时建立 |
| bag | modules/bag/ | 实现时建立 |

源码说明保留源文件名，模块测试放各自 tests/。共享约定进入 common/，项目约定进入 project/；模块的详细验证记录随模块保存。

## 当前开发位置

当前实现的是系统整体规划中的观测准备层 bag；这里的 RGB、Depth、CameraInfo、TF 和 builder 均属于该层。源码采用层/模块目录，现有同名文档仍按 modules/rgb、modules/depth 分类，不为尚未实现的其他层建立空说明。
