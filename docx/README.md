# vln_memory 文档导航

## 从哪里开始

| 需要查看的内容 | 入口 |
| --- | --- |
| 五模块分工、依赖和开发规范 | [整体架构](project/architecture.md) |
| 构建目标、安装与导出 | [CMakeLists 说明](project/CMakeLists.md) |
| 跨模块采样时间 | [sensor_types](common/sensor_types.md) |
| RGB 输入输出、错误码、示例和验证结果 | [RGB 核心](modules/rgb/rgb.md) |
| ROS Image 怎样接入 RGB | [RGB ROS 适配](modules/rgb/rgb_ros.md) |
| 构建和运行命令 | [项目 README](../README.md) |

## 当前目录

```text
docx/
├── README.md
├── project/
│   ├── architecture.md
│   └── CMakeLists.md
├── common/
│   └── sensor_types.md
└── modules/
    └── rgb/
        ├── rgb.md
        ├── rgb_ros.md
        └── tests/
            ├── test_rgb.md
            ├── test_rgb_ros.md
            └── rgb_consumer.md
```

## RGB 测试说明

- [核心行为测试](modules/rgb/tests/test_rgb.md)：颜色、行填充、所有权、异常输入和限额。
- [ROS 适配测试](modules/rgb/tests/test_rgb_ros.md)：真实 sensor_msgs 消息映射。
- [安装后消费者](modules/rgb/tests/rgb_consumer.md)：其他模块如何使用安装后的接口。

## 后续模块位置

| 模块 | 文档目录 | 当前状态 |
| --- | --- | --- |
| RGB | modules/rgb/ | 已实现 |
| Depth | modules/depth/ | 实现时建立 |
| CameraInfo | modules/camera_info/ | 实现时建立 |
| TF | modules/tf/ | 实现时建立 |
| bag | modules/bag/ | 实现时建立 |

源码说明保留源文件名，例如 rgb.cpp 对应 modules/rgb/rgb.md，test_rgb.cpp 对应 modules/rgb/tests/test_rgb.md。跨模块内容进入 common/，项目内容进入 project/；详细的模块验证记录随模块保存，避免总览不断膨胀。
