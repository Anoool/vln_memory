# vln_memory 文档

~~~mermaid
flowchart LR
  A["ROS 2 RGB-D bag"] --> B["BagBuilder"]
  S["RGB / Depth / CameraInfo / TF"] --> B
  B -->|"双图 Observation + evidence"| E["MemoryEngine"]
  E -->|"实体、关系、证据、回执"| W["SQLite 工作库"]
  E -.->|"完整记忆版本：规划"| V["MemoryViewer：规划"]
~~~

从 [项目 README](../README.md) 安装和启动。模块文档统一包含职责与边界、整体设计图、数据结构、公共 API、调用示例和错误处理。

| 模块 | 文档 |
| --- | --- |
| BagBuilder | [录包、配置、RGB-D 配对与真包验收](modules/bag/bag_builder.md) |
| MemoryEngine | [手动工具、深度定位与自动写入批次](modules/engine/memory_engine.md) |
| MemoryViewer | [检查界面和标注交接（规划）](modules/viewer/memory_viewer.md) |
| Memory | [正式观测、实体与 SQLite 契约](modules/memory/memory.md) |
| RGB | [彩色解码与 ROS 适配](modules/rgb/rgb.md) |
| Depth | [米制解码、重投影与区域定位](modules/depth/depth.md) |
| CameraInfo | [标定关联与有效模型](modules/camera_info/camera_info.md) |
| TF | [历史索引、插值与数值位姿](modules/tf/tf.md) |
| Common | [时间、图像网格及话题](common/common.md) |

项目顺序见[实施规划](project/system_plan.md)，原设计对照见[第 1、2 部分范围](project/sections_1_2_scope.md)，测试结论和旧文档查阅方式见[验收记录](reviews/verification.md)。
