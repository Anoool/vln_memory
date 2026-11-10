# 传感器公共类型

## 模块职责与边界

提供传感器时间、网格和话题名称的共享约定。比较时间由调用方声明时钟关系，上层负责配对、时钟标定和话题重映射。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["源 stamp / 网格 / 话题"] --> B["公共类型和基本检查"]
  B --> C["RGB、Depth、CameraInfo、TF"]
  C --> D["BagBuilder 配对与观测"]
~~~

## 数据结构

`SensorStamp` 保留有符号秒和非负纳秒，`ImageGeometry` 保存宽、高和坐标系。`SensorTimeOptions` 给出时钟关系和非负整数纳秒容差，结果里的 `delta_ns` 为 candidate − reference。零时刻及负秒合法，bag 记录时刻与采样时刻分开保存。

## 公共 API

| 公开头 | 入口 |
| --- | --- |
| [sensor_types.hpp](../../include/vln_memory/bag/common/sensor_types.hpp) | `SensorStamp`、`ImageGeometry`。 |
| [sensor_time.hpp](../../include/vln_memory/bag/common/sensor_time.hpp) | `checkSensorTimes(reference, candidate, options)`、`sensorTimeCodeName`。链接 `vln_memory::vln_memory_sensor_time`。 |
| [sensor_names.hpp](../../include/vln_memory/bag/common/sensor_names.hpp) | `isResolvedTopicName(topic)`，检查已解析的绝对话题名称。 |

## 快速开始与典型示例

~~~cpp
const vln_memory::SensorTimeOptions options{
  vln_memory::SensorClockRelation::same_clock, 30000000};
const auto match = vln_memory::checkSensorTimes(rgb_stamp, depth_stamp, options);
~~~

## 错误处理

`outside_tolerance` 仍可读取实际差值。`invalid_stamp`、`invalid_tolerance`、`clock_unknown` 或 `clock_mismatch` 无可用差值。话题名称检查失败返回 `false`，调用者提供正确绑定。
