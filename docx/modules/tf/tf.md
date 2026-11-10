# TF

## 模块职责与边界

为同一时钟和录包段建立离线 TF 历史，按图像采样时刻求数值位姿并保留所用端点。上游提供真实父子 frame 和来源，索引不外推或用最近位姿补空缺。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["/tf 与 /tf_static"] --> B["逐条映射并插入 TfIndex"]
  B --> C["坐标链与时间覆盖"]
  C --> D["逐边精确取样 / 插值"]
  D --> E["组合 T_target_source 和端点证据"]
~~~

静态边或精确命中直接使用原样本，动态边按门限选相邻端点，线性插值平移、最短弧插值旋转。反向边先插值再求逆，按源到目标顺序组合。`T_target_source` 满足 `p_target = R * p_source + t`。

## 数据结构

`TfScope` 标识时钟和段，`TfTransform` 保存原始父子帧、时间、米制平移、xyzw 四元数及消息来源。`TfTimeOptions` 给查询时钟段和插值间隔，`TfJumpLimits` 可限制相邻样本的运动跳变。`TfPoseResult` 持有查询结果、坐标链和端点证据，可在索引销毁后使用。

## 公共 API

| 公开头 | 入口 |
| --- | --- |
| [tf.hpp](../../../include/vln_memory/bag/tf/tf.hpp) | `TfIndex::insert`、`inspectChain`、`checkTime`、`lookupPose`、`inferInterpolationGapNs`。`transformTfPoint` / `inverseTransformTfPoint` 用数值位姿转换米制点。链接 `vln_memory::vln_memory_tf`。 |
| [tf_ros.hpp](../../../include/vln_memory/bag/tf/tf_ros.hpp) | `ingestRosTf` 将 `TFMessage` 逐条写入索引，返回处理和单条问题。链接 `vln_memory::vln_memory_tf_ros`。 |

## 快速开始与典型示例

~~~cpp
auto result = index.lookupPose("map", camera_frame, image_stamp, time_options);
if (result.ok()) {
  auto point_map = vln_memory::transformTfPoint(*result.pose, point_camera_m);
}
~~~

输入 `index` 应已读完对应段的 TF。RGB 和 Depth 各自使用原始采样时刻查询，查询底盘参考位姿时使用 RGB 时刻。

## 错误处理

不同钟段、缺链、无时间覆盖或插值间隔过大时查看时间状态和失败边。父帧冲突、环或不一致的同刻样本会拒绝插入并可阻断相关链。位姿失败时 `pose` 为空。数值点变换失败返回空值。
