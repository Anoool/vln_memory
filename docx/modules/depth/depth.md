# Depth

## 模块职责与边界

负责原始深度解码、深度到地图投影、Depth→RGB 重投影和选区代表点。深度单位、光学坐标系、配准依据由上游提供。像素尺寸相同只说明网格尺寸相同。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["16UC1 / 32FC1"] --> B["米制 optical-z"]
  B --> C["有效相机模型与历史 TF"]
  C --> D["地图点或 RGB 像素"]
  D --> E["区域分簇与代表点"]
~~~

深度单点用有效模型的 `z=1` 射线乘 optical-z，再从深度相机变换到地图。重投影使用 RGB 和 Depth 各自原始时刻的位姿。区域使用深度像素的连通性和深度差分簇，保留可信簇及实测代表点。

## 数据结构

`DepthImageView` 借用源字节。`DepthOptions` 显式给单位、`optical_z` 和可选有效范围或对齐声明。`DepthFrame` 拥有行优先 float32 米制深度，非法像素为 NaN，并记录源比例、单位和质量计数。矩形 `DepthBoundingBox` 采用半开区间；`DepthPolygon` 用 3–64 个原图整数像素中心。

## 公共 API

| 公开头 | 入口 |
| --- | --- |
| [depth.hpp](../../../include/vln_memory/bag/depth/depth.hpp) · [depth_ros.hpp](../../../include/vln_memory/bag/depth/depth_ros.hpp) | `decodeDepth(view, options, limits)`、`decodeRosDepth`、`checkDepthRgbAlignment`。分别链接 `vln_memory::vln_memory_depth` / `vln_memory::vln_memory_depth_ros`。 |
| [depth_projection.hpp](../../../include/vln_memory/bag/depth/depth_projection.hpp) | `projectDepthPixelToMap`、`projectDepthPixelToRgb`，返回地图点或 RGB 浮点像素。 |
| [depth_region_projection.hpp](../../../include/vln_memory/bag/depth/depth_region_projection.hpp) | `checkDepthRegionSelection`、`projectDepthRegionToMap`、`projectRgbRegionToMap`，返回计数、候选簇及可选代表点。两份投影头均链接 `vln_memory::vln_memory_depth_projection`。 |

## 快速开始与典型示例

~~~cpp
vln_memory::DepthOptions options;
options.input_unit = vln_memory::DepthUnit::millimeters;
options.convention = vln_memory::DepthConvention::optical_z;
auto decoded = vln_memory::decodeDepth(view, options);
if (decoded.ok()) {
  auto point = vln_memory::projectDepthPixelToMap(*decoded.frame, model, pose, map_frame, u, v);
}
~~~

`model` 与 `pose` 必须对应同一深度帧，位姿时间精确匹配该帧的原始时间。彩色选区的调用示例见 [MemoryEngine](../engine/memory_engine.md#快速开始与典型示例)。

## 错误处理

解码状态与像素质量分别检查。全无效帧仍可能解码成功，但无法定位。选区越界、自交或为空时修正输入。标定、时刻或位姿不符时没有投影点。支撑不足或多簇歧义时查看区域状态和候选，避免选用缺少依据的点。
