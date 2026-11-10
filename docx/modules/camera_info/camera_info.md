# CameraInfo

## 模块职责与边界

保留原始标定，检查它是否属于指定图像，并生成实际像素网格的有效相机模型。采集绑定、图像是否已校正、标定是否可静态复用由上游明确。CameraInfo 不求 TF，也不判断两路图像的物理注册。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["ROS CameraInfo / CameraInfoData"] --> B["校验与受限兼容"]
  B --> C["按话题、frame、时间和网格绑定"]
  C --> D["CameraModel"]
  D --> E["像素与原光学帧之间投影"]
~~~

动态标定在同一时钟下检查时间容差，静态复用依赖段内固定标定。按 ROI/binning 得到实际网格，raw 按畸变模型去畸变，rectified 使用已校正的投影并保留原光学帧语义。严格校验为默认。项目录包的 D8 零尾兼容只处理指定的有限零尾情况，原始 D 向量仍保留为证据。

## 数据结构

`CameraInfoData` 含源话题、frame、时间、宽高、畸变模型、K/D/R/P、ROI 和 binning。`CameraImageReference` 描述实际图像，`CameraInfoMatchOptions` 给出绑定、图像域及时间策略。`CameraModel` 持有实际图像网格及有效参数。反投影的射线在原光学 frame 下取 `z=1`。

## 公共 API

| 公开头 | 入口与链接 |
| --- | --- |
| [camera_info.hpp](../../../include/vln_memory/bag/camera_info/camera_info.hpp) | `decodeCameraInfo` 校验原始记录，`normalizeCameraInfo` 应用明确兼容策略，`checkCameraInfoImage` 检查关联。链接 `vln_memory::vln_memory_camera_info`。 |
| [camera_info_ros.hpp](../../../include/vln_memory/bag/camera_info/camera_info_ros.hpp) | `copyRosCameraInfo` / `decodeRosCameraInfo` 映射真实 ROS 消息。链接 `vln_memory::vln_memory_camera_info_ros`。 |
| [camera_model.hpp](../../../include/vln_memory/bag/camera_info/camera_model.hpp) | `buildCameraModel(info, image, options)`，以及 `pixelToCameraRay`、`cameraPointToPixel`。链接 `vln_memory::vln_memory_camera_model`。 |

## 快速开始与典型示例

~~~cpp
auto model = vln_memory::buildCameraModel(info, image, binding);
if (model.ok()) {
  auto ray = vln_memory::pixelToCameraRay(*model.model, u, v);
  if (ray.ok()) {
    // 射线乘以同一光学轴的米制 optical-z，得到相机坐标。
  }
}
~~~

## 错误处理

话题、frame、时钟、尺寸或采样时间不匹配时，查匹配状态并更正真实绑定。无效标定、模型不支持或像素越界时，结果没有可用模型或像素。是否放行零尾 D8 由明确的兼容配置决定。
