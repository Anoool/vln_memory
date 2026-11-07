# 从正式观测投影深度区域

公开接口为 `include/vln_memory/engine/observation_depth_region.hpp`，目标为 `vln_memory::vln_memory_observation_depth_region`。从观测数据集清单取得 preparation 根，打开 `PreparationReader`，把正式观测行交给 `deserializeObservation`。

```cpp
vln_memory::DepthBoundingBox box{512, 316, 528, 332};
auto projected = vln_memory::projectObservationDepthRegion(*opened.reader, *parsed.observation, box);
if (projected.ok()) {
  const auto & point = *projected.projection->representative;
  // point.pixel 为深度原图像素，point.point_map_m 为 map 米制坐标。
}
```

入口核对 manifest 指纹、map、配对 ID、两路消息身份、时钟与分段。回读完整深度后核对深度媒体路径、存储解释、相机模型、深度标定 ID、历史位姿、TF 端点和来源消息摘要，计算后复查文件状态。深度投影使用原始深度时间的位姿。输入为深度原图像素，RGB 注册与曝光同步 pending 可随观测保留。

返回值的 `code` 区分观测无效、来源不匹配、读取失败和缺深度、模型、位姿。进入几何阶段后，`projection` 包含 [区域投影](../depth/depth_region_projection.md) 的状态、点簇与计数。有效点数指投影成功的深度像素数。簇支持表达选区内的几何一致性，地图坐标仍依赖录制时的标定、尺度和轨迹依据。
