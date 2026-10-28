# sensor_types.hpp：采样时间与图像网格

头文件位于 include/vln_memory/bag/common/sensor_types.hpp，属于观测准备层的共享类型。

## 作用

定义 `vln_memory::SensorStamp`，供 RGB、Depth、CameraInfo、后续 TF 和 bag 组装共用，避免各模块自行定义不同的时间结构。

```cpp
struct SensorStamp {
  std::int32_t sec;
  std::uint32_t nanosec;
};
```

## 设计与自审

- 保留 ROS sec/nanosec，不用浮点秒关联消息，避免精度损失。
- sec 可为负数，nanosec 应位于 [0, 1e9)；零时间不是自动错误。
- 本类型不代表 UTC，也不包含 bag_id 或 clock_id。bag 模块在观测来源中维护时钟域，不能跨时钟直接比较。
- RGB、Depth、CameraInfo 各自校验纳秒字段。本文件只定义数据；显式时钟关系下的时间检查见 [sensor_time](sensor_time.md)。

RGB 输入与输出都直接使用此类型。后续模块按同一约定接入即可。

## ImageGeometry：公开图像网格

```cpp
struct ImageGeometry {
  std::uint32_t width;
  std::uint32_t height;
  std::string frame_id;
};
```

宽高与光学帧用于模块间核对布局；此类型不包含内参、时刻或几何注册证明。bag 层可以从 RGB 的公开输出构造 ImageGeometry{rgb.width, rgb.height, rgb.frame_id}，交给 checkDepthRgbAlignment 检查。

该类型拥有 frame_id 字符串，不借用来源消息。它是纯数据契约，不让 Depth 依赖 RGB 内部实现；CameraInfo 也使用它检查图像布局。详细规则见 [Depth 模块](../modules/depth/depth.md)。
