# sensor_types.hpp：跨模块采样时间

## 作用

定义 `vln_memory::SensorStamp`，供 RGB、后续 Depth/CameraInfo/TF 和 bag 组装共用，避免各模块自行定义不同的时间结构。

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
- 当前 decodeRgb 负责检查纳秒字段；本文件是无隐藏处理的数据类型，不提供未经定义的时间换算。

RGB 输入与输出都直接使用此类型。后续模块按同一约定接入即可。
