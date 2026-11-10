# RGB

## 模块职责与边界

把未压缩彩色图像解码为自有的紧密 `rgb8` 帧，保留源采样时刻、frame 和原像素网格。压缩图像解码与落盘由 BagBuilder 负责，ROS 适配只映射消息字段。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["ROS Image 或内存像素"] --> B["借用 RgbImageView"]
  B --> C["校验布局 / 统一通道"]
  C --> D["拥有像素的 RgbFrame"]
~~~

## 数据结构

输入支持 `rgb8`、`bgr8` 和 `mono8`。`RgbImageView` 借用像素、encoding 和 frame，调用期间须有效。输入 `step` 含行尾填充，`data_size` 等于 `step * height`。输出 `RgbFrame` 逐行紧密排列 RGB，`step = width * 3`，保留源编码、源步长和字节序，可在源消息销毁后使用。

## 公共 API

- [rgb.hpp](../../../include/vln_memory/bag/rgb/rgb.hpp) 提供 `decodeRgb(view, limits) → RgbResult`、`rgbErrorCodeName`。链接 `vln_memory::vln_memory_rgb`。
- [rgb_ros.hpp](../../../include/vln_memory/bag/rgb/rgb_ros.hpp) 提供 `decodeRosRgb(message, limits)`。链接 `vln_memory::vln_memory_rgb_ros`。

## 快速开始与典型示例

~~~cpp
const auto result = vln_memory::decodeRosRgb(rgb_message);
if (result.ok()) {
  const auto & frame = *result.frame;
  // 使用 frame.pixels，像素坐标与源图一致。
}
~~~

## 错误处理

编码、布局、时间或资源上限不符合输入契约时返回空帧，读 `code/message` 定位问题。系统分配异常按 C++ 异常处理。
