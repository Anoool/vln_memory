# rgb_ros.cpp / rgb_ros.hpp：ROS Image 适配

源码：[rgb_ros.cpp](../../../src/bag/rgb/rgb_ros.cpp)，头文件：[rgb_ros.hpp](../../../include/vln_memory/bag/rgb/rgb_ros.hpp)。

## 实施前规划与自审

适配器只负责 ROS 字段到 RgbImageView 的映射，随后调用 decodeRgb。自审要求：不订阅、不反序列化、不同步、不保留源消息引用，不实现另一套颜色或尺寸处理。

## 接口和用途

```cpp
RgbResult decodeRosRgb(
  const sensor_msgs::msg::Image & image, const RgbLimits & limits = {});
```

bag 模块取得已反序列化的 Image 后直接调用。本接口与 decodeRgb 使用完全相同的结果和错误码。

## 映射

- `header.stamp` -> SensorStamp。
- `header.frame_id` -> 光学帧名称。
- `width/height/encoding/step/is_bigendian` -> 原始布局。
- `data.data()/data.size()` -> 仅调用期有效的只读借用视图。

成功时返回的字符串和像素独立拥有，源消息释放或复用不影响输出。

## 如何接入

```cpp
#include <vln_memory/bag/rgb/rgb_ros.hpp>

vln_memory::RgbResult processRgbMessage(const sensor_msgs::msg::Image & image)
{
  return vln_memory::decodeRosRgb(image);
}
```

```cmake
find_package(vln_memory REQUIRED)
add_library(my_bag_reader src/my_bag_reader.cpp)
target_link_libraries(my_bag_reader PUBLIC vln_memory::vln_memory_rgb_ros)
```

适配库传递 sensor_msgs 和核心库依赖；调用方不需要链接 SessionManager 或 ToolManager。RGB 本身没有 vln_agent 依赖，后续由 Observation 数据契约与 Agent 衔接。

## 错误与验证边界

与核心一致：未支持编码返回 unsupported_encoding；空 frame_id 和零时间戳原样保留，能否定位由后续 TF/组装层判断。当前仅支持未压缩 Image。

真实 ROS 消息测试位于 `test/bag/rgb/test_rgb_ros.cpp`，执行方式见 [适配测试说明](tests/test_rgb_ros.md)，实际结果见 [RGB 验证记录](rgb.md#实施与验证记录)。缺少 ROS 环境时，不用仿造消息头冒充真实 ROS 集成测试。


## 2026-10-29 修复与注释复核

注明各字段组原样映射及临时借用终止位置，明确 step 包含消息真实行填充。 具体修复与验证记录见 [本次实施记录](../../project/sensor_validation_comments_plan.md)。


## 2026-10-29 人工审阅整理

补充适配函数的输入/输出与所有权说明，按采样身份、布局、借用缓冲区分组，合并重复注释。字段映射与公开接口保持；核心的分步审阅和本次验证见 [RGB 可读性重构](rgb.md#2026-10-29-人工审阅可读性重构)。
