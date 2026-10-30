# depth_ros.cpp / depth_ros.hpp：ROS Depth 适配

源码：[depth_ros.cpp](../../../src/bag/depth/depth_ros.cpp)，头文件：[depth_ros.hpp](../../../include/vln_memory/bag/depth/depth_ros.hpp)。

## 规划与自审

只把真实 Image 元数据和只读数据映射为 DepthImageView，调用 decodeDepth。所有语义与像素校验在核心复用；不订阅、不读取 bag、不猜测单位或注册。

## 接口

```cpp
DepthResult decodeRosDepth(
  const sensor_msgs::msg::Image & image, const DepthOptions & options,
  const DepthLimits & limits = {});
```

消息类型由参数限定为 sensor_msgs/msg/Image；其他消息没有重载。bag 层应在反序列化前检查话题类型。16UC1、32FC1 仍由核心检查，mono16 等不被静默当深度。

调用者须指定单位、optical_z，并提供可追溯的 RGB 注册配置（如果已确认）。结果与错误码与核心一致；源消息和 options 释放后输出仍有效。

## 如何使用

```cpp
#include <vln_memory/bag/depth/depth_ros.hpp>

vln_memory::DepthResult handleDepth(
  const sensor_msgs::msg::Image & message,
  const vln_memory::DepthOptions & configured_options)
{
  return vln_memory::decodeRosDepth(message, configured_options);
}
```

下游 find_package(vln_memory REQUIRED)，链接 vln_memory::vln_memory_depth_ros；目标会传递核心与 sensor_msgs 依赖。

本轮已在真实 Humble 环境构建并运行 Image 测试。见 [测试说明](tests/test_depth_ros.md)、[安装消费者](tests/depth_ros_consumer.md)、[验证结果](depth.md#验证结果)。


## 2026-10-29 修复与注释复核

整理并保留用户的信息头/时间戳说明，分开解释布局、单位来源和同步调用期间的借用。 具体修复与验证记录见 [本次实施记录](../../project/sensor_validation_comments_plan.md)。

2026-10-29 可读性调整：核心的注册配置现在使用并列的 alignment、rgb_grid、alignment_basis，质量直接从 DepthFrame 读取；ROS 入口签名与字段映射保持。
