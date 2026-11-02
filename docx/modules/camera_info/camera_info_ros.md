# camera_info_ros.hpp / camera_info_ros.cpp：ROS CameraInfo 薄适配

## 规划与自审

保持与 RGB/Depth 相同的结构：真实消息适配只映射字段，把所有判断交给独立 C++17 核心。话题由 bag 元数据传入，不从 frame_id 或相机命名猜测；不额外订阅消息，不读取 TF，不改写时间。

## 文件与接口

头文件 include/vln_memory/bag/camera_info/camera_info_ros.hpp；实现 src/bag/camera_info/camera_info_ros.cpp。

```cpp
CameraInfoResult decodeRosCameraInfo(
  const sensor_msgs::msg::CameraInfo & message, std::string_view topic);
```

映射 header.stamp/frame_id、width/height、distortion_model、D/K/R/P、binning、ROI 全部字段。ROI 保留原始语义，x_offset、y_offset、width、height、do_rectify 使用逐字段赋值，避免聚合初始化时混淆宽高顺序。全部输出拥有数据，不保留 message/topic 的借用引用。

bag reader 必须先验证话题类型为 sensor_msgs/msg/CameraInfo；本函数强类型接收，不能代替序列化层类型检查。结构、错误、配对和时间规则见 [核心](camera_info.md) 与 [时间约定](../../common/sensor_time.md)。

## 使用

```cpp
#include <vln_memory/bag/camera_info/camera_info_ros.hpp>

// msg 是已经反序列化的 CameraInfo，topic 来自 bag 元数据。
auto result = vln_memory::decodeRosCameraInfo(msg, topic);
if (!result.ok()) {
  // 把 result.code/message 交给上层 issues/pending 报告。
} else {
  // 用 result.info 与 RGB 或 Depth 的实际公开元数据做 checkCameraInfoImage。
}
```

find_package(vln_memory CONFIG REQUIRED)，链接 vln_memory::vln_memory_camera_info_ros；sensor_msgs、核心、公共时间目标的依赖向下游传递。VLN_MEMORY_ENABLE_ROS=OFF 时不构建/安装此适配头与目标。

## 验证

真实 Humble CameraInfo 测试检查全部标定、时间、ROI/binning 字段与所有权，并验证未标定错误继续来自核心。安装后的 ROS 消费者仅链接公开 ROS 目标并成功运行，见 [测试说明](tests/test_camera_info_ros.md)。


## 2026-10-29 修复与注释复核

说明完整矩阵/畸变字段映射和核心校验，不在适配层补默认标定或修改原始几何。 话题格式统一最长 247 字节。具体修复与验证记录见 [本次实施记录](../../project/sensor_validation_comments_plan.md)。

## 2026-11-03 统一兼容入口

copyRosCameraInfo(message, topic) 独立复制未经校验的全部 ROS 字段；供 reader 保存原始证据，返回值不能未经校验用于几何。decodeRosCameraInfo(message, topic, compatibility=strict) 复用该映射后调用 normalizeCameraInfo，结果与无 ROS 核心一致。旧两参数源码调用仍为严格模式；签名扩展须重新编译下游。

Reader 与 ROS API 共用这一映射，不再维护两份字段复制代码；末三项非零/非有限不会被截断。测试增加直接 ROS D8 兼容及默认严格检查。
