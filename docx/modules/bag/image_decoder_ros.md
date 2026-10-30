# image_decoder_ros.hpp / image_decoder_ros.cpp

## 职责与位置

[公开头](../../../include/vln_memory/bag/builder/image_decoder_ros.hpp)、[实现](../../../src/bag/builder/image_decoder_ros.cpp) 属于观测准备层 builder：将支持的 CompressedImage 适配为拥有型 Image，继续交给现有 RGB/Depth 核心。它不配对、不改时间、不选择标定，不推断单位或空间注册。

链接安装目标 `vln_memory::vln_memory_bag_builder`，包含 `<vln_memory/bag/builder/image_decoder_ros.hpp>`。无 ROS 核心不增加此依赖；ROS 构建使用已有 OpenCV、cv_bridge 与官方 compressed_depth_image_transport。

## 输入格式与限制

| role | 精确 format 白名单 | 输出 Image 编码 |
| --- | --- | --- |
| BagImageRole::rgb | `rgb8; jpeg compressed bgr8`、`bgr8; jpeg compressed bgr8` | bgr8，现有 RGB 核心再转 RGB8 |
| BagImageRole::depth | `16UC1; compressedDepth`、`16UC1; compressedDepth png` | 16UC1，原 uint16 数值 |

首轮不支持压缩 RGB PNG/mono、32FC1 compressedDepth、RVL 或无 format 声明的裸 JPEG；原始 Image 仍保留 RGB/Depth 核心原有编码支持。未支持格式返回明确失败，不按相似后缀猜测。

## 接口和返回值

```cpp
CompressedImageInfo inspectCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes);

CompressedImageResult decodeCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes);
```

| 参数/结果 | 格式与意义 |
| --- | --- |
| message | 只读真实 ROS 压缩消息；header、format 和 data 不被修改 |
| role | 显式 RGB/Depth 流身份，不能从 topic 名字猜 |
| max_pixels | 正整数，约束解码后 width×height |
| max_input_bytes | 正整数，同时约束压缩负载字节和解码后像素字节 |
| CompressedImageInfo | code/message、width/height、交给核心的 encoding、payload_offset；无像素、无借用引用 |
| CompressedImageResult | image 为拥有型 SharedPtr，code/message 拥有文本；失败 image 为空 |

先检查 `ok()` 再消费布局或 image。预检成功只说明头部/预算可解码，完整数据仍可能损坏。bad_alloc（含 OpenCV 内存不足）继续向上传播，其他受支持解码失败作为坏帧报告。

错误码：`none`、`unsupported_compressed_format`、`invalid_compressed_payload`、`limit_exceeded`。原始 Image/核心的其他错误保持原码；此模块不把配置或系统资源失败伪装成成功。

## 实际处理与所有权

1. 检查 format 和压缩字节预算。
2. RGB 有界扫描 JPEG SOF；Depth 按官方 ConfigHeader 大小定位并核对 PNG IHDR，先检查位深/通道和尺寸。
3. 用除法判断像素、解码字节和 ROS 行宽预算，避免乘法溢出，在像素分配前拒绝超限。
4. JPEG 用 OpenCV 解码为 BGR，并忽略 EXIF 自动旋转以保留相机网格；Depth 使用官方 codec。核对输出尺寸、编码、原 header 和字节预算。
5. 返回独立 Image；reader 回调仅在当前调用期间借用它，核心标准帧另外拥有输出字节。

16UC1 的 ConfigHeader 浮点参数不当作深度 scale；其原始配置头仍在 CDR。Depth PNG 解码不在这里乘 0.001，单位转换继续由显式 DepthOptions 决定。

预检最坏 O(B) 扫描 JPEG 字节，PNG 头部 O(1)，解码按当前帧像素处理。首遍只做预检；第二遍只拥有当前帧，不能缓存整包图像。

## 用法

```cpp
const auto decoded = vln_memory::decodeCompressedImage(
  message, vln_memory::BagImageRole::depth, 16777216, 134217728);
if (!decoded.ok()) {
  // 保留原消息及 decoded.code/message；不要通过改时间/改配掩盖坏帧。
  return;
}
const auto depth = vln_memory::decodeRosDepth(*decoded.image, explicit_depth_options);
```

上例调用 RGB/Depth 核心时应包含相应 `*_ros.hpp`。检查 bad payload 不会抹掉原 header 的集成路径见 [reader](bag_reader_ros.md)，零容差要求见 [BagBuilder 接口](../../interfaces/bag_builder_api.md)。

## 验证

[适配测试](tests/test_image_decoder_ros.md) 检查官方编码器生成的数据、RGB 通道、深度原值、原 header、预算、未知格式和坏负载；[集成测试](tests/test_bag_builder.md) 检查混合/压缩消息与相差 1ns 的拒绝。真实 bag 全量重跑证据另见 [零容差验收](../../reviews/rgbd_zero_tolerance_verification.md)。
