# test_image_decoder_ros.cpp

[测试源码](../../../../test/bag/builder/test_image_decoder_ros.cpp) 通过标准 JPEG 和官方 compressedDepth 编码器构造真实 CompressedImage；这些是构造测试，不是设备曝光验证。

## 覆盖

- JPEG 解码 BGR 与 RGB 核心的通道顺序；带 EXIF 旋转提示仍保留相机原网格。
- uint16 的 0、1、256、1000 原值、字节序以及负秒/非零纳秒 header 保留。
- 原 format 与显式 PNG 后缀；未知 32FC1/RVL 格式拒绝。
- 压缩字节、解码像素/字节预算；巨大 PNG IHDR 与截断 JPEG/PNG。
- 预检成功但 PNG 主体损坏时返回失败。

## 运行

```bash
source /opt/ros/humble/setup.bash
ctest --test-dir build/rgbd-strict-ros -R image_decoder_ros --output-on-failure
```

依赖和公开接口见 [image_decoder_ros](../image_decoder_ros.md)。安装后接口调用同时由 bag_builder_consumer 验证。
