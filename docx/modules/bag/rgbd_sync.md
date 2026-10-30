# rgbd_sync.hpp / rgbd_sync.cpp

## 用途与文件

[头文件](../../../include/vln_memory/bag/builder/rgbd_sync.hpp) 定义拥有型索引/结果；[实现](../../../src/bag/builder/rgbd_sync.cpp) 验证、排序、二分及统计。无 ROS；复用 sensor_time。

## 接口

pairRgbDepth(rgb, depth, options) 返回 RgbdSyncResult。输入 RgbdStreamIndex 有 topic、clock_id、segment_id、frames；每个 RgbdFrameRef 有 message_id、message_order、SensorStamp。ID 两流全局唯一；order 在各流内唯一。输入排列可变，order 必须稳定。

options 显式 same_clock、clock_basis、非负 max_delta_ns，合计 max_frames 默认 1000000。先检查 result.ok()，再读 report；配置/引用错误无报告。报告含 pairs、unmatched_rgb、unused_depth、数量及最大绝对差；空流也可成功。

每个 pair 拥有两侧引用，delta_ns=Depth-RGB；时间不被修改。RGB 顺序、一对一、Depth 单调、最近候选；等距选较早时间，同刻选较小 order。不是最大匹配数/最小总误差算法。unused Depth 不是坏帧。

## 所有权、复杂度与使用

输入同步借用，内部只排序指针，输出独立持有字符串。排序 O(R log R + D log D)，每 RGB O(log D)，辅助空间 O(R+D)。按上下文构造两个索引，配置选项后调用；具体参考 [安装消费者](../../../test/install_consumer/rgbd_sync_consumer.cpp)。图像解码、标定、TF、文件保存由 builder 调用，不进入本核心。

## 验证

见 [test_rgbd_sync](tests/test_rgbd_sync.md)。只有通过实测的内容才写入实施记录。
