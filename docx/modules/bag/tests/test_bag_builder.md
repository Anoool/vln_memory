# test_bag_builder.cpp

[源码](../../../../test/bag/builder/test_bag_builder.cpp) 对应 CTest bag_builder_ros。使用官方 rosbag2 Writer 和真实 ROS 类型支持，将 CDR 写入磁盘 SQLite；用未选择的大消息强制分卷，再调用实际 buildBag。

## 验证内容

RGB 行填充/色彩、uint16 毫米转换、NaN/全无效、RGB-D 相等及近似边界差值、未配对、重复记录时间、负 bag 时间、迟到静态/动态 TF、重复广播和空 TFMessage。检查所有原始 CDR 引用及不虚构位姿。

独立变体覆盖错误消息类型、静态模型变化、TF 冲突、图像时间重启、容量失败、旧结果覆盖拒绝、跨遍源文件改变、单文件范围标记和配置未知/重复/负容量。

## 产物与范围

每次在 build 下独立 test_artifacts/run_<计时值> 创建录包和数据集，latest.txt 指向最近成功用例。demo_config.yaml 可直接运行 CLI，输出应选新的目录。生成物不入 Git。

这些是磁盘格式/库的构造回归，不是实际设备录制；未验证物理同步、标定精度或 map 数值位姿。

## 保留的零容差回归

当前主 fixture 的两对 header 已改为精确相等，旧 ±4ms 是第一版历史用例。新增相差 1ns 全拒绝、JPEG+compressedDepth、原 RGB+压缩 Depth 混合流、坏压缩体保留 header/固定配对、负数/小数/溢出 YAML 以及负数 C++ API 拒绝。错误类型 fixture 改为非图像消息，CompressedImage 已是受支持类型。

上述零门限变体验证近似配对数为 0、原始媒体保留、物理同步 unknown。独立 NumPy 校验检查两图原 stamp 完全相等及两流计数守恒；新增 [压缩适配测试](test_image_decoder_ros.md)。真实数据与构造数据分别见 [验收](../../../reviews/rgbd_zero_tolerance_verification.md)。

## 最近配对回归

原始/压缩两组夹具使用 Depth−RGB=-1ns（跨秒）及 +2ns。门限 0/1/2 验证严格拒绝、边界包含和超差拒绝，检查原 stamp/策略/门限/exact/正负号。正门限保留真实相等对；无对状态 no_pairs/问题 no_rgbd_pairs 与严格模式区分，预算仍生效。正门限 YAML/C++ 实际接受，输出由独立 Python 从原始 CDR 回查。
