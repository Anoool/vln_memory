# bag_config.hpp / bag_config.cpp

## 用途

[头文件](../../../include/vln_memory/bag/builder/bag_config.hpp) 定义 BagBuildOptions；[实现](../../../src/bag/builder/bag_config.cpp) 负责严格 YAML 加载和独立配置校验。ROS 版 builder 链接 yaml-cpp，原传感器核心不新增此依赖。

## 接口

loadBagBuildOptions(string yaml_path) → 拥有型 BagBuildOptions；未知/重复字段、错误标量或不支持枚举抛 BagBuildError(invalid_config)。路径可在 CLI 覆盖，最终必须调用 validateBagBuildOptions 或 buildBag。

validateBagBuildOptions(const BagBuildOptions&) → vector<string> 全部诊断；无文件访问，不改输入。检查话题/frame、同钟及分段依据、独立非负时间阈值、深度语义、注册声明与资源约束。文件完整性、实际类型及输出路径由 reader/buildBag 再检查。

## 参数

路径 bag_uri/output_directory；单段 clock_id/clock_basis/segment_id/segment_basis/clock_relation；map_frame/body_frame；sync_tolerance_ns/tf_max_gap_ns/max_header_reorder_ns。rgb_camera/depth_camera 直接复用 CameraInfoMatchOptions，depth 复用 DepthOptions；不添加多层绑定包装。

max_messages/max_frames/max_calibrations/max_tf_entries/max_serialized_bytes/max_output_bytes，以及 RGB/Depth 核心和 TF 唯一样本上限分别生效。上限不是恶意 CDR 反序列化的绝对内存沙箱。

## 使用

配置示例见 [vln_memory.yaml](../../config/vln_memory.md)。固定标定策略需要段内不变依据；近似同步容差必须按实际采集填写。原始/已注册深度要选正确的 CameraInfo，不能从编码猜毫米或从宽高猜注册。

额外配置约束：尺寸按非负十进制解析再检查 uint32 范围；已注册声明必须与 RGB/Depth 两个光学 frame 一致；未注册 Depth 不允许静默共用 RGB CameraInfo。

## 零容差约束

sync_tolerance_ns 保留字段但仅允许 0，YAML 加载和 validateBagBuildOptions/buildBag 的 C++ 入口都拒绝非零；没有自动回退。CameraInfo max_delta_ns 和 tf_max_gap_ns 独立，不因此锁为零。配置只声明单位与时钟依据，不能由帧率或设备名猜测。
