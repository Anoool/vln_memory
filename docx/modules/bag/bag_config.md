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

## 最近时间配对（builder_version=3）

sync_tolerance_ns 为非负 int64 纳秒最大绝对时差。0 严格，正值显式开启最近候选；YAML 省略/C++ 默认仍为 0，负数/小数/溢出拒绝。示例显式 30000000（30 ms 试用值），不自动扩大门限。C++ 设置 options.sync_tolerance_ns = 30000000 后调用 buildBag(options)。

输出保留原始时间和真实差值；标定与 TF 阈值独立。配置不能证明曝光同步。实际效果见 [最近配对验收](../../reviews/rgbd_nearest_pairing_verification.md)。
