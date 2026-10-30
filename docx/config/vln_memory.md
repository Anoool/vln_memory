# vln_memory.yaml

[实际配置](../../config/vln_memory.yaml) 仅有本版 bag 段，未实现的 MemoryEngine/Viewer 不创建空配置。所有键严格检查，拼错/重复/负容量拒绝。

## 必填语义

input/output 为路径，可由 CLI 覆盖。clock_relation=same_clock、clock_id/clock_basis、segment_id/segment_basis 表示用户的采集声明；程序不会根据时间相近证明硬件同步。图像时间回退超过 max_header_reorder_ns 终止，多 epoch 应先明确分段。

rgb_camera/depth_camera 各自填写 image_topic/camera_info_topic/optical_frame/binding_basis/image_domain/time_policy/max_delta_ns；static_calibration 必须证明段内固定，扫描还会核对模型是否变化。

depth 明确 input_unit=meters/millimeters、convention=optical_z；alignment=unknown/not_aligned 不填目标网格；aligned_to_rgb 必须有 alignment_basis、rgb_width/rgb_height/rgb_frame，且与真实配对图像一致。

## 独立阈值与上限

sync_tolerance_ns 是两图差，标定 max_delta_ns 是图像与标定差，tf_max_gap_ns 是 TF 相邻端点最大间隔，均为非负整数纳秒，0 表示严格/精确。不得因匹配率低自动放宽。

min_valid_depth_ratio 在 [0,1]，全无效深度始终失败。limits 各项必须正数；max_tf_entries 限制原始 TF 条目（含重复），max_tf_samples 限制索引唯一样本，max_output_bytes 计原始 CDR、标准图像和文本总字节。

## 格式与使用

目前仅未压缩 SQLite ROS 2 bag；CompressedImage、MCAP、ROS 1 和自动跨段处理不支持。示例单位/容差不是设备事实，使用前按真实录制配置填写。
