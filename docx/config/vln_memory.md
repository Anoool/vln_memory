# vln_memory.yaml

[实际配置](../../config/vln_memory.yaml) 仅有本版 bag 段，未实现的 MemoryEngine/Viewer 不创建空配置。所有键严格检查，拼错/重复/负容量拒绝。

## 必填语义

input/output 为路径，可由 CLI 覆盖。clock_relation=same_clock、clock_id/clock_basis、segment_id/segment_basis 表示用户的采集声明；程序不会根据时间相近证明硬件同步。图像时间回退超过 max_header_reorder_ns 终止，多 epoch 应先明确分段。

rgb_camera/depth_camera 各自填写 image_topic/camera_info_topic/optical_frame/binding_basis/image_domain/time_policy/max_delta_ns；static_calibration 必须证明段内固定，扫描还会核对模型是否变化。

depth 明确 input_unit=meters/millimeters、convention=optical_z；alignment=unknown/not_aligned 不填目标网格；aligned_to_rgb 必须有 alignment_basis、rgb_width/rgb_height/rgb_frame，且与真实配对图像一致。

## 独立阈值与上限

sync_tolerance_ns 接受非负 int64 整数纳秒，表示允许的最大绝对时差（含边界）。0 仅接受原始 header 相等；正值按 RGB 基准、单调一对一最近候选选择，等距取较早 Depth，同刻取较小来源顺序。两路 stamp 不变，delta_ns=Depth−RGB。不是全局最多配对或最小总误差算法。

BagBuildOptions 默认值和 YAML 省略值仍为 0。示例显式设置 sync_tolerance_ns: 30000000（30 ms）作为当前试用值，需按实际运动误差预算调整，程序不自动扩容。负数、小数或超出 int64 的数在加载时拒绝。

标定 max_delta_ns 是图像与标定差。tf_max_gap_ns 控制 TF 相邻端点最大间隔，省略时在首遍扫描后按 map 到相机和底盘所需链的动态样本估计。每条样本数至少为 3 的动态边取相邻间隔下中位数的两倍，再取最大值。没有可用估计时使用 0。显式非负纳秒值优先，0 只接受精确动态样本。manifest 记录实际门限与 tf_gap_policy。该门限反映采样周期，多速率链共享门限。

可选 tf_max_translation_step_m 和 tf_max_rotation_step_rad 分别限制插值端点之间的平移量和最短旋转角，值须有限且大于零。两图标定和 TF 各用本图原始时刻；没有运动补偿或设备时钟校正。

min_valid_depth_ratio 在 [0,1]，全无效深度始终失败。limits 各项必须正数；max_tf_entries 限制原始 TF 条目（含重复），max_tf_samples 限制索引唯一样本，max_output_bytes 计原始 CDR、标准图像和文本总字节。

## 格式与使用

目前仅未压缩 SQLite ROS 2 bag；支持原 Image 及白名单压缩图像，详见 [压缩适配](../modules/bag/image_decoder_ros.md)。MCAP、ROS 1 和自动跨段处理不支持。示例单位/时钟声明不是设备事实，使用前按真实录制配置填写；输出预算包含标准 float32 NPY，实际大小可能显著超过压缩 bag。

## 2026-11-03 相机兼容

rgb_camera/depth_camera 新增 compatibility：省略/API 默认 strict；显式 plumb_bob_zero_tail_d8_v1 用于当前真实 bag 的有限零尾 D8。未知值拒绝；共享 CameraInfo 话题不能配置不同兼容策略。

image_domain 必须根据实际生产链声明 raw/rectified；JPEG/PNG 是编码，alignment 是跨流注册，两者都不能决定图像域。静态标定复用仍须显式声明并由索引核查全段一致；不会为修复一张缺同刻 CameraInfo 的图像自动扩大时间门限。

实际真实运行配置和结果见 [相机模型验收](../reviews/camera_model_verification.md)。
