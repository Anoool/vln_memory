# vln_memory.yaml

[实际配置](../../config/vln_memory.yaml) 仅有本版 bag 段，未实现的 MemoryEngine/Viewer 不创建空配置。所有键严格检查，拼错/重复/负容量拒绝。

## 必填语义

input/output 为路径，可由 CLI 覆盖。clock_relation=same_clock、clock_id/clock_basis、segment_id/segment_basis 表示用户的采集声明；程序不会根据时间相近证明硬件同步。图像时间回退超过 max_header_reorder_ns 终止，多 epoch 应先明确分段。

rgb_camera/depth_camera 各自填写 image_topic/camera_info_topic/optical_frame/binding_basis/image_domain/time_policy/max_delta_ns；static_calibration 必须证明段内固定，扫描还会核对模型是否变化。

depth 明确 input_unit=meters/millimeters、convention=optical_z；alignment=unknown/not_aligned 不填目标网格；aligned_to_rgb 必须有 alignment_basis、rgb_width/rgb_height/rgb_frame，且与真实配对图像一致。

## 独立阈值与上限

sync_tolerance_ns 接受非负 int64 整数纳秒，表示允许的最大绝对时差（含边界）。0 仅接受原始 header 相等；正值按 RGB 基准、单调一对一最近候选选择，等距取较早 Depth，同刻取较小来源顺序。两路 stamp 不变，delta_ns=Depth−RGB。不是全局最多配对或最小总误差算法。

运行 `bag_builder --sync-tolerance-ms 30` 可将本次门限显式设为 30000000 ns。CLI 接受非负整数毫秒并检查纳秒范围，原 YAML 保持原值。BagBuildOptions 默认值和 YAML 省略值均为 30000000（30 ms）。示例显式设置相同门限。需要按 15 Hz 的一帧周期配对时，可设为 66666667。程序保留每对的原始时间差，供后续几何验收。负数、小数或超出 int64 的数在加载时拒绝。

标定 max_delta_ns 是图像与标定差。tf_max_gap_ns 控制 TF 相邻端点最大间隔，省略时在首遍扫描后按 map 到相机和底盘所需链的动态样本估计。每条样本数至少为 3 的动态边取相邻间隔下中位数的两倍，再取最大值。没有可用估计时使用 0。显式非负纳秒值优先，0 只接受精确动态样本。manifest 记录实际门限与 tf_gap_policy。该门限反映采样周期，多速率链共享门限。

可选 tf_max_translation_step_m 和 tf_max_rotation_step_rad 分别限制插值端点之间的平移量和最短旋转角，值须有限且大于零。两图标定和 TF 各用本图原始时刻；没有运动补偿或设备时钟校正。

min_valid_depth_ratio 在 [0,1]，全无效深度始终失败。limits 各项必须正数；max_tf_entries 限制原始 TF 条目（含重复），max_tf_samples 限制索引唯一样本，max_output_bytes 计原始 CDR、标准图像和文本总字节。

bag.depth_storage 可选 metric_npy 或 source_png16。默认 metric_npy 保留米制 NPY。source_png16 只将 16UC1 保存为无损源整数 PNG，32FC1 继续写米制 NPY。录包示例显式启用 PNG。读取时按逐帧存储描述处理，见 [文件契约](../interfaces/bag_builder_api.md)。

## 格式与使用

目前仅未压缩 SQLite ROS 2 bag；支持原 Image 及白名单压缩图像，详见 [压缩适配](../modules/bag/image_decoder_ros.md)。MCAP、ROS 1 和自动跨段处理不支持。示例单位/时钟声明不是设备事实，使用前按真实录制配置填写；输出预算包含实际媒体、原始 CDR 和文本。

## 2026-11-03 相机兼容

rgb_camera/depth_camera 新增 compatibility：省略/API 默认 strict；显式 plumb_bob_zero_tail_d8_v1 用于当前真实 bag 的有限零尾 D8。未知值拒绝；共享 CameraInfo 话题不能配置不同兼容策略。

image_domain 必须根据实际生产链声明 raw/rectified；JPEG/PNG 是编码，alignment 是跨流注册，两者都不能决定图像域。静态标定复用仍须显式声明并由索引核查全段一致；不会为修复一张缺同刻 CameraInfo 的图像自动扩大时间门限。

实际真实运行配置和结果见 [相机模型验收](../reviews/camera_model_verification.md)。

## CDR 保存策略

bag.save_raw_cdr 接受未加引号的 true/false，API 与 YAML 省略默认 false。v7 在 effective_options 中记录实际值。false 时 messages 和嵌套 source 的 raw_path 为 null，true 时保存原始 CDR。输入反序列化字节预算及来源摘要计算继续生效，输出预算累计实际生成的文件。使用说明见 [精简转换](../modules/bag/preparation_compactor.md)。
