# bag_builder.hpp / bag_builder.cpp

## 用途与外部接口

[头](../../../include/vln_memory/bag/builder/bag_builder.hpp)、[实现](../../../src/bag/builder/bag_builder.cpp) 串联整个第一版；主接口为 BagBuildResult buildBag(const BagBuildOptions&)。

输入必须声明一个同钟数据段、话题/光学 frame、深度单位/注册依据及独立时间阈值。返回状态/错误/产物路径与数量，不返回整包像素。ok() 表示完整发布，可有 issues；prepared_pairs 只是传感器准备通过数量，不是已计算 map 几何。

## 调用路径

配置检查 → 创建唯一工作目录 → 官方 reader 首遍索引/原始 CDR → rgbd_sync → CameraInfoIndex → 第二遍逐图 decodeRosRgb/decodeRosDepth、实际 RGB 网格核对、历史 TF 查询 → 媒体/JSONL → 引用/输入复核 → 完整 manifest/发布。

采样时间保留；RGB/Depth 分别查询自己时刻的 TF；body 在 RGB 时刻查询。CameraInfo 选择复用统一兼容/匹配规则并生成实际模型；现有 TF 索引提供数值 map 位姿和样本引用，关键帧及正式 observations.jsonl 待实现。

## 失败和产物

输入与输出路径不得包含彼此，旧输出不覆盖。只在新建 .partial 工作目录中写入；失败返回 work_directory，正常输出路径不发布；成功清空 work_directory。发布保证完整目录的原子可见性，不是断电持久化事务。

坏图像保留 raw/问题；全部无效深度不能准备通过；标定/TF/注册缺项使配对 pending。未使用 Depth 记录为 not_selected_by_policy，不单凭此当坏帧。不可反序列化 TF/标定、容量不足、存储错误或源变化属于整次失败。

输出集合、参数格式和示例见 [接口文档](../../interfaces/bag_builder_api.md)；命令见 [CLI](bag_builder_main.md)，配置见 [配置说明](../../config/vln_memory.md)。测试见 [集成验证](tests/test_bag_builder.md)。

审查更新：坏帧及数值质量不通过会有独立 pending 行；文件发布用 Linux renameat2(RENAME_NOREPLACE)，即使并发出现空目录也拒绝覆盖。numeric_quality_passed 不表示标定/TF/物理注册已经通过。

## 可配置最近配对（builder_version=3）

synchronize 将 sync_tolerance_ns 传入已有 pairRgbDepth，检查最大绝对差不超过门限；0 模式额外检查严格不变量。writePairs 按 delta_ns==0 填 exact，并保存 policy、sync_tolerance_ns、两个原始 stamp。RGB 是索引参考，两图标定/TF 分别查询自身时刻。

0 策略为 exact_header_one_to_one_v1，正门限为 rgb_reference_monotonic_nearest_one_to_one_v1；后者不保证全局最优，实际相等的一对仍 exact=true。summary/manifest/CLI 共用策略名接口，JSON schema 仍为 bag_preparation/1。

empty_stream/partial/all_paired 仅说明有效时间索引覆盖；无对时零模式为 no_exact_pairs/问题 no_exact_rgbd_pairs，正门限为 no_pairs/问题 no_rgbd_pairs。capture_sync_status 保持 unknown。prepared_for_geometry 要求数值位姿及既有模型、注册声明和深度质量检查通过。运动补偿与曝光同步属于后续独立验证。

两种图像均逐帧转换，坏帧不改配，未配对媒体仍保存。当前与历史结果见 [最近配对](../../reviews/rgbd_nearest_pairing_verification.md)、[历史零容差](../../reviews/rgbd_zero_tolerance_verification.md)。
