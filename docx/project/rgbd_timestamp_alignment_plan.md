# RGB–Depth 时间戳对齐修复规划

> 第 1–4 节记录编码前核查与待选路线；用户已选择最近配对，实施契约与结果见第 5–6 节。

日期：2026-11-02。当前请求：使 RGB 与 Depth 的时间戳尽可能相等。此前明确要求零容差，因此本轮先确定“采集端改善”或“允许离线近似配对”的规则，再修改相应实现。

## 1. 实际代码与环境核查

| 文件或环境 | 当前情况 |
| --- | --- |
| include/vln_memory/bag/builder/rgbd_sync.hpp | pairRgbDepth 已支持同一时钟段内、单调一对一的最近候选；保留两个原始时间戳，明确不是全局最优算法 |
| src/bag/builder/rgbd_sync.cpp | 排序后每条 RGB 二分查找；等距取较早 Depth，同刻取较小来源顺序 |
| include/vln_memory/bag/builder/bag_config.hpp | sync_tolerance_ns 默认 0，注释约定 BagBuilder 仅允许零容差 |
| src/bag/builder/bag_config.cpp | YAML 和 C++ 配置校验拒绝非零容差 |
| src/bag/builder/bag_builder.cpp | synchronize 强制零容差，并检查严格不变量；配对、summary、manifest 写 exact 策略 |
| config/vln_memory.yaml | sync_tolerance_ns: 0 |
| build/rgbd-strict-20261030/audit_records/last_summary.json | 历史真实 bag：RGB 3474、Depth 3415、解码 6889、严格配对 0；不是本轮重新提取 |
| vln_memory / vln_agent | 没有现成的 RealSense 采集启动文件 |
| 当前 WSL | /opt/ros/humble/share 未发现 RealSense 包；/dev 未发现 video 设备，不能声称完成相机实测 |

原图及提取数据此前已清理。本轮没有重新解压或生成大体积图像结果。

## 2. 两种目标与实施边界

### 路线 A：保留零容差，改善后续录制

输入：录制机实际驱动版本、设备支持的 RGB/Depth profile、设备与 TF 时间域依据。

1. 在录制入口启用受实际版本支持的 enable_sync，选择相同且受设备支持的帧率，检查驱动最终生效 profile。
2. 需要按 RGB 像素使用深度时，独立启用并检查 align_depth.enable，绑定 aligned_depth_to_color 的图像和 CameraInfo。
3. 保存参数、版本、color/depth metadata 和原始设备时间。enable_sync 收集最近帧并使用共同 ROS 标签，标签相等不证明曝光同时。
4. 先短录制，再以现有 BagBuilder 的零容差接口验收；保留未配对记录和两个原始时间戳。
5. 当前环境无设备，配置检查/构造测试与真实设备验收分别记录。

输出：新录制配置、采集说明、短 bag 的严格匹配结果与设备时间证据。旧 bag 的采集时差不能通过此路线修复。

### 路线 B：允许离线最近时间配对

仅在用户确认调整此前零容差要求后实施。

输入：明确、非负、有限的 sync_tolerance_ns；同 clock/segment 及其依据。默认值仍为 0，非零阈值必须显式配置，不能根据单个 bag 静默扩大。

1. 复用 pairRgbDepth，不增加第二套同步器；首先采用已有 RGB 基准、单调一对一最近候选规则。
2. 修改 YAML/C++ 校验及 synchronize，将明确配置传入既有核心。所有输出检查实际绝对差值不超过门限；零配置继续检查完全相等。
3. summary、manifest、pair 使用与实际算法一致的策略和门限；非零差值必须 exact=false，不能继续统一写 exact_header_one_to_one_v1。
4. 原始 RGB/Depth header 不修改。需要统一索引时使用明确的 RGB 参考时间；两路图像及各自 CameraInfo/TF 仍按自己的采样时间处理。
5. 空配对和配对状态需区分严格与近似语义；capture_sync_status 保持 unknown，直到有采集证据。
6. 增加相等、1 ns 差、门限内外、重复帧、一对一、乱序、跨时钟段、原时间未改的回归；检查 CLI/JSONL 和安装后接口。
7. 先输出时间索引统计再决定是否需要全量媒体重跑，避免重复生成约 16 GB 数据。

输出：可选近似配对能力、原始时间与真实 delta_ns、明确策略/门限、未配对原因。不能保证每张图都有配对，也不能将已有贪心规则描述成全局时差最小。

## 3. 自审与验收

- 不用 bag 接收时间替代 header，不覆盖 Depth 时间，不按帧号硬配，不重复使用一张 Depth。
- 不根据最近邻中位数自动做固定时钟偏移；只有独立时钟校准依据才讨论派生校正。
- 时间匹配、物理曝光同步、空间注册分开验收。
- 复用现有模块与拥有型来源接口；不新增深层结构、重复配置或无必要依赖。
- 原匹配算法排序 O(R log R + D log D)，查询 O(R log D)，额外空间 O(R+D)；若以后要求全局最优，需另定匹配目标和资源预算。
- 选择路线后先补充具体改动清单，再逐步实现；源码同名说明、接口文档与文档导航同步更新，完成后通读调用链。
- Git 修改保留在 codex/rgbd-timestamp-alignment，合并由用户审阅后决定。

## 4. 官方依据

2026-11-02 查阅。当前上游行为不能证明旧 bag 使用了同一驱动版本。

- [RealSense ROS wrapper 参数](https://github.com/realsenseai/realsense-ros)：enable_sync、RGBD 与 align_depth.enable 的职责。
- [RealSense frame_callback 实现](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/src/base_realsense_node.cpp)：frameset 使用共同发布时刻；应另外保留各流原始设备时间证据。

当前状态：完成代码/环境核查与实施前规划；尚未改变严格配对规则，尚未进行新的真实 bag 或设备验收。

## 5. 用户选择后的实施拆分（2026-11-02）

用户已明确允许按最近时间配对，并保留原始时间戳和差值。采用路线 B。

1. 同步核心只新增 rgbdSyncPolicyName(max_delta_ns) 静态策略名接口，集中服务 builder/CLI；不改变 pairRgbDepth 的算法、输入结构与所有权。
2. BagBuildOptions 的默认容差及 YAML 省略值仍为 0；配置允许非负 int64。仓库示例显式填写 30000000 ns（30 ms）作为本次试用值，需按实际运动误差预算调整，程序不自动扩容。
3. synchronize 传递配置，验证报告的最大绝对时差未超门限；零模式保留严格不变量。pair 的 exact 按实际 delta 计算，policy 和 sync_tolerance_ns 在 pair/summary/manifest/CLI 中一致。
4. 正容差无配对返回 no_pairs/问题 no_rgbd_pairs；零模式保留 no_exact_pairs/no_exact_rgbd_pairs，空流仍为 empty_stream。manifest builder_version 升为 3，JSON schema 仍为 bag_preparation/1，消费者必须检查实际策略。
5. RGB stamp 是配对参考时刻；不新增可覆盖源时间的 aligned_stamp。标定和 TF 仍各用本图时间。近似配对允许进入传感器准备流程，但不证明数值几何、运动补偿或曝光同步。
6. 集成回归使用负 1 ns/正 2 ns 的原始和压缩图像，验证零门限、边界、超差、正负号、原时间/TF 查询、状态及序列化。独立 Python 按原始 CDR 再核算。
7. 真实输入先恢复约 990 MB 的源 DB，仅做全部时间戳审计；复用正式核心比较 0/5/10/20/30/40/50 ms。再抽取短片段运行正式 BagBuilder 验证媒体/JSONL，避免全量生成约 16 GB。临时源和短片段媒体验证后清理，保留小型审计记录。
8. 修改后运行当前 ROS 全套 CTest、安装下游验证、文档链接/差异检查。源码同名模块说明、接口/配置、历史零容差验收入口与导航同步更新。

自审：沿用已有核心/依赖/平局与一对一规则；没有时钟改写、全局最优或物理同步承诺。新策略名函数只返回静态字符串，不分配；实际图像仍逐帧解码。以上为编码前确定的接口和验证范围。

## 6. 实际实施与自审

已按第 5 节实现，完整结果见 [最近配对验收](../reviews/rgbd_nearest_pairing_verification.md)。15/15 ROS 回归与 10 个安装消费者通过；全部 6889 个原始图像 header 经正式安装核心和独立参考比较，30 ms 为 2671 对；一秒真实图像片段正式提取 25 张/7 对，CDR/header/像素/delta 无差异。

通读配置→核心→两图几何检查→JSONL→汇总/CLI：门限只在匹配选择处生效，原始 stamp/CameraInfo/TF 时间保留，近似输出未伪称 exact 或物理同步。默认 0 与示例显式 30 ms 区分清楚，历史零容差记录已标注当前规则变化。新增函数与测试已按职责拆分，算法复杂度/所有权保持原契约。

本轮未处理标定 D8、TF 数值求解与设备同步。临时大文件清理和最终文件检查记录在 build/rgbd-nearest-20261102，原始 ZIP 不变。
