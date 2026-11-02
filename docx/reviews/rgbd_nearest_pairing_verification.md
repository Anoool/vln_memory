# RGB–Depth 最近时间配对验收

日期：2026-11-02。用户已允许最近时间配对，要求保留原始时间戳与真实差值。当前 manifest builder_version=3，schema 为 bag_preparation/1。本记录区分完整原始时间索引核查、短片段正式媒体提取和未验证的设备物理同步。

## 1. 实际修改及接口

| 文件 | 实际职责/变更 |
| --- | --- |
| include/vln_memory/bag/builder/bag_config.hpp；src/bag/builder/bag_config.cpp | sync_tolerance_ns 接受非负 int64；默认/省略 0，负/小数/溢出拒绝 |
| include/vln_memory/bag/builder/rgbd_sync.hpp；src/bag/builder/rgbd_sync.cpp | 新增静态策略名 rgbdSyncPolicyName；原 pairRgbDepth 算法/结构/所有权不变 |
| src/bag/builder/bag_builder.cpp | 传递门限，检查不变量；按真实差计算 exact，输出策略与门限，区分正门限空结果 |
| src/apps/bag_builder_main.cpp | CLI 与 manifest/summary/pair 使用同一策略接口与门限 |
| config/vln_memory.yaml | 显式试用 30000000 ns（30 ms），改 0 恢复严格 |
| test/bag/builder/test_bag_builder.cpp；test_bag_output.py；test_rgbd_sync.cpp | 原始/压缩图像的边界、正负差值、原 CDR 时间与 TF 查询、策略、守恒、配置和预算回归 |

完整参数/返回/所有权见 [接口文档](../interfaces/bag_builder_api.md)，配置见 [vln_memory](../config/vln_memory.md)，编码前拆分见 [规划](../project/rgbd_timestamp_alignment_plan.md)。

0 策略为 exact_header_one_to_one_v1；正值为 rgb_reference_monotonic_nearest_one_to_one_v1。每条 pair 保存 rgb_stamp、depth_stamp、delta_ns（Depth−RGB 十进制字符串）、exact、policy、sync_tolerance_ns。RGB 是索引参考，两图标定/TF 各用本图时间。capture_sync_status 保持 unknown。

## 2. 完整 bag 时间索引重新核查

只恢复 ZIP 中的源 DB，SHA-256 为 5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d，与此前真实录制一致。读取全部 RGB 3474、Depth 3415，共 6889 个原始 header；不解码全包媒体。

使用安装后的正式 pairRgbDepth 对以下门限逐个执行，再用独立 Python 全扫描贪心参考逐对比较。所有来源、delta、门限及一对一守恒一致；原始时间保留。输入的 same-clock 是沿用此前审计的诊断声明，不能据数值接近证明物理时钟关系。

| 最大时差 ms | 配对数 | RGB 匹配率 | 已配对绝对差中位数 ms | 已配对最大绝对差 ms |
| --- | --- | --- | --- | --- |
| 0 | 0 | 0.00% | — | — |
| 5 | 45 | 1.30% | 2.396928 | 4.904960 |
| 10 | 105 | 3.02% | 5.223936 | 9.948928 |
| 20 | 738 | 21.24% | 16.745984 | 19.997952 |
| 30 | 2671 | 76.89% | 25.120000 | 29.979136 |
| 40 | 3230 | 92.98% | 26.140160 | 39.913984 |
| 50 | 3271 | 94.16% | 26.253056 | 48.124160 |

全部已配对记录 exact=false，因为原始时间戳交集仍为 0。30 ms 得 2671 对、未配对 RGB 803、未用 Depth 744。40/50 ms 会增加允许的误差；程序没有自动使用更宽门限。此表不是全局最优匹配结果，而是与既有 RGB 基准、单调一对一最近候选规则一致的结果。

选择门限需要运动误差预算。例如平移误差量级约为速度乘时间差；角运动还会放大远处像素错位。30 ms 仅是本次显式试用值，不构成设备/几何精度验收。

## 3. 一秒真实图像片段正式提取

从原 DB 按最早图像 header 起 1 秒选取图像与 CameraInfo，保留全部 TF 消息帮助历史查询。子集保留原始 SQLite 消息 ID、record time 与 CDR，未修改 header；使用单 .db3 作为输入，明确不是完整目录 bag 或完整时段 TF 的独立录制。

安装后的 BagBuilder 实际结果：

| 项目 | 结果 |
| --- | --- |
| 选定消息 | 7335，含全部 TF、26 条 CameraInfo、25 条图像 |
| RGB / Depth / 成功解码 | 13 / 12 / 25 |
| 30 ms 最近配对 | 7，严格相等 0 |
| 最大绝对时差 | 28.086784 ms |
| 提取状态 / 时间匹配 | completed_with_issues / partial |
| prepared_pairs | 0 |
| 提取耗时 / 最大 RSS | 1.13 s / 89668 KiB |
| 独立核查 | 7335 条 CDR 与原始 DB/子集一致；25 张原始 header 和 RGB/Depth 像素一致；7 对 delta 一致，错配引用/重复使用/超门限均为 0 |

独立图像核查复用上一轮的 JPEG/uint16 PNG 解码、PNG/NPY 比较。Depth 一致性仅证明按显式毫米配置转换正确，真实物理尺度仍待设备证据确认。

第一次直接运行安装前缀 CLI 时，因未设置该前缀的动态库搜索环境而未启动；补齐 LD_LIBRARY_PATH 后重跑成功。这不是解码或配对异常。正式运行无解码错误、时间越界或时间改写。

## 4. 保留的问题

- invalid_distortion：26 条。原 CameraInfo 的 plumb_bob/D8 仍被严格核心拒绝，本轮未做 D8→D5 兼容。
- pair_pending：7 条。两侧标定不可用、当前 tf_max_gap_ns=0 下两图及 body 没有精确动态 TF 样本、注册依据 unknown；每对均明确记录原因。
- outside_tolerance：6 条 RGB。30 ms 内没有所选 Depth，保留原始媒体/未配对原因。
- 物理曝光同步、深度尺度、空间注册、数值位姿、运动补偿均未因近似配对自动完成。

7 对真实匹配不等于 7 个完整可用 Observation；全包 2671 是时间索引配对数，不是本轮生成了 2671 对完整媒体。

## 5. 构造回归与下游验证

- ROS 全套 CTest：15/15 通过，编译沿用 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror。
- 原始/压缩夹具分别使用 -1 ns 跨秒及 +2 ns，门限 0/1/2 验收严格、边界和超差；正门限下恰好相等仍 exact=true。
- 正门限空结果为 no_pairs/no_rgbd_pairs；零配置兼容 no_exact_pairs/no_exact_rgbd_pairs。
- 独立 Python 从原始 CDR 按端序读取 stamp，验证 frames/pairs 与各图 TF 查询时间一致；计数、真实差值与策略一致。
- 10 个安装后下游消费者编译/运行通过；全包时间审计本身也只链接安装后的核心公开接口。
- 600 组已有核心参考算法用例继续通过，配对算法没有改变。
- 最终再次 CTest 15/15 通过，351 处文档链接有效，git diff --check 通过；完整调用链复核无时间改写，旧匹配算法除新增策略名函数外逐字一致。

## 6. 生成数据与 Git 边界

原始 ZIP 未改。验证后清理本轮临时 DB、子集 DB 和短片段媒体，仅保留小型时间索引、统计、核查脚本/日志和 audit_records 快照；快照缺少媒体/原始 CDR，不能作为可消费 preparation 数据集。

本轮临时输入/媒体实际清理 1,057,971,021 字节；审计目录约 3 MB，整体 build 约 222 MB。实际验证目录为 build/rgbd-nearest-20261102（忽略，不入 Git）。源码、分类接口/模块文档与本验收记录保存在 codex/rgbd-timestamp-alignment，合并留待用户审阅。
