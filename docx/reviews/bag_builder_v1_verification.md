# BagBuilder 第一版验收与人工检查入口

日期：2026-10-30。范围是未压缩 SQLite ROS 2 bag 的读取/转换/传感器准备，不是原设计第 1、2 部分全部完成，也不是数值几何/关键帧/完整 Observation。

## 实际结果

使用官方 rosbag2 Writer 和真实消息类型支持构造两个 SQLite 分卷，经实际安装后的 CLI 和包外 C++ API 提取。图像有行填充，包含负 bag 记录时间、同刻记录、迟到 TF、重复静态 TF、空 TFMessage、坏 CDR 和全无效深度。

| 项目 | 实际数量/结果 |
| --- | --- |
| 配置消息读取 | 13 条；另外 1 条未选择话题只进入 inventory |
| RGB / Depth | 4 / 3 条，其中 RGB 含 1 条坏 CDR |
| 标准媒体成功 | 6 帧，RGB PNG、Depth 米制 NPY |
| RGB-D 配对 | 2 对，原始 header 时差 +4ms / -4ms |
| 传感器准备通过 | 2 对；pose/effective_model 仍未计算 |
| 未配对 RGB / 未使用 Depth | 1 / 1；后者同时为全无效数值深度，两种原因分别记录 |
| 问题 | 3 条：坏 CDR、全无效深度质量、未配对 RGB |
| TF 原始条目 / 唯一样本 / 重复 | 4 / 3 / 1 |
| 状态 | completed_with_issues；输出完整，可检查每条问题 |

## 验证证据

- 严格编译：-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror。
- 无 ROS 核心 CTest：8/8，通过。
- 标准 colcon build/test/test-result：14 项，0 errors/0 failures/0 skipped。
- 安装后纯 C++ 消费者：6/6；ROS 安装消费者：10/10。新增配对、标定索引和完整 builder 导出目标均实际运行，builder 还用 YAML 提取了磁盘 bag。
- PNG 用 OpenCV 独立解码检查红/蓝通道。NPY 用 Python/NumPy 独立检查 shape、小端 float32、1/2/3 米及 NaN；JSON/raw SHA/引用校验通过。
- 同输入/同配置重复导出的所有文件逐字节一致；实际已安装 CLI 与包外 C++ API 的 28 个产物也逐字节一致。
- 回归覆盖：类型错误、注册配置矛盾、未知/重复/负容量及尺寸、同刻/静态标定冲突、TF 冲突、图像重启、旧输出拒绝、输出预算失败、跨遍输入改变和单文件范围。
- 新增 C++ 文件的 ament_uncrustify（120 列）检查通过；304 个仓库内文档链接及 git diff --check 检查通过。

这是构造磁盘包的工程验证，没有真实设备录包、物理标定/同步或 map 数值位姿验收。

## 内存检查的实际边界

配对和标定索引核心在 ASan/UBSan 下执行，开启 detect_leaks=1，通过。完整 ROS 提取链在 ASan/UBSan 下执行通过（该次 detect_leaks=0）。

开启完整 ROS LeakSanitizer 时，退出报告 389 字节/7 个分配，栈涉及 libclass_loader 和动态插件初始化。为确认归属，另建只调用官方 SequentialReader 打开/关闭一个 SQLite 文件的独立程序，不包含或链接 vln_memory；它复现完全相同的 389 字节/7 个分配。因此没有宣称“完整 ROS 泄漏检查全绿”，也没有向项目添加广泛泄漏抑制规则。

探针与原始日志保留在 build/bag-v1-lsan-baseline/；项目检查日志在 build/bag-v1-asan-*。检测同时发现并修复了本项目反序列化边界的 ROS 错误状态清理，回归检查运行后不残留全局错误状态。

## 人工先看哪里

- 实际配置：[vln_memory](../config/vln_memory.md)。
- 公开接口与 JSONL 格式：[BagBuilder API](../interfaces/bag_builder_api.md)。
- 调度/发布：[bag_builder](../modules/bag/bag_builder.md)；读取/来源：[bag_reader_ros](../modules/bag/bag_reader_ros.md)。
- 每个函数的参数、所有权、单位/失败在公开头和函数注释中说明。
- 实施与复杂度复核：[实施记录](../project/bag_builder_v1_implementation.md)。

本地已生成可检查样例 build/bag-v1-demo/：input_bag/、demo_config.yaml、dataset/（实际 CLI 产物）、consumer_dataset/（实际包外 API 产物）。先打开 dataset/bag_manifest.json 和 reports/summary.json，再按 frames/pairs/标定/TF 引用查看 evidence。样例位于 Git 忽略目录。

## Git 版本

分支 codex/bag-builder-v1；既有工作快照 d746af1，配对核心 28c47a8，首个完整读取闭环 87f915c。审查修复与最终验证单独提交，稳定入口为 tag bag-builder-v1；未合并 main、未推送。

比较命令：git diff d746af1..bag-builder-v1 --stat；查看提交：git log --oneline --decorate。
