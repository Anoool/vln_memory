# BagBuilder 第一版实施记录

日期：2026-10-30。依据 [整合方案](rosbag_integration_plan.md) 与 [AGENTS.md](../../AGENTS.md) 实施。

## 编码前框架与边界

1. 保存现有暂存内容为 Git 基线 d746af1，在 codex/bag-builder-v1 分支逐步提交；不合并 main。
2. 先实现独立 rgbd_sync：只接收拥有型时间引用；排序、二分、单调一对一选择；复用 sensor_time。错误无半份报告，空流可有完整报告。
3. 再实现配置/来源、CameraInfo 候选索引、官方 ROS reader、媒体/JSONL、薄 buildBag/CLI。每次落地真实行为，不创建占位类。
4. 第一版只支持一个显式声明的 clock/segment，配置需提供依据；图像时间回退超过明确的重排限额时终止，不推断重置后继续配对。多时钟段自动划分不在本版。
5. 未压缩 SQLite 目录（含 metadata 分卷）及单独 .db3；绑定话题类型必须核对；提取全部配置图像，未配对也保存。
6. 两遍顺序读取：第一遍完整反序列化但只保留元数据/标定/TF，第二遍转换落盘；数值位姿、有效投影模型和关键帧继续留在 S2，不伪造正式 Observation。
7. 所有新增函数含函数级注释；接口使用扁平字段和少量有意义的公共类型；函数按校验、检索、转换、保存分拆。

## 验证计划

配对已知答案/边界/参考算法；配置错误和标定歧义；真实 rosbag2 Writer 生成的 SQLite bag（分卷、负记录时间、同刻重复、坏帧、迟到 TF）；图像/NPY 独立回读；输出引用/失败发布/重复运行；全部现有核心及 ROS 测试、安装消费者。

构造 bag 验证与真实采集验收分开陈述。目前未提供真实设备 bag。

## 复杂度目标

配对排序 O(R log R + D log D)，每 RGB 二分候选 O(log D)；候选选择不逐图全扫。标定索引一次排序，逐图二分；TF 复用现有按 child/时间索引，不逐帧生成整段覆盖。大图只保留当前处理所需临时缓冲，所有输出和索引设置预算，写入失败不发布完整 manifest。

## 实施结果

各阶段完成后补充实际接口、验证结果和提交，不预先写成已完成。

## 首次完整回归

严格编译 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror：ROS 构建 14/14 CTest 通过；包括 8 个核心、4 个真实消息适配、磁盘 bag 构造及独立 Python/NumPy。配对核心版本 28c47a8，既有代码/规划基线 d746af1。

发现并修复 Humble Writer 可写负起点但 MetadataIo 用 uint64 解码失败：仅通过官方 metadata 注入点处理内存副本并恢复有符号起点，输入文件不变；结合 seek 最小记录时间避免静默漏帧。文件身份查表缓存分卷切换，不逐消息做 canonical 文件系统调用。

后续进行整条调用链自审、安装消费、最终文档/格式及必要边界补充，结果在本文件继续追加。

## 整体框架复核与改进

完整读取闭环已保存为 87f915c。继续通读配置→索引→选择→转换→JSONL→发布：

- 提前拒绝注册目标与两相机 optical_frame 矛盾，以及未注册深度共用 RGB 标定；尺寸解析拒绝负数/溢出。
- 单个含 NaN 的坏标定不误报为“静态模型变化”；添加对应核心回归。
- 坏 CDR 没有可用 header，导出 metadata_available=false/stamp=null；坏帧及全无效数值深度独立进入 pending。
- 图像转换、记录组装和问题上报拆成短函数；保留 source_data_bytes 方便人工检查布局。
- 分卷路径仅在切换时解析；完整源文件 SHA 从五遍减为三遍，第二遍仍逐条核对来源/摘要，最终仍做完整校验。
- 发布使用 Linux 原子不覆盖 renameat2，关闭后的完整目录才对外可见；这不承诺断电持久化或断点恢复。
- manifest 保存所有有效预算/阈值。依赖键按照 [ROS 官方 rosdep 数据](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) 使用 nlohmann-json-dev（CMake 包仍为 nlohmann_json）；未初始化本机 rosdep 全局数据库。

后续安装/内存检查完成后追加最终证据。

安装后实际运行发现：仅调用 rgbd_sync 的独立核心消费者，其可执行文件 RUNPATH 不能自动用于第二层 sensor_time 依赖。已为 Linux 安装共享库设置 $ORIGIN，使库自身解析同目录依赖；重新安装后需运行全部消费者验证，不仅编译。

## 最终验证

无 ROS 核心 8/8，标准 colcon 测试 14 项零失败/跳过；安装后独立核心 6 个、ROS 10 个消费者实际运行。Linux $ORIGIN 修复已通过不加载本包环境的核心消费者验证；已安装 CLI 和包外 buildBag 均实际导出合成分卷样例。

配对/标定索引 ASan/UBSan + LeakSanitizer 通过；完整 ROS 链 ASan/UBSan（detect_leaks=0）通过。ROS detect_leaks=1 有 389 字节 class_loader 退出诊断，独立官方 SequentialReader（不链接本项目）同样复现，未掩盖为通过。详见 [验收记录](../reviews/bag_builder_v1_verification.md)。

本地 review 样例在 build/bag-v1-demo/，实际 CLI 结果 4 RGB/3 Depth、6 成功媒体、2 对 ±4ms 配对、2 对准备通过、3 条问题；数值位姿/有效模型/关键帧仍为 false。原始设计附件未修改。

收口检查：CLI/API 的 28 个实际产物逐字节一致，304 个仓库文档链接有效，AGENTS.md 399 词。最终源码/接口按整条调用路径复核，生产函数分离校验、检索、转换、报告与发布，新增 C++ 风格检查通过。
