# 真实录包 CameraInfo 兼容与有效模型验收

日期：2026-11-03。分支 codex/camera-model-compatibility。文档基线 c1c7ab3，实施计划 28cf72a，实现提交 025d24d。原始录包来自用户提供的 camera_record.zip；本次恢复全部 DB/metadata 并运行完整 bag，未改原始 ZIP。

## 1. 修改与边界

统一 normalizeCameraInfo 将显式有限零尾 plumb_bob D8 派生为 D5，原始 D8/严格诊断/CDR 保留。copyRosCameraInfo 为 ROS/reader 共用字段映射；直接 ROS、无 ROS 匹配、索引和模型入口使用同一兼容规则。严格调用者不会因已有派生缓存而绕过策略。

模型库为实际网格生成 K/D/R/P、raw/rectified 和 frame，raw 用 OpenCV 逆畸变并正向核验残差；rectified 用 P 内参及 R 转置回原光学 frame。输出射线 z=1，不是单位向量。双目非零 Tx/Ty 的校正投影、需要变换的校正 ROI 当前明确不支持。既有配对算法和所有源采样时间不改。

builder_version=4，JSON 仍为 bag_preparation/1。每帧和每对的有效模型标志为实际结果；全局布尔表示至少一帧成功，camera_models 给完整数量。数值 TF 位姿、物理注册、关键帧和完整记忆业务不因此完成。

## 2. 原始输入与配置

- ZIP SHA-256：c06ac1e3beccfc62b1562a0c7e4d73bdb9536d4575c3ecebfb158191f54e6452。
- DB SHA-256：5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d。
- ZIP 解压检查成员路径、符号链接、CRC；DB 以只读方式使用，独立审计 SQLite integrity_check=ok。
- RGB 为 JPEG CompressedImage，Depth 为 16UC1 compressedDepth PNG；两路实际 header frame 均 camera_color_optical_frame。
- 显式 30 ms 最近配对、plumb_bob_zero_tail_d8_v1、static_calibration。此前全包审计发现每流一个固定模型，本次索引再次检查全段一致性；固定复用不改写 CameraInfo 时间。
- image_domain=raw 是本次沿用的显式生产链声明；frame/尺寸相同不证明空间注册，alignment 仍 unknown。深度按显式毫米转换，物理尺度/时钟与曝光同步仍需独立采集依据。
- 实际保存配置 build/camera-model-20261103/real_config.yaml；可移植录包配置为 [camera_record.yaml](../../config/camera_record.yaml)，用户替换路径后运行。

## 3. 完整真实 bag 结果

| 项目 | 结果 |
| --- | --- |
| 原始消息 | 21441 |
| 原始 CameraInfo / 成功零尾 D8→D5 | 7268 / 7268 |
| RGB / Depth / 成功解码 | 3474 / 3415 / 6889 |
| 实际相机模型 | 6889，全部 raw，保留实际图像网格 |
| 30 ms 配对 / 严格相等 | 2671 / 0 |
| 未配对 RGB / 未用 Depth | 803 / 744 |
| prepared_pairs | 0 |
| 总体状态 / 进程退出 | completed_with_issues / 0 |
| 耗时 / 最大 RSS | 14:53.99 / 117712 KiB，约 115 MiB |
| 数据集文件总字节 | 16404527947 |

验收时完整产物为 build/camera-model-20261103/full_output，约 16 GB。随后按用户要求于 2026-11-03 清理完整媒体和输入副本，保留小型摘要/日志/元数据快照，详见 [清理记录](build_cleanup_20261103.md)。这些生成数据不上传 GitHub，原始 ZIP 保留。

全部 2671 对已具有两图有效模型，但仍 pending。原因计数：

- body_tf_unavailable：2671。
- depth_rgb_alignment_unverified：2671。
- depth_tf_unavailable：2671。
- rgb_tf_unavailable：2671。

tf_max_gap_ns=0 仅接受精确动态样本；本轮不把放宽参数当作已经完成插值/数值位姿。注册仍 unknown。当前问题计数 pair_pending=2671、outside_tolerance=803；invalid_distortion 不再作为不可用标定 issue，原始严格诊断仍保存在 camera_info.jsonl。

## 4. 独立逐条核查

[审计源码](../../test/bag/builder/audit_camera_dataset.py) 从原 DB 构建按 topic/record time/CDR SHA 的多重集合，核查全部导出 CDR 的字节摘要及重复计数；反序列化原始 CameraInfo，比较全部原始字段、时间、D5 派生及来源。

对全部 6889 张图独立解码 JPEG/16UC1 PNG，逐像素比较导出 RGB PNG 和米制 float32 NPY（含 NaN）；用矩阵 A*K/A*P 独立求实际网格参数，核对模型和图像/标定引用；重算原始整数纳秒差、一对一配对和各图 TF 查询时间。CDR/header/参数/模型/像素/配对差异均为 0。

执行方式（加载 ROS Humble 环境）：

    python3 test/bag/builder/audit_camera_dataset.py --db <原始.db3> --dataset <完整输出> --report <新报告.json>

该脚本针对当前录包声明：完整单 DB、raw、毫米 16UC1、plumb_bob 零尾 D8；不称为任意设备的通用物理验收。

## 5. 最终安装版本真实子集复验

完整提取后，收口直接调用的转换标志和共用 ROS 字段映射，并用最终安装版 CLI 再跑同源一秒子集。子集保留原始 SQLite ID、record time/CDR，图像/CameraInfo 为一秒窗口，全部 TF 保留；不是新的独立完整录制。

7335 消息、26 条原始/派生标定、25 张图像及 25 个模型、7 对，completed_with_issues，退出 0；同一独立审计检查全部子集 CDR、header、模型、媒体和配对差异为 0。剩余问题仍为 7 条 pair_pending、6 条 outside_tolerance。产物 final_short_output 随后按用户要求清理，保留小型审计快照。

## 6. 构造回归与安装接口

- 严格编译 -Wall/-Wextra/-Wpedantic/-Werror/-Wconversion/-Wsign-conversion。
- ROS CTest 16/16、无 ROS CTest 9/9 通过。
- ROS 安装消费者 11/11、无 ROS 安装消费者 7/7 编译运行通过，包含独立模型 API。
- 测试覆盖严格/显式 D8、非零尾/非有限/未知策略、原值保留、直接 ROS/匹配/索引策略一致、缓存不能绕过严格模式。
- 数值测试使用 K != P、非单位 R、ROI/binning 独立答案、三种非零畸变的独立正向公式与逆解、越界/坏模型/未支持投影。
- 实际序列化 SQLite 回归覆盖 D8 读取→派生→选择→模型→配对→JSON/CLI，坏候选与半成品发布规则保持。
- 安装 CLI 的第一次子集启动因外层 shell 覆盖了动态库环境而未启动；明确安装 lib 和 /opt/ros/humble/lib 后成功。所有实际提取/审计的最终退出状态为 0。

文档检查：73 份 Markdown、391 处本地链接/章节引用通过；git diff --check 通过。

## 7. 后续与 Git

按用户要求，preparation 的读取/引用/筛选放在后续 API 输入组织阶段，不先实现最小 Observation。下一步为 TF 数值位姿及深度几何/注册依据，随后再做记忆工作库、工具、构建与 Viewer。完整原设计第 1、2 部分仍未完成。

计划、实现、验收文档分别提交；公开仓库 [Anoool/vln_memory](https://github.com/Anoool/vln_memory) 的功能分支供审阅，main 不自动合并。录包、生成媒体/日志和凭据不入 Git。
