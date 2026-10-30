# camera_record：真实 rosbag 提取与问题分析
> 历史记录：本文针对 e14efd2 第一版。用户随后要求清理旧提取数据并重跑；旧 build/camera-record-audit-20261030 已删除，输入保留至新目录。当前修复与新证据见 [零容差验收](rgbd_zero_tolerance_verification.md)，本文旧日志路径不再作为现存文件入口。

日期：2026-10-30。设备由用户确认为 Intel RealSense D435i。核查基线为 `e14efd2` / `bag-builder-v1`，本报告保存在 `codex/camera-record-audit`。本轮只做实际提取、核查和修复方案，没有修改产品源码、原 bag 或正式配置。

## 1. 结论与验收边界

**现有 BagBuilder v1 不能直接处理这份 bag：输入是压缩图像，v1 只接受 Image。** 实际 CLI 返回 `topic_type_mismatch`、退出码 1，未发布完整数据集。`decoded_frames=0` 和 `issue_count=0` 是预检阶段失败的结果，不代表 bag 没有图像或运行成功。

为继续核查数据，另用只读诊断工具提取全部 21,441 条消息，并用已安装的实际 C++ 核心及官方 compressedDepth 解码器交叉验证。3,474 张 RGB、3,415 张 Depth 均可解码，6,889 帧的导出文件摘要、归一化 RGB 像素、原始 Depth 像素交叉检查全部一致，独立提取错误记录为零。

诊断导出不等同于 BagBuilder 新增了压缩能力，也不是完整 Observation/memory。数值 map 位姿、关键帧、有效投影模型与物理标定准确度仍未完成验收。

## 2. 输入、完整性与话题清单

原文件：`C:/Users/咕咕噜哩/xwechat_files/wxid_kdxjaxm91nsr22_13af/msg/file/2026-10/camera_record.zip`。

| 项目 | 实测结果 |
| --- | --- |
| ZIP 大小 | 859,535,780 字节 |
| ZIP SHA-256 | `c06ac1e3beccfc62b1562a0c7e4d73bdb9536d4575c3ecebfb158191f54e6452` |
| 数据库大小 | 990,035,968 字节 |
| 数据库 SHA-256 | `5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d` |
| 存储 | ROS 2 bag / SQLite3 / CDR，无 bag 级压缩 |
| 记录时长 | 243.272663071 秒 |
| ZIP CRC / SQLite 完整性 | 通过 / `PRAGMA integrity_check = ok` |

| 话题 | 消息类型 | 数量 | 图像实际格式 |
| --- | --- | ---: | --- |
| `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 3,474 | `rgb8; jpeg compressed bgr8` |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 3,415 | `16UC1; compressedDepth`，内嵌 PNG |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 3,633 | — |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 3,635 | — |
| `/tf` | `tf2_msgs/msg/TFMessage` | 7,279 | — |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 5 | — |
| **合计** | | **21,441** | |

这份 bag 只有上述六个话题，未记录 IMU、相机驱动参数或设备元数据话题；不能仅根据 D435i 型号补造这些信息。

## 3. 实际执行过程与报错

1. 安全展开 ZIP：检查相对路径、目录逃逸、符号链接及 CRC；以只读方式打开 SQLite，核对实际 SQL 计数与元数据。
2. 用实际话题配置运行未修改的 v1 CLI。配置中的单位/同一时钟为诊断假设，未作为物理验收依据；本次在类型预检即终止，尚未进入这些语义检查。
3. 独立逐条反序列化。RGB 保存原 JPEG；Depth 保存原 compressedDepth 中的无损 uint16 PNG，另写来源与元数据 JSONL。没有将未知尺度强行写成已验证米制深度。
4. C++ 诊断程序使用官方 compressedDepth 解码器、cv_bridge，以及本项目真实 RGB/Depth/CameraInfo/TF/rgbd_sync 接口再检查一遍。
5. 对全部 6,889 帧比较文件 SHA、RGB 规范化像素 SHA、Depth 原始像素 SHA、尺寸和有效像素比例；检查配对、标定关联、TF 链与覆盖，并抽看六组代表画面。

| 阶段 | 实际结果 | 含义 |
| --- | --- | --- |
| 原 v1 CLI | 退出 1；`topic_type_mismatch` | 产品缺少压缩图像入口，不是 CDR/JPEG 损坏 |
| 独立提取 | 退出 0；21,441 消息；0 解码错误 | bag 内已有消息可读 |
| C++ 全量检查 | 退出 0；6,889 图像核心返回 `none` | 经诊断适配后现有图像核心可用 |
| 原始 CameraInfo 核心检查 | 7,268 条均 `invalid_distortion` | 标定元数据与当前严格模型规则不兼容 |
| 仅对副本规范化后的 CameraInfo 检查 | 7,268/7,268 通过 | 此特殊零尾项情形有明确兼容路径 |
| TF 插入 | 7,287/7,287 `inserted` | 无冲突、无非法条目、无重复拒绝 |
| 像素与导出交叉检查 | 6,889 帧，0 不一致 | 未发现本次提取引入的通道/数值/文件错误 |

诊断工具首次构建还出现过包前缀未指定、ament 链接关键字混用、整型模板参数推导不一致三项配置/编译错误；修正后严格警告编译和运行通过。这些是诊断工具构建问题，不计作 bag 解码错误。诊断 C++ 编译使用 `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror`。

已安装官方 `ros-humble-compressed-image-transport`、`ros-humble-compressed-depth-image-transport` 2.5.5 供核查；未因此修改本项目依赖声明。JPEG 本身是有损媒体，本次一致性比较针对 bag 内已有 JPEG 的解码结果。

## 4. RGB / Depth：数据可读，但时间未严格同步

全部图像为 1280×720；RGB 为 uint8，Depth 为 uint16。两种图像的全部 header.frame_id 都是 `camera_color_optical_frame`。

四个图像/CameraInfo 流均无重复 header 时间戳、无时间倒退。RGB 图像的 header 间隔中位数 66.608128 ms，Depth 为 66.879488 ms，接近 15 Hz；最大间隔分别为 269.626112 ms、332.210944 ms，存在明显缺帧或输出停顿区间。不能仅凭 bag 区分上游未发布、压缩处理、DDS 传输或 recorder 未收到。

逐张 RGB 独立找最近 Depth 时，绝对 header 时差中位数 **26.454016 ms**，P95 **39.0352 ms**，最大 **159.788032 ms**。没有严格同时间戳帧。这个最近邻统计允许不同 RGB 指向同一 Depth，仅用于描述时间分布。

下表才是实际调用 `pairRgbDepth()` 的**单调、一对一、RGB 锚定最近贪心策略**结果：

| 最大时差 | 配对数 | 未配对 RGB | 未使用 Depth |
| ---: | ---: | ---: | ---: |
| 0 ms | 0 | 3,474 | 3,415 |
| 5 ms | 45 | 3,429 | 3,370 |
| 10 ms | 105 | 3,369 | 3,310 |
| 20 ms | 738 | 2,736 | 2,677 |
| 30 ms | 2,671 | 803 | 744 |
| 40 ms | 3,230 | 244 | 185 |
| 50 ms | 3,271 | 203 | 144 |

因此不能把“40 ms 内最近邻有 3,330 张 RGB”写成“3,330 对一对一 RGB-D”。另一种按最早可行项匹配、优先保留数量的诊断算法在 40/50 ms 下得到 3,241/3,320 对，说明当前局部最近策略不承诺最大配对数量；这是目标选择的区别，不是本次解码错误。下一版应保留已说明的策略，不为增加数量而静默改变语义。

bag 记录时间减 header 时间：RGB 中位数约 44.29 ms，Depth 约 86.88 ms。这个差值也包括时间域/调度等因素，不能直接当成单项网络延迟。应使用确认在同一时间域内的图像 header 做配对；record 时间独立保留，用于来源和迟到诊断。不能用 record 时间覆盖 header，也不能把两图时间平均后冒充采集时间。

30 ms 和 40 ms 是本次测量的候选容差，不是通用合格标准。即使时间容差通过，两次曝光仍有时间差；运动场景应在后续几何层分别查询各自时刻的位姿，并做运动误差门限或时间补偿。新录制应核对驱动的 RGB/Depth 同步、frameset、时间域和帧率配置，并用实测 header 分布验收。

## 5. CameraInfo：D8 元数据、缺少同刻记录和注册语义

### 5.1 全部标定被拒绝的原因

两个流都写 `distortion_model=plumb_bob`，但 D 数组长度是 8。ROS CameraInfo 对该模型约定 5 项；当前 `src/bag/camera_info/camera_info.cpp::validateDistortion()` 精确检查长度，因此原始 7,268 条全部失败。见 [ROS 2 Humble CameraInfo 定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)。

本 bag 每条 D 的末 3 项都严格为 0。保留原始记录，在副本中只取前 5 项后，全部 7,268 条通过现有核心的畸变、矩阵、ROI/binning 与尺寸校验。每个流全程只有一个不随时间变化的几何模型，RGB/Depth 的 K/D/R/P 数值也相同。这证明可按明确特殊规则兼容；不能据此证明真实物理标定无误。

建议继续保留核心默认严格规则，在 ROS/Bag 适配边界增加显式的“plumb_bob 零尾 D8 转 D5”兼容策略：必须模型匹配、系数有限、长度恰为 8、尾 3 项严格为零。原始 CameraInfo JSON/CDR 保留；派生记录明确输出 `effective_d`、`normalization_applied`、策略版本及来源，标定索引消费派生模型。非零尾项或未知模型继续拒绝，不能盲目截断或擅自改模型名称。采集端若能正确输出 D5，可同时消除源头不一致。

### 5.2 图像与标定时间关联

| 流 | 同刻标定匹配图像 | 图像总数 | 只有标定、没有同刻图像的时间戳 |
| --- | ---: | ---: | ---: |
| RGB | 3,473 | 3,474 | 160 |
| Depth | 3,415 | 3,415 | 220 |

唯一没有同刻 RGB 标定的图像是数据库 row 6423，header 为 `1793283781739397888` ns，最近标定相差 66.508032 ms。不能因为 CameraInfo 比图像多就默认每张图都有同刻标定。

修复 D8 兼容后，本 bag 可在明确验证同一标定版本的前提下考虑现有 `static_calibration` 策略，复用全程不变的模型，保留其原始时间和引用；不能伪造同刻 CameraInfo。后续模型变化、重启或来源歧义必须重新划分适用范围。

### 5.3 是否已经对齐 RGB

Depth 图像和 Depth CameraInfo 使用 color optical frame，尺寸和标定又与 RGB 相同，**元数据与注册到 RGB 的情况相符**。仅凭这些字段和六组画面，仍不能验收真实空间注册，也不能确认驱动版本和 `align_depth` 参数。当前按 `alignment=unknown` 处理有依据；应补采集配置/参数证据，并用静态边缘、标定板或重投影检查确认，再声明 `aligned_to_rgb`。

时间配对、空间注册和深度尺度需要分别验收；设备型号只能提供背景，不能替代这些证据。

## 6. Depth：零值较多，原始值应保留

| 指标 | 实测结果 |
| --- | ---: |
| 总像素数 | 3,147,264,000 |
| 0 值像素 | 1,967,568,298，占 **62.5168%** |
| 正值像素比例 | **37.4832%** |
| 65535 饱和值 | 0 |
| 正值原始计数 min / P5 / median / P95 / max | 134 / 548 / 919 / 2,324 / 10,000 |
| 单帧正值比例 min / median / max | 12.7737% / 33.4847% / 75.6255% |

没有整张全零的深度帧。六组代表画面中，近处墙面正值覆盖较高，部分复杂室内画面的表面、远处和边缘有大面积空洞；不能仅凭当前图像判断空洞分别由距离、匹配失败、表面材质、注册或编码前裁剪中的哪项造成。

按 [REP 118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)，标准 16 位深度约定毫米、0 为无效。若本录制遵循此约定，正值中位数对应 0.919 m、P95 为 2.324 m，最大 10 m。实际 driver/depth scale 参数尚未独立确认，所以本次文件保留 raw counts，不将这个假设标成已验证尺度。

本次 compressedDepth 的 PNG 起始位置在 12 字节配置头之后；官方 C++ 解码结果与独立 PNG 解码原始像素一致。该头中的浮点参数用于特定编码，不能拿来当此 16UC1 的米制 scale；参考 [Humble 官方 codec](https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_depth_image_transport/src/codec.cpp)。未来 32FC1、RVL 或其他格式必须各自声明支持并测试，不能统一硬切 12 字节处理。

后续处理建议：保留 raw 值和无效掩码；在尺度有依据时转米制，0 转无效值；输出逐帧与目标区域的有效比例、深度分布和拒绝原因。当前全图比例门限不能替代目标选区质量检查。不能用自动补洞生成的值冒充原始测量，也不能从已丢失/裁剪的源数据恢复真实距离。

## 7. TF：链完整，覆盖门限和数值位姿仍需区分

7,284 条 TF 消息包含 7,287 个 transform，全部成功进入现有 `TfIndex`，共 11 个 frame、10 条边。实际相机到 map 的链是：

```text
camera_color_optical_frame
  -> camera_color_frame
  -> camera_depth_frame
  -> camera_link
  -> base_link
  -> odom
  -> map
```

上图表示查询链经过的 frame；原始父子定义为 `map -> odom -> base_link -> camera_link` 等，不应据此颠倒 TransformStamped 的数值方向。`map -> odom` 为静态单位变换；相机所在链使用动态 `odom -> base_link`，不是另一分支的 `camera_init -> body`。

`odom -> base_link` 有 4,853 样本，中位间隔 50.001124 ms；仅一个间隔超过 100 ms，最大 199.976632 ms。其覆盖区间为 `1793283709.022377246` 至 `1793283951.773046337`，覆盖全部图像 header 范围。`camera_init -> body` 另有 2,426 样本，不能因名称接近就与 base_link 的位姿混用。

| `tf_max_gap_ns` | RGB covered / 总数 | Depth covered / 总数 |
| ---: | ---: | ---: |
| 0 | 0 / 3,474 | 0 / 3,415 |
| 100,000,000（100 ms） | 3,473 / 3,474 | 3,414 / 3,415 |
| 200,000,000（200 ms） | 3,474 / 3,474 | 3,415 / 3,415 |

默认 0 是“只接受精确动态样本”的严格策略，这份 bag 没有图像恰落动态 TF 样本；报 `gap_too_large` 不是缺少 map 链。把门限设为 200 ms 只能让本次端点覆盖检查全部通过，不能保证运动插值精度。

`TfIndex::checkTime()` 目前返回可用历史端点，不计算数值 pose。后续 S2 要实现平移插值、四元数插值、求逆和链组合，保留使用的样本、间隔、时刻及质量标记；对近 200 ms 的缺口按运动/误差预算拒绝或降级。静态 TF 的旧 header 不应按动态过期规则拒绝。

## 8. 修复方案：按依赖分步推进

以下均为建议，尚未写入产品代码或改变默认策略。每步独立审查、文档和 Git 提交，优先用这份 bag 回归，再补小型确定性坏输入测试；真实录制及其全部媒体不进 Git。

| 顺序 | 模块和实际接入点 | 第一轮具体工作与接口约束 | 验收与预防边界 |
| --- | --- | --- | --- |
| P0-1 | RGB/Depth ROS 适配；`bag_reader_ros.cpp`、`bag_builder.cpp` | 按实际消息类型和 format 分发；第一步声明本 bag 的 JPEG RGB、16UC1/PNG compressedDepth 支持。压缩适配输出现有核心所需 Image/像素视图，复用 RGB8/Depth 转换。两遍读取采用同一规则；原 CDR、原 header、格式与来源继续保留。 | 全部 6,889 帧进入正式 builder；RGB 无通道交换、Depth 原始像素一致。输入字节、解码尺寸、资源预算和坏负载须检查；其他编码明确报不支持。可覆盖已声明格式，不能保证所有未来 codec。 |
| P0-2 | CameraInfo ROS/Bag 适配、`camera_info_index.cpp`、JSONL | 实现上述显式零尾 D8 规范化，保留 raw 与派生参数及策略，核心严格规则保留；核对固定标定选择及那一张缺同刻 info 的 RGB。 | 本 bag 7,268 条派生标定通过；尾项非零、NaN、模型未知仍拒绝。防止同类元数据误拒绝，不保证物理标定精度。 |
| P1-1 | `rgbd_sync.cpp`、`bag_config.cpp`、配置/报告 | 保持一对一策略；清楚输出采用的时间域、两个原始时刻、delta、阈值、未配对原因；按用途评估 30/40 ms，不能静默改时间戳。标定和 TF 阈值独立。 | 重现 2,671/3,230 对及未配对计数，无重复消耗 Depth。可避免错误关联/统计误报，不能修复曝光不同步。 |
| P1-2 | TF 核心及后续 S2 位姿层 | 先明确非零覆盖门限与大缺口策略，再补数值插值/逆/组合及各时刻的相机/底盘位姿；区分 base_link 与 body 来源。 | 覆盖和 pose 分别验收；已知轨迹、逆向链、旋转、边界与缺口对照验证。门限放宽不能杜绝运动误差。 |
| P1-3 | Depth 质量与后续投影层 | 确认尺度/注册依据；保留无效掩码，支持逐帧及选区质量拒绝、深度分簇；低质量进入 pending。 | 无效像素不变成有效点、尺度未知不宣称可信 map 几何。能阻止污染 memory，不能消除物理空洞。 |
| 采集改进 | D435i 驱动、image_transport、DDS/recorder | 保存 driver/固件/参数和时钟依据；监控各阶段计数、压缩耗时、QoS、CPU/磁盘；记录同步/注册配置并做实测。 | 解释图像与 info 的不一致来源并量化改进。不能仅从 best-effort QoS 推断丢帧位置，也不能承诺调整一个参数就零丢帧。 |

第一步的调用路径保持简单：`rosbag Reader -> 类型/format 校验 -> 压缩或 Image 适配 -> 现有 RGB/Depth 核心 -> 标定索引/配对/TF 诊断 -> preparation JSONL/媒体`。不为兼容本 bag 新建完整中间 rosbag，不把诊断脚本当正式 API。

实现时按 AGENTS.md 先逐模块写输入输出/错误与所有权，再编码；新增文件在 include/src 相同模块分类，配套同名文档。全量处理按消息流式解码，不同时保留所有大图。时间和标定继续使用已建立的排序/索引，避免逐 RGB 全扫全部 Depth/CameraInfo。

## 9. 本次诊断输出：位置与接口

审计根目录：`/home/dev/agent/vln_memory/build/camera-record-audit-20261030/`。Windows 可通过 `\\wsl.localhost\Ubuntu-22.04\home\dev\agent\vln_memory\build\camera-record-audit-20261030` 访问。所有输入、媒体和诊断源码保留在被忽略的 build 中。

```text
camera-record-audit-20261030/
├── input/camera_record/{metadata.yaml,camera_record_0.db3}
├── independent_extract/
│   ├── rgb/row_*.jpg                 原 bag JPEG 字节
│   ├── depth_raw/row_*.png           原 embedded PNG，uint16 raw counts
│   ├── messages.jsonl               21,441 条源消息引用
│   ├── frames.jsonl                 6,889 条图像及文件/像素摘要
│   ├── camera_info.jsonl            7,268 条原标定
│   ├── tf.jsonl                     7,287 条原 transform
│   └── errors.jsonl                 空：独立提取无解码失败
├── v1_direct_config.yaml / v1_direct.log / v1_direct_exit.txt
├── independent_summary.json / independent_extraction.log
├── core_probe_summary.json / core_frame_checks.jsonl / core_probe.log
├── pairs_30ms.jsonl / pairs_40ms.jsonl
├── cross_checks.json / final_artifact_checks.json
├── contact_sheet.png / timing_quality.png
└── scripts/                         本次专用诊断工具，不是安装后的 API
```

| 文件 | 输入/来源及字段 | 返回信息/作用 |
| --- | --- | --- |
| `messages.jsonl` | 原 SQLite 消息；`row_id`、`topic`、从 0 开始的 `topic_ordinal`、`record_ns`、`cdr_bytes`、`cdr_sha256` | 按数据库指纹+row_id 回溯 CDR；本地 row_id 不是跨 bag 的永久 ID |
| `frames.jsonl` | 来源字段，加 `kind`、`format`、`stamp_ns`、`sec`、`nanosec`、`frame_id`、`width`、`height`、`dtype`、`path`、文件与像素摘要 | path 相对 independent_extract；RGB 像素摘要基于规范 RGB8，Depth 基于原 uint16；深度记录另有原值质量统计 |
| `camera_info.jsonl` | 来源/header，加 `calibration`：尺寸/frame/model/D/K/R/P/binning/ROI | 原始标定；未将规范化副本覆盖为原值 |
| `tf.jsonl` | 来源、transform_index、header、parent/child、translation_m、rotation_xyzw | 原始变换；不是已组合的 map 相机 pose |
| `pairs_30ms.jsonl` / `pairs_40ms.jsonl` | 实际核心的 `rgb_key`、`depth_key`（topic#ordinal）及 `delta_ns`；两个 stamp 通过 frames.jsonl 查询 | 本次 bag 作用域内的一对一配对诊断；不是正式 Observation |
| `core_probe_summary.json` / `cross_checks.json` | 实际核心状态及独立结果交叉检查 | 标定、配对、TF 阈值结果和零像素不一致的证据 |

纳秒总时间与 delta 使用十进制字符串，避免常见 JSON/JavaScript 双精度整数精度损失；读取时用 64 位整数或 BigInt，不用 float。图像名中的数字是 SQLite row_id，不是连续图像序号。

例如 `frames.jsonl` 中真实首张 RGB 的关键字段如下（完整信息在原 JSONL）：

```json
{"row_id":21,"topic":"/camera/color/image_raw/compressed","topic_ordinal":0,"record_ns":"1793283709795263196","stamp_ns":"1793283709178912000","frame_id":"camera_color_optical_frame","kind":"rgb","width":1280,"height":720,"dtype":"uint8","path":"rgb/row_000021.jpg"}
```

查看深度原值应使用 `cv2.imread(path, cv2.IMREAD_UNCHANGED)` 并检查 dtype 为 uint16；不要以普通 8 位照片读取方式作为距离数据。contact_sheet 的黑色为 0，伪彩色按 300–3000 raw counts 显示，仅用于查看分布，不是米制真值。六组样例使用最近 Depth，其中部分时差超过 30 ms，不能当作已通过准备检查的观测。

## 10. 证据索引、复核及版本保存

- 输入指纹/完整性：`archive_manifest.json`、`inventory.json`、`inspection.log`。
- 产品失败：`v1_direct.log`、`v1_direct_exit.txt` 与 partial 工作目录；没有完整输出 manifest。
- 独立提取与耗时：`independent_extraction.log`、`independent_resources.log`，约 70.10 秒、最大 RSS 215,164 KB。
- 真实核心检查：`core_probe.log`、`core_probe_resources.log`，约 118.41 秒、最大 RSS 80,460 KB；与上一项是不同运行，不能作为正式 builder 性能承诺。
- 最终文件/JSONL 复核：3,474 JPG、3,415 PNG；各 JSONL 行数与上表一致；`final_artifact_checks.json` 确认全部 frame/尺寸；`cross_checks.json` 的 `pixel_and_export_mismatches=[]`。
- 本次实际使用版本 `e14efd2` 和 tag `bag-builder-v1` 保留。审计分支只提交本报告与文档导航，不提交录制、图像或 build；不合并、不推送、不声称已完成以上产品修复。

本报告可证实文件完整性、现有消息可解码、提取像素一致性以及核心对这些输入的实际反应。相机的真实深度尺度、注册精度、标定精度、时钟/曝光同步与 map 几何准确度仍需采集配置和物理实验验证。
