# 真实录制问题清单与 RGB–Depth 严格同步优先方案

日期：2026-10-30。依据：用户要求优先解决严格同步、当前实际代码和 [camera_record 审计](../reviews/camera_record_extraction_audit.md)。本轮交付问题清单、同步定义、实施步骤与验收依据；尚未修改产品源码或相机配置。

## 1. 当前问题与对应解决方案

| 优先级 | 已核实的问题/缺口 | 解决方案 | 不能据此承诺的结果 |
| --- | --- | --- | --- |
| P0 当前重点 | 原始 RGB/Depth 时间戳交集为 0；最近绝对时差中位数 26.454016 ms | 保留离线零容差严格筛选；从采集端验证帧组、各流时间域和实际采样时差，再录制并验收 | 离线不能将已有不同曝光时刻恢复成同一时刻 |
| P0 同步证据 | 旧 bag 没有设备 metadata、驱动参数和原始设备时间证据 | 新录制保留原始每流 metadata、参数快照、版本和帧组关联；报告区分 header 相等与物理同步证据 | `exact=true` 不能证明曝光同时 |
| P0 接入依赖 | 正式 reader 仅支持 Image；本 bag 是 JPEG RGB + compressedDepth | 在 ROS 适配层支持声明过的压缩格式，接入既有核心；同步索引依据原 header，媒体解码不改时间 | 本文不是压缩能力已实现的声明 |
| P1 | CameraInfo 的 plumb_bob 带 D8，7,268 条均被严格核心拒绝 | 显式、可追溯地对末 3 项全零的 D8 副本规范化为 D5，保留原值；其他情形继续拒绝 | 软件校验通过不证明物理标定准确 |
| P1 | 1 张 RGB 无同刻 info；另有 160 个 RGB、220 个 Depth 的 info 时刻没有同刻图像 | 标定版本明确且模型不变时复用固定标定；记录缺失；从采集/压缩/DDS/recorder 分段计数定位损失 | 数量差不能直接定位丢帧环节，无法补回未记录的原图 |
| P1 | TF 零间隔门限下图像均无精确动态样本；当前只查链和历史端点 | 依据运动误差预算设置独立 TF 间隔门限；后续补插值/逆/组合，异常缺口拒绝或降级 | 200 ms 覆盖全部图像不等于数值 pose 精确 |
| P1 | Depth 尺度/物理注册依据不完整 | 保存驱动深度单位/scale 和注册配置，并用已知距离和重投影检查验收；未知保持 unknown | D435i 型号、同 frame/尺寸/K 不能替代物理验证 |
| P1 | Depth 零值占 62.5168% | 保留无效掩码，按帧和目标选区报告质量，低质量进入 pending | 不能靠填洞恢复真实测量或保证物理空洞消失 |

关键帧、数值投影和正式 Observation 等其余缺口继续按 [系统规划 S2](system_plan.md) 跟踪，不混入这次同步核心修改。

## 2. 必须分别验收的两项同步条件

### 2.1 消息时间戳严格匹配

在同一已声明且有依据的 clock/segment 内：

```text
rgb.header.stamp == depth.header.stamp
delta_ns == 0
每条 RGB、Depth 来源最多使用一次
```

当前配置 `bag.sync_tolerance_ns: 0` 已实现该条件，不新增同义的 exact 模式开关。不同 segment、时钟关系未知、坏 stamp 或重复来源按既有校验拒绝。原始 header 不改写；record 时间只作来源，不替代采样时间。

### 2.2 采集时间与曝光同步证据

相同 ROS header 可能来自驱动对 frameset 使用统一标签，不能从中反推出原始曝光同时。应保存各流原始设备时间、单位、clock_domain、frame_number 和可用曝光元数据；有同域或经验证的映射时，才评估真实采样时差。

物理同步要求必须结合允许的时间误差和传感器能力验收，不能用“header 差值为 0 ns”代替物理精度指标。缺少证据时标记 unknown；跨时间域的数值不直接相减，独立流 frame_number 也不要求数值相等。

## 3. 代码核查：现有严格筛选已经可用

| 文件 | 当前实际职责 | 本轮结论 |
| --- | --- | --- |
| `include/vln_memory/bag/builder/rgbd_sync.hpp` | 拥有型来源、时钟段、整数纳秒容差、配对/未配对结果 | 复用现有公开契约 |
| `src/bag/builder/rgbd_sync.cpp` | 校验、排序、单调一对一最近匹配，复核 checkSensorTimes | 容差 0 已拒绝不相等时间，不重写算法 |
| `src/bag/common/sensor_time.cpp` | 同域、合法 stamp、差值和容差校验 | 保留整数纳秒和错误语义 |
| `src/bag/builder/bag_builder.cpp` | streamIndex/synchronize；写两图 stamp、delta、exact 及 summary 计数 | `RgbdSyncResult::ok()` 仅说明算法完成，不能当作采集同步已验收 |
| `src/bag/builder/bag_reader_ros.cpp` | 话题类型核对、反序列化和来源索引 | 压缩图像仍是正式 reader 的阻塞点 |
| `test/bag/builder/test_rgbd_sync.cpp` | 已知答案、1 ns 超差、边界、来源/segment、参考算法 | 本轮已有 rgbd_sync_core 1/1 通过 |

当前算法成本为排序 O(R log R + D log D)，随后每 RGB 约 O(log D) 查找；不为严格模式另写一个重复同步器。算法输出拥有型引用，不装图像，不改变输入生命周期。

正式 `rgbd_pairs.jsonl` 已有 `rgb_message_id`、`depth_message_id`、两个 stamp、字符串 `delta_ns` 和 `exact`；summary 已有 exact/approximate/未配对计数。后续只在必要处补充采集同步的证据状态和引用，不重复建立一套消息模型。

提取完成与同步验收分开：允许保留全量未配对证据和问题报告；零配对不能称为严格 RGB-D 数据集。未来下游消费要检查配对/质量结果，不仅检查 buildBag().ok()。

## 4. 旧 bag 怎样处理

本轮重新读取全部 6,889 条图像索引，以 Python 整数纳秒计算集合交集，结果仍为：

```text
RGB = 3474
Depth = 3415
intersection(original_rgb_stamp, original_depth_stamp) = 0
```

这与实际 C++ 核心的零容差结果一致。保留全部图像/标定/TF 和未配对来源；这份旧 bag 的严格时间戳匹配结果应为 0 对。正式 CLI 当前会先因压缩消息类型退出，不能把诊断结果描述成正式 builder 已成功发布。

不能通过以下操作宣称修复：扩大为 30/40 ms 后仍标 strict；给 Depth 覆盖 RGB 时间；减去最近邻中位数作为固定时钟校正；按图像序号配对；复用一张 Depth；用 bag 接收时间替换采样时间。深度重投影或运动补偿若以后实现，也必须作为派生结果保留方法、参考时刻和误差，不能伪装原始严格同步。

## 5. 新录制：从 D435i 采集侧解决来源问题

现有两个仓库未发现可直接调整的 D435i 启动配置；本次 bag 也未记录录制机的驱动/参数。以下为需按实际安装版本落实的采集方案，尚未在设备上运行。

1. 保存录制机实际 ROS wrapper、librealsense、固件版本和参数；用设备支持列表确认 RGB/Depth 的真实 profile，设置相同受支持帧率，并核对最终启用结果。
2. 在实际驱动支持的参数中启用 `enable_sync`，使用 SDK 帧组输出。官方说明它收集最近帧并使用统一标签，不能把这一开关承诺为硬件曝光零误差。
3. 若后续用 RGB 像素索引 Depth，另外启用并验收 `align_depth.enable`，使用真实 aligned_depth_to_color 话题；空间注册和时间同步分别检查。继续录制当前 builder 将支持的 Image 或压缩图像话题，不要求先引入专用 RGBD 消息依赖。
4. 同时录制 color/depth 原始 metadata、各自 CameraInfo、所需 TF，并保存参数快照。metadata 原字段/单位/domain 保留，确认它们关联的是对应原始帧；检查每个流的 frame_number 是否重复或跳变，避免把同一物理帧的重复发布误当新测量。
5. 先做短时试录，核对 header 严格匹配、设备时间证据、两流帧率、缺帧和重启，再做长录制。帧组无法提供所需物理时间精度时，应评估设备同步能力和采集方案，不能仅在 ROS 层统一时间戳。

官方参考（2026-10-30 查阅，不能据此认定旧 bag 使用该版本）：

- [RealSense ROS wrapper 参数与 enable_sync](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)：同标签帧组与空间注册配置。
- [RealSense frame_callback / publishMetadata](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/src/base_realsense_node.cpp)：帧组统一 ROS 时间及每帧 metadata 的 frame_timestamp、clock_domain、frame_number。
- [ROS 2 Humble ExactTime](https://github.com/ros2/message_filters/blob/humble/include/message_filters/sync_policies/exact_time.h)：按消息 timestamp 组织严格匹配；只作对照，不为离线索引额外引入在线订阅队列。

## 6. 下一轮实现顺序与人工审阅边界

| 步骤 | 工作 | 验收 |
| --- | --- | --- |
| A：现有严格筛选核验 | 本轮已核查零容差核心、现有输出、真实 bag 时间戳交集 | 现有测试通过；真实 bag 0 对；保留源时间。本步骤已完成，不等于采集同步修复完成 |
| B：reader/适配接入 | 下一轮先让 Image/CompressedImage 都提供原始 header/来源，再完成本 bag 压缩格式转换；两遍读取一致 | 原始 stamp 无变更；按相同来源调用既有 pairRgbDepth，旧 bag 仍为 0 严格对 |
| C：明确同步证据 | 在现有 summary/来源体系补充最少的匹配策略及采集证据状态，缺失为 unknown；新增字段有版本和文档 | exact header 与设备同步状态可分别检查；不把 extraction completed 解释为同步合格 |
| D：采集配置和新 bag 验收 | 在录制机匹配驱动版本、profile、frameset、metadata，执行短录制 | 有严格配对结果、来源与未配对清单；物理同步证据另验收，无法证明时保持 unknown |

B/C 编码前分别确定输入输出、所有权、错误和文件格式；保持五模块层次、短函数与函数级注释，修改放 codex 分支逐步提交。具体源码同名说明继续放 docx/modules/bag 等对应目录；本文放 docx/project。

新增回归要覆盖：相等 stamp 正例、1 ns 不等负例、同刻多消息一对一、乱序与跨段/时钟未知、原时间不被修改、旧 bag 无交集以及“统一 header 但原设备时间不同”只通过消息匹配而不通过采集同步证据验收。构造测试与真实设备测试分别记录。

## 7. 本轮自审与实际交付

已通读同步头文件、实现、测试以及 builder 的来源索引/配对/JSONL/发布路径；确认 exact 字段、两个原始 stamp 和主要统计已存在。没有为了严格同步重复增加 mode、同步器类或改写来源时间。

本轮只新增本规划与导航入口，原审计报告和 bag-builder-v1 保留。现有核心测试通过、真实数据交集重新核对为零；驱动采集配置未改、压缩接入/证据字段/新采集均仍待实现或设备验收。方案将用户要求的严格同步放在当前最高优先级，同时如实保留其采集端依赖。

## 8. 零容差实施前拆分（2026-10-30）

本轮用户已要求实际修复并坚持零容差，按下列接口边界逐步实施：

1. BagBuilder 配置/API 校验只接受 sync_tolerance_ns=0，非零直接 invalid_config；既有通用 pairRgbDepth 独立 API 保留兼容，但 builder 不调用近似策略。
2. 新增 builder/image_decoder_ros.hpp/.cpp，负责压缩负载的格式、尺寸/字节预算预检和逐帧解码。首轮支持 JPEG RGB8/BGR8，以及 16UC1 compressedDepth PNG；其他编码明确拒绝。PNG/JPEG 头部先检查解码尺寸，再使用已有 OpenCV 与官方 compressedDepth codec，不能无边界解压。
3. reader 首遍只读取压缩消息 header/布局与来源，不缓存所有像素；第二遍逐帧解码后复用现有 RGB/Depth。坏负载保留可用 header 与 CDR，记录解码错误，不伪装为坏 CDR或改配别的帧。
4. BagImageRecord 增加扁平的原消息类型、format、负载解码错误字段；BagBuildResult/summary/manifest/CLI 明确时间戳匹配状态。保留提取完成但零配对的证据，不假造严格 RGB-D 数据集；物理同步状态保持 unknown。
5. 原始 Image、混合/压缩流、精确相等与相差 1ns、重复时间一对一、坏压缩负载、非零容差配置均做回归；独立 Python 检查所有已发布 pair 的 delta 为零和引用守恒。
6. 真实 camera_record 完整运行正式 builder，预期 3474 RGB、3415 Depth、6889 解码、0 对；逐帧核对原始时间与图像/深度。D8 校验失败及物理尺度/注册未知继续如实报告。

源码仍属于五模块中的 builder；新适配文件公开注释说明所有权、预算和错误；分类文档同名放 docx/modules/bag/，测试说明在其 tests/。不修改原 bag，不改动原 header，不改采集设备参数。

自审：复用现有时间核心；不新增同义模式开关，不通过扩大阈值处理旧 bag；只增加支持真实输入和明确验收状态所需接口，解码保持逐帧内存，元数据查找不全表扫描。以上为编码前方案，完成后追加实际验证结果。
