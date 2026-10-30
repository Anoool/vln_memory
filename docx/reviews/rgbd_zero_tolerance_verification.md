# RGB–Depth 零容差修复与真实 bag 重新提取验收

> 2026-10-31 清理更新：用户要求缩减 build 占用，本文所述完整提取 output 与 ZIP 的解压输入副本已删除；原始 camera_record.zip 保留。build 从约 18.04 GB 降到 165 MB，移除了约 17.88 GB。历史 manifest/summary/issues 已归档到 build/rgbd-strict-20261030/audit_records/ 的 last_* 文件，结合 verification.json 和运行日志供复查；该归档不是可直接消费的完整 preparation 数据集。重跑前需重新展开原 ZIP 或修改输入路径。最新 rgbd-strict-ros、rgbd-strict-install、rgbd-strict-consumer 仍保留，安装版 CLI 启动检查通过。以下各节保留 2026-10-30 实测历史。

日期：2026-10-30。分支：codex/rgbd-zero-tolerance。用户要求实际修复、坚持零容差，并在重新读取前清理旧提取数据。本记录对应最终安装版本，不以旧诊断脚本代替正式 BagBuilder。

## 1. 实际结论

正式 CLI 已能读取这份 D435i bag 的压缩 RGB/Depth，完整扫描 21,441 消息，解码 6,889 帧成功；全量独立回查的原始 CDR、header 与像素不一致均为 0。零容差筛选输出 0 对，符合原始两流时间戳没有交集的事实。

本轮交付严格筛选与真实输入支持，不生成同刻曝光，不改写原始时间，不自动放宽为近似匹配。未配对图像及原始消息完整保存；数据集完成状态为 completed_with_issues，时间匹配状态为 no_exact_pairs。

## 2. 改了哪些文件、怎样复用五模块

| 文件/模块 | 实际职责与本轮变化 |
| --- | --- |
| builder/bag_config.hpp/.cpp、config/vln_memory.yaml | sync_tolerance_ns 仅接受 0，YAML 与 C++ 入口拒绝非零；标定/TF 阈值保持独立 |
| builder/image_decoder_ros.hpp/.cpp | 新增压缩适配；格式/尺寸/字节预算先检查，再用 OpenCV 和官方 compressedDepth codec 逐帧解码；保留 header |
| builder/bag_reader_ros.hpp/.cpp | 接受 Image/CompressedImage，首遍只预检压缩头，第二遍解压；CDR 与负载错误分开，坏负载仍保留真实时间 |
| builder/bag_types.hpp | 扁平增加源消息类型/format/负载错误、timestamp_match_status；像素不放进全包索引 |
| builder/bag_builder.cpp | 固定以 0 调用现有 pairRgbDepth，复核 exact 不变量；零对也保留媒体和未配对清单，输出独立状态 |
| apps/bag_builder_main.cpp | 薄 CLI 输出匹配策略、状态及采集证据 unknown，不复制配对算法 |
| RGB / Depth 核心与 ROS Image 适配 | 复用原实现，RGB8 与显式单位深度转换不重写 |
| CameraInfo / TF | 保留原始校验和历史查询；D8 兼容、数值位姿不在本轮混改 |
| CMakeLists.txt / package.xml | 新头/源/测试安装，增加 cv_bridge 和 compressed_depth_image_transport 依赖；独立核心目标不引入 ROS 解码依赖 |

同名模块与测试说明分类存放，入口为 [image_decoder_ros](../modules/bag/image_decoder_ros.md)、[reader](../modules/bag/bag_reader_ros.md)、[BagBuilder 接口](../interfaces/bag_builder_api.md)。

调用路径：

```text
ROS bag 原消息 / 原 CDR / 原 header
  -> reader 第一遍：Image/CompressedImage 元数据索引
  -> 现有 pairRgbDepth(max_delta_ns=0)
  -> reader 第二遍：当前帧解压 -> 现有 RGB/Depth 核心
  -> 原始媒体/标定/TF/未配对与问题 JSONL
  -> 原子发布 preparation 目录
```

## 3. 严格筛选与接口契约

每条已发布 pair 必须满足两个原始 stamp 完全相等、delta_ns="0"、exact=true；RGB/Depth 均一对一、不复用、不按序号凑对、不用 record 时间替换 header。空配对的 max_abs_delta_ns="0" 是空统计默认值，不能解释为已经测得零时差。

独立通用 pairRgbDepth API 保留原兼容参数；生产 BagBuilder 只传零，非零配置在启动前失败。如果内部结果违反 exact_count/max_abs_delta_ns 不变量，整次按 strict_sync_violation 拒绝。

新增 CompressedImageInfo/CompressedImageResult 的输入、单位、返回和错误详见同名文档。压缩白名单为 RGB8/BGR8 JPEG、16UC1 compressedDepth PNG（含旧无 png 后缀写法）；其他压缩编码明确报告 unsupported_compressed_format，不误解码为 PNG。原始 Image 保留既有支持。

BagImageConsumer 只在回调内借用解压 Image；返回空指针时用 deserialization_error/payload_error_code 区分错误。坏 JPEG/PNG 保留 header，不能伪装为没有采样时间的坏 CDR。C++ 公共结构新增成员，下游需要重编译，不能将旧二进制混用新库。

CLI/manifest/summary 增加 timestamp_match_policy、timestamp_match_status 和 capture_sync_status。当前采集证据为 unknown；header 相等不能单独证明硬件曝光同时。输出 schema 仍为 bag_preparation/1，manifest 的 builder_version=2。

## 4. 先清理，再重新读取

已验证所有递归删除和输入迁移的绝对路径都在仓库 build 内，未触及用户原始 ZIP。清理/保留范围：

- 删除旧 build/camera-record-audit-20261030 生成树：6,977 文件、1,007,984,090 字节，包括原诊断导出、JSONL/统计、失败 partial 和临时诊断构建。
- 保留原输入 bag，移至 build/rgbd-strict-20261030/input/camera_record；原始 ZIP 没有修改。
- 删除旧 Windows 预览 camera_record_contact_20261030.png 与 camera_record_timing_20261030.png。
- 审查中间输出已替换，当前只保留一份最终 output；清理列表在新目录 cleanup.json。

原始数据库内容 SHA-256 仍为 `5704b033b590e01151bb56534c3068ea817f90836744975c51ef70b7dbc5890d`，与首次原包核查一致。正式 reader 在两遍读取中继续验证文件/元数据指纹和逐消息内容。

## 5. 最终正式 CLI 实测

使用安装到 build/rgbd-strict-install 的最终程序；日志为 build/rgbd-strict-20261030/cli.log 和 resources.log。实际退出码 0，最终目录发布成功，没有残留 partial。

| 指标 | 实际值 |
| --- | ---: |
| 消息扫描 | 21,441 |
| RGB | 3,474 |
| Depth | 3,415 |
| 图像成功解码 | 6,889 |
| 图像解码失败 | 0 |
| exact_pairs / approximate_pairs | 0 / 0 |
| 未配对 RGB / 未使用 Depth | 3,474 / 3,415 |
| prepared_pairs | 0 |
| TF 原始条目 / 成功插入 | 7,287 / 7,287 |
| issues | 10,743 |
| timestamp_match_status | no_exact_pairs |
| capture_sync_status | unknown |
| 完成状态 | completed_with_issues |

最终 wall time 为 2 分 50.38 秒，最大 RSS 111,928 KiB（约 109.3 MiB）；单次本机测量，不作为设备或其他 bag 的性能承诺。

最终输出 28,339 文件，共 16,397,126,077 字节（约 16.4 GB / 15.27 GiB）。标准 float32 深度 NPY 比原始 uint16 PNG 大，输出预算本次显式设为 28 GiB；没有把此数据集的预算写回默认配置。

本次深度按显式 millimeters 配置转换，原始计数/CDR 保留。逐像素验证的是转换一致性，不能据此确认设备物理 scale 已验证；alignment 与采集同步证据仍为 unknown。

## 6. 全量独立回查

verify_output.py 直接只读查询原 SQLite，不使用已清理的旧图像/JSONL作为预期值。读取每条源 CDR，核对记录时间、topic ordinal、SHA 与导出 raw；读取原压缩 header，逐张比较输出 stamp/frame/format/尺寸和源字节数。

- 21,441 条原始消息逐字节核对，无不一致。
- 3,474 张 JPEG 独立解码后与标准 PNG 比较，无通道或像素不一致。
- 3,415 张 uint16 PNG 独立解码，按显式毫米配置计算预期 float32，逐像素比较 NPY，零值对应 NaN，全部一致。
- 6,889 个原始 header 没有改写；整数纳秒交集仍为 0。
- 配对清单为空、近似对为 0、两流计数守恒、未配对数据保留。

完整结果见新目录 verification.json/verification.log，工具源码 verify_output.py 留在当前 build 内，不属于安装 API。深度总像素 3,147,264,000，零值 1,967,568,298，与原包初次统计一致。

## 7. 剩余 issues 的实际含义

| code | 数量 | 含义/后续 |
| --- | ---: | --- |
| invalid_distortion | 7,268 | 原 CameraInfo 为 plumb_bob+D8；本轮未改动该问题，应按已规划零尾兼容策略处理 |
| outside_tolerance | 3,474 | 每张 RGB 都没有原始同刻 Depth，零容差如实拒绝 |
| no_exact_rgbd_pairs | 1 | 数据集级提示没有严格时间戳配对 |

这些不是 10,743 次图像解码错误。未使用的 3,415 张 Depth 在 pending 单独记录，不自动标记坏帧。TF 时间门限仍为诊断配置的 0，逐图覆盖失败状态可在 frames.jsonl 查询；原始 TF 已保留，不宣称已算出数值 pose。CameraInfo、注册依据、物理深度尺度、数值位姿和关键帧继续作为独立工作项。

## 8. 测试、整体审查与版本

- 严格编译通过：-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror。
- 最终 CTest 15/15，通过既有核心/ROS、压缩适配、磁盘集成及独立 NumPy 检查。
- 新增回归：精确相等与 1ns 不等；真实零时刻；同刻重复一对一；未知/不同 clock 拒绝；原 Image/压缩/混合流；坏负载保留 header；未知 codec、尺寸/字节预算、截断；非零 YAML/C++ 配置拒绝；JPEG EXIF 不能旋转相机网格。
- 安装后 10 个消费者编译并实际运行；最终库又通过包外消费者实际构建压缩 fixture。
- 首轮测试发现 YAML 加载入口仍允许读入非零值，已补齐拒绝并回归通过。合成坏 CDR 会触发 ROS 底层反序列化诊断，这是测试故意输入；实际真实 bag 没有此类解码失败。
- 通读配置→索引→压缩预检/解码→核心→配对→JSONL/发布，补齐函数级注释；布局用除法防溢出，JPEG 不自动旋转，坏 CDR 不冒充零时刻。
- 首遍不缓存解压图像；第二遍逐帧释放。复用现有排序/二分配对和索引，没有逐图全表扫描或增加新的同步器层。

原 bag-builder-v1 标签保留。当前 codex 分支保存规划和本轮实现，录制/媒体/生成结果不进 Git，不合并、不推送。

## 9. 查看与复现

最终目录：`/home/dev/agent/vln_memory/build/rgbd-strict-20261030/output`。

```text
rgbd-strict-20261030/
├── input/camera_record/             保留的输入缓存
├── config.yaml                     真实话题、零容差及显式诊断语义
├── output/
│   ├── bag_manifest.json
│   ├── messages.jsonl / frames.jsonl / camera_info.jsonl / tf.jsonl
│   ├── rgbd_pairs.jsonl             空：无严格对
│   ├── reports/{summary.json,issues.jsonl,pending.jsonl}
│   └── evidence/{raw,rgb,depth}/
├── cli.log / resources.log
├── cleanup.json / archive_provenance.json
└── verify_output.py / verification.json / verification.log
```

在仓库根目录复现（输出必须换成尚不存在的新目录）：

```bash
source /opt/ros/humble/setup.bash
source build/rgbd-strict-install/share/vln_memory/local_setup.bash
ros2 run vln_memory bag_builder --config build/rgbd-strict-20261030/config.yaml \
  --output /absolute/path/new_strict_output
```

接口、参数和返回约定见 [BagBuilder 手册](../interfaces/bag_builder_api.md)，原规划与逐步实施见 [严格同步方案](../project/rgbd_strict_sync_plan.md)。
