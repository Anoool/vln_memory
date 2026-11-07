# 离线构建输入、关键帧与批次

实现日期：2026-11-05。公开接口为 `include/vln_memory/engine/build_input.hpp`，实现为 `src/engine/build_input.cpp`，链接目标为 `vln_memory::vln_memory_build_input`。核心库不依赖 ROS 或模型服务。

## 输入、输出与职责

`buildInputManifest(preparation, output, options)` 消费已发布的 v5/v6/v7 preparation，通过现有索引 reader 核对每对数据。输出到尚不存在且与输入不重叠的新目录，目录内包含 `build_input_manifest.json`（`schema_version=build_input/1`）。共用既有 partial 目录及 Linux 不覆盖原子发布机制；失败不发布完整目录，可能保留失败工作目录。

这是离线构建的输入清单，记录选择决策及原配对引用。发布正式观测时，[观测数据集入口](observation_dataset.md) 直接接收 preparation 并复用本模块的选择逻辑。实际几何操作继续校验像素、标定、位姿及其来源依据。

## 用途筛选

| purpose | 要求 |
| --- | --- |
| rgb | 读取成功并有 RGB 媒体路径 |
| depth_map | 读取成功并有米制 DepthFrame、Depth 模型与 Depth 时刻的 map 相机位姿 |
| prepared_rgbd | 同时满足以上两项，有 RGB 模型和 RGB 时刻的 map 位姿，且来源 pair.status 为 prepared_for_geometry |

用途只表达当前操作所需输入。depth_map 不依赖 RGB 注册；它也不证明深度尺度或物理定位精度。prepared_rgbd 沿用来源 preparation 的状态，曝光同步状态与依据继续从原记录核查。底盘位姿仅在启用采样/步进规则时必需。

候选筛选使用 Reader 完整回读，因此 rgb 用途也会核验配对的深度媒体，损坏时记录 read_failed。正式观测数据集随后用元数据模式组装已入选记录。逐对处理后释放像素，仅保留轻量来源和决策。单对读取失败保留错误码/消息，其他配对继续；全局打开错误、可检测 source_changed 和发布错误禁止发布。

## 采样规则与批次

先以 RGB 原始整数 sec/nanosec 稳定排序，同刻保留输入源顺序。source_index 保留源 JSONL 行序。所有 eligible 候选必须显式共享 bag_id/map_frame/clock_id/segment_id，拒绝跨上下文采样。

- 不设置 distance_m/yaw_rad 时选择全部 eligible 候选。
- distance_m 为相对上一关键帧的三维 map 欧氏距离；yaw_rad 为底盘绕 map Z 的最短角差。设置的任一条件达到阈值即选择。
- 首个可用候选及缺姿态/不合用途断点后的首个可用候选直接选择，原因记 first_after_gap。缺失/无效底盘位姿不伪造为原点。
- 可选 max_step_m/max_step_yaw_rad 比较相邻可用候选。超过上限记 pose_jump，本段之后均记 trajectory_discontinuity，不自动猜测新地图或恢复轨迹；等于上限仍允许。
- 阈值必须有限且严格为正，yaw 阈值不超过 pi。不设置跳变上限表示调用方未提供该项限制；已声明的来源段及物理轨迹仍需独立核实。
- 选中记录按序分组，每批最多 batch_size 条，尾批可以较小。批次和 selection_index 从 0 开始，未选记录为 null。

`selectBuildInputFrames(frames, options)` 是上述轻量采样函数，接收已核验的 eligibility、来源和可选位姿，不读取媒体。配置、ID、时间戳或上下文错误抛出 invalid_argument，配置/上下文验证在修改选择结果前完成。

## 清单字段与回读

根记录保存 preparation_root、实际 options、排序规则、candidate/eligible/selected 计数和 batches（pair_id 数组）。frames 保留每一候选的源行序、pair_id、源消息 ID、bag/map/clock/segment、原始两路时间及有符号十进制字符串 delta_ns、来源状态、capture_sync_status、pending_reasons、读取状态、用途拒绝原因、采样原因、可选底盘位姿、selection_index/batch_index。

清单中的 pair_id 仍由 `PreparationReader::loadPair` 回读原始配对。preparation 和证据在消费期间必须保持稳定；清单不内嵌整份模型/TF/像素，也不独立证明物理标定、注册或同步。读取器路径/大小/修改时间检查的保证范围沿用其模块文档。

## CLI

两种构建模式均提供 `memory_build_input`。示例参数只是演示，须根据采集条件显式选择：

```bash
build/standalone-install/lib/vln_memory/memory_build_input \
  --input /absolute/path/to/preparation \
  --output /absolute/path/to/new_selection \
  --purpose rgb --batch-size 3 \
  --distance-m 0.8 --yaw-deg 25
```

CLI 将角度转为弧度，清单保存实际内部值。可另传 --max-step-m 和 --max-step-yaw-deg。成功返回 0（包括合法空输入或全部被筛除），业务/发布失败返回 1，参数解析失败返回 2。应检查清单 selected 计数判断是否有模型输入，不能只检查进程退出码。

C++ 调用示例：

```cpp
vln_memory::BuildInputOptions options;
options.purpose = vln_memory::BuildInputPurpose::rgb;
options.batch_size = 3;
options.distance_m = 0.8;  // 明确的试用参数，按采集条件确定。
auto result = vln_memory::buildInputManifest(preparation, new_output, options);
if (!result.ok()) {
  // 记录 result.code / result.message / result.work_directory。
}
```

## 验证与开销

回归源 `test/engine/test_build_input.cpp` 覆盖位移累计、yaw 环绕、缺/坏位姿、断点、步进边界及跳变、分批、负/零/同刻时间、重复 ID、配置和跨上下文拒绝。可传入现有 preparation 和独立输出父目录进行磁盘集成，核对来源/pending、三种用途、发布计数、已有目标/路径重叠、坏媒体和无效配置。安装消费者复用同一检查程序。

设配对数 P，采样与批次阶段时间 O(P log P)、轻量空间 O(P)，像素内存只随当前回读的一对及解码临时空间增长。读取阶段开销沿用索引 reader，当前每对仍恢复完整深度。

2026-11-05 的核心 11/11、ROS 20/20、两种安装模式与固定真实短包结果见 [验收](../../reviews/build_input_verification.md)。
