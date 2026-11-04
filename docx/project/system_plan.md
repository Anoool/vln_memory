# VLN 空间语义记忆规划：当前只交付原设计第 1、2 部分

更新：2026-11-03。本规划依据用户提供的 VLN_SPATIAL_SEMANTIC_MEMORY_DESIGN.md（更新 2026-10-27）及用户最新范围要求，替换此前同时展开完整在线查询/导航接入的总体路线。原始附件不修改。

**当前目标：原文第 1 部分“核心约定与数据规范”及第 2 部分“离线构建流程、工具与规范”。当前已实现观测准备层基础与 BagBuilder preparation 导出，有效模型与数值位姿已接入 preparation，后续记忆业务尚未实现，两个原文部分都未完成。**逐项证据见 [范围与进度对照](sections_1_2_scope.md)。

## 1. 本轮范围与完成定义

目标闭环：

```text
ROS 2 bag + config/vln_memory.yaml（目前支持 bag 段）
  -> BagBuilder：来源/时间段、RGB-D 配对、标定/TF 数值解析
  -> preparation JSONL + evidence/ + pending/质量报告
  -> 离线构建输入：读取和核对引用、按用途筛选、关键帧/批次选择
  -> MemoryEngine：几何、观测与实体/关系、构建读取与修改工具、模型批次/恢复
  -> 原文 1.6 规定的完整版本、索引与原子发布 manifest
  -> MemoryViewer：map 实体/关系、图像/选区/轨迹/位姿联动及命名标注
```

原文第 2.2 节已经要求 search_entities/get_observation/project_depth/spatial_query，因此构建所需读取能力在范围内。原文第 2.9/2.10 节要求 Viewer 与人工标注，也在范围内。

当前暂不交付：第 3 部分完整在线自然语言查询编排、全空间操作目录与视角问答、独立 Agent 在线查询插件组、导航目标生成/执行。可选向量检索、embedding、跟踪不作为前置条件。不会为这些内容提前创建源码或目标。

“第 1 部分完成”需要规范对应真实的数据结构、校验、存储/序列化和版本证据；“第 2 部分完成”需要三个实际运行模块和端到端验收。不能以写了设计或有单元测试替代。

## 2. 当前真实实现及可复用基础

| 已有内容 | 当前能力 | 明确未完成 |
| --- | --- | --- |
| RGB | rgb8/bgr8/mono8 到紧密 rgb8，输入检查与独立输出；builder 已接入原 Image/JPEG 并保存 PNG | 单帧模块不承担物理同步或相机几何 |
| Depth 与单点投影 | 16UC1/32FC1、米制 optical-z、质量统计。独立投影库实现单点 Depth→map 和 Depth→RGB 候选像素 | 真实几何验收、物理尺度与注册依据、区域投影 |
| CameraInfo | 完整 K/D/R/P、显式绑定、ROI/binning 网格和时间检查。支持 D8 兼容、有效模型及单点射线，builder 可索引并选择标定 | 物理标定精度验收 |
| TF | 静/动态原始消息、child/时间索引、链/区间交集、端点、插值/求逆/组合及 map 相机和底盘位姿 | 整段轨迹跳变与物理定位精度验收 |
| common | SensorStamp/ImageGeometry、候选时差与统一名称检查；builder 已保留消息来源和显式 clock/segment | 正式 Observation 的统一 map/版本契约 |
| BagBuilder | SQLite bag 两遍读取、Image/指定压缩图像解码、严格/最近配对、标定候选与 TF 历史检查、CDR/PNG/NPY/JSONL 及 pending 导出 | 深度几何、关键帧和后续记忆业务 |
| 构建与文档 | 无 ROS 核心、ROS 适配、安装目标与独立投影消费者 | 完整 Observation、MemoryEngine/Viewer 和完整记忆产物 |

当前 vln_memory 中没有 Entity/Relation/Observation 正式业务类型和持久化，没有原文八个构建工具。vln_agent 的 SessionManager/ToolManager 是可复用基础；EnvironmentMemory 是逐观测历史，不计作永久空间记忆。详见 [范围对照第 6 节](sections_1_2_scope.md)。

## 3. 实施顺序与逐步验收

| 阶段 | 原文对应 | 交付 | 验收重点 | 状态 |
| --- | --- | --- | --- | --- |
| S0 数据契约/配置 | 1.1–1.6、2.10 | map/clock/来源/版本、Observation 与实体/关系字段规则，配置 schema/加载与校验 | 单位、引用、缺项、版本语义明确；配置不静默猜数据含义 | 部分完成：传感器/准备数据契约及 bag 配置已落地；正式业务类型、map/版本和其他配置待补 |
| S1 自动配对与读取 | 2.1、2.10 | 先实现无 ROS rgbd_sync，再 reader、话题类型/编码与时间索引 | 确定配对、平局/一对一、时钟段边界、来源可回查 | 主要能力已实现；全包时间索引与真实短片段提取已验证，物理同步/尺度/注册仍待验收 |
| S2 完整离线观测 | 1.3、2.1、2.3、2.10 | 有效相机模型、TF 位姿求解、注册/时间质量、关键帧、证据与 Observation 输出 | 真实 bag 的两图时间/位姿、标定与文件引用一致，缺项进入 pending | 已有标定兼容、有效模型、数值位姿和单点 Depth→map/Depth→RGB；真实深度几何验收与观测组织待完善 |
| S3 几何与工作库 | 1.2/1.4/1.6、2.3–2.7 | 点/区域到 map、对象/空间、关系、事务存储、基本索引与版本 | 已知几何答案、永久 ID、代表点、inside/region_id、revision | 待实现 |
| S4 工具与模型构建 | 1.5、2.1/2.2/2.4/2.6 | 八工具、上下文/图像、批次循环、进度/幂等恢复 | 真实模型服务、多视图 ID 复用、轮数上限/失败不误报完成、重复 call_id 不重复写 | 待实现 |
| S5 纠错/标注与发布 | 2.7–2.10 | 四纠错、关系刷新、注释历史、完整记忆产物和原子 manifest | 整体提交/引用修复、人工信息保留、几何缓存失效、半成品不发布 | 待实现 |
| S6 Viewer 与闭环验收 | 2.1/2.9/2.10 | 地图/关系/图像/轨迹联动、按实体/关系/时间定位观测、人工命名与回读 | 同一发布版本可检查，标注走共用写入路径，重开后结果一致 | 待实现 |

S0 已部分落地。按用户最新要求，当前不先实现最小 Observation；既有 preparation 的读取/引用/筛选在后续 API 输入组织时处理。map/版本与 Entity/Relation 随实际业务落地，不为所有阶段生成空类。RGB-D 自动配对与第一版 reader/准备导出已实现，实际接口见 [BagBuilder](../interfaces/bag_builder_api.md)；该方案现在明确归属于 S1，不再把它当整个第 1、2 部分的交付范围。

## 4. S0–S2：观测准备层如何收口

### 4.1 数据与配置

- 观察视角与物体自身朝向分开；参照不足返回待补充/未知，不猜测语义正面。该约定保留，完整第 3.4 节视角问答暂不开发。
- 世界几何属于同一 map；map_id、geometry_revision、米/弧度、坐标轴和时钟段在配置/manifest 中明确。相同 map_id 不足以证明不同 bag 已配准。
- 正式区分源消息采样时间、bag 记录时间、处理时间，统一来源消息 ID；零时刻真实保留，未知时钟不跨域关联。
- config/vln_memory.yaml 是最终交付物，包含 dataset/topics/frames/preprocess/build/model/geometry。当前已实现该文件的 bag 段及解析/校验；其他系统层配置仍未实现，不把原文完整 YAML 示例算已支持。
- 原文示例的 sync_tolerance_ms=30、depth_scale_to_m=0.001、keyframe_distance_m=0.8、keyframe_yaw_deg=25 都需按实际采集配置确认。深度单位/比例冲突应拒绝；已有米制深度不能再乘 0.001；角度配置转换到内部弧度须明确。
- 未注册深度需要自己的标定话题和 frame，不能因为示例只有一个 camera_info 字段就默认共用 RGB 内参。

### 4.2 配对与读取

保留 [配对方案](rgbd_sync_plan.md) 的原始 stamp、单时钟段、RGB 基准、一对一/单调最近候选及可追溯失败规则。metadata 先索引、后选择解码，避免把整包像素留在内存。

reader 负责 bag 存储/类型/反序列化、来源与稳定顺序，当前纯时间检查/ROS 适配不能替代 reader。输入是否有时间重启要结合采集依据处理，不能把重启数据简单全局排序。

### 4.3 历史位姿与相机几何

TF 模块已在链与样本选择之上实现插值、逆变换和组合。Builder 导出 map 相机与底盘位姿、查询时刻和端点来源。时间超限和插值跳变通过独立状态报告。

CameraModel 已实现实际网格 K/D/P、ROI/binning 和单点射线：raw 去畸变，rectified 经 P 内参与 R 逆旋转。非 optical-z 值在模型明确后换算；未注册深度先按标定/外参重投影。支持范围按真实输入验证，未支持的组合进入明确错误/pending，不能默认生成单位模型。

### 4.4 Observation、关键帧和证据

以下保留后续记忆业务的观测要求；当前不新增独立最小 Observation 层，先复用 preparation，读取映射和筛选在 API 输入组织时处理。

一条 Observation 引用一对明确配对的图像，保留原文字段并使时间语义无歧义：camera_pose_map 对应 RGB 参考时刻，Depth 保留自己时刻的相机位姿（可增加 depth_camera_pose_map），body_pose_map 对应 RGB 索引时刻。schema 阶段确定具体字段，不用一个未注明时间的位姿替代两者。

记录两张图各自 frame/stamp、CameraInfo 原始模型/选择版本、有效投影参数、原始与标准媒体、bag 来源、clock/segment、配对策略/时差、TF 查询与质量。未定位/坏帧保留证据进入 pending。

关键帧依据有效底盘位姿的位移/转角及明确采样策略选择，输出 observation_id 引用；处理首帧、静止、缺姿态、定位跳变，不把每张解码图像自动称关键帧。BagBuilder 负责 preparation 与原始证据。离线构建组织关键帧和业务观测，完整记忆版本发布时导出 observations.jsonl 及相关证据。

## 5. S3：原文第 1 部分的数据与几何业务

### 5.1 实体、空间和关系

Entity 同时支持 kind=object/region，永久 ID、semantic_label、名称/别名/属性、状态、观测引用和 revision。物品位置是已观测表面的代表点；空间代表点必须引用确认在内部的 body_pose_map，polygon_map/orientation_map 未确认则留空。

SemanticRelation 支持有证据的 near/on/hanging_on/inside 及 connects_regions 多端点连接。数值 near 保留距离定义、阈值、实测距离和 geometry_revision；on/hanging_on 不从中心高低或距离硬推。空间 inside 与 region_id 在同一修改中同步，指向物品的 inside 不改空间索引。

### 5.2 project_depth 与索引

point/bbox/polygon 使用原图像素，适配先还原缩略/拼图坐标；验证边界。根据有效相机模型和深度计算 map 点；区域返回可信簇、代表点、点数/范围/背景与多簇歧义，include_points 返回点集引用。invalid_depth/missing_map_pose/ambiguous_region 等错误保持明确依据。

工作库优先使用 SQLite 单 writer 支持事务与恢复，再导出原文 1.6 JSONL。基本 ID/名称/区域成员/关系端点索引必须有效；空间检索先用可验证的小规模实现，向量索引为可选增强。存储选择是内部实现方案，不扩展成独立在线服务。

## 6. S4–S5：MemoryEngine、八工具及发布

### 6.1 构建工具范围

| 工具 | 本轮用途 |
| --- | --- |
| search_entities | 构建时召回旧对象/空间与 ID/revision/证据 |
| get_observation | 取回原始 RGB/Depth、标定、相机/底盘 map 位姿与关联选区 |
| project_depth | 程序计算像素点/区域的 map 几何与质量 |
| spatial_query | 构建需要的有效 near/on/hanging_on/inside、关系/几何/观察依据；数值结论版本匹配 |
| create_entity | 经观测/定位/引用校验后建立永久 ID、几何与关系 |
| update_entity | expected_revision、字段合并、观测追加、关系及几何更新 |
| correct_memory | update/merge/split/retract，理由/证据/版本、引用重定向或重分配、整体提交 |
| memory_annotate | 人工/模型共用名称、别名、备注处理及注释历史/名称索引刷新 |

### 6.2 模型循环与 Agent 复用

MemoryEngine 分批组织 1–4 张观测、相关旧实体与历史视图，运行 function calling。setTools/addInputs/request/takeResponse 可经 ModelSession 适配复用现有 SessionManager；已有 previous_response_id 自动续接时不能重复追加整段历史，必要时新实例建立新批次上下文。

核心业务不依赖 API 外壳。当前 vln_agent 的 functionCallOutput 使用内容数组，而原文示例业务载荷是 JSON 字符串：由适配层对接并用实际服务验证文本/图像与工具协议，不能直接改坏现有客户端。api_key_env、模型地址/超时等配置仍需显式落实，不假定现有配置原生支持。

BuildDispatcher 持有原始 call_id/name/arguments，执行前验证并持久记录调用结果。ToolManager 的活动去重不能替代该记录。调用修改/执行结果在同一事务提交，重试同 call_id 复用结果；冲突参数返回错误。批次进度与调用结果分开，超时/拒绝/工具轮数耗尽不能误判完成，恢复时从已提交工作库重组上下文。

### 6.3 修改、纠错和版本

物品多视图沿用 ID；同图不同实例独立 ID。证据/位置矛盾进入 unresolved/needs_review，未定位新实体不以原点入库。静态物品融合与移动物体的时间状态分开，空间沿用确认的内部代表点。

update/merge/split/retract 必须在一个事务里维护引用、关系、索引、region_id 与历史；自动更新遵循字段白名单并保留人工名称、别名和原始证据。几何/观测归属变化使受影响关系进入复核、旧数值缓存失效；更新 geometry_revision 后重算几何，不能只改版本号。

导出版本包含 manifest、entities/observations/relations/annotations JSONL、evidence/reports/indexes；JSON 不能直接写 NaN/Inf。先验证完整性再原子发布 manifest，构建工作库/临时文件不能成为有效完整版本。memory_version、geometry_revision、记录 revision、schema_version 含义分别固定，不能混用。

## 7. S6：原文要求的 MemoryViewer

Viewer 需在 map 中展示全部已定位实体与语义连接，联动 RGB/Depth 与像素选区、机器人轨迹和观测位姿；可以按实体、关系、时间定位观测。

名称、别名、备注通过与 MemoryEngine 共用的 memory_annotate 处理函数写入并记录历史，重开后回读；不得只修改 Viewer 内存或任意改已发布 JSONL。采用单 writer/事务与版本发布，先实现可检查、可标注的闭环，不另建一套不一致的存储。

本轮不将 Viewer 扩展成自然语言在线导航工作台。其原文 2.10 明确要求的可视化/定位/标注能力仍是必需验收项。

## 8. 目录与依赖约定

真实源码仍以 include/vln_memory/bag、src/bag 和 test/bag 分类，详细树见 [当前架构](architecture.md)。新增模块已位于 bag/builder，包含配对、读取、标定选择、配置、媒体/JSONL 和调度；数值几何/正式 Observation 继续分步落实。

后续真实业务实现时才创建 memory（模型/几何/存储/修改）、engine（上下文/会话适配/工具构建）、viewer（检查标注）、apps（薄命令入口）与 config。当前不创建 plugins 在线查询目录，也不注册未实现工具，不生成空目标。vln_agent 不反向依赖这些准备库，模型会话适配保持单向复用。

文档：project 为当前范围/实施计划，interfaces 为已实现接口，modules 为同名代码说明与 tests，reviews 保留历史核查/后续修复记录。规划接口与真实已实现接口必须区分。

## 9. 本轮验收清单

完整验收以 [第 1、2 部分对照第 7 节](sections_1_2_scope.md) 为准，至少包括：

- 真实 bag 到可回查的观测/关键帧/证据和 pending；
- 已知几何投影、多簇/无效输入与独立 RGB/Depth 历史位姿；
- 八工具与真实模型多视图构建；永久 ID/空间归属/版本一致性；
- 幂等恢复、四类纠错、关系刷新、人工信息保留；
- 原文文件集合、有效索引、单 writer 原子发布；
- Viewer 按实体/关系/时间联动检查和标注回读；
- 三个入口使用实际 config/vln_memory.yaml，支持字段和错误有验证。

投影基线 6d809e8 的无 ROS 核心测试 10/10、非录包 ROS 测试 15/15 通过，两种安装模式的投影消费者独立编译运行通过。真实全包提取仍以 v4 的 [相机模型验收](../reviews/camera_model_verification.md) 为准。v5 的真实位姿覆盖、投影落点和当前配对准备状态待验证。原文第 1、2 部分完成前，README 和进度报告不得写成完整离线记忆系统已经交付。

## 10. 当前最近行动

### 10.1 2026-11-03 相机模型进展

D8 兼容、原始/派生标定、reader/索引/直接 ROS 匹配贯通，raw/rectified 有效模型和像素射线已实现，见 [实施](camera_model_implementation.md) 与 [验收](../reviews/camera_model_verification.md)。

BagBuilder v4 的历史全包验收：21441 消息、7268 原始 D8、6889 成功解码图像及模型，30 ms 下 2671 对。原始参数、时间及 CDR 保留。当时的 prepared_pairs=0，原因是 v4 验收所用的精确动态 TF 策略与注册依据不足。完整媒体和临时输入已于 2026-11-03 按用户要求清理；build/camera-model-20261103 仅保留小型摘要、日志、配置和审计快照，当前构建/安装保留。完整数据集需从原始 ZIP 重建，见 [清理记录](../reviews/build_cleanup_20261103.md)。

### 10.2 后续任务与依赖

| 顺序 | 工作 | 交付/验收 |
| --- | --- | --- |
| 已实现 | TF 数值位姿 | 复用 checkTime 端点完成插值、求逆和链组合。各图用自己的时间，body 用 RGB 时间。按本轮要求使用构造样本和 tf2 对照验证 |
| 已实现 | 单深度像素到 map | 复用米制 optical-z、有效相机模型与该帧历史 TF 位姿，输出一个地图点或明确失败码。见 [单点投影接口](../modules/depth/depth_projection.md) |
| 已实现 | 单点 Depth→RGB | 按两帧各自时刻的位姿求几何候选像素。见 [单点投影接口](../modules/depth/depth_projection.md) |
| 已实现 | 深度媒体压缩 | 通过 bag.depth_storage 选择源 16UC1 无损 PNG，保留旧米制 NPY。每帧记录恢复米制值所需的元数据 |
| 固定样本通过 | 真实短 bag 数值链路 | v6 PNG/NPY 回读、原始 TF 端点复核和三个投影点均通过。物理尺度和 RGB/Depth 注册精度仍需外部参考 |
| 单对读取已实现 | preparation 读取与引用 | loadPreparedPair 使用已保存的配对 ID，校验来源、标定、位姿和媒体。复用 v5/v6 准备数据并保留 pending 原因。见 [读取接口](../modules/bag/preparation_reader.md) |
| 随离线构建推进 | 关键帧/批次输入选择 | 依据有效位姿和明确采样规则选择需要的观测，保留稳定来源和像素坐标；与 API 输入组织协同落地 |

### 10.3 后续业务

继续 S3 几何投影/实体关系/SQLite 工作库与索引 → S4 八工具/真实模型批次及幂等恢复 → S5 纠错/人工标注/关系刷新/完整版本原子发布 → S6 Viewer 联动与端到端验收。当前仍不展开完整在线查询与导航。原文第 1、2 部分的完整验收目标保留；已有 preparation 可作为后续观测组织的输入。
