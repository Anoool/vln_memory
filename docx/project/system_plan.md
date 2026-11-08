# VLN 空间语义记忆规划：当前只交付原设计第 1、2 部分

更新：2026-11-08。本规划依据用户提供的 VLN_SPATIAL_SEMANTIC_MEMORY_DESIGN.md（更新 2026-10-27）及用户最新范围要求，替换此前同时展开完整在线查询/导航接入的总体路线。原始附件不修改。

**当前目标：原文第 1 部分“核心约定与数据规范”及第 2 部分“离线构建流程、工具与规范”。当前已实现观测准备层基础与 BagBuilder preparation 导出，有效模型与数值位姿已接入 preparation，区域投影、物品创建和空间创建已接入，两个原文部分都未完成。**逐项证据见 [范围与进度对照](sections_1_2_scope.md)。

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

| 模块 | 当前能力 |
| --- | --- |
| RGB / Depth / CameraInfo / TF | 图像解码、米制 optical-z、显式时钟和数据段、相机模型与原始标定、带来源的历史数值位姿。 |
| BagBuilder / PreparationReader | 从 SQLite 录包配对和导出，按保存的配对 ID 索引回读，支持完整像素与元数据两种模式，新导出绑定媒体摘要。 |
| BuildInput / Observation | 按用途选择配对、按位姿选关键帧并分批；已实现正式观测、JSON 校验、组装、带文件摘要的数据集发布及按 ID/批次回读。 |
| 深度几何 / Entity | 单点 Depth→map、Depth→RGB 候选像素及深度原图 bbox/polygon；物品保存表面代表点，空间使用经确认的历史底盘锚点。 |

实体为拥有型记录，可经 SQLite 工作库事务写入、追加观测、补定位和按 ID 回读。关系、原文八个构建工具、完整版本发布和 Viewer 待实现。vln_agent 的 SessionManager/ToolManager 可作为未来的会话与工具适配基础，实际复用边界见 [范围对照第 6 节](sections_1_2_scope.md)。当前层次、命令和产物见 [架构](architecture.md)。

## 3. 实施顺序与逐步验收

| 阶段 | 原文对应 | 交付 | 验收重点 | 状态 |
| --- | --- | --- | --- | --- |
| S0 数据契约/配置 | 1.1–1.6、2.10 | map/clock/来源/版本、Observation 与实体/关系字段规则，配置 schema/加载与校验 | 单位、引用、缺项、版本语义明确；配置不静默猜数据含义 | 部分完成：传感器、preparation、Observation 和 Entity 的基础契约已实现；关系与完整系统配置待补 |
| S1 自动配对与读取 | 2.1、2.10 | 先实现无 ROS rgbd_sync，再 reader、话题类型/编码与时间索引 | 确定配对、平局/一对一、时钟段边界、来源可回查 | 主要能力已实现；全包时间索引与真实短片段提取已验证，物理同步/尺度/注册仍待验收 |
| S2 完整离线观测 | 1.3、2.1、2.3、2.10 | 有效相机模型、TF 位姿求解、注册/时间质量、关键帧、证据与 Observation 输出 | 真实 bag 的两图时间/位姿、标定与文件引用一致，缺项进入 pending | 已有标定兼容、有效模型、数值位姿和单点 Depth→map/Depth→RGB；输入用途筛选、位姿关键帧和批次清单已实现；正式 Observation 记录、批量组装与数据集发布已实现，物理几何依据待完善 |
| S3 几何与工作库 | 1.2/1.4/1.6、2.3–2.7 | 点/区域到 map、对象/空间、关系、事务存储、基本索引与版本 | 已知几何答案、永久 ID、代表点、inside/region_id、revision | 部分完成，已有区域投影、物品及空间创建和 SQLite 事务工作库 |
| S4 工具与模型构建 | 1.5、2.1/2.2/2.4/2.6 | 八工具、上下文/图像、批次循环、进度/幂等恢复 | 真实模型服务、多视图 ID 复用、轮数上限/失败不误报完成、重复 call_id 不重复写 | 待实现 |
| S5 纠错/标注与发布 | 2.7–2.10 | 四纠错、关系刷新、注释历史、完整记忆产物和原子 manifest | 整体提交/引用修复、人工信息保留、几何缓存失效、半成品不发布 | 待实现 |
| S6 Viewer 与闭环验收 | 2.1/2.9/2.10 | 地图/关系/图像/轨迹联动、按实体/关系/时间定位观测、人工命名与回读 | 同一发布版本可检查，标注走共用写入路径，重开后结果一致 | 待实现 |

S1 和 S2 已形成可复用的数据链。S3 已能创建物品和空间，在原 ID 上追加观测证据。已能选择现有观测证据为原实体补定位。下一项是实体关系。

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

录包读取器处理存储、类型、反序列化、来源和稳定顺序。PreparationReader 读取已保存配对，分别提供元数据与完整媒体入口。录制中的时间重启应结合采集依据处理，保持原有数据段。

### 4.3 历史位姿与相机几何

TF 模块已在链与样本选择之上实现插值、逆变换和组合。Builder 导出 map 相机与底盘位姿、查询时刻和端点来源。时间超限和插值跳变通过独立状态报告。

CameraModel 已实现实际网格 K/D/P、ROI/binning 和单点射线：raw 去畸变，rectified 经 P 内参与 R 逆旋转。非 optical-z 值在模型明确后换算；未注册深度先按标定/外参重投影。支持范围按真实输入验证，未支持的组合进入明确错误/pending，不能默认生成单位模型。

### 4.4 Observation、关键帧和证据

已发布的 Observation 聚合两图、各自模型与位姿、底盘位姿和来源，采用 RGB 原始时间作为参考 stamp。观测数据集入口复用 PreparationReader 与现有选择逻辑，具体契约见 [Observation 参考时刻与字段关联](observation_time_contract.md)。

stamp 等于 rgb_stamp，rgb_stamp 与 depth_stamp 保留各自原始时间，配对差值及门限继续保存。camera_pose_map 和 body_pose_map 对应 RGB 参考时刻，depth_camera_pose_map 对应 Depth 时刻。各路模型按图像网格和采样时刻绑定。单一 width/height/intrinsics 视图需有共同网格、光轴、rectified 模型和注册依据。

记录两张图各自 frame/stamp、CameraInfo 原始模型/选择版本、有效投影参数、原始与标准媒体、bag 来源、clock/segment、配对策略/时差、TF 查询与质量。未定位/坏帧保留证据进入 pending。

关键帧依据有效底盘位姿的位移/转角及明确采样策略选择，输出 observation_id 引用；处理首帧、静止、缺姿态、定位跳变，不把每张解码图像自动称关键帧。BagBuilder 负责 preparation 与原始证据。离线构建组织关键帧和业务观测，完整记忆版本发布时导出 observations.jsonl 及相关证据。

## 5. S3：原文第 1 部分的数据与几何业务

### 5.1 实体、空间和关系

当前 Entity 支持 kind=object/region、永久 ID、semantic_label、状态、观测引用与 revision。物品使用实测表面代表点，空间使用经确认在内部的历史 body_pose_map。名称、别名、属性、空间边界和朝向随相应业务及证据规则接入。

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

现有源码按 `bag`、`memory`、`engine` 和 `apps` 分层。准备层提供传感器、几何及回读原语，memory 层维护业务记录，engine 层组合输入、发布观测和创建实体，apps 层提供命令。源码树与依赖见 [当前架构](architecture.md)。

后续工作库和关系放在 memory，工具与模型批次放在 engine，检查及标注放在 Viewer。模型会话适配可单向复用 vln_agent。

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

工作库基线的核心 CTest 为 16/16、ROS CTest 为 29/29。真实短包验证了媒体回读、历史位姿、数值投影及物品写入和重开回读。已定位空间使用带明确内部确认的构造录包验证。第 1、2 部分的完整交付仍以上述各项验收为准。

## 10. 当前最近行动

### 10.1 已可用的链路

`bag_builder` 导出带来源和媒体摘要的 preparation；`memory_observation_dataset` 复用 Reader、关键帧和批次逻辑发布正式观测；C++ 接口从同源观测创建物品或空间。各模块用法见 [文档导航](../README.md)。

### 10.2 后续任务与依赖

| 顺序 | 模块 | 交付与验收 |
| --- | --- | --- |
| 1 | SQLite 工作库（已实现） | 单写入者事务、实体按 ID 写入和回读、精确观测引用。已定位及待定位物品和空间在重启后保持原 ID；无效引用或写入错误整笔回滚。 |
| 2 | 实体更新与关系 | 已支持按 expected_revision 向原 ID 追加证据和补定位。定位引用可按观测 ID 和修订回读。下一步实现有证据的 inside/near，空间归属与关系在同一事务更新，数值结果绑定 geometry_revision。 |
| 3 | MemoryEngine | 复用正式观测数据集读取器，接入原文八项构建工具。先用固定调用验证多视图复用和 call_id 幂等，再用真实模型完成小批次。 |
| 4 | 纠错、标注与版本发布 | 实现修改、合并、拆分、撤回和人工标注历史，校验实体/观测/关系引用与索引后原子发布完整版本。 |
| 5 | MemoryViewer | 地图、实体、关系、图像选区及轨迹联动；通过共用标注入口修改名称等字段，重开后可回读。 |

### 10.3 设备验收

物理深度尺度、RGB/Depth 注册与曝光同步可与业务模块并行验收。现有短包的数值对照基于当时配置的模型、比例和轨迹；这些声明及其未知状态继续随观测和证据保留。

### 历史验收

- BagBuilder v4 的历史全包记录为 21441 条消息、7268 条原始 D8 标定、6889 帧成功解码图像及模型，30 ms 下 2671 对。该阶段的精确动态 TF 策略和注册依据使 prepared_pairs 为 0。见 [相机模型验收](../reviews/camera_model_verification.md) 与 [产物清理](../reviews/build_cleanup_20261103.md)。
- 独立选帧、正式记录和 Reader 组装边界的阶段验证见 [BuildInput](../reviews/build_input_verification.md)、[Observation](../reviews/observation_verification.md) 与 [Reader](../reviews/preparation_reader_assembly_metadata_verification.md)。
