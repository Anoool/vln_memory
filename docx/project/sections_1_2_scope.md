# 原设计第 1、2 部分：范围、现状与验收对照

更新：2026-11-03。依据为用户提供的《VLN_SPATIAL_SEMANTIC_MEMORY_DESIGN.md》（文内更新 2026-10-27）。本轮用户要求：当前只完成原文第 1、2 部分。原始附件只读，不修改；本文为仓库内部的范围与追踪文件。

这里的“部分 1、2”指原文的顶层标题，不是同步方案里的交付 A/B，也不是仅完成 RGB/Depth 两个子模块：

- 第 1 部分：核心约定与数据规范（1.1–1.6）。
- 第 2 部分：离线构建流程、工具与规范（2.1–2.10）。

## 1. 当前实际完成到哪里

当前 vln_memory 的源码仍集中在 include/vln_memory/bag 与 src/bag：RGB、Depth、CameraInfo、TF 链/覆盖、公共时间与名称检查。新增 builder 第一版及 config/vln_memory.yaml 的 bag 段，已能读取/转换/配对并输出 preparation 文件；没有 engine/memory/viewer 与正式 Observation，也没有原文八个业务工具。实际接口见 [BagBuilder](../interfaces/bag_builder_api.md)。

因此：原文 1.1、1.3、2.1、2.3、2.10 有部分准备代码，其余业务尚未实现；没有任何一个原文顶层部分可以标记为完成。已有原始 TF 不是 map 数值位姿；现有 builder 已有实际时间配对，但不证明物理同步或空间注册；已有 float 深度不是 project_depth。

状态定义：“基础部分”表示有可复用原语但未满足条款完整验收；“未实现”表示在 vln_memory 中没有对应业务入口/产物。原文只作为目标依据，不将其中示例当现有文件。

## 2. 第 1 部分逐项对照

| 原文条款 | 当前状态 | 已有代码/证据 | 尚缺的实际能力 |
| --- | --- | --- | --- |
| 1.1 坐标、单位与信息保留 | 基础部分 | 米制 optical-z、SensorStamp、CameraInfo 原值、TF 父子链/样本来源；原始 CDR 和标准媒体已落盘 | 统一 map_id/geometry_revision/单位契约，真正 map 相机/底盘位姿、像素到 map 与空间关系依据 |
| 1.2 Entity（含 1.2.1 空间） | 未实现 | 无 Entity/Region 模型或存储 | 永久 ID、物品/空间统一实体、已观测代表点、可选边界/朝向、证据/revision、未定位 pending |
| 1.3 Observation | 基础部分 | 已导出 RGB/Depth 媒体；frames/pairs 保存尺寸、原始时间、媒体路径、标定候选及 TF 端点/来源 | 正式 Observation 结构/校验/序列化、关键帧及正式引用校验。数值位姿与有效模型已在准备数据中实现 |
| 1.4 SemanticRelation | 未实现 | 无语义关系业务 | near/on/hanging_on/inside、门的多端点 connects_regions、证据/状态、去重、inside 与 region_id 同步 |
| 1.5 版本、证据与工具规范 | 未实现（有外部通用基础） | vln_agent 有会话与通用 function_call 外壳；vln_memory 没有业务工具 | 工具注册/分发、版本上下文、状态/证据校验、构建轮次与持久执行记录 |
| 1.6 文件组织与索引 | 基础部分 | 已导出 preparation manifest/来源/图像/标定/TF/配对/报告；不是完整 memory 版本 | manifest、entities/observations/relations/annotations JSONL、evidence/reports/indexes、ID/名称/空间/关系索引 |

## 3. 第 2 部分逐项对照

| 原文条款 | 当前状态 | 已有代码/证据 | 尚缺的实际能力 |
| --- | --- | --- | --- |
| 2.1 输入与总体流程 | 基础部分 | BagBuilder 两遍读取、原始/指定压缩图像解码、严格/最近配对、标定候选/TF 检查及准备导出；真实短片段已验证 | 数值几何/有效模型、关键帧、正式观测、模型构建、检查标注闭环（reader/配对/标定选择/准备导出已实现） |
| 2.2 构建工具 | 未实现 | 通用 ToolManager 不是这八个工具 | search_entities/get_observation/project_depth/spatial_query/create_entity/update_entity/correct_memory/memory_annotate |
| 2.3 深度定位 | 基础部分 | 深度米制、模型原始参数、历史链/端点选择 | TF 插值/求逆/组合，有效内参/畸变/ROI 处理，point/bbox/polygon 到 map，区域分簇/歧义、质量及失败结果 |
| 2.4 创建/更新协议 | 未实现 | 无永久实体及乐观 revision 校验 | 字段合并/观测追加、引用/版本校验、永久 ID、几何定位、关系更新 |
| 2.5 空间加入记忆 | 未实现 | 无空间实体业务 | 从确认在区域内的 body_pose_map 取代表点，证据积累，inside/region_id 一致性 |
| 2.6 同一实体/几何更新 | 未实现 | 通用 ToolManager 只对活动调用去重，不具备所需持久幂等 | 同观测区域去重、构建 call_id 执行结果复用、身份/位置冲突状态、静态融合/移动状态、轨迹更新后重算 |
| 2.7 关系刷新 | 未实现 | 无关系状态与几何缓存 | 更新/撤回/needs_review、几何版本与距离缓存失效、原始证据保留 |
| 2.8 通用纠错 | 未实现 | 无 correct_memory | update/merge/split/retract 整体提交、引用重定向/重分配、理由与 revision_history、关系/归属/索引同步 |
| 2.9 人工命名 | 未实现 | 无该实体 ID/注释接口 | display_name/aliases/note、annotations 历史、名称索引回读、Viewer 调用共用写入处理 |
| 2.10 最小实现与发布 | 基础部分 | C++17/ROS 包、分类文档、现有消息测试和安装消费者 | 完整 Observation、MemoryEngine/MemoryViewer（已实现 bag 配置/CLI/准备导出）、单 writer 进度、完整版本与原子 manifest 发布、真实数据闭环 |

截至 2026-11-03，当前源码重新构建后 ROS CTest 16/16、无 ROS 核心 9/9 通过；最新验收见 [相机模型验收](../reviews/camera_model_verification.md)。构造回归与真实 bag 时间索引/短片段验证分别记录，均不等于第 1、2 部分整体验收。

## 4. 范围收敛后，哪些必须保留

第 2.2 节的四个读取工具依然要实现，尽管它们也出现在第 3.2 节：

- search_entities：构建时召回已有实体，帮助沿用永久 ID。
- get_observation：构建/纠错时取回图像与深度证据。
- project_depth：程序计算几何，不能由模型直接生成世界坐标代替。
- spatial_query：满足构建所需的 near/on/hanging_on/inside 与证据核对；数值 near 必须带明确距离定义与版本。

第 2.9/2.10 节明确包含 MemoryViewer，不能因为“只做前两部分”就把它全部推迟。当前验收要求 map 中显示实体/关系、联动 RGB/Depth/像素选区、轨迹/位姿、按实体/关系/时间定位观测、编辑名称/别名/备注并回读。

Entity、SemanticRelation、Observation、索引和版本虽然在原文第 1 部分叫“规范”，仍需实际实现，不能只写 Markdown/JSON 示例就算完成。

## 5. 当前不作为交付目标的内容

- 第 3 部分完整在线查询编排与用户自然语言查询流程。
- 第 3.4 节的完整空间操作目录及全部视角问答：between、sort_along、left/right_from_view、intrinsic_front 等按完整在线查询需求再扩展；构建必要的基本几何不因此删除。
- 独立在线 Agent 查询插件组、导航工具接入与目标点生成。
- 向量检索、embedding、跟踪等可选增强不是默认前置条件。
- 分布式构建、多 writer、自动跨地图配准等超出本轮必要闭环的扩展。

原文 2.6 要求的“已配准数据进入同一 map、更新轨迹后重算”仍须支持；不在此新增一个自动求解任意多 bag 配准的子系统。人工标注/纠错与原始证据保留也不能删去。

## 6. 与 vln_agent 的实际复用边界

| 现有组件 | 能复用什么 | 不能计作已经完成什么 |
| --- | --- | --- |
| SessionManager | setTools/addInputs/request/takeResponse；已有 previous_response_id 续接机制 | 不是 MemoryEngine，没有自动完成批次/数据入库/八工具业务 |
| ToolManager / ToolPluginBase | 通用定义/调度/协议包装，可在适配层复用已有约定 | execute 只有 arguments；已完成 call_id 不再持有，不是持久幂等/事务层 |
| functionCallOutput | 现有响应包装支持文本与图片内容 | 不是空间记忆结果 schema；原文业务 JSON 与实际 content array 的协议要经适配并验证 |
| EnvironmentMemory | 逐观测存储/历史检索经验，可参考事务与字段处理 | object_id 是 observation 范围内的键，不是跨观测 Entity 永久 ID；无 map 几何、空间实体、通用纠错/版本发布 |

当前不修改 vln_agent 的这些已有能力。后续依赖保持 memory 业务核心独立，engine 的模型适配可以单向复用 vln_agent；不创建 vln_agent↔vln_memory 循环依赖。

## 7. 第 1、2 部分完成的最低验收

1. 一个已声明 map/时钟上下文的真实短 bag，可以生成含标定、独立 RGB/Depth 时间/位姿、关键帧、来源和证据的观测数据集；缺项形成 pending，无伪造位置。
2. point/bbox/polygon 投影有已知答案；越界、无效深度、缺 TF、区域混入背景/多簇返回明确结果。
3. 使用实际模型服务，经 Responses function calling 跑完至少一个多视图批次；八个工具有参数/返回验证及可回查执行记录。
4. 同一实体多视图沿用 ID，同图不同实例分开；空间代表点来自确认的内部底盘位姿；inside 与 region_id 一致。
5. create/update 与四种纠错均通过事务/引用/乐观版本检查；中断后同 call_id 不重复修改，人工信息与证据保留。
6. 几何/观测归属改变后相关关系进入复核，旧数值距离不会因缓存被误用；geometry_revision 与数据匹配。
7. 实际生成原文 1.6 文件集合和有效索引；完整记忆版本以单 writer 原子发布 manifest，缺半成品/断引用不能被发布。
8. Viewer 满足原文 2.10 的地图/关系/图像/轨迹联动及实体/关系/时间定位，并通过共用 memory_annotate 写入名称/别名/备注，重开后能回读。
9. config/vln_memory.yaml 的每个已支持字段在三个程序入口生效；不把原文示例的 30ms/0.001/0.8m/25deg 等数值当通用正确默认值。

以上全部通过才标记第 1、2 部分完成。RGB-D 自动配对与 preparation 导出已实现；相机兼容/有效模型已实现；TF 数值位姿已实现，最近待办为深度几何，preparation 的读取/引用/筛选在后续 API 输入组织时处理，具体交付与验收见 [整体规划第 10 节](system_plan.md#10-当前最近行动)。

## 2026-10-30 实施更新

BagBuilder 第一版已经通过实际序列化 SQLite 构造磁盘回归，包含分卷、负记录时间、媒体/JSONL 和失败发布；真实设备 bag、数值 map 位姿、有效相机模型及关键帧仍待验证/实现。进度条款不能因此标记原文两个部分完成。

## 2026-11-02 当前进度更新

- 能力基线：24af181（codex/rgbd-timestamp-alignment）；builder_version=3，schema=bag_preparation/1。省略/API 默认 0，正门限需显式配置；保留原始 stamp、真实差值和两图各自标定/TF 查询时间。
- S0 部分落地；S1 主要读取/配对能力已实现并有真实数据验证；S2 只有准备基础；S3–S6 尚未实现。
- 全包 6889 个图像 header：严格 0 对，30 ms 下 2671 对；本轮只做全包时间索引核查。一秒真实图像片段解码 25 张、匹配 7 对，prepared_pairs=0，没有生成正式 Observation。
- 当前阻塞：plumb_bob/D8 参数数目不兼容、零插值间隔下缺少精确动态 TF、深度/RGB 注册依据 unknown；数值位姿、有效模型、关键帧仍待实现。近似配对不补足曝光同步、深度物理尺度或运动补偿。
- 临时 DB 和短片段媒体已清理，audit_records 为不含完整媒体/CDR 的审计快照；下一轮验收需重新准备输入。详细证据见 [最近配对验收](../reviews/rgbd_nearest_pairing_verification.md)。

## 2026-11-03 当前进度更新

原始 D8 与 CDR 保留，统一显式兼容贯通 reader/索引/匹配/模型/输出；有效相机模型及单点射线已实现。完整真实 bag 为 21441 消息、7268 标定、6889 解码图像/模型、2671 对。TF 数值位姿、物理注册、记忆工作库/工具/Viewer 仍未实现或验收，完整原文两部分仍未完成。

按用户最新要求，不把最小 Observation 作为当前前置任务。既有 preparation 的读取、引用和筛选放在后续 API 输入组织时处理；后续业务仍须保留原设计要求的观测证据、时间、几何和引用完整性。以上旧日期的更新保留历史含义。详见 [当前系统规划](system_plan.md#10-当前最近行动)。

### 2026-11-03 TF 数值位姿更新

BagBuilder v5 复用历史端点求数值位姿并写入 preparation。省略 TF 门限时按本段动态样本间隔估计，显式设置优先。构造样本验证结果见 [TF 数值位姿](../modules/tf/tf_pose.md)。单个深度像素的 [map 投影](../modules/depth/depth_projection.md) 已实现。跨流重投影、真实深度几何验收与后续记忆业务仍待完成。
