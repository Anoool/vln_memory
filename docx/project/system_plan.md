# VLN 空间语义记忆系统整体规划

更新：2026-10-28。本文保存整个系统的目标、边界和分步实现顺序。它是设计规划，不能将未来模块视为已经实现。

**当前开发范围：观测准备层 bag。已实现 RGB、Depth、CameraInfo 与候选时间检查；历史 TF、bag 读取/自动配对/导出尚待实现。**现有源码和测试的分类见 [当前架构](architecture.md)，各模块详细接口见 [文档导航](../README.md)。

## 1. 系统目标与数据流

将 ROS 2 RGB-D bag 转成可追溯的空间语义记忆，供 Agent 查询并辅助导航：

```text
ROS 2 bag
  -> 观测准备层 BagBuilder
  -> Observation JSONL + RGB/Depth/CameraInfo/TF 原始证据
  -> MemoryEngine 分批观察、召回旧记录、调用模型工具
  -> 记忆核心校验与提交：实体、区域、关系、纠错和标注
  -> 完整、不可变的记忆快照
  -> Agent 查询插件 / MemoryViewer
  -> 导航适配层选择可达接近点，再调用现有导航工具
```

世界几何统一为明确的 map 基准，单位米、弧度。像素选区由模型提出，世界坐标由深度、标定和历史位姿计算。物体表面代表点不直接作为机器人导航终点。

## 2. 分层职责和当前状态

| 层 | 主要职责 | 当前状态 |
| --- | --- | --- |
| bag：观测准备 | 传感器消息处理、时间匹配、标定/TF、媒体和 JSONL 导出 | 正在开发；RGB/Depth/CameraInfo 已实现 |
| memory：记忆核心 | 几何、实体/区域/关系、查询、修改、工作库、快照 | 规划 |
| engine：离线构建 | 上下文组织、模型会话、工具分发、进度与恢复 | 规划 |
| plugins：Agent 接入 | 将只读记忆查询接入现有 ToolManager | 规划 |
| viewer：检查与标注 | 地图、轨迹与证据联动，人工复核入口 | 规划 |
| apps：薄命令入口 | prepare/build/inspect/annotate 等参数解析和模块组装 | 规划 |

层之间通过公开数据或工具契约结合。记忆核心不依赖 ROS、模型客户端或 Agent 工具类型；适配代码负责协议转换。只为已经开始实现的模块创建源码，不为未来层生成空类或空目录。

## 3. 当前观测准备层的五个模块

这里的 bag 同时是层名。其内部第五个“bag 读取与组装模块”使用 builder 目录，避免 bag/bag 含义不清。

| 模块 | 输入 | 输出 | 源码归属 | 状态 |
| --- | --- | --- | --- | --- |
| RGB | 原始图像字段 / ROS Image | 标准 rgb8、采样身份与错误 | bag/rgb | 已实现 |
| Depth | Image、显式单位/深度含义、注册声明 | 米制 float32、质量、RGB 网格检查 | bag/depth | 已实现 |
| CameraInfo | 标定消息、真实 topic、图像绑定/时间策略 | 完整标定与图像尺寸/frame/时间检查 | bag/camera_info | 已实现 |
| TF | TF/TFStatic 与采样时刻 | map 中的相机、底盘位姿及依据 | bag/tf | 待实现 |
| builder | bag、提取配置及以上四模块 | 完整 Observation、证据和报告 | bag/builder | 待实现 |

SensorStamp、ImageGeometry 以及显式时钟关系的候选时差检查位于 bag/common。它们是观测准备层共用类型，不提前承担未来 Entity、Relation 等业务。

### 3.1 RGB 与 Depth 的已实现边界

- RGB 校验 rgb8/bgr8/mono8，输出独立持有的 rgb8，保留原图尺寸和采样身份。
- Depth 支持 16UC1/32FC1，必须显式指定米/毫米与 optical_z；处理大小端、行填充和无效值。
- Depth 对齐检查验证“已声明注册”与实际 RGB 网格元数据一致，不靠相同尺寸推断配准，不完成标定或时间同步。
- 二者输出内存数据。原始证据保存、文件命名、跨消息同步和最终 JSONL 由 builder 统一完成。

详细说明：[RGB](../modules/rgb/rgb.md)、[Depth](../modules/depth/depth.md)。

### 3.2 CameraInfo 已实现范围与 TF 后续目标

- CameraInfo 保留真实 topic、header、K/D/R/P、畸变模型、ROI、binning，校验完整标定、显式光学 frame 和图像尺寸。raw/rectified 必须声明；校正 ROI 需要后续几何计算时明确返回不支持。
- 固定标定可按显式策略和依据复用；逐帧标定核验同一时钟与时间容差。自动选消息、动态标定有效区间由后续 builder 实现，不能整包取最后一条。
- TF 按采样时间处理 map<-body、body<-camera_optical 及组合变换，支持静态与动态链。
- 历史查询使用索引/分块窗口，不依赖默认短时 Buffer 覆盖整包，不用 latest 填历史位姿。
- 缺失变换、外推超限、重定位跳变等形成明确报告，不写入假位置。

CameraInfo 详细规划与接口见 [模块说明](../modules/camera_info/camera_info.md)。公共时差检查不自动改变任何时间；采用哪个时刻、写入与在线控制规则见 [时间约定](../common/sensor_time.md)。

### 3.3 BagBuilder 的目标流程

1. 检查 bag 存储、话题类型、编码、标定和 TF 时间覆盖。
2. 第一遍建立消息时间、CameraInfo、TF 与来源索引。
3. 以 header.stamp 为依据，确定 RGB-D 配对、标定和历史位姿。
4. 第二遍调用各消息模块，解码并保存原始与标准化媒体。
5. 输出有效 Observation、pending、异常与质量统计。
6. 导出关键帧引用、检查文件与引用完整性，再发布完整数据集。

默认保留有效配对，关键帧使用 observation_id 索引，便于后续重选。同步容差、深度帧复用和标定匹配策略必须明确。记录 bag 接收时间与传感器采样时间，不混成同一个时间。

### 3.4 Observation 交接契约

一条观测对应一组明确配对的原始 RGB-D，各张图保留独立时间。主要字段：

- id、schema_version、clock_id、map_id、frame_id、geometry_revision；
- 以 rgb_stamp 为观测索引，分别保留 rgb_stamp、depth_stamp、CameraInfo 原始 stamp 与 bag 接收时间；媒体相对路径、尺寸与原始格式；
- camera_model、intrinsics、完整 CameraInfo 引用；
- depth_unit=m、depth_convention=optical_z；
- RGB 与 Depth 各自采样时刻的 camera_pose_map；RGB 索引时刻的 body_pose_map，quaternion_xyzw；明确标记每个位姿的查询时刻，不能把一个模糊的 camera_pose_map 复用于两个时刻；
- bag 来源、源消息 ID、同步时差和有效深度比例。

缺深度、标定或位姿的记录保留证据进入 pending，不以原点或单位旋转补齐。纯观测数据集还不是语义记忆，不提前生成 memory_version。

```text
dataset/session_01/
  manifest.json
  observations.jsonl
  camera_info.jsonl
  tf_dynamic.jsonl
  tf_static.jsonl
  keyframes.jsonl
  evidence/
  reports/pending.jsonl
  reports/issues.jsonl
  reports/summary.json
```

## 4. 记忆核心层目标

### 4.1 数据模型

- Entity：物品与 kind=region 空间共用永久 ID、名称/别名、描述、属性、观测引用和 revision。
- 物品位置表示已观测表面的代表位置，保留逐观测几何与已观测范围，支持重算和拆分。
- 空间代表点必须引用确认位于内部的机器人位姿；polygon_map 可暂缺。
- Relation：near/on/hanging_on/inside，以及连接、功能等关系；支持有角色的多端点关系，如门连接两个空间。
- 人工标注、来源和修订历史独立记录；自动修改使用字段白名单保护人工确认。

### 4.2 查询与修改

QueryService 只取得只读视图，统一名称/属性/区域/关系检索和空间计算。MutationService 持有可写工作库，集中维护创建、更新、纠错与标注。

- inside(object, region) 为有效空间归属事实，region_id 是同步派生索引；物品 inside 柜子不改空间归属。
- near 定性结论与数值距离有不同依据，距离注明 xy/xyz、代表点/表面及几何版本。
- on/hanging_on 使用视觉或可靠几何依据，不能用中心点高低远近代替。
- merge/split/retract 在一个事务内修复观测、几何、关系、索引与旧 ID 引用。
- 静态多视图融合与移动物体时间状态分开；位置冲突不平均成中间假位置。

### 4.3 工作库、证据和快照

构建时 SQLite 工作库是唯一事实源；JSONL 是检查通过后的导出，索引可重建。原始证据不可变，保存来源与校验信息。

每个 release 包含完整的 manifest、observations/entities/relations/annotations JSONL、证据、报告与索引。先写临时目录并校验，再形成不可变版本，最后原子切换 current.json。查询固定同一版本；临时计算缓存放在快照外，不回写已发布记录。

区分 schema_version、已发布 memory_version、geometry_revision 与单记录 revision。更新轨迹/标定后重算几何并使旧数值关系失效，不能只改版本号。相同 map_id 不能证明多个 bag 已配准。

## 5. MemoryEngine 目标

MemoryEngine 管批次状态、上下文、重试、取消与恢复；ContextBuilder 召回少量相关实体与历史共视图；ModelSession 包装现有模型会话；BuildDispatcher 处理完整 function_call 与本地执行记录。

目标工具：

- 查询：search_entities、get_observation、project_depth、spatial_query。
- 写入：create_entity、update_entity、correct_memory、memory_annotate。
- 构建控制：finish_batch，明确结束当前批次。

查询定义只维护一份，构建时合并写入/控制工具；未实现工具不提前注册。

按批次使用短模型上下文，持久事实以工作库为准。每次工具修改和执行结果在同一事务提交，不跨模型请求持有写事务。同 call_id 的参数变化报冲突；新 call_id 的重复内容仍受观测、选区、身份和 revision 等约束。

工具提交与批次完成分别记录。空回复、拒绝、超时、轮数上限不自动算完成；支持 completed_with_pending。重启从已提交事实重组上下文，不假设服务端链或 transcript 一定能精确恢复。

## 6. 与现有 vln_agent 的结合

| 现有组件 | 规划的复用方式 | 边界 |
| --- | --- | --- |
| SessionManager | 离线 ModelSession 持有独立实例 | 遵守现有 previous_response_id 增量链，不重复追加整段历史；无 reset 时通过新实例建立新链 |
| ToolManager / ToolPluginBase | 四个在线只读查询插件 | execute 只有 arguments，不能当作离线持久幂等入口 |
| SharedResources | 同一管理器内四插件共享 MemoryRuntime | 固定只读快照，核对目录与版本，确保在途查询引用有效 |
| ToolResult / utils | 在适配层包装数据和实际图片 | 核心不依赖 API 外壳；get_observation 要提供模型能看到的图像 |
| EnvironmentMemory | 保留原有逐观测历史用途 | 其局部对象 ID 不直接冒充全局 Entity |
| Nav2 工具 | 上层选择可达接近点或观察点后调用 | 物品表面位置不是机器人目标位置 |

模型服务需验证图像、工具、WS、续接与可选字段兼容性；“OpenAI-compatible”不足以证明全支持。api_key_env 属于需明确落实的配置扩展，不假定现有客户端已支持。

当前 RGB/Depth 不链接 vln_agent。适配在未来 engine/plugins 层实现，不形成 vln_agent 与 vln_memory 的循环依赖。

## 7. MemoryViewer 与人工复核

地图实体/关系、机器人轨迹与采样位姿，联动 RGB/深度、选区和点集证据。显示版本、状态和关系依据。

Viewer 初期只读快照；人工名称、别名和备注通过受控标注入口，在构建停写时使用同一修改事务并发布新版本。需要构建中编辑时，再向唯一 writer 提交命令。人工修改不直接改 JSONL，也不另建不受控 writer。

## 8. 源码与文档组织

当前真实代码以层/模块分类：

```text
include/vln_memory/bag/
  common/sensor_types.hpp
  rgb/rgb.hpp, rgb_ros.hpp
  depth/depth.hpp, depth_ros.hpp
src/bag/
  rgb/rgb.cpp, rgb_ros.cpp
  depth/depth.cpp, depth_ros.cpp
test/bag/
  rgb/test_rgb.cpp, test_rgb_ros.cpp
  depth/test_depth.cpp, test_depth_ros.cpp
```

后续 CameraInfo、TF、builder 分别在 bag/camera_info、bag/tf、bag/builder 增加真实实现。记忆核心、构建、插件、Viewer 层开始实现时，才建立 memory、engine、plugins、viewer 等同级源码目录；命令行参数组装再放 apps。

include 与 src 使用相同层/模块布局；导出的头文件保持该目录结构。现有函数、命名空间和库目标保持，调用方更新 include 路径。安装后独立消费者用来检查实际公开接口。

文档入口为 docx/README.md：project 保存系统目标与目录/构建约定；common 保存共享类型说明；modules/rgb、modules/depth 保存当前 bag 子模块的同名代码说明，tests/ 保存测试说明。未来新层的文档再按真实模块扩展，不生成空文档。

## 9. 分步实施与验收

| 阶段 | 交付 | 验收 | 状态 |
| --- | --- | --- | --- |
| bag-1 | RGB、Depth 核心与真实 ROS 适配 | 已知数据、格式/单位/对齐边界、安装消费者 | 已完成 |
| bag-2 | CameraInfo 与历史 TF | 标定匹配、已知变换、缺失/超限/跳变报告 | 下一阶段 |
| bag-3 | bag 读取、同步、证据和 Observation JSONL | 短真实 bag、引用完整、统计一致、可回查落点 | 待实现 |
| memory-1 | 几何、实体/区域、工作库与只读查询 | 投影、同物体多视图、同类实例区分 | 待实现 |
| engine-1 | 工具构建、批次终态与恢复 | 提交后中断不重复、轮数耗尽不误报成功 | 待实现 |
| memory-2 | 关系刷新、快照与在线查询插件 | 版本固定、归属一致、旧距离失效 | 待实现 |
| review | 完整纠错、标注和 Viewer | merge/split 引用修复、人工信息保留、证据联动 | 待实现 |

优先完善当前 bag 层，不并行搭建未用到的其他层空壳。每个小模块遵循“规划与自审 -> 实现与注释 -> 针对性验证 -> 整体通读 -> 分类同名说明 -> Git 暂存审阅”。已有修改留在 codex/ 分支，合并由用户审阅后决定。
