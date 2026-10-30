# TF 可读性整理

日期：2026-10-30。依据仓库 AGENTS.md，本轮只整理 TF 的数据契约和代码表达，不新增位姿求解，也不改变索引、时间覆盖、冲突阻断和错误优先级。

## 编码前规划

- 输入仍是原始 TF 或真实 TFMessage，以及明确的时钟/数据段、来源与查询时刻；输出仍是插入结果、链/覆盖报告和历史样本端点。
- 将 TfTransform 的来源 topic、message_id、transform_index 直接放在顶层，移除只包装这些字段的 TfSource。
- 将 TfTimeOptions 的 clock_id、segment_id、max_interpolation_gap_ns 放在同一层；TfChainReport 的 clock_id、segment_id 也直接可读。TfScope 保留为索引上下文契约。
- 将类内 Edge 移到 detail 命名空间，命名为 TfEdgeHistory；不引入 PImpl、继承或兼容包装。
- 保留 SensorStamp、TfTimeInterval、可选动态包络以及 before/after 原始样本：这些分别表达时间、完整区间、无动态范围和插值证据，不拆成易失配的重复字段。
- 拆分祖先收集、边报告、覆盖区间生成和单边样本选择等连贯步骤；展开嵌套三元表达式，使用说明角色和单位的变量名，补齐函数注释。
- 同步迁移 TF 核心/ROS 测试、安装消费者和文档完整示例；只调整调用字段，不放宽原有断言和错误语义。

## 工作区与验证约定

已有多模块暂存修改，本轮保留当前 codex/rgb-readability 分支，不切换共享工作区；完成后仅暂存本轮文件，不提交或合并。

重构前快照保存在临时目录。验证依次包括：严格无 ROS 核心回归、真实 ROS 2 Humble、重构前后诊断对照、安装目标消费者、文档示例和链接检查。所有生成物写入临时目录，不将构造消息通过写成真实 bag 验收。

## 复杂度与所有权目标

保留 child/时间有序索引和二分端点定位；逐图查询不扫描历史全量样本。覆盖仍只扫描查询路径上的动态历史并求区间交集。原始时间、四元数、首次重复来源和拥有型返回值保持。重构不增加 ROS 运行依赖或额外历史深拷贝。

## 实施完成与自审

- TfTransform 使用 source_topic/source_message_id/source_transform_index；TfTimeOptions 使用 clock_id/segment_id/max_interpolation_gap_ns；TfChainReport 直接携带 clock_id/segment_id。移除 TfSource。
- 类内 Edge 改为 detail::TfEdgeHistory，内部字段 parent_frame 明确父节点角色；不作为业务调用接口。
- 抽取 wouldCreateCycle、collectAncestors、describeEdge、buildCoverageIntervals、intersectIntervals、selectEdgeSamples 和查询验证；主流程可按“校验—拓扑—覆盖—选择”阅读。无嵌套三元表达式；生产函数、辅助函数和相关测试入口补齐职责/边界注释。
- 时间仍使用整数纳秒；静态/精确端点优先、无外推、真实零时刻、首次重复来源、冲突阻断及错误优先级保持。失败直接修改本地结果，避免原先错误 lambda 按值复制完整报告。
- 原有断言仅迁移字段路径/配置初始化，未放宽预期；另补报告上下文、来源字符串/序号独立所有权以及多个同时错误的诊断顺序。
- 同步核心/ROS 示例、接口手册、历史复现片段、安装消费者及分类导航；公开字段迁移表位于 TF 核心说明第 12 节。此调整需要下游重新编译，不应继续使用旧字段路径。

## 复杂度复核

索引仍是 child 有序表和每边整数纳秒有序表。已有动态边插入定位为 O(log E + log N_e)；新增边额外沿祖先检查环。链发现沿两端祖先查表并定位共同祖先，不扫描全图。单边覆盖生成 O(N_e)，两个区间序列相交 O(I_left + I_right)；整链依次计算交集，不把首末包络当覆盖。逐图 checkTime 仅发现路径并对各动态边二分定位 O(log N_e)，不扫描整段历史。

额外内存仍用于祖先路径、覆盖区间及拥有型返回值；历史样本不为求覆盖而深拷贝。TfTransform 按值输入的复制/移动约定不变；来源和原始四元数没有被重写。索引继续只允许先插入、后并发只读查询。

## 实际验证

1. 严格编译参数 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror：无 ROS 核心 CTest 6/6，真实 ROS 2 Humble CTest 10/10，零失败/跳过。分步扁平化及流程拆分后均运行核心回归。
2. 以同一临时探针分别编译重构前快照与当前 TF：840 次插入、16,800 次链/覆盖、218,400 次时间查询，共 236,040 条规范化输出记录（17,257,892 字节）逐字节一致。比较涵盖状态/完整消息、失败 frame、上下文、路径、区间、原始时间/几何/来源，以及所选 before/after。
3. 安装到临时前缀后，只使用公开导出目标：无 ROS 的 4 个消费者、ROS 的 7 个消费者全部严格编译并运行通过，包括 TF 与图像元数据和真实 TFMessage 交接。
4. 从当前 Markdown 提取 TF 核心与 ROS 两份完整示例，经安装目标严格编译并运行，分别输出 1..3; edges=2 和 /tf_static。
5. 本轮文档的 103 个本地文件/章节链接有效；本轮 C++ 文件无超过 120 字符的行、无行尾空白，git diff --check 通过。快照指纹核对确认修改仅涉及 TF、其测试/消费者及配套文档，其他模块内容保持。

验证产物及重构前快照位于 /tmp/vln-tf-readability-zwiuzj31；项目源码及 Git 不包含生成物。所有测试使用构造数据，不代表真实 bag 的坐标链/物理标定或整体验收。当前仍只提供历史样本与覆盖，不新增数值位姿求解。
