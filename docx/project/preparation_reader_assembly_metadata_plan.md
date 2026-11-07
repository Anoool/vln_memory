# PreparationReader 组装返回契约

日期：2026-11-06。按用户已讨论方案先构建、再自查；本文件记录本轮落实的边界与后续衔接。

## 数据路径

PreparationReader 统一解析已保存 preparation 事实；engine/observation_builder 后续只接收类型数据及显式地图业务上下文；memory/observation 负责正式记录契约与 JSON。生产构建器无需再读取五类源 JSONL 或解释历史存储差异。

## 本轮实现

PreparedCalibration 保存原始候选、绑定参数、实际匹配/时序与原始/规范化状态；PreparedMessageSummary 返回当前配对引用的消息闭包；PreparedDepthStorage 返回每帧实际存储和源单位解释。额外补齐 body_frame、配对门限、TF 限制、来源 manifest 指纹、注册检查与规范相对路径。

新旧入口共用 readPair 和私有 preparation_metadata 解析。loadPreparedPair 仍为选择性扫描，openPreparation 仍为全局键/schema 索引；单对语义错误在 loadPair 返回，不阻断其他配对。默认完整深度读取行为保持，metadata_only 留作后续任务。

不新增 PreparationReader→memory 依赖，不修改 preparation 文件格式，不在读取层决定 map_id/geometry_revision/map_basis 或观测 ID。公开值独立拥有数据。扩展 C++ 结构后下游需重新编译。

## 实现后自查

检查复制/生命周期、精确整数、缺项状态、历史 NPY、v7 无 CDR、单对错误隔离、原始标定与保存模型的对应关系、路径约束及直接组装。修正全局打开范围扩大、目录项复制整份相机配置、模型形状/规范化矛盾、非有限标定头字段绕过及负注册状态兼容问题。详见 [验收与审查](../reviews/preparation_reader_assembly_metadata_verification.md)。
