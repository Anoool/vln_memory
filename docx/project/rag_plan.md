# RAG 实施方案

## 当前范围

已实现的离线链路为 BagBuilder → MemoryEngine → SQLite 工作库 → 已发布记忆。RAG 接在发布之后，读取实体文本、关系、坐标和观测证据。第一版实现本地建索引和只读查询。SQLite 继续负责构建、标注、纠错和批次恢复。[发布目录格式](../modules/engine/memory_publication.md)

## 模型与索引

首版用 [intfloat/multilingual-e5-small](https://huggingface.co/intfloat/multilingual-e5-small) 在本地 CPU 编码，输出 384 维向量。当前实体描述主要是英文，验收使用中文查询。实体文本加 `passage: `，查询加 `query: `，两侧使用相同的模型修订和归一化方式。视觉模型负责离线识别，Agent 的会话模型负责理解指令和生成回答。[构建流程](../modules/engine/memory_engine.md)

如果小模型在实测中反复混淆相似物品，用同一组问题对比 [Qwen3-Embedding-0.6B](https://huggingface.co/Qwen/Qwen3-Embedding-0.6B) 的首位命中与推理时间。

每个实体生成一条文本，以人工名称或语义标签开头，拼入别名、描述和选定属性。索引存到发布目录之外，例如本项目的 `build/rag/<memory_version>/`。

| 文件 | 内容 |
| --- | --- |
| `vectors.npy` | `N × dimension` 的 `float32` 矩阵，一行一个实体。默认 E5-small 为 384 维。 |
| `entity_ids.json` | 依矩阵行顺序保存实体 ID。 |
| `index.json` | 索引格式、发布版本、发布清单 SHA-256、模型与分词器摘要、编码配置、维度及行数。 |

每次加载检查索引与发布版本的清单、行数、维度和实体 ID 是否一致。新发布版本、更换 Embedding 模型或编码配置时重建索引。当前默认使用同一个 E5-small INT8 ONNX 导出。YAML 的 `environment` 选择批量和 CPU 线程数，桌面和 Jetson 共用模型和索引。板端验收项见 [部署说明](../modules/engine/memory_rag.md#jetson-nano-部署)。

## R1 · 本地检索

已提供 [本地建索引与查询 CLI](../modules/engine/memory_rag.md)。

1. 从 `entities.jsonl` 读取已发布实体，按 `entity_id` 合入 `annotations.jsonl` 的名称和别名，生成向量及行映射。
2. 查询优先处理唯一的名称、别名或标签精确匹配。其他查询编码后，用 NumPy 矩阵点积排序，默认返回最多 3 个候选。记录匹配方式和得分，相似度排序结果按候选返回。
3. 原版纯向量测试中，黑色背包排第 1，灰棕色背包排第 3。通过 B4 标注发布中文名称和别名后，两者均可精确命中。「打印机」仍返回候选，跨版本索引会提示重建。

## R2 · 回查坐标、关系和观测

结果返回实体 ID、标签、匹配方式、`position_map`、`map_id`、`frame_id`、`geometry_revision` 和 `memory_version`。按实体的 `location_observation` 返回观测 ID、修订和选区，用 MemoryEngine 现有 `get_observation` 回看原图。

调用方指定 `hanging_on` 等关系时，从同版本 `relations.jsonl` 按实体 ID 取原始有序端点、另一端和支持观测。缺少对应记录时报告证据不足。向量用于候选排序，数值距离沿用发布记录。

R1 与 R2 共用以下 CLI。当前 [config/memory_rag.yaml](../../config/memory_rag.yaml) 指向 217 条观测的正式包及单份已建索引。首次建索引或更换发布、模型时使用 `build-index`。

~~~bash
build/rag/venv/bin/python tools/memory_rag.py query \
  --text "black backpack hung on chair backrest" --relation hanging_on
~~~

历史四帧验收从 `hanging_on` 查到椅子和支持观测，并从背包实体回查 RGB 1607、Depth 1575 和原始选区。当前正式包的结果见 [217 条观测验收](../reviews/verification.md#217-条观测与共享索引验收)。四帧版本位于原项目的 `/home/dev/agent/vln_memory/build/minimal-real-bag-20261117/site_s4-model-v1`，可以只读使用。发布目录也可以按 [README](../../README.md) 重新生成后传入。

## R3 · 接入 vln_agent

常驻 Python 进程加载已发布版本和索引，通过 ROS 2 服务暴露同一套查询逻辑。`vln_agent` 的 `ToolManager` 注册 `search_memory` 插件，返回现有 `ToolResult.data`。先用 `/vln/tool_call` 验证真实查询，再从会话查看模型调用和工具结果。发布版本变更时重建索引并重启检索进程。[启动与配置](../modules/engine/memory_rag.md#接入-vln_agent)

检索结果携带实体及观测引用，RGB-D 工具负责现场确认。在线导航另行验收，历史物体坐标用于缩小搜索范围，停车点依据当前可通行区域选择。

## R4 · 发送用户消息前召回

Agent 将首条及后续用户指令送入独立的 `retrieve_context` ROS 入口，强制执行向量检索。先按名称、别名和 YAML 同义词组筛选相关实体，再将按向量排序的候选及同版定位证据随原消息送给模型。没有候选、服务失败或超时则发送原消息。模型可继续通过 `search_memory` 发送明确的实体描述和关系问题。两条路径复用一个编码器和索引，无需重建已有索引。[接口与配置](../modules/engine/memory_rag.md#接入-vln_agent)
