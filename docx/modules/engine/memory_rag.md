# 本地记忆检索

`tools/memory_rag.py` 读取 `spatial_memory/1` 发布目录的清单、实体和人工标注。建索引将已归一化的实体向量写到发布目录外。查询返回同一版本的实体、地图坐标和定位观测引用。用 `--relation` 回查已发布的关系与支持观测。

## 安装和模型

使用 Python 3.10 或更新版本。以下命令从仓库根目录执行。示例权重是 [E5-small 官方 ONNX 导出](https://huggingface.co/intfloat/multilingual-e5-small/tree/ccc66d3/onnx) 中的 INT8 CPU 版本，当前验证环境支持 AVX-512 VNNI。第一次下载模型，之后编码在本地执行。

~~~bash
python3 -m venv build/rag/venv
build/rag/venv/bin/python -m pip install -r tools/requirements-rag.txt

mkdir -p build/rag/e5-small
model_url=https://huggingface.co/intfloat/multilingual-e5-small/resolve/ccc66d3bcd826f577e26b9a4072cc5fe3a7ad6a3/onnx
curl -fL "$model_url/tokenizer.json" -o build/rag/e5-small/tokenizer.json
curl -fL "$model_url/model_qint8_avx512_vnni.onnx" -o build/rag/e5-small/model_qint8_avx512_vnni.onnx
~~~

系统未提供 `venv` 的 pip 支持时，可安装 Ubuntu 的 `python3-venv`。模型搬到其他位置时，在 YAML 的 `model_dir` 填写包含这两个同版文件的目录。

## 建索引和查询

默认读取 [config/memory_rag.yaml](../../../config/memory_rag.yaml)，其中 `release`、`index_dir`、`model_dir` 和 `top_k` 分别设置发布目录、索引目录、模型目录和候选数量。相对路径从 YAML 所在目录解析。默认指向本地中文标注验收版本；换数据时修改配置或使用另一份配置。新建索引时，`index_dir` 应尚不存在且位于发布目录之外。

~~~bash
build/rag/venv/bin/python tools/memory_rag.py build-index
build/rag/venv/bin/python tools/memory_rag.py query --text "黑色背包" --relation hanging_on
build/rag/venv/bin/python tools/memory_rag.py --config /path/to/rag.yaml query --text "黑色背包"
~~~

索引包含 `vectors.npy`、`entity_ids.json` 和 `index.json`。发布清单或模型修订变更后，使用新目录生成对应索引。发布文件摘要、实体 ID 顺序、索引行数和向量维度会在读取时校验。

| 查询状态 | 含义 |
| --- | --- |
| `exact` | 在名称、别名和原语义标签中找到唯一的精确匹配。 |
| `candidates` | 同一模型编码查询，按余弦相似度排序。得分用于比较候选。 |
| `empty` | 该发布版本没有实体。 |

## 坐标、关系和观测

查询结果携带 `map_id`、`frame_id`、`geometry_revision` 和 `memory_version`。实体命中包括人工名称、定位状态、`position_map`、定位观测 ID 与修订和原始选区。双图路径相对于发布目录，配准和同步状态沿用原观测记录。可把返回的观测 ID 传给同一发布版本的 [`get_observation`](memory_engine.md#公共-api)。

`--relation` 接受 `inside`、`near`、`on`、`hanging_on` 和 `connects_regions`。返回支持的同版关系、原始有序端点、关联实体、`basis` 和支持观测 ID。数值距离沿用发布记录。缺少相应证据时返回 `relation_status: insufficient_evidence` 和空关系列表。

版本、文件和参数错误写入 stderr，命令返回非零退出码。

## 四帧中文查询验收

原版 `site_s4-model-v1` 的纯向量检索将「黑色背包」排第 1，将「灰棕色背包」排第 3。通过 [`memory_annotate`](memory_annotate.md) 在工作库副本补上中文名称和别名，重新发布并建索引后，「黑色背包」「灰棕色背包」「灰褐色背包」分别精确命中对应实体。

本地验收产物在 `build/rag/r2/`。已用 `site_s4-model-zh-v1` 和同级的 `index/` 验证 `--text "黑色背包" --relation hanging_on`，查到灰色转椅和两条支持观测。黑色背包的定位证据为 RGB 1607、Depth 1575，可回读双图并复算出相同代表点。


## 接入 vln_agent

先构建并安装 `vln_agent`，启用其 YAML 中的 `memory` 工具组。Agent 和检索进程使用同一 ROS domain，两个 YAML 的 `service_name` 保持一致。启动检索服务时，先 source Agent 的 ROS 工作空间，Python 才能导入 `vln_agent/srv/SearchMemory`。

~~~bash
source /opt/ros/humble/setup.bash
source /path/to/agent_workspace/install/setup.bash
cd /home/dev/agent/vln_memory_RAG
build/rag/venv/bin/python tools/memory_rag_server.py
~~~

服务默认使用 `config/memory_rag.yaml`。启动时校验发布和索引，首个向量查询加载模型，后续重用。查询工具参数为 `text` 和 `relation`，只查实体时给 `relation: null`。服务返回实体、坐标、关系和观测引用，以 JSON 放入 `ToolResult.data`。Agent 的 YAML 配置等待超时，换发布版本时重建索引并重启检索进程。

本地验收使用 `site_s4-model-zh-v1`。已验证 ROS 工具服务和模拟 Responses 会话均能通过 `search_memory` 查到黑色背包及灰色转椅，并从 ROS 用户输出话题收到模拟回复。工作区里默认 Fast DDS 的跨进程请求出现超时，该轮验收使用 `rmw_cyclonedds_cpp` 和回环网络。会话模型效果与实车导航分别验收。
