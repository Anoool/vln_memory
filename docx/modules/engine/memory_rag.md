# 本地记忆检索

`tools/memory_rag.py` 读取 `spatial_memory/1` 发布目录的清单、实体和人工标注。建索引将已归一化的实体向量写到发布目录外。查询返回同一版本的实体、地图坐标和定位观测引用。用 `--relation` 回查已发布的关系与支持观测。

## 安装和模型

使用 Python 3.10 或更新版本。以下命令从仓库根目录执行。桌面和 Jetson 共用 [Xenova 的 E5-small INT8 ONNX 导出](https://huggingface.co/Xenova/multilingual-e5-small/tree/761b726dd34fb83930e26aab4e9ac3899aa1fa78/onnx)，下载权重后在本地编码。

~~~bash
python3 -m venv build/rag/venv
build/rag/venv/bin/python -m pip install -r tools/requirements-rag.txt

mkdir -p build/rag/e5-small-arm64
model_url=https://huggingface.co/Xenova/multilingual-e5-small/resolve/761b726dd34fb83930e26aab4e9ac3899aa1fa78
curl -fL "$model_url/tokenizer.json" -o build/rag/e5-small-arm64/tokenizer.json
curl -fL "$model_url/onnx/model_quantized.onnx" -o build/rag/e5-small-arm64/model_quantized.onnx
~~~

系统未提供 `venv` 的 pip 支持时，可安装 Ubuntu 的 `python3-venv`。YAML 中的 `model_path` 和 `tokenizer_path` 分别指向 ONNX 文件与同版 `tokenizer.json`。

## 建索引和查询

CLI 和 ROS 检索服务默认共读 [config/memory_rag.yaml](../../../config/memory_rag.yaml)。相对路径以 YAML 所在目录为基准。

| 配置 | 当前值 |
| --- | --- |
| `release` | `../build/rag/e2e-200-20261118/site_s4-bag217-deepseek-v1`，包含实体、关系、观测及双图证据。 |
| `index_dir` | `../build/rag/e2e-200-20261118/index`，包含 53 个实体的 384 维向量。 |
| `environment` | `desktop` 用于当前桌面环境，批大小 16，CPU 线程数 4。部署到 Jetson Nano 时选 `jetson`，批大小和线程数均为 2。 |
| `model_id`、`model_revision`、`model_path`、`tokenizer_path` | 两个环境共享 E5-small INT8 ONNX 及其分词器，由这组字段指定。 |

**有匹配的索引时直接查询。** 当前版本可用英文实体标签精确查证关系，中文描述可走向量召回。通过 `--config /path/to/rag.yaml` 可读取另一份配置，参数放在 `query` 或 `build-index` 前。

~~~bash
build/rag/venv/bin/python tools/memory_rag.py query \
  --text "black backpack hung on chair backrest" --relation hanging_on
build/rag/venv/bin/python tools/memory_rag.py query --vector-only --text "黑色背包"
~~~

**首次构建或编码配置变更时，建一次索引。** 更换发布版本、模型、分词器、编码长度、ONNX 输出、池化或文本前缀时，将 `index_dir` 指向发布目录之外的一个新目录，再运行：

~~~bash
build/rag/venv/bin/python tools/memory_rag.py build-index
~~~

`index_dir` 内有 `vectors.npy`、`entity_ids.json` 和 `index.json`。`memory_rag/2` 清单记录发布摘要、模型与分词器摘要和编码设置，查询启动时会校验。**切换 `environment` 只会改变推理参数，直接复用这三个文件。** `top_k` 可独立调整。

更换 ONNX 模型时，在同一 YAML 设置输出名、维数、池化、填充词元和查询前缀。兼容的模型使用自包含的 ONNX 文件与可独立加载的 `tokenizer.json`，接收 int32 或 int64 的 `input_ids`，可选 `attention_mask` 和 `token_type_ids`。池化可以是 `mean`、`cls` 或 `none`，分别对应按掩码求均值、取首词元和使用二维句向量。向量会统一归一化。

旧配置中的 `model_dir` 需要改成显式的 `model_path` 和 `tokenizer_path`，缺少的参数可从默认 YAML 补齐。旧版 `memory_rag/1` 索引可从原发布目录重建，发布数据可直接复用。

| 查询状态 | 含义 |
| --- | --- |
| `exact` | 在名称、别名和原语义标签中找到唯一的精确匹配。 |
| `candidates` | 同一模型编码查询，按余弦相似度排序。得分用于比较候选。 |
| `empty` | 该发布版本没有实体。 |
| `no_relevant_match` | 自动召回没有找到名称、别名或类别词依据，返回空候选。 |

## Jetson Nano 部署

原版 Jetson Nano 使用 Cortex-A57 ARM64。NVIDIA 的 JetPack 4 基于 Ubuntu 18.04，ROS 2 Humble 的官方目标平台为 Ubuntu 22.04 ARM64。以下配置面向具备 Python 3.10、兼容 ARM64 CPU 的 ONNX Runtime，以及与 Agent 匹配的 ROS 2 运行环境。板上先检查 `uname -m`、`python3 --version` 及本机依赖导入，再启动服务。[NVIDIA JetPack 说明](https://developer.nvidia.com/embedded/faq) · [ROS 2 Humble 目标平台](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst#humble-hawksbill-may-2022---may-2027) · [ONNX Runtime ARM64 安装包](https://pypi.org/project/onnxruntime/1.22.1/#files)

将 [统一 YAML](../../../config/memory_rag.yaml) 中的 `environment` 改成 `jetson`。共同的编码长度限制为 128 词元，Jetson 环境的批大小和线程数均为 2。部署时使用桌面已构建的同一份索引。

~~~bash
build/rag/venv/bin/python -c 'import onnxruntime, tokenizers; print(onnxruntime.get_available_providers())'
build/rag/venv/bin/python tools/memory_rag.py query --text "肩带放在椅背的随身包"
~~~

部署时把整个发布目录、单份 `index/`、ONNX 文件和分词器复制到目标机，按实际目录填写 YAML 的 `release`、`index_dir`、`model_path`、`tokenizer_path`。先在板上运行查询示例，再启动检索服务。板端联调时测量首次语义查询和常驻内存，再按实测延迟配置 Agent 的 `timeout_sec`。

## 坐标、关系和观测

查询结果携带 `map_id`、`frame_id`、`geometry_revision` 和 `memory_version`。实体命中包括人工名称、定位状态、`position_map`、定位观测 ID 与修订和原始选区。双图路径相对于发布目录，配准和同步状态沿用原观测记录。可把返回的观测 ID 传给同一发布版本的 [`get_observation`](memory_engine.md#公共-api)。

`--relation` 接受 `inside`、`near`、`on`、`hanging_on` 和 `connects_regions`。返回支持的同版关系、原始有序端点、关联实体、`basis` 和支持观测 ID。数值距离沿用发布记录。缺少相应证据时返回 `relation_status: insufficient_evidence` 和空关系列表。

版本、文件和参数错误写入 stderr，命令返回非零退出码。

## 四帧中文查询验收

原版 `site_s4-model-v1` 的纯向量检索将「黑色背包」排第 1，将「灰棕色背包」排第 3。通过 [`memory_annotate`](memory_annotate.md) 在工作库副本补上中文名称和别名，重新发布并建索引后，「黑色背包」「灰棕色背包」「灰褐色背包」分别精确命中对应实体。

本地验收产物在 `build/rag/r2/`。已用 `site_s4-model-zh-v1` 及对应索引验证 `--text "黑色背包" --relation hanging_on`，查到灰色转椅和两条支持观测。黑色背包的定位证据为 RGB 1607、Depth 1575，可回读双图并复算出相同代表点。


## 接入 vln_agent

先构建并安装 `vln_agent`。Agent 和检索进程使用同一 ROS domain。启动检索服务时，先 source Agent 的 ROS 工作空间，Python 才能导入 `vln_agent/srv/SearchMemory`。一个 RAG 进程提供两个使用同一协议的 ROS 服务。

| 路径 | 触发方式 | 配置对接 |
| --- | --- | --- |
| `search_memory` | 模型根据问题发出工具调用。精确名称先匹配，其他描述走向量检索。可带 `relation` 核对关系。 | Agent 工具组的 `service_name` 对应 RAG 的 `service_name`。 |
| `retrieve_context` | Agent 在发送用户消息前先用目标名称、别名和 YAML `context_aliases` 筛选实体，再对原句编码排序。 | Agent `memory_context.service_name` 对应 RAG 的 `context_service_name`。 |

自动路径附加 `{"source":"memory_rag","data":{...}}` 到同一条用户消息。`data` 包含发布版本、地图、候选相似度与定位证据引用，候选数量由 RAG YAML 的 `top_k` 控制。`context_aliases` 中每一组是同类实体的中英文近义词，例如「座椅」和 `chair`。筛选支持「参照物旁边的目标」「目标在参照物旁边」等常见句式；参照物单独命中时不会成为自动候选。没有目标匹配时，自动路径保留原用户消息，模型仍可主动查证。`--context` 可本地复现自动入口，例如：

~~~bash
build/rag/venv/bin/python tools/memory_rag.py query --context --text "实验室里的绿帘"
build/rag/venv/bin/python tools/memory_rag.py query --context --text "白色设备柜旁边的亮紫色直升机在哪里"
~~~

设置 `memory_context.enabled: false` 可关闭自动路径。检索失败或达到该组的 `timeout_sec` 时，Agent 继续处理原消息。

~~~bash
source /opt/ros/humble/setup.bash
source /path/to/agent_workspace/install/setup.bash
cd /home/dev/agent/vln_memory_RAG
build/rag/venv/bin/python tools/memory_rag_server.py
~~~

服务默认使用 `config/memory_rag.yaml`。启动时校验发布和索引，首个向量查询加载模型，后续重用。查询工具参数为 `text` 和 `relation`，只查实体时给 `relation: null`。服务返回实体、坐标、关系和观测引用，以 JSON 放入 `ToolResult.data`。Agent 的 YAML 配置等待超时，换发布版本时重建索引并重启检索进程。

当前正式包 `site_s4-bag217-deepseek-v1` 已通过本地联调。模拟 Responses 会话首轮收到 3 条自动候选，主动调用 `search_memory` 返回黑色背包的精确命中和 `hanging_on` 关系。当前联调使用 x86_64 WSL、`rmw_cyclonedds_cpp` 和回环网络。Jetson 真机运行、线上会话模型效果与实车导航属于后续验收。
