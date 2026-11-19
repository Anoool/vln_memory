# MemoryViewer

## 模块职责与边界

用 Streamlit + Plotly 浏览已发布的空间记忆。点选实体可查看定位、同实体的其他观测和关系证据；「全部图片」每页展示 6 组 RGB-D，可放大任一组，或在侧栏固定一组原尺寸 RGB 和深度预览。页面只允许保存实体的显示名称、别名和备注，无需启动 ROS、加载编码模型或重新构建记忆。

## 页面布局

- 侧栏：标题、发布版本、实体/观测/关系计数，「实体与标注 / 全部图片」切换，以及当前页面的选择控件（实体下拉框，或图库页码与原图固定）。
- 实体与标注：左侧为俯视实体图，选中实体为橙色并自动避让邻近点标注名称，与其有已发布关系的实体为黄色并以虚线相连；支持拖动平移、工具栏缩放、双击复位和点选。右侧检查器显示实体名称与摘要标签（有人工名称时附带模型原标签，另含坐标、观测数、关系数），下设三个页签：
  - 观测证据：选择证据来源，RGB 与深度预览上下排列，并可「放大」在弹窗查看原尺寸图像与定位选区；
  - 详情与关系：模型描述、属性和已发布关系，「查看」跳转到关系另一端的实体；
  - 人工标注：可编辑时显示表单，只读时说明原因。
- 全部图片：两列卡片，每张卡片的「放大」在弹窗里显示原尺寸图像。

代码分为三个文件：[`memory_viewer.py`](../../../tools/memory_viewer.py) 负责读取发布包和页面流程，[`viewer_map.py`](../../../tools/viewer_map.py) 生成实体图，[`viewer_style.py`](../../../tools/viewer_style.py) 提供页面样式与转义后的 HTML 片段。项目根目录的 `.streamlit/config.toml` 设置浅色主题并隐藏部署工具栏，从项目根目录启动时生效。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["已发布的空间记忆"] --> B["俯视坐标图和实体列表"]
  B --> C["选中实体"]
  C --> D["定位观测和选区"]
  C --> E["关系及其支持观测"]
  A --> F["全部图片：分页浏览"]
  C --> G["显示名称、别名和备注"]
  G --> H["MemoryEngine 保存标注"]
~~~

发布文件由 Viewer 只读打开。页面调用现有 [`memory_annotate`](../engine/memory_annotate.md) 保存标注到工作库，立即重读同源、同实体修订的标注来更新界面。图像、坐标、选区、模型原标签和关系都不能从页面修改。Agent 要使用新标注时，仍需发布新版本并重建索引。地图底图、轨迹和语义搜索留作后续实现。

## 数据结构

`manifest.json` 提供版本、地图坐标系和文件摘要。坐标图读取 `entities.jsonl` 中的 `position_map`，横纵轴单位为米。显示名称优先取 `annotations.jsonl`。定位观测从实体的 `location_observation` 与 `observation_refs` 取同一修订的选区；关系按照 `relations.jsonl` 的原始方向及其 `evidence_observation_ids` 回查。两者都从同包的 `observations.jsonl` 和 `evidence/` 读取图像。深度图的灰度拉伸仅用于显示，不改原图，也不将未确认的配准或同步状态显示为已验证。

## 启动

在 WSL 进入项目根目录。Streamlit 会从这里读取 `.streamlit/config.toml`，新版主题和侧栏样式才能按预期加载。Viewer 使用独立的 `build/viewer/venv`；已经装过依赖就直接启动。无需启动 ROS、重新建库或加载 RAG 模型。

首次安装依赖：

~~~bash
cd /home/dev/agent/vln_memory_RAG
python3 -m venv build/viewer/venv
build/viewer/venv/bin/python -m pip install -r tools/requirements-viewer.txt
~~~

如果 WSL 的系统 Python 缺少 `ensurepip`，可改用 `python3 -m venv --without-pip build/viewer/venv`，再调用已有的 pip 安装到目标环境。例如，这台机器可用 `/home/dev/miniconda3/bin/python -m pip --python "$PWD/build/viewer/venv" install -r tools/requirements-viewer.txt`。依赖只装进 Viewer 环境。

日常在前台启动：

~~~bash
cd /home/dev/agent/vln_memory_RAG
build/viewer/venv/bin/python -B -m streamlit run tools/memory_viewer.py \
  --server.address 127.0.0.1 --server.port 8507 --server.headless true \
  --browser.gatherUsageStats false -- --config config/memory_viewer.yaml
~~~

浏览器打开 [本地 Viewer](http://127.0.0.1:8507)，前台运行时按 `Ctrl+C` 停止。若希望关闭终端后服务继续运行，可从项目根目录使用：

~~~bash
nohup build/viewer/venv/bin/python -B -m streamlit run tools/memory_viewer.py \
  --server.address 127.0.0.1 --server.port 8507 --server.headless true \
  --browser.gatherUsageStats false -- --config config/memory_viewer.yaml \
  >> build/viewer/viewer.log 2>&1 < /dev/null &
printf '%s\n' "$!" > build/viewer/viewer.pid
~~~

如果端口被占用，先用 `ss -ltnp 'sport = :8507'` 查看监听 PID，确认是旧的 MemoryViewer 再用 `kill PID` 停止。后台启动失败时查看 `tail -n 30 build/viewer/viewer.log`。可用 `curl --noproxy '*' -fsS http://127.0.0.1:8507/_stcore/health` 检查服务；返回 `ok` 后，再打开页面检查侧栏和观测证据是否正常显示。

默认使用 [config/memory_viewer.yaml](../../../config/memory_viewer.yaml)。`release` 指向发布目录；`annotations.engine` 和 `annotations.engine_config` 指向 Engine 和同源工作配置，这三条路径均相对 Viewer YAML。Engine 从项目根目录启动。只需浏览时可传入仅含 `release` 的 YAML；服务未绑定回环地址、工作库不匹配时也会保持只读。若替换了同一路径下的发布包，请重启 Viewer。

## 快速验收与错误处理

在 `site_s4-bag217-deepseek-v1` 中点选 `black backpack hung on chair backrest`。定位证据应显示约 `X -3.52、Y 4.80` 米与 RGB 框。将证据来源切为「挂在上面」，应看到这条关系自己的另一帧，且不叠加定位帧的选区。界面检查 JSONL 的摘要与记录数以及坐标、引用；证据缺失时在右侧报告，其他实体仍可浏览。

固定到 217 条观测的正式包时，可在相同的环境运行回归：

~~~bash
build/viewer/venv/bin/python -B test/viewer/check_memory_viewer.py
~~~

本地回归覆盖点选后的定位证据、关系证据、关系跳转、同实体的其他观测及全包图库末页。界面里选择实体后，在「人工标注」页签填写「显示名称」「别名（每行一个）」「备注」，点击「保存标注」即可。空白字段会清除对应人工标注，随后名称回退到模型原标签。
