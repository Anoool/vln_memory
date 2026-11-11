# BagBuilder

## 模块职责与边界

读取项目的 ROS 2 SQLite RGB-D 录包，发布可独立回读的双图观测和证据。BagBuilder 负责时间配对、标定、历史位姿和关键帧，不负责语义识别、实体写入或模型分批。当前配置面向 `camera_record` 的 JPEG 彩色图、`compressedDepth`、两路 CameraInfo 和 TF。

## 核心概念与整体设计

~~~mermaid
flowchart LR
  A["ROS 2 bag + YAML"] --> B["扫描图像时间、标定与 TF"]
  B --> C["按 header 配对并解码"]
  C --> D["关键帧与正式 Observation"]
  D --> E["原子发布观测目录"]
  E --> F["MemoryEngine"]
~~~

以 RGB 为基准做单调一对一最近配对，同差优先较早的 Depth，同刻优先来源顺序。容差含边界，设为 `0` 只配原始时间完全相等的帧。关键帧由底盘位移或转角触发。RGB 和 Depth 分别按各自的原始时刻取相机位姿，底盘使用 RGB 时刻。时间接近只说明满足配对门限，配准与曝光同步依据单独记录。

## 当前录包的 RGB-D 验收

`camera_record_0.db3` 中，彩色与深度图均为 1280×720，使用彩色光学坐标系和同一有效相机模型。三张低运动帧的四条边缘剖面在原像素网格上的偏差中位数为 14.5 px，高于本次的 5 px 验收线。`635` 帧只有 17 个两侧深度均有效的跳变像素，未用于剖面量化。将深度临时按无畸变网格解释也无法消除低运动帧的偏差。

运动样本 `1578` 的椅座底部边缘在双时刻 TF 重投影后由 8.50 px 降到 2.01 px，`1633` 的桌腿边缘由 5.50 px 降到 2.13 px。测量值来自指定剖面。原始两路图像 header 没有完全相同的时间戳，122 条关键帧配对的绝对时间差中位数为 25.2 ms。

BagBuilder 对实测源文件及原 30 ms 配置写入 `alignment_basis` 和 `capture_sync_basis`。该录包的 `depth_alignment` 和 `capture_sync_status` 均保留 `unknown`。原图、原始 header、配对及双时刻位姿可以继续用于重投影。用以下脚本复查指定帧的叠加图和剖面测量。

~~~bash
python3 test/bag/builder/audit_camera_record_rgbd.py /path/to/observations build/rgbd-audit
~~~

## 数据结构

| 输入或产物 | 内容 |
| --- | --- |
| [vln_memory.yaml](../../../config/vln_memory.yaml) | `dataset` 给录包、输出目录、地图 ID/几何版本/依据。`topics`、`frames` 给实际话题和坐标系。`preprocess` 给配对容差（毫秒）、深度比例和可选位移/转角（米/度）。 |
| `observations.jsonl` | 每行一个 [Observation](../memory/memory.md#数据结构)。直出记录均有 `rgb_media` 和 `depth_media`，路径相对于输出目录。 |
| `evidence/` | 彩色 PNG、深度媒体、来源和观测清单、媒体 SHA-256 索引、`pending.jsonl` 和 `summary.json`。源 `16UC1` 深度使用无损 PNG，读取时按记录比例转为米制 optical-z。 |
| 历史 preparation | `bag_manifest.json` 绑定 `messages.jsonl`、`frames.jsonl`、`rgbd_pairs.jsonl`、`camera_info.jsonl`、`tf.jsonl` 和报告。用 `pair_id` 回读保存的配对。v5–v7 兼容米制 NPY / 源 PNG；v7 的原始 CDR 可选，`bag.save_raw_cdr` 默认 `false`。 |

默认示例用 `30 ms`、`0.001`、`0.8 m` 和 `25°`。两项关键帧门限都省略时保留所有可回读的双图对。深度比例由录包实际单位决定，毫米用 `0.001`，米用 `1`。现有工程路径从启动目录解析。旧 `bag` 配置另见 [camera_record.yaml](../../../config/camera_record.yaml)。

## 公共 API

| 入口 | 输入与返回 |
| --- | --- |
| `bag_builder --config FILE` | 项目 YAML → 正式观测目录。旧 `bag` YAML 可用 `--format preparation`。 |
| [bag_builder.hpp](../../../include/vln_memory/bag/builder/bag_builder.hpp) | `buildBagObservations(options, context, selection)` 直出正式观测。`buildBag(options)` 发布 preparation。两者返回 `BagBuildResult`，含状态、原因、计数及目录。链接 `vln_memory::vln_memory_bag_builder`（ROS 构建）。 |
| [bag_config.hpp](../../../include/vln_memory/bag/builder/bag_config.hpp) | `loadBagBuildOptions(path)` 加载旧 `bag` 格式。`validateBagBuildOptions(options)` 返回配置问题。 |
| [rgbd_sync.hpp](../../../include/vln_memory/bag/builder/rgbd_sync.hpp) · [camera_info_index.hpp](../../../include/vln_memory/bag/builder/camera_info_index.hpp) | `pairRgbDepth(...)` 配对同一时钟段的流。`CameraInfoIndex::select(...)` 根据图像与声明选择标定。 |
| [preparation_reader.hpp](../../../include/vln_memory/bag/builder/preparation_reader.hpp) | `openPreparation(root)` → `reader->pairs()` / `loadPair(id)` / `loadPairMetadata(id)`。`loadPreparedPair(root, id)` 可单独读一对。只读既有配对、标定、TF 和媒体，返回的配对独立拥有数据。链接 `vln_memory::vln_memory_preparation_reader`。 |
| [preparation_compactor.hpp](../../../include/vln_memory/bag/builder/preparation_compactor.hpp) | 将旧 preparation 复制为新目录，去除原始 CDR，保留配对 ID、元数据和媒体。CLI 为 `preparation_compact`。 |

底层读包、压缩解码与写入接口见 [builder 公开头文件](../../../include/vln_memory/bag/builder)。

## 快速开始与典型示例

从仓库根目录运行。填好 `dataset.bag_path` 和地图信息，`dataset.memory_dir` 指向尚不存在的目录。

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config config/vln_memory.yaml
~~~

构建后可在输出目录查看 `observations.jsonl` 和 `evidence/summary.json`。处理历史 preparation 时可用以下命令。

~~~bash
ros2 run vln_memory bag_builder --config config/camera_record.yaml --format preparation
ros2 run vln_memory preparation_compact --input old_preparation --output compact_preparation
~~~

## 错误处理

| 情况 | 处理 |
| --- | --- |
| CLI 退出 `2` | 参数或配置错误。检查 YAML、录包路径和话题绑定。 |
| CLI 退出 `1` | 整次构建失败。检查不支持的录包、来源变化、读写错误或输出已存在等原因。已有目录不被覆盖，失败工作目录供排查。 |
| CLI 退出 `0`，但有坏帧或未配对帧 | 可回读的观测已经发布。查 `evidence/pending.jsonl` 和 `summary.json`。缺标定或位姿的双图观测可带 pending 原因，启用关键帧时缺底盘位姿的帧由选帧报告记录。 |
| preparation 回读失败 | `pair_not_found` 查 ID，`invalid_reference` 查来源与引用，`invalid_media` 查载荷，`source_changed` 重新打开稳定的数据目录。 |

录包验收和实际数值边界见 [验收记录](../../reviews/verification.md)。
