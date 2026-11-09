# BagBuilder 直出正式观测

默认命令读取 ROS bag 并在同一次运行中完成配对、媒体提取、标定和历史 TF 查询、选帧以及正式观测发布。观测组装和选帧复用现有 C++ 函数。执行入口位于 [bag_builder_main.cpp](../../../src/apps/bag_builder_main.cpp)。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config /absolute/path/vln_memory.yaml \
  --bag /absolute/path/recording --output /absolute/path/memory \
  --map-id room_map --geometry-revision trajectory-v1 --map-basis "Recorded map and TF"
```

- `--config` 指向录包配置。`--bag`、`--output` 和 `--sync-tolerance-ms` 可覆盖当次设置。输出路径须尚未存在。
- 地图 ID、几何版本、地图依据必填，地图 frame 取 YAML 的 `bag.map_frame`。
- `--purpose` 默认 `rgb`，可选 `depth_map` 或 `prepared_rgbd`。`--batch-size` 默认 4，可用 `--distance-m`、`--yaw-deg`、`--max-step-m`、`--max-step-yaw-deg` 配置选帧。
- `--format preparation` 使用历史准备目录流程，C++ 旧接口 `buildBag` 也沿用该流程。

结果根只包含 `observations.jsonl` 和 `evidence/`。媒体以相对路径引用。标准媒体来自时间配对成功的帧，启用关键帧筛选时，证据目录可保留未选入正式观测的配对媒体。配置开启 `save_raw_cdr` 时还会保留原始消息。源清单和媒体索引均放在 `evidence/`。

BagBuilder 在新建临时目录中写同一套最终文件，引用校验和关闭写流后原子发布。退出 0 表示已发布，可带问题。检查 `observations`、`pending_observations`、`prepared_pairs` 和 `evidence/issues.jsonl` 以了解可用数量。退出 1 表示构建失败，退出 2 表示参数或配置错误。

MemoryEngine 从已发布的目录读取正式观测。可用当前的 [语义构建命令](../engine/semantic_build.md) 传入结构化实体操作，媒体回读见 [观测读取器](../engine/observation_dataset_reader.md)。
