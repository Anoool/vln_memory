# 观测数据集发布

公开库为 `vln_memory::vln_memory_observation_dataset`，接口见 `include/vln_memory/engine/observation_dataset.hpp`。输入为已发布的 preparation、选择配置与显式地图上下文。核心和 ROS 安装都提供 `memory_observation_dataset`。

```bash
memory_observation_dataset --input /path/to/preparation --output /path/to/new_dataset \
  --purpose rgb --batch-size 4 --map-id sample --geometry-revision trajectory-v1 \
  --map-frame map --map-basis "Recorded map and TF for this acquisition"
```

输出目录包含：

- `observation_manifest.json` 保存 `observation_dataset/1` 版本、preparation 根与 manifest 指纹、地图上下文、采样参数、计数及文件引用。
- `observations.jsonl` 保存按选择顺序排列的 `observation/1` 记录。
- `batches.jsonl` 按批次索引记录正式观测 ID。批次引用顺序与观测文件一致。
- `build_input_manifest.json` 保留全部候选、原配对批次与选择理由。
- `reports/issues.jsonl` 记录读取失败、用途不满足和合法 pending 的具体原因。

媒体路径以来源 preparation 根解析。输入应保持稳定。读取器核对元数据、媒体和 CDR 的规范路径、大小及修改时间。完整筛选解码 RGB 和深度以核验载荷，对已保存的 CDR 核对 SHA-256，组装入口复用索引执行元数据读取。正式发布前复核读过的文件状态及来源提供的媒体摘要。新导出和精简转换的 preparation 支持媒体内容校验，历史无摘要产物按文件状态检查。

坏候选从选择中排除并记报告。入选项组装失败、来源变化或写入失败会中止发布，保留 partial 工作目录。已有目标及与输入重叠的路径会拒绝。合法空选择可以发布。
