# 构建与真实 bag 输出清理（2026-11-03）

用户要求删除不必要输出以节约空间。本次从 codex/camera-model-compatibility 的 6d47c65 新建 codex/build-cleanup-20261103，实际删除仅在 /home/dev/agent/vln_memory/build 内；源码和原始 ZIP 不动。

## 删除前检查

工作区干净，Git 没有跟踪 build；无 BagBuilder/审计/构建任务正在使用输出。逐项解析拟删除绝对路径，确认位于真实 build 根目录内，拒绝符号链接目标和 build 根本身。

原始 ZIP 存在，大小 859535780 字节，SHA-256 重新核对为 c06ac1e3beccfc62b1562a0c7e4d73bdb9536d4575c3ecebfb158191f54e6452。先保存小型元数据/汇总及测试日志，再删除已验收可重建结果。

## 实际清理

| build 下的目录/文件 | 删除文件数 | 文件字节数 |
| --- | ---: | ---: |
| camera-model-20261103/full_output | 28339 | 16404527947 |
| camera-model-20261103/final_short_output | 7369 | 64494448 |
| camera-model-20261103/input | 2 | 990039449 |
| camera-model-20261103/short_input.db3 | 1 | 3325952 |
| camera-model-ros/test_artifacts | 2641 | 66392612 |
| rgbd-strict-ros | 4489 | 204420436 |
| rgbd-strict-install | 122 | 19097252 |
| rgbd-strict-consumer | 165 | 2410482 |

合计删除 43128 个文件、17754708578 字节（约 17.75 GB）。build 的文件内容占用由 17811171664 字节（约 17.81 GB）降至约 61 MB；统计为 WSL 项目目录中的文件大小。

## 保留内容

- 原始 Windows ZIP、全部源码/测试脚本/配置和 Git 记录。
- 当前 camera-model-ros、camera-model-core、两种 install 和消费者目录，继续用于增量构建/接口核对。
- camera-model-20261103 下的独立全包/子集审计 JSON、运行/资源/构建日志、来源指纹与配置。
- audit_records/full_output 与 audit_records/final_short_output 保存 manifest/summary 和少量首尾 JSONL 样本；没有媒体/原始 CDR，不是可消费 preparation 数据集。
- 当前 CTest 日志与退役旧构建的小型日志快照，以及更早轮次的小型审计记录。

cleanup_preflight.json、cleanup_progress.json、cleanup_summary.json 在忽略的验证目录记录实际绝对路径及计数。生成快照/日志不入 Git。

## 重新生成

从原始 ZIP 恢复 camera_record 目录，使用 [camera_record.yaml](../../config/camera_record.yaml) 替换输入与新输出路径，重新运行 BagBuilder。完整媒体已经删除，旧 real_config/final_short_config 中的输入/输出路径属于历史运行记录，不能假定仍存在。

CTest 的 test_artifacts 会由测试夹具按需重新创建；清理后未为重复验证重新生成大型夹具。先前 16/16 ROS、9/9 核心和真实 bag 零差异验收仍是历史已执行结果，详见 [相机模型验收](camera_model_verification.md)。

本轮只改变生成文件的保留状态和说明，不改变代码行为，也不合并 main。
