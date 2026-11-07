# 构建输入筛选、关键帧与批次验收

日期：2026-11-05。实现分支：codex/build-input-selection。该交付推进 S2 离线输入组织，正式 Observation、记忆工作库与模型业务循环继续实施。

## 软件回归

- 无 ROS 核心 CTest 11/11 通过，日志位于 build/build-input-core/ctest.log。
- ROS CTest 20/20 通过，含录包夹具生成、构建输入磁盘集成、原有 reader/NumPy/适配回归；日志位于 build/build-input-ros/ctest.log。
- 核心及 ROS 两种安装模式的 build_input_consumer 编译、链接与采样检查通过；已安装 memory_build_input 无需额外设置本包库路径即可运行 --help。
- 针对位移累计、yaw 环绕、缺/坏位姿、断点、跳变上限等号与越界、固定分批、负/零/同刻时间、重复 ID、非法阈值/时间和跨上下文的检查均通过。
- 磁盘集成核对来源/pending、新目录重复构建清单一致、三种用途、已有目标/输入路径重叠拒绝、坏媒体逐对失败和无效配置禁止发布。

## 固定真实短包

输入为 build/cdr-real-check-mpttQq/export_source_png16（v7），同时用 v6 metric_npy 验证已安装核心消费者；已安装 ROS 消费者验证 v7。原始 preparation 和媒体保持不变，损坏测试仅操作独立副本。

输出及完整检查记录位于 build/build-input-acceptance-9cxvdmo2；acceptance.json、逐条日志及 run_acceptance.py 保留在该目录。

| 显式配置 | 候选数 | 选中数 | 批次数 |
| --- | --- | --- | --- |
| rgb，不采样，batch_size=3 | 10 | 10 | 4 |
| depth_map，不采样，batch_size=3 | 10 | 10 | 4 |
| prepared_rgbd，不采样，batch_size=3 | 10 | 0 | 0 |
| rgb，distance_m=0.1、yaw=10 度，batch_size=3 | 10 | 1 | 1 |

采样结果与直接读取来源帧 map_body_tf 后独立计算的参考一致。十对继续保留 depth_rgb_alignment_unverified，capture_sync_status 未被修改。该样本的注册、物理深度尺度与曝光同步仍未获得独立物理验收；depth_map 可用性表示输入字段齐备，不能替代逐像素投影和物理标定验证。

空配对数据集成功发布空清单；未知选项、负 batch_size 和非有限阈值被拒绝。实际运行了公开 C++ 入口、CLI 和安装后消费者。

## 复跑

```bash
ctest --test-dir build/build-input-core --output-on-failure
ctest --test-dir build/build-input-ros --output-on-failure
python3 build/build-input-acceptance-9cxvdmo2/run_acceptance.py
```

真实短包检查脚本每次在 build 下创建新的独立输出目录。模块契约见 [build_input](../modules/engine/build_input.md)。
