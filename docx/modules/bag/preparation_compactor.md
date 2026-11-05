# Preparation 精简转换

[公开头](../../../include/vln_memory/bag/builder/preparation_compactor.hpp) 提供 `compactPreparation(source, output)`，链接 `vln_memory::vln_memory_preparation_reader`。输入为已发布的 v5/v6/v7 preparation，输出为明确声明 `save_raw_cdr=false` 的 v7 新目录。

ROS 构建和安装后可使用以下命令。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory preparation_compact \
  --input /absolute/path/to/preparation \
  --output /absolute/path/to/new_compact_preparation
```

无 ROS 构建可运行 `build/preparation-cdr-core/preparation_compact`，参数相同。`--help` 返回用法。成功退出 0，转换失败退出 1，参数错误退出 2。

结果包含 completed、code、message、output_directory、work_directory、messages、pairs_checked 和 raw_bytes_omitted。最后一项统计源数据实际保留而目标省略的 CDR 字节数，净目录体积还受 JSON 路径缩短及转换来源字段影响。失败计数只表示已执行的进度。

转换过程读取源目录，创建独立 partial 工作目录，复制 JSONL、报告及实际媒体，统一将 messages 和嵌套 source 的 raw_path 设为 null。消息、帧、标定和 TF 来源字段在转换前逐项核对。源消息原长度和 SHA-256 继续保留。

目标 manifest 保留 bag_id、时间/地图上下文、配置和配对状态，将格式版本标记为 7，声明 save_raw_cdr=false。conversion、conversion_source_directory、conversion_source_builder_version 记录本次转换来源。

目标的每个配对通过完整 PreparationReader 读取后，使用与 BagBuilder 共用的 Linux renameat2 不覆盖发布逻辑生成最终目录。既有目标和源/目标包含关系均被拒绝。失败保留 work_directory，源文件由读取操作访问。

转换时按源文件的大小、修改时间及规范路径核对常规变化，输入应为稳定的已发布数据。消息来源校验暂存 O(N) 元数据，后续验证前释放；像素按当前配对加载。

新导出使用 `bag.save_raw_cdr`，API 和 YAML 默认 false。true 模式仍保存原始消息，并与历史数据一样核对 CDR 路径和长度。v7 的保存策略字段必须是 JSON 布尔值，缺项或类型错误明确返回元数据错误。

PreparedPair 的 rgb_raw_evidence 和 depth_raw_evidence 现在为 optional 路径，省略时为空。这是公开 C++ 字段类型变化，下游需重新编译并检查可选值。

验证结果见 [CDR 存储验收](../../reviews/cdr_storage_verification.md)。源消息重放及原始字节分析需要源 bag，精简数据独立提供提取后的媒体与结构化信息。
