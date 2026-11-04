# Preparation 配对读取

[公开头](../../../include/vln_memory/bag/builder/preparation_reader.hpp) 提供 loadPreparedPair，输入 preparation 根目录与已保存的 pair_id。链接 vln_memory::vln_memory_preparation_reader，可在 ROS 构建或独立构建中使用。

读取器在当前目录查找 v5/v6 已发布 bag_preparation/1 的配对和两帧，校验标定、原始来源及 TF 端点引用。两图沿用各自的原始采样时刻、保存的配对结果和 pending 原因。

结果持有两帧元数据、可选模型和位姿、RGB 底盘位姿、证据路径和可选米制 DepthFrame。RGB 媒体校验相对路径和 PNG 头部的尺寸、格式后，返回绝对路径供消费端按需解码。NPY 支持 BagBuilder 写出的 v1.0 二维 C 顺序小端 float32，v5 的深度范围取自 manifest。v6 源 PNG 以单通道 uint16 读取，复用深度解码的比例、范围和无效值处理。

文件缺失、路径越界、格式损坏和断引用返回结构化错误码。合法缺项以空字段返回。pending 配对仍可读取，各消费操作检查自己需要的字段。

[实现](../../../src/bag/builder/preparation_reader.cpp)按文件扫描 JSONL，仅保存当前配对引用的行。时间随元数据行数和深度像素数增长，额外空间主要用于单帧深度。原始 CDR 检查引用及文件长度，字节对照和独立位姿重算由验收工具负责。

[回归程序](../../../test/bag/builder/test_preparation_reader.cpp)使用 BagBuilder 生成的构造夹具，覆盖 v6 PNG/NPY、v5 兼容格式、断引用、错时刻位姿、位姿缺项和坏 NPY。CTest 的 preparation_reader 用例复用 bag_builder_ros 生成的产物。可在仓库根目录用本地真实短包复跑：

```bash
build/depth-projection-core/test_preparation_reader \
  build/depth-png-ab-20261104/metric_npy \
  build/depth-png-ab-20261104/source_png16 \
  build/depth-geometry-acceptance-20261104/run-1.json
```
