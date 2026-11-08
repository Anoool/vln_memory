# 正式观测数据集读取器

公开头为 [observation_dataset_reader.hpp](../../../include/vln_memory/engine/observation_dataset_reader.hpp)，链接 `vln_memory::vln_memory_observation_dataset`。ROS 和无 ROS 构建均提供。

```cpp
#include <vln_memory/engine/observation_dataset_reader.hpp>

auto dataset = vln_memory::openObservationDataset(observation_directory);
if (dataset.ok() && !dataset.reader->batches().empty()) {
  auto batch = dataset.reader->loadBatch(0);
  const auto & ids = dataset.reader->batches().front().observation_ids;
  auto observation = dataset.reader->loadObservation(ids.front());
  // 需要投影或读取媒体时再打开来源。
  auto preparation = dataset.reader->openSourcePreparation();
  // 便携包自动打开内置 preparation，外部引用可显式传入实际路径。
}
```

- `openObservationDataset` 校验清单、固定文件路径、摘要、批次序号和记录覆盖顺序。它检查每行的来源、地图与修订上下文，拒绝重复 ID，核对记录及 pending 计数。结果持有只读索引，支持合法空数据集。
- `batches()` 提供发布顺序中的批次及 ID。`loadObservation(id)` 定位到单条原始行，`loadBatch(index)` 复用一个流读取该批。读取时核对打开后观察到的文件状态和行内容，并用 `deserializeObservation` 做完整语义校验。批次中任一条失败则整批返回错误。
- `openSourcePreparation()` 默认取清单中的路径。便携包使用同目录下的 `preparation/`，整个目录迁移后自动解析。引用模式可传入实际路径。打开来源时校验绑定的 manifest SHA-256 和 map frame，并按已发布观测逐一核对配对 ID、两路消息 ID、原始时间和时差。显式相对路径以当前进程目录解析；清单中的相对提示以观测目录解析。独立查看观测时可暂不打开 preparation。
- 返回的 `Observation` 由调用方持有，保留原始时间、模型、位姿和 pending 原因。图像载荷继续由 `PreparationReader` 读取，实体和投影调用可复用已打开的来源。

| 数据集清单 | 完整性行为 |
| --- | --- |
| `observation_dataset/2` | 清单要求四个文件的 SHA-256，覆盖观测、批次、选帧记录及问题报告。观测行仍是 `observation/1`。 |
| `observation_dataset/1` | 兼容既有发布产物，按原清单核对路径、计数、上下文和引用。无发布时文件摘要的历史产物标记 `file_integrity_verified=false`。 |

`observation_not_found` / `batch_not_found` 表示索引中缺项。`invalid_manifest`、`invalid_batch`、`invalid_observation` 和 `integrity_mismatch` 分别用于清单、批次、记录及摘要错误。打开后发现文件状态、观测清单字节或目标行内容变化会返回 `source_changed`。来源不匹配由 preparation 返回 `invalid_reference`。摘要以交付清单为基线，原始录制的物理尺度、注册和同步依据仍来自观测及设备验收。

打开阶段流式扫描元数据，保留 ID、偏移、行摘要和批次目录，内存不随像素增长。按 ID 查找为 O(log N)，只读目标记录。完整记录校验延迟到回读时执行。连接来源时使用 preparation 的配对索引核对引用，像素由后续几何或媒体读取操作按需校验。验证覆盖 ROS 发布夹具及两种安装包消费者，见 [构建说明](../../project/CMakeLists.md#验证)。
