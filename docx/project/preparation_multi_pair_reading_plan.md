# Preparation 多对数据读取方案

日期为 2026-11-05。索引 reader 阶段已实现，metadata_only 阶段保留为后续设计。

当前公开接口为 openPreparation(root)、reader.pairs() 与 reader.loadPair(pair_id)。loadPair 沿用完整深度读取契约。下文带 mode 参数的接口和两种模式表描述后续目标，已实现接口见 [模块说明](../modules/bag/preparation_reader.md)。

## 1. 目标与当前依据

本轮交付一个可重复访问同一 preparation 数据集的读取器。调用方能够列出全部已保存配对、按 ID 读取其中一对，并选择读取元数据或同时恢复米制深度。输入继续采用已发布的 BagBuilder v5/v6 `bag_preparation/1`。

现有 [preparation_reader.hpp](../../include/vln_memory/bag/builder/preparation_reader.hpp) 提供 `loadPreparedPair(root, pair_id)`，返回独立持有数据的 `PreparedPair`。[实现](../../src/bag/builder/preparation_reader.cpp) 对每次调用重新扫描配对、帧、标定、TF 和消息 JSONL，随后读取深度像素。逐对循环时，会重复解析同一批元数据。

本轮在读取层提供目录内多对访问能力。用途筛选、关键帧和批次选择由后续离线构建入口消费这些接口。阶段依据见 [系统规划第 10 节](system_plan.md#102-后续任务与依赖)。

## 2. 总体设计

流程为打开目录、建立元数据偏移索引、列出配对、按 ID 读取。

```text
openPreparation(root)
  -> 校验 manifest 与元数据文件
  -> 各 JSONL 扫描一次，记录 ID 到文件偏移的映射
  -> 构建轻量配对目录
  -> PreparationReader
       pairs()                           -> 配对目录
       loadPair(id, metadata_only)        -> 元数据、引用与媒体路径
       loadPair(id, with_depth)           -> 上述数据及可选米制深度
```

索引保存记录 ID、二进制文件偏移与行长度，配对目录保存后续遍历需要的小型字段。单条完整 JSON 在实际读取时解析。这样内存主要用于索引及调用方当前持有的结果。

一份 reader 对应一个 preparation 根目录及其明确的时钟段。每次读取使用局部文件流，索引在打开成功后只读。

## 3. 公开接口草案

继续使用现有公开头和链接目标 `vln_memory::vln_memory_preparation_reader`。JSON 类型和文件偏移细节保留在实现文件内。

```cpp
// 轻量目录项。时间保持原始 sec/nanosec。
struct PreparationPairInfo
{
  std::string pair_id;
  std::string rgb_message_id;
  std::string depth_message_id;
  SensorStamp rgb_stamp;
  SensorStamp depth_stamp;
  std::int64_t delta_ns{0};
  std::string status;
  std::vector<std::string> pending_reasons;
};

enum class PreparationLoadMode
{
  metadata_only,
  with_depth
};

// 下列声明展示主要操作，省略生命周期声明和私有索引。
class PreparationReader
{
public:
  const std::vector<PreparationPairInfo> & pairs() const noexcept;

  PreparationReadResult loadPair(
    std::string_view pair_id, PreparationLoadMode mode) const;
};

// open 结果拥有 reader，采用与现有读取结果相同的 code/message/ok 形式。
struct PreparationOpenResult
{
  PreparationReadCode code{PreparationReadCode::io_error};
  std::string message;
  std::unique_ptr<PreparationReader> reader;

  bool ok() const noexcept;
};

PreparationOpenResult openPreparation(
  const std::filesystem::path & preparation_root);
```

`PreparationOpenResult` 在成功时持有 reader，在失败时提供错误码和消息。`PreparationReader` 独占自身索引，目录引用的有效期为 reader 的生命周期。

`pairs()` 保留 `rgbd_pairs.jsonl` 的原始行顺序。重复时间戳、零时间戳和负秒时间戳原样保留。目录项是保存信息的摘要，引用一致性在 `loadPair` 时核验。

`loadPair` 返回现有 `PreparationReadResult` 与 `PreparedPair`。结果中的模型、位姿、路径和可选深度继续独立持有，reader 销毁后仍可使用结果。reader 自身只保留元数据索引。

典型调用方式如下。

```cpp
auto opened = openPreparation(preparation_root);
if (!opened.ok()) {
  // 记录 opened.code / opened.message。
  return;
}

const auto & reader = *opened.reader;
for (const auto & info : reader.pairs()) {
  auto result = reader.loadPair(
    info.pair_id, PreparationLoadMode::metadata_only);
  if (!result.ok()) {
    // 记录当前 pair_id 及错误，调用方决定是否继续。
    continue;
  }
  // 消费本对元数据，后续根据用途选择需要完整深度的配对。
}
```

## 4. 打开与读取的职责

### 打开数据集

1. 解析规范化根目录及 manifest，复用现有数据集类型、schema、builder_version 和发布状态校验。
2. 顺序扫描 `rgbd_pairs.jsonl`、`frames.jsonl`、`camera_info.jsonl`、`tf.jsonl`、`messages.jsonl`。
3. 校验各行的 JSON 语法、schema、索引键及键唯一性。TF 键继续使用 message_id 和 transform_index。
4. 解析配对目录所需的 ID、两路时间、时差、状态和 pending 原因。
5. 保存元数据文件的规范路径、大小和修改时间，发布完整 reader。

索引在文件的二进制模式下记录偏移及长度，并检查数值转换和范围。目录项从配对行生成，媒体和关联记录的完整语义在逐对读取时校验。

合法空配对文件返回成功及空目录。重复 ID 会导致打开失败，因为索引无法确定该 ID 的唯一记录。任一打开错误都只返回错误结果。

### 按 ID 读取一对

1. 检查元数据文件是否仍与打开时的文件信息一致。
2. 按索引读取该配对及引用的两帧。
3. 收集当前配对引用的标定、TF 端点和源消息 ID，按偏移读取对应行。
4. 调用现有的帧关联、相机模型、位姿链、来源、路径和媒体校验逻辑。
5. 根据读取模式处理深度媒体，读取结束前再次检查元数据文件信息。
6. 返回独立持有的 `PreparedPair` 或当前配对的结构化错误。

来源证据检查只遍历当前配对实际引用的记录。来自全数据集索引的无关标定或 TF 记录不会进入当前配对的证据集合。

### 两种读取模式

| 检查或产物 | metadata_only | with_depth |
| --- | --- | --- |
| 原始时间、配对来源、模型与位姿 | 核验并返回 | 核验并返回 |
| 标定、TF 端点与源消息引用 | 核验 | 核验 |
| 原始 CDR 路径与文件长度 | 核验 | 核验 |
| RGB 路径及 PNG 头部网格 | 核验 | 核验 |
| Depth 路径、存储单位、格式头、尺寸及资源范围 | 核验 | 核验 |
| NPY 声明的样本区长度 | 核验 | 核验 |
| Depth 像素、有效值与质量计数 | 由完整读取阶段核验 | 读取并核验 |
| PreparedPair.depth | 空值 | 媒体存在且校验通过时持有米制深度 |

读取模式由调用方显式传入。`metadata_only` 返回成功表示元数据及上述基础媒体检查通过；像素质量结论由 `with_depth` 给出。

导出记录中合法的 null 媒体、缺模型或缺位姿继续保留为空。pending 原因随结果返回。RGB 继续采用媒体路径及 PNG 头部检查的现有契约。

## 5. 错误处理与文件生命周期

| 场景 | 处理 |
| --- | --- |
| 非 v5/v6、schema 或发布类型不支持 | open 返回 unsupported_dataset |
| JSON 损坏、必要目录字段格式错误 | open 返回 invalid_metadata |
| 重复索引键 | open 返回 invalid_reference |
| 必需文件打不开或读取失败 | 返回 io_error |
| 查询不存在的 pair_id | loadPair 返回 pair_not_found |
| 某对的帧/标定/TF/消息引用缺失或矛盾 | 该次 loadPair 返回 invalid_reference |
| 某对媒体头、深度样本或存储参数损坏 | 该次 loadPair 返回 invalid_media |
| 合法 pending 或合法可选字段缺失 | 读取成功，保留状态、原因及空字段 |
| 元数据在打开后发生可检测的改变 | loadPair 返回新增 source_changed |

全局索引或目录无法解析时，打开失败。索引成功后，某一对的关联或媒体错误只影响该次读取，调用方可以继续读取其他 ID。每次失败均返回空 pair，消息尽量带文件路径、行号或相关 ID。

新增 `source_changed` 追加到现有枚举末尾，保留既有枚举值。新 reader 面向已发布、读取期间保持稳定的数据集。文件大小和修改时间检查能够发现常规修改；相同大小且恢复修改时间的替换超出该检查的保证范围。

媒体路径在每次读取时重新进行根目录约束和文件检查。CDR 继续核对引用及长度，读取器的校验范围以现有契约为基础。

## 6. 复用与兼容

原有 `loadPreparedPair(root, pair_id)` 继续保留两参数签名及现有默认行为，包括完整深度读取、v5 NPY 回退、pending 可读取以及错误时空 pair。

旧入口保留按当前配对选择记录的路径，新 reader 使用偏移索引取记录。两条路径共用配对组装、引用校验和媒体恢复逻辑。

这一区分需要保留。新 reader 的 open 校验整个索引键和目录；将旧入口直接改成 open 的包装，会使无关记录的 schema 或重复 ID 错误影响旧调用方。新接口的全局打开校验与旧接口的选中记录校验分别测试。

实现重构按内部职责分开组织。

- JSONL 扫描和偏移查找负责定位记录。
- 配对组装负责当前配对的元数据、来源和几何引用。
- 媒体检查与深度恢复负责路径、格式和像素质量。

这些逻辑继续位于现有 preparation_reader 模块。reader 的私有状态封装索引，公开契约保持扁平。

## 7. 时间与空间开销

令 B 为元数据总字节数，N 为索引记录数，J 为一对引用的记录字节数，Q 为一对引用的记录数，D 为深度像素数。

沿用标准库有序映射建立 ID 索引，打开耗时约为 O(B + N log N)。逐对读取约为 O(Q log N + J)，完整深度读取再增加 O(D)。这里按正常有界的记录 ID 长度估算。

索引和配对目录占用 O(N) 级空间，实际还取决于 ID 与 pending 字符串长度。深度空间由调用方当前持有的读取结果决定。逐对处理时仅需要当前一帧深度及其解码临时空间。

相较逐对调用旧入口，新路径把 JSONL 全文件解析集中到打开阶段。重复访问某对时仍按索引读取它实际引用的行。

## 8. 实施顺序与涉及文件

| 步骤 | 内容 | 主要文件 |
| --- | --- | --- |
| 1 | 补充目录项、读取模式、open 结果及 reader 操作声明，并写接口注释 | preparation_reader.hpp |
| 2 | 从现有入口提取可共享的配对组装、证据检查和媒体恢复逻辑，复跑旧回归 | preparation_reader.cpp |
| 3 | 实现 JSONL 偏移索引、配对目录、文件变化检查及新入口 | preparation_reader.cpp |
| 4 | 实现 metadata_only，并保证其检查范围与 with_depth 明确区分 | preparation_reader.cpp |
| 5 | 增加针对性测试，验证 ROS 与无 ROS 安装后的接口 | test_preparation_reader.cpp、test/install_consumer |
| 6 | 更新模块说明、文档导航及系统规划中的本项进度 | docx/modules/bag/preparation_reader.md 等 |

沿用现有库目标。实施时按能够单独审阅的步骤提交到 `codex/` 分支。

## 9. 验收条件

1. 同一已发布目录可列出全部配对，顺序与源 JSONL 一致，零/负/相同时间戳均保留。
2. 所有正常配对的 `with_depth` 结果与旧接口在时间、来源、模型、位姿、pending 和深度值上逐项一致。
3. v5 NPY、v6 NPY 和 v6 PNG 均通过读取回归。
4. metadata_only 可用于遍历元数据，其 depth 为空；完整模式可检测仅出现在深度样本区的损坏。
5. 打开时的格式/重复键失败、逐对断引用、合法缺项、空数据集和未知 ID 各有针对性用例。
6. 两对中一对的媒体损坏后，另一对仍可读取；全局不可索引的元数据错误会使 open 失败。
7. 修改元数据大小或修改时间后，reader 返回 source_changed；新建 reader 能读取新的稳定数据集。
8. reader 销毁后，已返回的配对结果仍可用于已有投影接口。
9. 构造较多配对时，每个 JSONL 的顺序建索引扫描只发生在 open 阶段，逐对读取通过偏移定位。通过内部测试观察或一次文件访问跟踪核实。
10. 复用现有构造录包夹具和固定真实短包完成验证。真实短包测试记录实际可用配对数和原始 pending 原因。

当前已完成索引建立、目录列举、按 ID 完整读取和旧接口兼容验证。后续继续实现 metadata_only，并验证两种模式的媒体校验边界。
