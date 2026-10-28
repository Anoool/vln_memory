# depth.cpp / depth.hpp：Depth 模块

## 实施前总体规划

本模块沿用 RGB 的核心/ROS 适配分层：sensor_msgs/msg/Image -> decodeRosDepth -> DepthImageView -> decodeDepth -> DepthFrame 或结构化错误。

Depth 核心使用 C++17，不依赖 RGB 实现、ROS 节点、CameraInfo、TF 或 bag。共享 SensorStamp 与 ImageGeometry 作为公开数据契约；bag 层完成实际配对、配置来源、原始证据保存和 JSONL 组装。

## 分部分规划与预审

1. **公开数据类型。** 明确原始编码、数值类型、源单位、optical-z 深度约定、RGB 对齐声明和输出所有权。预审：编码不能替代单位声明，不以相同分辨率推断对齐。
2. **数值解码。** 逐行按 16UC1/32FC1 读取，支持大小端与填充；检查乘法溢出和处理上限。预审：不把 uint8 缓冲区强转为 uint16/float 指针，避免未对齐访问和别名问题。
3. **标准化与质量。** 显式米/毫米换算；零、负值、非有限值、范围外和不可表示值输出 NaN 并分类计数。预审：全无效帧仍是成功解码，质量统计明确为零有效点。
4. **对齐契约。** 未知/未对齐可解码；声明已对齐必须附 RGB 网格和依据，检查深度布局与该网格一致。后续再与实际配对 RGB 的 ImageGeometry 检查。预审：这只是验证上游声明与元数据，不执行几何注册，不替代时间同步。
5. **ROS 适配与构建。** 以真实 Image 类型限定输入，映射字段并复用核心；新增独立 core/ROS 库和测试目标。预审：不复制第二份解码逻辑，不改变 RGB 的转换行为。
6. **验证与复核。** 已知数值、单位、大小端、奇数行填充、非对齐内存、无效值、对齐声明、异常元数据、边界、源生命周期、生成样本；真实 Humble、安装后消费者、完整文档链接与差异复核。

## 深度约定

- 输入消息：未压缩 sensor_msgs/msg/Image；压缩深度或 DisparityImage 需要另外的适配器。
- 数值编码：16UC1 为 uint16，32FC1 为 IEEE-754 float32。编码名称区分大小写。
- 源单位必须显式声明为米或毫米。REP 118 的标准组合是 32FC1/米与 16UC1/毫米；模块不根据编码静默推断。显式非标准组合仍按声明转换并保留来源。
- 深度含义必须显式声明。当前只接受 optical_z；射线距离需要 CameraInfo 才能转换，返回 unsupported_convention。
- 输出：紧密排列的 float32 米制 optical-z，保留源格式、单位、比例、采样身份、对齐声明和质量统计。
- aligned_to_rgb 需要非空依据以及非空 RGB 光学帧和正尺寸；其布局、frame_id 必须与深度一致。unknown/not_aligned 不携带目标 RGB 网格。

依据：[ROS REP 118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)、[Humble Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)。

## Git 审阅流程

现有 RGB 保存在 main 基线，本模块与必要的共享接口/构建/文档修改在 codex/depth-module 开发。完成后 git add，保留暂存差异供人工审阅，不自动合并。

## 代码与公开接口

源码：[depth.cpp](../../../src/bag/depth/depth.cpp)，头文件：[depth.hpp](../../../include/vln_memory/bag/depth/depth.hpp)。真实消息适配见 [depth_ros](depth_ros.md)。

```cpp
DepthResult decodeDepth(
  const DepthImageView & input, const DepthOptions & options,
  const DepthLimits & limits = {});
DepthAlignmentResult checkDepthRgbAlignment(
  const DepthFrame & depth, const ImageGeometry & rgb);
```

稳定报告名称由 depthErrorCodeName、depthUnitName、depthNumericTypeName 和 depthAlignmentStatusName 提供。

### 输入字段与调用者责任

| 数据类型/字段 | 约定 |
| --- | --- |
| DepthImageView | 调用期只读借用；支持的 encoding 严格为 16UC1/32FC1 |
| stamp / frame_id | 原始身份；不改写时钟、光学帧或零时间 |
| step / data_size | step 包含填充；长度必须等于 step*height |
| is_bigendian | 仅允许 0/1；与 CPU 大小端无关 |
| DepthOptions.input_unit | 必填 meters 或 millimeters；比例分别为 1.0、0.001 |
| DepthOptions.convention | 必须为 optical_z；未给出或射线距离明确失败 |
| min_depth_m / max_depth_m | 正深度还须落在闭区间内；min 有限且 >=0，max>min，允许 +Inf |
| DepthLimits | 默认 16 Mi 像素、128 MiB 输入，计入行填充 |

限额约束的是本次转换，不限制上游消息分配；输出占像素数乘以 sizeof(float) 字节并有元数据开销。分配失败等系统异常不转换为坏帧。

### 输出与无效值

DepthFrame 拥有像素和全部字符串，源消息、options 释放或复用后仍可使用。

- depth_m：行优先的 float32 米制 optical-z，无行填充；encoding=32FC1，step=width*4。
- source_encoding / source_numeric_type：保留原始编码与 uint16/float32 类型。
- source_unit / source_scale_to_m：保留显式单位与实际换算比例，避免下游二次换算。
- source_step / source_is_bigendian：原布局来源。
- min_depth_m / max_depth_m：保留本次过滤范围，便于解释质量报告。
- alignment：保留原始对齐声明、RGB 目标网格和依据。
- statistics：有效点数、无效点数、有效比例和互斥原因计数。

无效原因包括零值、负数、非有限值、范围外及 float32 不可表示值；输出统一为 quiet NaN。原始特殊值应由 bag 层从原始消息保存，不从标准化结果反推。16UC1 的 65535 不默认当无效值，如需过滤，应按设备范围显式配置。

全无效图像返回成功且 valid_ratio=0：这表示消息已正确解码，不表示具备可投影的有效深度。后续 bag/几何层据统计判断是否进入 pending。NaN 保存在二进制深度中，不直接写成 JSON 数值。

### RGB 对齐检查的边界

| 声明状态 | 解码与检查结果 |
| --- | --- |
| unknown | 可解码；即使尺寸/frame_id 相同也保持 unknown |
| not_aligned | 可解码；检查返回 not_aligned，不能直接用 RGB 像素索引深度 |
| aligned_to_rgb | 需要非空 basis 与目标 RGB 网格；网格/光学帧须匹配本深度，随后与实际配对 RGB 再检查 |

checkDepthRgbAlignment 的状态还包含 invalid_reference、inconsistent_declaration、size_mismatch、frame_mismatch。只有 alignedToRgb() 为真时，声明与实际 RGB 元数据才一致。

这不验证像素内容、CameraInfo、注册算法或采样时差。basis 应来自已核实的驱动/注册配置或标定 ID，不能因为话题含 aligned、分辨率相同，就自动填入“已对齐”。实际配准与未对齐重投影属于后续基于 CameraInfo/TF 的能力。

共享网格来自 RGB 的公开字段，例如 ImageGeometry{rgb.width, rgb.height, rgb.frame_id}；Depth 不链接 RGB 实现。参见 [共享数据类型](../../common/sensor_types.md)。

### 错误结果

先检查 result.ok()，再访问 frame。失败没有部分帧，使用 code/message 写报告。

| 分类 | 错误码 |
| --- | --- |
| 布局与数据 | invalid_dimensions、invalid_stamp、unsupported_encoding、invalid_endianness、invalid_step、invalid_data |
| 容量 | invalid_limits、size_overflow、size_limit_exceeded |
| 语义 | missing_unit、unsupported_unit、missing_convention、unsupported_convention |
| 配置一致性 | invalid_range、invalid_alignment |

一个输入有多处问题时返回最先发现的错误。默认构造的 DepthResult 没有帧，code=none 本身不表示成功。

## 最小完整用法

```cpp
#include <vln_memory/bag/depth/depth.hpp>
#include <iostream>
#include <vector>

int main()
{
  // 原始 uint16 little-endian 值为 1000，单位由配置明确声明为毫米。
  const std::vector<std::uint8_t> bytes{0xE8, 0x03};
  vln_memory::DepthImageView input;
  input.frame_id = "rgb_optical";
  input.width = 1;
  input.height = 1;
  input.encoding = "16UC1";
  input.step = 2;
  input.data = bytes.data();
  input.data_size = bytes.size();

  const vln_memory::ImageGeometry rgb_grid{1, 1, "rgb_optical"};
  vln_memory::DepthOptions options;
  options.input_unit = vln_memory::DepthUnit::millimeters;
  options.convention = vln_memory::DepthConvention::optical_z;
  options.alignment = {vln_memory::DepthAlignment::aligned_to_rgb,
    rgb_grid, "verified_registered_stream_config"};
  const auto result = vln_memory::decodeDepth(input, options);
  if (!result.ok()) {
    std::cerr << vln_memory::depthErrorCodeName(result.code) << ": " << result.message;
    return 1;
  }
  const auto alignment = vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb_grid);
  return result.frame->depth_m[0] == 1.0F && alignment.alignedToRgb() ? 0 : 1;
}
```

链接 vln_memory::vln_memory_depth。ROS 使用 decodeRosDepth，详见同名适配说明；安装后组合 RGB 与 Depth 的完整例子见 [depth_consumer](tests/depth_consumer.md)。

## 验证结果

2026-10-28，实际检查：

| 检查 | 结果 |
| --- | --- |
| C++17 严格告警，含 conversion/sign-conversion/Werror | 通过 |
| Depth 11 组行为测试，含 600 组生成样本 | 通过 |
| 无 ROS 核心 CMake/CTest：RGB、Depth | 2/2 通过 |
| 真实 Humble/ament colcon 构建 | 通过 |
| colcon 测试：RGB/Depth 核心与两种真实 Image 适配 | 4/4 通过，无跳过 |
| AddressSanitizer、UndefinedBehaviorSanitizer、泄漏检查 | 通过 |
| 安装后独立核心消费者：RGB 与 Depth 网格契约 | 编译、运行通过 |
| 安装后独立 ROS 消费者：sensor_msgs 与适配库传递依赖 | 编译、运行通过 |
| 本页完整 C++ 示例 | 编译、运行通过 |
| XML、分类同名说明、67 个本地链接/章节与 git diff 格式 | 通过 |

本轮测试包含真实 ROS 消息类型，但没有实际相机或 bag 数据；真实注册正确性仍须在 bag/CameraInfo 阶段核验。测试源码与执行方式见 [核心测试](tests/test_depth.md)、[ROS 测试](tests/test_depth_ros.md)。

## 整体自审

- 单位与数值编码独立；没有隐式毫米比例，也没有 ray range 假转换。
- 多字节读取不依赖输入地址对齐，使用明确字节序和 memcpy 解释 float 位模式。
- 输入错误在像素读取和输出分配之前返回；输出独立拥有数据和声明。
- 对齐声明检查与实际 RGB 检查共用规则，未知状态不会升级为对齐。
- 质量统计和范围来源完整；全部无效的图像与坏消息分开表达。
- 公共图像网格类型只增加共享契约，RGB 转换实现保持原样。
- 代码、ROS 适配、测试和同名说明按层组织，供后续 bag 模块直接调用。


## 审阅入口

本地仓库：/home/dev/agent/vln_memory，分支 codex/depth-module。RGB 基线提交 f8fe7c4 保留在 main。本次新增模块与配套修改完成后暂存；使用 git diff --cached 查看，尚未自动合并。
