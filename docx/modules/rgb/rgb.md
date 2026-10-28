# rgb.cpp / rgb.hpp：RGB 图像处理模块

## 本文件完成什么

源码：[rgb.cpp](../../../src/bag/rgb/rgb.cpp)，公开接口：[rgb.hpp](../../../include/vln_memory/bag/rgb/rgb.hpp)。

本模块接收一张未压缩图像的字段和只读像素视图，检查输入并输出独立持有的紧密 rgb8 帧。适用于后续 bag 提取、PNG 保存、深度同步及模型图片输入前的数据准备。

本轮不在 RGB 内实现 bag 读取、RGB-D 同步、PNG 写入或最终 Observation JSONL；这些由 bag 模块组装，避免五模块互相耦合。

## 实施前规划与自审

1. 元数据、尺寸运算、容量上限、缓冲区依次校验，再读取像素和分配输出。
2. RGB 原样复制、BGR 交换红蓝、mono8 复制到三个通道；逐行跳过填充。
3. 公共输入使用借用视图，输出拥有像素及字符串。
4. 预期坏帧用返回码表达；系统内存异常不伪装成传感器数据错误。
5. 自审：各接口能独立于 ROS 使用；输入不被修改；输出像素坐标不改变；没有隐藏定位或时间转换。

## 接口

```cpp
RgbResult decodeRgb(const RgbImageView & input, const RgbLimits & limits = {});
const char * rgbErrorCodeName(RgbErrorCode code) noexcept;
```

### RgbImageView：输入

| 字段 | 含义与要求 |
| --- | --- |
| stamp | 原始采样时间；sec 可负、零合法，nanosec < 1e9 |
| frame_id | 原始光学帧标识；不自动补值 |
| width / height | 正数，保持输入像素布局 |
| encoding | 严格支持 rgb8、bgr8、mono8；名称区分大小写 |
| step | 一行完整字节数，包含行尾填充 |
| is_bigendian | 只接受 0/1；8 位通道不需要字节交换 |
| data / data_size | 只读缓冲区；长度严格等于 step*height |

输入像素及 string_view 指向的字符串只需在函数调用期间有效；调用期间不得被其他线程修改。函数不会保存输入引用。

### RgbFrame：成功输出

- `pixels`：按行 R,G,B 排列的拥有型 vector，无行填充。
- `encoding="rgb8"`，`step=width*3`，像素长度为 `step*height`。
- `stamp/frame_id/width/height` 保留来源信息。
- `source_encoding/source_step/source_is_bigendian` 保留格式来源，便于追溯和后续 JSONL 记录。
- 不裁剪、不缩放，原图像素坐标保持不变。

mono8 输出保留灰度信息，但不产生本不存在的颜色。

### RgbResult：如何判断结果

先调用 `ok()`，成功后读取 `frame.value()`；失败时 `frame` 为空，使用 `code/message` 写报告。不要把默认构造的结果或失败结果当有效帧。

| 错误码 | 原因 |
| --- | --- |
| invalid_limits | max_pixels 或 max_input_bytes 为零 |
| invalid_dimensions | 图像宽或高为零 |
| invalid_stamp | nanosec 超出范围 |
| unsupported_encoding | 压缩、16 位或未支持编码等 |
| invalid_endianness | 标志不是 0/1 |
| invalid_step | 行跨度小于一行编码像素所需字节数 |
| size_overflow | 尺寸计算或标准输出 stride 不可安全表示 |
| size_limit_exceeded | 超出单帧像素数或源字节数上限 |
| invalid_data | 空指针、截断或多余的源字节 |

`none` 表示无错误。一个输入有多个问题时返回首先发现的错误，不承诺枚举全部问题。

默认上限为 16 Mi 像素、128 MiB 输入（含行填充）。输出最多为 max_pixels*3 字节，另有元数据开销；这不限制此前的 ROS 反序列化内存，也不是整个进程的内存预算。

## 使用示例

```cpp
#include <vln_memory/bag/rgb/rgb.hpp>
#include <iostream>
#include <utility>
#include <vector>

int main()
{
  std::vector<std::uint8_t> bgr{0, 0, 255, 0, 255, 0};
  vln_memory::RgbImageView input;
  input.stamp = {100, 0};
  input.frame_id = "camera_color_optical_frame";
  input.width = 2;
  input.height = 1;
  input.encoding = "bgr8";
  input.step = 6;
  input.data = bgr.data();
  input.data_size = bgr.size();

  auto result = vln_memory::decodeRgb(input);
  if (!result.ok()) {
    std::cerr << vln_memory::rgbErrorCodeName(result.code)
              << ": " << result.message << '\n';
    return 1;
  }
  auto frame = std::move(result.frame.value());
  // frame.pixels = {255,0,0, 0,255,0}，即红色、绿色。
  return frame.pixels.size() == 6 ? 0 : 1;
}
```

编译时链接 `vln_memory::vln_memory_rgb`，使用 C++17。

## 后续模块怎样使用

- bag 层反序列化 ROS Image 后优先调用 `decodeRosRgb`。
- RGB-D 匹配使用 `stamp`；关联时仍需 bag 提供 clock_id 和源消息 ID。
- PNG 导出使用 `pixels/width/height/step`。若使用 OpenCV，需按其接口转换成 BGR 后再 imwrite。
- CameraInfo 检查与本帧尺寸/光学帧对应关系；TF 负责定位，不由 RGB 推断。
- 最终 JSONL 可使用 source_* 和错误码；RGB 返回的是 C++ 数据，不自行拼接 JSON 字符串。

## 验证和复核

测试与命令见 [核心测试说明](tests/test_rgb.md)，实际验证结果见下方实施记录。重点人工复核三段代码：校验顺序、逐行转换、拥有型输出；无需进入 ROS 网络或其他四模块。

## 实施与验证记录


2026-10-28，已完成本轮 RGB 模块。

### 文件与接口复核

- 通读了公共头文件、核心实现、ROS 适配、CMake 安装导出及使用示例。
- 核心不依赖 ROS/vln_agent；ROS 适配只映射字段并调用核心；没有后续模块的空实现。
- 校验在像素访问与分配前完成，整数乘法先检查溢出；支持带行填充的 rgb8/bgr8/mono8。
- 输出拥有像素、frame_id 和源编码字符串，不依赖 ROS 消息或输入字符串的生命周期。
- 保留原始时间和空 frame_id，不伪造定位；像素不缩放裁剪，便于后续 CameraInfo/Depth/TF 使用。
- PNG/NPY 保存、跨消息同步、bag 读取和最终 JSONL 属于后续 bag 组装层，本轮没有声称完成这些能力。
- 每个 C++ 源文件的说明按模块归入 docx/modules/rgb/；测试说明位于该目录的 tests/，公共头文件与同名实现共同说明。

### 实际验证

| 检查 | 结果 |
| --- | --- |
| Linux g++ 11 C++17 严格告警（含 conversion/sign-conversion，Werror） | 通过 |
| RGB 核心 9 组行为测试，含 900 组生成样本 | 通过 |
| AddressSanitizer / UndefinedBehaviorSanitizer / leak 检查 | 通过 |
| Windows MinGW + CMake/Ninja 独立配置与编译 | 通过 |
| 正式 CTest rgb_core | 1/1 通过 |
| 安装后独立 find_package 消费者，构建并运行 | 通过 |
| docx/modules/rgb/rgb.md 的完整 C++ 使用示例 | 编译并运行通过 |
| package.xml XML 解析、文件结尾与源码同名文档检查 | 通过 |
| ROS 2 Humble/ament 构建与真实 Image 适配测试 | 初版未执行；Depth 集成阶段已在真实 Humble 环境通过，见下方补充 |

Windows 构建工具不支持以 UNC 为链接工作目录，验证构建改在本任务本地临时目录执行；这不是 RGB 代码错误。源码位于 /home/dev/agent/vln_memory，Linux 核心验证位于本包 build/standalone。未以仿造 ROS 头文件替代真实集成验证。

真实 ROS 环境中仍需执行 README 的 colcon build/test，验证 sensor_msgs 头文件、ament 导出及 rgb_ros_adapter；随后 bag 模块可直接调用 decodeRosRgb。

## 注释与文档分类复核

2026-10-28，根据人工审阅反馈调整：

- 实现注释补充在检查顺序、8 位字节序、行填充、容量计算、通道转换和输出所有权处。
- 公开结构体按采样身份、输入布局、借用缓冲区、输出与来源格式分组说明；不逐行解释赋值。
- ROS 适配及关键测试补充意图说明，便于从接口追踪到验证点。
- docx 根目录只保留导航；项目、共享类型、RGB 模块和 RGB 测试分别归类。整体架构不再保存 RGB 的详细测试历史。
- 编辑前后比较了去注释后的 C++ 词法内容，保持一致；本次没有修改转换行为或接口。
- 严格 C++17 语法检查通过；33 个本地文档链接与章节引用、源码同名说明覆盖和旧路径检查通过。

分类导航见 [文档首页](../../README.md)，整体开发约定见 [项目架构](../../project/architecture.md)。

## Depth 集成阶段补充

2026-10-28，环境已具备真实 ROS 2 Humble。新增 Depth 的严格构建中同时编译并运行了 rgb_core 与 rgb_ros_adapter，两项均通过；安装后 RGB 消费者也通过。此前“环境未提供”的文字仅记录初版验证边界。共享 SensorStamp 保持原样，新增 ImageGeometry；RGB 转换源码没有修改。

参见 [Depth 集成验证](../depth/depth.md#验证结果)。
