# camera_info.hpp / camera_info.cpp：CameraInfo 校验与图像关联

## 1. 编码前分步规划

当前开发观测准备层 bag 的第三个模块。保留 RGB/Depth 的输出接口，不引入 TF、bag 读写、OpenCV 或 vln_agent 依赖。

1. 公共时间检查放 bag/common/sensor_time：整数纳秒计算 candidate-reference；调用方明确同一时钟，显式容差，返回差值和超限状态，不改写时间。
2. CameraInfoData 对应 CameraInfo 字段并拥有其字符串/数组；额外保留 bag 提供的真实话题。decodeCameraInfo 按值接收，校验后移动到结果；错误返回稳定代码和原因。
3. 校验消息时间、绝对话题、frame、非零标定分辨率、K/D/R/P 有限性、K[0] 未标定标记、支持模型系数数量、内参结构、R 正交旋转、P 结构。保留 K 与 P 的差异和双目平移，不能混用。
4. ROI 使用完整未 binning 坐标，binning 0 归一解释为 1；检查范围和整除，得到实际图像网格。不能直接用标定 width/height 匹配采集图像，也不猜测任意 resize 的新内参。
5. checkCameraInfoImage 使用显式话题映射、预期光学 frame、配置依据和原始/校正图像类型，核对指定图像的话题、尺寸、frame、时间。frame 名称不证明坐标轴；ROI.do_rectify=true 的校正网格需要几何库，当前明确拒绝该组合。
6. 标定匹配默认为逐帧时间检查；固定标定必须显式声明策略和依据，才允许跳过采样时间差检查。动态标定的版本/有效区间选择由后续 builder 实现，不能整包取最后一条。
7. ROS 薄适配只复制真实 sensor_msgs::msg::CameraInfo 字段和外部话题，复用核心，不在消息层推断 topic、optical frame 或时钟。
8. 增加核心边界测试、真实 ROS 适配测试、安装后的 RGB/Depth/CameraInfo 协作消费者；完成后通读接口、实现和文档并运行验证。

## 2. 规划自审

- 话题来自 bag 元数据，CameraInfo 消息自身不含话题；来源与图像绑定分别记录。
- 光学轴约定为 x 右、y 下、z 前；只核对显式 frame 映射，不靠字符串后缀认证。
- “有效”是结构/数值可用，不代表物理标定准确；原始证据由 builder 保留，失败不返回半成功模型。
- RGB、Depth、CameraInfo 保留各自 header 时间。没有实际 bag，不能宣称设备已同步。
- Observation 以 RGB 时间为索引；深度点变换查 depth 时间的 TF，RGB 相机位姿查 RGB 时间。二者差异大时拒绝或后续做明确运动补偿，不取平均时间。
- bag 接收时间用于追溯和延迟分析；不能默认替代采样时间。在线控制还需要当前机器人状态，历史感知点先在采样时刻变到 map，再交由实时规划/控制使用。
- 时间/标定声明不等于物理配准；必须继续调用 Depth 的注册检查。

## 3. 文件与模块职责

| 文件 | 实际工作 |
| --- | --- |
| include/vln_memory/bag/camera_info/camera_info.hpp | 独立持有的标定字段、图像引用、流绑定、错误和公开函数 |
| src/bag/camera_info/camera_info.cpp | 标定数值与 ROI/binning 检查，图像匹配，调用公共时间检查 |
| include/vln_memory/bag/camera_info/camera_info_ros.hpp | 真实 ROS 消息适配签名 |
| src/bag/camera_info/camera_info_ros.cpp | 把 CameraInfo 和外部 topic 映射为核心字段 |
| include/vln_memory/bag/common/sensor_time.hpp | 同时钟关系、容差、带符号时差的公共契约 |
| src/bag/common/sensor_time.cpp | 无浮点、无时间改写的整数纳秒比较 |

本模块只链接 vln_memory_sensor_time，不链接 RGB/Depth 实现；两种帧的公开宽高/frame/stamp 足以构造 CameraImageReference。来源 topic 仍由后续 builder 提供。

## 4. 输入、输出与错误

```cpp
CameraInfoResult decodeCameraInfo(CameraInfoData input);
CameraInfoMatchResult checkCameraInfoImage(
  const CameraInfoData & info, const CameraImageReference & image,
  const CameraInfoMatchOptions & options);
```

CameraInfoData 拥有 topic、frame_id、distortion_model、D/K/R/P、原始 stamp、标定尺寸、ROI、binning。按值接收避免对源消息的生命周期依赖；传入左值会复制，std::move 可以转移所有权。

CameraInfoResult 必须先检查 ok() 再读取 info。失败无部分模型，code/message 可直接交给未来 issues.jsonl；原始失败消息仍应由 builder 留作证据。系统内存分配失败等 C++ 异常不伪装成坏标定。

| 校验 | 规则 |
| --- | --- |
| topic | 已解析的绝对 ROS 话题，不能含空 token、数字开头 token、空白或 remap 表达式 |
| frame | 非空、无前导 /、无空白/控制字符；不强制命名后缀 |
| stamp | nanosec < 1e9；负秒和零时刻允许 |
| 分辨率 | 标定 width/height 均非零 |
| 未标定 | K[0]==0 返回 uncalibrated，不补单位内参 |
| 畸变 | plumb_bob 精确 5 项、rational_polynomial 精确 8 项、equidistant 精确 4 项；全部有限 |
| K | 正焦距，标准 pinhole 结构，结构零/一绝对容差 1e-9 |
| R | R*R^T≈I 且 det≈+1，绝对容差 1e-5；反射、全零 R 不接受 |
| P | 正焦距、标准 3x4 投影结构；保留 Tx/Ty，允许与 K 不同 |
| ROI | 默认几何零值表示完整标定网格，否则非空且完全在标定分辨率内 |
| binning | 0 按 1 解释；ROI 宽高分别整除 bx/by 才接受，不猜驱动舍入 |

未支持的畸变模型返回 unsupported_distortion_model，表示本实现不能使用，不等于证明相机物理标定错误。当前采用完整 CameraInfo 契约：即使调用方只看原始图，R/P 仍须有效；只填 K、R/P 留零的非完整消息会明确失败，不自动补齐。

CameraInfoMatchResult 返回 matched()、稳定 code/message、标定细分 calibration_code、计算后的 expected_width/height、可选 timing、used_static_calibration。按检查顺序返回第一个失败；timing 只有实际运行时间检查时才有值。可通过 cameraInfoErrorCodeName/cameraInfoMatchCodeName/sensorTimeCodeName 持久化名称，不保存枚举序号。

## 5. 与图像的绑定和网格

CameraInfoBinding 必须明确 camera_info_topic、image_topic、optical_frame、basis 和 image_domain(raw/rectified)。binding 是设备配置的声明，检查器核对实际话题与图像元数据。一个 CameraInfo 可为多个流使用，但每个流都必须单独建立明确绑定。

frame 名不必带 optical 后缀，也不能仅凭后缀证明光学轴。相机 frame 与图像 frame 必须都等于配置中的 optical_frame；相机 link/body frame 不能不经变换冒充光学 frame。实际轴与外参必须结合设备配置、URDF 和 TF 核查；TF 链连通和四元数有效本身不能证明物理安装正确。

例如标定网格 640x480，ROI=(100,60,320,240)，binning=(2,3)，期望图像是 160x80。输出保留原始 K/P，不输出可直接用于该裁剪图像的“有效内参”；后续几何模块必须结合 ROI/binning 调整主点、焦距，原始图还需按畸变模型处理。不能把完整分辨率 K 直接用于 ROI 图像反投影。

rectified 且 roi.do_rectify=true 时必须计算校正后的 ROI，当前返回 unsupported_rectified_roi；raw 图像可使用原始 ROI。这里不做去畸变、resize、裁剪、相机外参或深度重投影。

## 6. 时间策略与整体使用顺序

1. builder 读取话题类型后反序列化，调用 RGB/Depth/CameraInfo 解码函数；不以字节布局相似代替 ROS 类型检查。
2. builder 以 RGB 为观测索引，调用 checkSensorTimes(rgb.stamp, depth.stamp, options) 检查已选候选帧；选择最近帧、平局规则、复用和有效区间不在本函数中实现。
3. 对每张图分别选择对应 CameraInfo，并调用 checkCameraInfoImage；RGB 内参与未注册 Depth 内参可以不同，不能盲目共享。
4. 已注册深度需要继续调用 checkDepthRgbAlignment；CameraInfo 匹配成功不能代替物理配准声明。
5. TF 层分别使用 RGB/Depth 各自采样时刻查历史变换；缺失、超差或未支持情况由 builder 记入 pending，不写入假坐标。

synchronized 为默认标定策略；需要明确同一时钟和非负容差，0 表示严格同时。timing.delta_ns=CameraInfo stamp-image stamp，超过容差也保留差值供统计。

static_calibration 是显式配置承诺：basis 必须说明该数据段内标定/操作参数固定的依据，才可不检查 CameraInfo 的采样时差。此时 timing 为空、used_static_calibration=true，原 stamp 原样保存。它不授权跨标定版本复用，也不自动推断静态；动态标定的有效区间选择留给 builder。

详细的时间取舍、在线控制和未来 JSONL 约定见 [sensor_time 说明](../../common/sensor_time.md)。

## 7. 完整使用示例

```cpp
#include <vln_memory/bag/camera_info/camera_info.hpp>
#include <iostream>

int main()
{
  using namespace vln_memory;
  CameraInfoData source;
  source.topic = "/rgb/camera_info";
  source.frame_id = "rgb_optical";
  source.stamp = {10, 0};
  source.width = 640;
  source.height = 480;
  source.distortion_model = "plumb_bob";
  source.d = {0, 0, 0, 0, 0};
  source.k = {500, 0, 320, 0, 500, 240, 0, 0, 1};
  source.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  source.p = {500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0};
  const auto decoded = decodeCameraInfo(source);
  if (!decoded.ok()) {std::cerr << decoded.message; return 1;}

  CameraImageReference rgb{"/rgb/image_raw", {640, 480, "rgb_optical"}, {10, 0}};
  CameraInfoMatchOptions options;
  options.binding = {"/rgb/camera_info", "/rgb/image_raw", "rgb_optical",
    "device_profile_v1", CameraImageDomain::raw};
  options.time = {SensorClockRelation::same_clock, 0};
  const auto checked = checkCameraInfoImage(*decoded.info, rgb, options);
  if (!checked.matched()) {std::cerr << checked.message; return 1;}
  std::cout << checked.expected_width << 'x' << checked.expected_height << '\n';
  return 0;
}
```

下游 find_package(vln_memory CONFIG REQUIRED)，链接 vln_memory::vln_memory_camera_info，公共时间依赖自动传递。ROS 接入链接 vln_memory::vln_memory_camera_info_ros，见 [ROS 适配](camera_info_ros.md)。

## 8. 验证与人工复核

2026-10-28：严格编译参数 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror 下，无 ROS 核心 4/4、真实 ROS 2 Humble 7/7 测试通过，零跳过。独立安装消费者验证 RGB/Depth/CameraInfo 和 ROS 适配的头文件、目标、依赖及运行结果。CameraInfo 与公共时间的完整文档示例均已严格编译并运行；107 个本地链接/章节引用及全部 C++ 同名分类文档检查通过。CameraInfo 和时间边界程序的 ASan/UBSan 与泄漏检查通过。

复核公开字段、错误路径、ROS 映射、安装导出及 RGB/Depth 合作调用：没有修改 RGB/Depth 的解码行为，没有把时间、相机模型和深度注册混成一个成功标志。测试均为构造数据；没有实际 bag，因此设备时差分布、时钟关系与物理标定精度尚不能验证。

测试说明：[核心](tests/test_camera_info.md)、[ROS 映射](tests/test_camera_info_ros.md)、[安装消费者](tests/camera_info_consumer.md)。

## 9. 对照的官方实现与约定

- [ROS 2 Humble CameraInfo 定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)：采样时间、光学 frame、K/P、ROI/binning 的语义。
- [ROS image_geometry PinholeCameraModel](https://github.com/ros-perception/vision_opencv/blob/humble/image_geometry/src/pinhole_camera_model.cpp)：保留完整标定，按 ROI/binning 派生网格/内参；后续投影优先复用成熟几何库。本模块没有复制其去畸变算法。
- [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)：光学轴与单位约定。
