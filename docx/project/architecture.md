# 当前源码组织与观测准备层 bag

当前最终交付范围是原设计第 1、2 部分，逐条进度见 [范围对照](sections_1_2_scope.md)，实施顺序见 [收敛后的系统规划](system_plan.md)。**当前只在开发观测准备层 bag，RGB、Depth、CameraInfo 已实现，第四个 TF 模块已实现链/覆盖索引。**

## 源码目录约定

```text
include/vln_memory/bag/
├── common/{sensor_names,sensor_types,sensor_time}.hpp
├── rgb/{rgb,rgb_ros}.hpp
├── depth/{depth,depth_ros}.hpp
├── camera_info/{camera_info,camera_info_ros}.hpp
└── tf/{tf,tf_ros}.hpp
src/bag/
├── common/sensor_time.cpp
├── rgb/{rgb,rgb_ros}.cpp
├── depth/{depth,depth_ros}.cpp
├── camera_info/{camera_info,camera_info_ros}.cpp
└── tf/{tf,tf_ros}.cpp
test/
├── bag/common/test_sensor_time.cpp
├── bag/rgb/{test_rgb,test_rgb_ros}.cpp
├── bag/depth/{test_depth,test_depth_ros}.cpp
├── bag/camera_info/{test_camera_info,test_camera_info_ros}.cpp
├── bag/tf/{test_tf,test_tf_ros}.cpp
└── install_consumer/
```

- include、src、模块测试按“层/模块”对应，头文件安装后保留同样路径。
- bag/common 放观测准备层共用类型和无 ROS 时间检查；不放相机业务、ROS 订阅或未来实体类型。
- 后续本层添加 bag/builder；最后一个是 bag 读取和组装入口，避免 bag/bag 的重复含义。
- 原文第 1、2 部分要求的记忆、构建和 Viewer 在真实实现时才添加同级目录；独立在线查询 plugins 暂不开发，不提前生成空类。
- 函数签名、vln_memory 命名空间和已导出库目标保持，公开 include 路径使用新分类。

## 本层五个模块的职责与接口方向

| 模块 | 输入 | 输出 | 状态 |
| --- | --- | --- | --- |
| RGB | 原始图像字段或 ROS Image | 独立持有像素的标准 rgb8 帧，或结构化错误 | 已实现 |
| Depth | 深度图像、明确单位与注册声明 | 米制深度、质量及 RGB 网格检查 | 已实现 |
| CameraInfo | CameraInfo、真实话题及显式图像绑定 | 完整标定、有效性/网格/frame/时间检查 | 已实现 |
| TF | TFMessage、真实 topic、clock/segment 及查询时刻 | 坐标链、共同时间覆盖、带来源的历史样本选择 | 链/覆盖已实现；位姿求解后续 |
| bag 读取与组装（builder） | bag 文件及提取配置 | 组合 Observation、媒体与 JSONL 报告 | 后续实现 |

五模块属于同一个 vln_memory 包。bag 负责反序列化、跨消息同步、来源 ID、PNG/NPY 保存和 JSONL 组装，调用前四个数据处理模块；前四模块不反向依赖 bag，也不相互读取对方的内部状态。

跨模块时间与图像网格使用 [共享数据类型](../common/sensor_types.md)，保留原始整数采样时间；来源时钟、map 基准和观测组合由 bag 层负责。

## 当前 RGB 调用链

ROS Image -> decodeRosRgb -> RgbImageView -> decodeRgb -> RgbFrame / RgbErrorCode

- 输入视图只在调用期间借用消息数据，输出独立持有像素和字符串。
- 核心库使用 C++17，不依赖 ROS 或 vln_agent；ROS 适配层仅映射字段。
- 接口、实现规划和验证记录放在 [RGB 模块说明](../modules/rgb/rgb.md)。
- 构建目标与导出方式见 [CMakeLists 说明](CMakeLists.md)。

## 当前 Depth 调用链

ROS Image -> decodeRosDepth -> DepthImageView + DepthOptions -> decodeDepth -> DepthFrame

- 16UC1/32FC1 编码只说明数值类型，单位与 optical_z 必须显式指定。
- 输入单位、过滤范围、无效值原因和注册声明保留在输出中。
- ImageGeometry 让 Depth 检查实际配对 RGB 网格，无需链接 RGB 实现。
- 不以相同尺寸推断注册，不承担标定、时间同步或未对齐重投影。
- 实施计划、接口、示例及验证记录见 [Depth 模块](../modules/depth/depth.md)。

## 当前 CameraInfo 与时间检查调用链

ROS CameraInfo + 实际 topic -> decodeRosCameraInfo -> decodeCameraInfo -> CameraInfoData

CameraInfoData + 图像元数据 + 显式绑定 -> checkCameraInfoImage -> CameraInfoMatchResult

- 话题绑定和光学 frame 由设备配置明确；不根据后缀或尺寸推断关联。
- 标定 K/D/R/P、ROI/binning 保留原值，实际图像尺寸按操作参数检查；校正 ROI 的不支持情况明确报告。
- 公共 checkSensorTimes 可独立检查 RGB-D，CameraInfo 在逐帧策略中调用它；静态标定显式复用但不伪造时间相等。
- Observation 以 RGB 时间索引，Depth 几何使用 Depth 时刻 TF；TF 样本选择已实现，几何求解、图像配对和 JSONL 仍待实现。
- 详细规划、接口及示例见 [CameraInfo](../modules/camera_info/camera_info.md)，跨模块时间和控制约定见 [sensor_time](../common/sensor_time.md)。

## 当前 TF 调用链

TFMessage + 实际 topic/message_id -> ingestRosTf -> TfIndex::insert

- inspectChain(target,source,policy) 检查指定链与共同时间区间，分开报告拓扑与覆盖。
- checkTime(target,source,image.stamp,options) 按采样时刻定位原始样本；RGB/Depth 分别调用，不采用 bag 接收时刻。
- /tf_static 不受动态发布时段限制，但属于明确的数据段；/tf 有首末范围和内部缺口。
- 矛盾 parent、静动态类型、静态变化、同刻值及闭环会阻断相关 frame，不自动选最后一条。
- 核心使用有上限的内存历史索引，不依赖在线 Buffer 的短缓存。返回的是样本依据，位姿组合尚未实现。
- 逐文件职责和使用见 [TF 核心](../modules/tf/tf.md)、[ROS 适配](../modules/tf/tf_ros.md)。

## 开发与人工审阅约定

1. 先规划一个小模块的输入、输出、职责、错误和依赖，再自审是否满足整体框架。
2. 按已确定接口实现；注释集中说明字段组、检查顺序、非显然分支、所有权和错误边界。
3. 不逐行重复代码含义，不在说明中声称尚未实现的功能已可用。
4. 完成后通读公开头、实现、构建设置及示例，检查与其他模块的接口是否一致。
5. 执行与改动范围匹配的验证；真实依赖缺失时明确记为未执行。
6. 同名 Markdown 跟随模块分类，头文件和同名源文件合并说明；测试说明归入模块的 tests/。
7. 使用本地 Git 保存基线；新增模块及其配套修改放在 codex/模块分支，完成后 git add，供用户审阅后合并。构建、安装与测试输出保持在忽略的 build/ 内。

## 文档组织规则

- project/：整体架构和构建设置，不累积各模块的详细测试历史。
- common/：跨模块共用类型及约定。
- modules/<模块>/：模块接口、实施规划和使用说明。
- modules/<模块>/tests/：该模块测试源码的同名说明。
- docx/README.md：唯一总导航，后续增加模块时同步更新。

完整分类导航见 [文档首页](../README.md)。

## 本次目录整理复核

2026-10-28：迁移 13 个头文件、实现和模块测试，比较了 16 个 C++ 文件的词法内容，除公开头文件路径外没有行为变更。同步更新 CMake、安装路径、消费者和同名说明。

从全新构建/安装目录验证：无 ROS 核心 2/2、真实 Humble 4/4 测试通过；核心与 ROS 安装后的独立消费者均通过，新安装中没有旧平铺头文件。RGB/Depth 的完整文档示例编译运行通过，76 个本地链接及章节引用有效。整体系统规划与当前 bag 层的实现状态已分别列明。

以上目录整理与 Depth 已本地提交至 codex/depth-module。CameraInfo 后续在 codex/camera-info-module 开发，保持独立审阅与暂存，不自动合并 main。


## CameraInfo 完成复核

2026-10-28：新增 CameraInfo 核心/真实 ROS 适配及 common/sensor_time；通读公开数据、标定验证、匹配结果、ROS 映射和 CMake 导出。匹配结果保留具体 calibration_code，时间结果保留有符号 delta_ns，静态标定不伪造时间相等。

严格编译下核心 4/4、真实 Humble 7/7 测试通过；核心与 ROS 的全部安装消费者通过；新增两个完整文档示例编译运行通过；CameraInfo/时间边界程序的 ASan/UBSan/泄漏检查通过，107 个本地文档链接/章节及同名说明检查通过。此阶段的代码与分类文档随后本地提交为 d9b030b，仍未合并 main。


## TF 完成复核

2026-10-29：实现 /tf、/tf_static 的逐条接入，按 child/整数时间索引，返回拓扑、每边时间包络、共同覆盖区间、失败 frame 与有来源的历史样本。整体通读公开头、索引、ROS 映射、安装导出及图像元数据交接；无跨模块隐藏状态。

严格编译下核心 5/5、真实 Humble 9/9、全部安装消费者、两个完整文档示例、TF 的 ASan/UBSan/泄漏检查通过；133 个本地链接/章节和所有 C++ 同名分类文档检查通过。测试数据为构造消息，尚未用实际 bag 核查设备的真实链与时间覆盖。

CameraInfo 基线在原分支本地提交 d9b030b；TF 与配套修改位于 codex/tf-module，完成后 git add，未自动合并 main。位姿求解、跳变检测、bag 自动分段和 JSONL 仍待后续实现。


## 共享名称修复与注释

当前 F5 来源话题校验问题已通过 common/sensor_names 统一修复，生产头/实现均已补清楚职责与边界注释；后续功能状态保持。见 [实施记录](sensor_validation_comments_plan.md)。
