# 观测准备层源码核查与能力汇总

日期：2026-10-29。审查对象为当前工作树，分支 codex/tf-module，包含此前已暂存的 TF 开发结果；不能只按 HEAD 判断。第 1–9 节保留首次核查记录（其中 F5 标记后续修复状态），当前修复结果见第 10 节。具体字段和参数见 [接口手册](../interfaces/observation_preparation_api.md)。

> 当前进度入口（2026-11-02）：本文保留 2026-10-29 的核查及后续修复历史。Reader、配对、指定压缩图像解码、CDR/PNG/NPY 和 preparation JSONL 此后已实现；正式 Observation、数值位姿、有效模型与关键帧仍待实现。当前状态见 [系统规划](../project/system_plan.md) 和 [最近配对验收](rgbd_nearest_pairing_verification.md)，下文旧日期的“未实现”不得直接作为当前结论。

## 1. 结论与审查方法

**当前完成的是四类传感器消息的处理基础及其元数据检查，尚未完成“从 rosbag 提取同步 RGB-D、标定、历史位姿和关键帧”的完整观测构建流程。**

本次实际读取：10 个公开头、9 个实现、CMakeLists.txt、package.xml、包配置模板、真实消息测试与安装消费者，并核对核心测试入口与关键断言；从全新目录构建和运行现有测试，补做四模块组合与边界复现。发现内容按功能缺口、可复现问题和设计限制区分，未把旧说明或通过的测试当作端到端完成证据。

本轮变更以文档为主：新增集中接口手册与本报告，修正整体规划中过时的代码树，增加导航。没有补写缺失的 builder，也没有修改运行时行为；问题“发现”不等于“已修复”。

## 2. 按用户目标逐项验收

| 目标 | 实际状态 | 源码证据与影响 |
| --- | --- | --- |
| C++ 核心数据处理 | 已实现 | 5 个核心目标：RGB、Depth、时间检查、CameraInfo、TF；不是后续 memory 核心层 |
| ROS 图像消息 | 已实现，限定编码 | Image 薄适配调用核心；不订阅、不打开 bag、不解压 CompressedImage |
| 从 rosbag 读取/提取 | 未实现 | 无 bag/builder 目录、Reader/反序列化入口；构建配置无 rosbag2 依赖 |
| RGB-D 同步 | 只实现候选时差检查 | checkSensorTimes 不寻找候选，不定义最近帧/平局/复用/队列规则 |
| RGB-D 物理配准 | 只检查上游声明 | checkDepthRgbAlignment 不做重投影；未注册深度不能直接按 RGB 像素读取 |
| 标定提取/校验 | 消息字段与校验已实现 | 保留 K/D/R/P、ROI/binning，关联实际图像；从 bag 提取、标定版本索引/选择未实现 |
| 可直接投影的有效相机模型 | 未完整实现 | 不输出 ROI/binning 后的有效 K/P，不执行去畸变，部分校正 ROI 明确拒绝 |
| /tf、/tf_static | 消息接入、索引、链/覆盖已实现 | 每条 header 时间、来源与静动态语义保留 |
| 历史位姿 | 只完成样本选择依据 | checkTime 返回 before/after/inverted，没有插值、求逆和组合后的 pose |
| 关键帧 | 未实现 | 无选择策略/实现/类型/索引写入接口；不把抽取图像等同关键帧 |
| Observation、媒体与 JSONL | 未实现 | 无 Observation 类型、PNG/Depth 资产保存、JSONL writer、manifest/pending 输出或命令入口 |
| 完整真实 bag 验收 | 未完成 | 当前测试使用构造消息，没有真实 bag 数据集回查与完整性验证 |

没有 reader 不能说“已经从 rosbag 提取”；有消息适配不等于能反序列化 bag。有历史 TF 原始变换不等于已经获得图像时刻 map 中的相机/底盘位姿。

## 3. 主要发现（按影响排序）

### F1：完整流程缺少 reader/组装入口（P1，功能未交付）

[CMakeLists.txt](../../CMakeLists.txt) 只构建库和测试；[package.xml](../../package.xml) 没有 rosbag2 读取依赖；include/src 没有 builder、Observation、keyframe 或 writer 实现。不能提供 bag 路径并得到最终输出。

需要补：bag 元数据/存储插件与类型检查、反序列化、统一来源封装、流程组装与实际可运行入口。core 和 *_ros 的分离适合复用，但它们不承担这些职责。

### F2：时间检查尚不等于 RGB-D 配对/标定选择（P1，功能未交付）

[checkSensorTimes 实现](../../src/bag/common/sensor_time.cpp) 只比较已给定的两个时间；[checkCameraInfoImage](../../src/bag/camera_info/camera_info.cpp) 只核对已给定标定与图像。需要 reader 提供多个候选后的确定性配对、容差、平局、深度复用、乱序和 clock epoch 处理，以及动态标定有效区间/版本选择。

不同模块保存 stamp 的格式相同，不证明时钟已对齐。SensorStamp 不含 clock_id，CameraInfo 的 same_clock 和 TF 的 scope 都由调用方声明，builder 尚未统一维护这些证明。

### F3：历史 map 位姿没有计算（P1，功能未交付）

[tf.hpp](../../include/vln_memory/bag/tf/tf.hpp) 的 TfSelectedEdge/TfTimeCheck 和 [tf.cpp](../../src/bag/tf/tf.cpp) 的 checkTime只复制静态/精确样本或插值端点。没有求解后的 position/quaternion，没有旋转插值、逆变换和链组合。

后续应按明确方向计算 T_target_source，并验证已知旋转/平移、反向链、跨零时刻和两个图像采样时刻。小时间间隔不能代替重定位跳变检查。

### F4：关键帧、证据、Observation 与导出未实现（P1，功能未交付）

没有关键帧策略、稳定 observation_id/来源关联、媒体保存与引用、JSONL/manifest/质量报告。即便四模块手动组合成功，也不会产生这些文件。后续还应定义可恢复性、半成品处理和文件引用完整性验收。

### F5：直接 TF 核心与 ROS 入口的话题校验不一致（P2，后续已修复）

首次核查时，[tf.cpp](../../src/bag/tf/tf.cpp) 的 validate 只检查 source.topic 长度>=2、首字符 / 及非空 message_id。实际复现：TfIndex::insert 接受 source.topic="/bad topic"。

[tf_ros.cpp](../../src/bag/tf/tf_ros.cpp) 的 binding 校验更严格，拒绝含空格的配置话题。常规 ROS 接入路径有这道保护，但公开核心可独立调用，来源格式约束并不一致。影响为来源记录可能不合法；不是已经复现的坐标运算错误。首次核查建议统一/复用话题语法校验并加入核心回归；后续已按该建议完成修复，见第 10 节。

最小独立复现（修复前打印 inserted；当前应打印 missing_provenance，且索引仍为空）：

```cpp
#include <vln_memory/bag/tf/tf.hpp>
#include <iostream>

int main()
{
  using namespace vln_memory;
  TfIndex index({"clock_1", "segment_1"});
  TfTransform sample;
  sample.parent_frame = "map";
  sample.child_frame = "base";
  sample.source_topic = "/bad topic";
  sample.source_message_id = "bag:message:1";
  std::cout << tfInsertCodeName(index.insert(sample).code) << '\n';
  return 0;
}
```

### F6：没有统一观测质量门控，解码成功不能直接发布（P2，集成前置条件）

[RGB 输出](../../src/bag/rgb/rgb.cpp) 保留空 frame；[Depth 转换](../../src/bag/depth/depth.cpp) 允许全无效图仍 ok()；注册检查只核对声明与网格；TF covered 只表示样本足够。核查程序实际复现前两项。

这符合当前各模块分工，但缺少组装层去联合判断是否可定位、有效深度比例、标定、同步、TF 和地图上下文。未来只写 if(rgb.ok() && depth.ok()) 就输出有效 Observation 会错误。应分别保留原始证据，给出 pending/有效/退化的明确状态。

## 4. 每个现有小模块的源码检查结果

| 模块 | 已核实的实现 | 限制/接入要求 | 判断 |
| --- | --- | --- | --- |
| RGB | 编码选择、乘法溢出/资源检查先于读像素、逐行去填充、BGR/RGB 交换、mono 展开、拥有型输出 | 仅 rgb8/bgr8/mono8；空 frame 原样保留；无压缩解码、保存、同步 | 可作为限定格式的单帧处理器 |
| Depth | uint16/float32、明确端序、memcpy 位模式、显式单位与 optical_z、范围和互斥无效统计 | 无物理配准；全无效可成功；NaN 不能直接变 JSON 数值；不支持径向 range/Disparity/压缩 | 可作为标准深度转换器，组装须检查质量 |
| 时间 | 整数秒/纳秒校验、带符号差值、闭区间容差、拒绝未知/不同域 | 无匹配算法或时钟校准，默认严格相等 | 可检查候选，不是同步器 |
| CameraInfo | K/D/R/P 数值与结构、完整 R/P、显式话题/光学 frame/图像 domain、ROI/binning 实际尺寸 | 只支持三种模型固定系数数；完整 R/P 要求可能不兼容部分驱动；校正 ROI 拒绝；未生成有效内参 | 作为完整标定消息校验可用，真实设备兼容性待验 |
| TF | child/时间索引、乱序插入、静态去重、父子/值/类型/环冲突、区间交集/空洞、精确/端点选择 | 无位姿求解；保守阻断祖先可能使局部可算链也失败；内存索引仅计数上限；scope 不含真实段起止 | 链/覆盖检查可用，不能替代位姿求解器 |
| ROS 适配 | 核对 Image/CameraInfo/TFMessage 字段映射与输出所有权；TF 逐条来源和错误 | 强类型签名不代替序列化层类型检查；无 bag 读取；TFMessage 可部分成功 | 可供未来 reader 调用 |

CameraInfo 的 static_calibration 仅凭显式策略与 basis 接受固定段声明，不自动验证整个 bag 内是否变标定；TF 不验证物理安装方向。不要将结构校验和物理真实性混淆。

## 5. 接口是否便于其他模块使用

当前优点：

- 公开头与实现分开，目录按 bag/模块分类；核心没有反向依赖 ROS/vln_agent。
- ImageGeometry 与 SensorStamp 允许跨模块传元数据，Depth/TF 不读取 RGB 内部状态。
- 借用范围与拥有型输出明确，不依赖源消息后续生命周期。
- 错误类型化且有稳定名称；默认结果无数据不会误判解码成功。
- C++17/include/ROS 消息依赖随公开目标传递，安装消费者实际能编译运行。

需要人工调用者特别留意：

1. 成功谓词有 ok/matched/alignedToRgb/accepted/chainExists/covered，含义不同，不能统一偷换成“观测有效”。手册逐项列明，不在审查阶段做破坏性改名。
2. RGB/Depth 的话题、bag 接收时间、来源 ID、clock/map 上下文没有统一封装；CameraInfo/TF 又有不同来源字段。builder 需要持有统一外层 provenance/context，不能重新依赖全局变量串联。
3. 数值标准化不等于文件格式标准化：DepthFrame::depth_m 是 float 数组，不是 ROS Image 字节序列；RGB 是 RGB 顺序，交给 OpenCV 保存还需按其约定处理。
4. 公开结构允许调用方改字段。修改后应重新校验；不要用一个曾经成功的结果为后来改写的数据背书。
5. TF 索引应先建立后只读查询；在插入期间并发查询不受支持。报告是当时的快照，后续冲突可使旧结论失效。

结论：**现有接口足以手动组合消息检查，有安装后可引用的证据；尚缺统一 Observation/来源/质量和流水线接口，不能评价为整层已打通。**

## 6. 人工可读性与代码规范

已核实：C++17、明确命名空间、const 输入/拥有型输出、显式整数和单位、结构化错误、较清楚的校验阶段注释；未引入单实现工厂/插件框架。实际严格编译通过，不能据此推断所有运行时语义均正确。

可改进之处（P3，未影响本轮已验证调用）：没有 .clang-format/.clang-tidy/.editorconfig 统一配置，也没有格式检查接入 CTest；源码仍有少量>120字符长行（CameraInfo 223 行，TF 287/288 行），部分消费者把多项赋值放一行。CameraInfo/TF 的局部话题校验函数存在重复，错误检查风格也有少量差异。后续应在单独的可审阅修改中统一，不能把“编译过”写成“全面符合格式规范”。

本次还发现系统整体规划第 8 节代码树仍只列 RGB/Depth，容易低估 CameraInfo/TF 的现有内容；已按实际树修正，并增加本报告和接口手册入口。

## 7. 本轮实际执行的验证

- 全新无 ROS 核心构建：-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror；CTest 5/5。
- 全新真实 ROS 2 Humble colcon 构建：同等严格参数；9 tests，0 failures/errors/skipped。
- 全新安装后的消费者：核心 4 个、ROS 模式 7 个，均实际运行通过；不借用源码 include 路径。
- 四模块真实接口串联：RGB/Depth 解码、候选时差、注册声明、CameraInfo 关联、分别按 RGB/Depth stamp 检查 TF，运行通过。未生成 pose/Observation/文件，不能冒充端到端测试。
- 边界复现：全无效深度仍 ok、空 RGB frame 保留、K-only/R/P为零被拒绝、TF 核心接受 /bad topic。
- 构建与探针位于忽略的 build/observation-audit-* 下；文档示例及链接检查结果见本报告末尾。

测试是构造数据。未知：真实 bag 的存储类型/消息类型、驱动编码、硬件单位、时钟同步/重启、TF 实际覆盖、标定物理精度、关键帧效果、导出完整性。这些不能靠已有单元测试结论替代。

## 8. 下一步按依赖收口

1. 修复已复现的话题校验差异，并明确真实设备编码/标定支持范围。
2. 完成 TF 数值求解与有效相机几何，使用有已知答案的旋转/平移和 ROI 测试。
3. 实现 bag reader、统一来源/context、候选配对与标定版本选择。
4. 组装 Observation，实施联合质量门控和 pending；再实现关键帧及媒体/JSONL/manifest 输出。
5. 用短真实 bag 做从输入到落盘文件、来源、时间、位姿、关键帧的闭环验收。

这是当前源码对应的剩余工作，不是已经提供的接口。当前阶段更准确的名称是“观测准备层的消息基础与检查库”。

## 9. 文档与变更复核

集中接口手册的完整 C++/CMake 示例已按安装后目标严格编译运行，输出 10; delta_ns=5000000。F5 最小复现从本文提取编译运行，输出 inserted，问题仍未修复。全库 178 个本地链接/章节引用检查通过。核查前后 36 个运行源码/公开头/测试/构建文件 SHA-256 完全一致；此次没有修改运行时或测试逻辑。


## 10. 后续修复与注释整理

F5 已修复：common/sensor_names.hpp 定义统一校验，TF 核心、ROS binding、CameraInfo 复用。非法来源返回既有结构化错误，不改变索引；Humble 247 字节限制统一应用，合法 remap/隐藏话题保留支持。新增公共回归、核心失败不变性检查、真实 RMW 对照。接口与约束见 [sensor_names](../common/sensor_names.md)。

RGB/Depth、CameraInfo、TF、公共类型/时间及 ROS 适配的 19 个生产文件已补职责与关键边界注释；保留用户新增的信息头/时间戳解释，展开已有长分支和部分消费者的多语句赋值。新增 .editorconfig 统一 UTF-8、LF、两空格和行尾设置；它是编辑器约定，不是完整 C++ 格式化/lint 检查器。

F1–F4 是后续功能交付项，F6 是组装门控前置要求，仍保持原状态；不通过改变解码成功语义伪装它们已经完成。首次核查的源码指纹与旧测试数量属于历史证据；本次修复后的验证另行记录在 [实施记录](../project/sensor_validation_comments_plan.md)。
