# tf.hpp / tf.cpp：离线 TF 坐标链与时间覆盖索引

## 1. 编码前整体规划

这是观测准备层 bag 的第四个模块。本轮目标是处理 /tf、/tf_static，检查指定坐标链及其时间覆盖，向后续 builder 提供有来源依据的历史 TF 数据。

| 部分 | 文件 | 职责 |
| --- | --- | --- |
| 公开契约 | include/vln_memory/bag/tf/tf.hpp | 原始变换、来源、索引、链报告、覆盖区间及查询结果 |
| 核心索引 | src/bag/tf/tf.cpp | 数值校验、按 child 和时间索引、拓扑检查、覆盖交集、查询时刻检查 |
| ROS 契约 | include/vln_memory/bag/tf/tf_ros.hpp | 真实 TFMessage 与来源话题的接入函数 |
| ROS 适配 | src/bag/tf/tf_ros.cpp | 区分动态/静态、逐条映射、返回插入/重复/错误报告 |
| 验证 | test/bag/tf/ 与 test/install_consumer/ | 核心边界、真实消息、安装接口与图像采样时间衔接 |

## 2. 分步实现与接口自审

1. 定义 source/target 方向：消息保存 T_parent_child，把 child 坐标变到 parent；链按 source 到 target 排列，每条边标注是否需要求逆。
2. 一个索引只属于显式 clock_id 与 segment_id；图像时刻查询必须提供相同上下文。跨 bag/时钟重启/地图版本不能混成一张树。
3. 插入前校验 frame、stamp、有限平移、近单位四元数和来源；单位约定为米，旋转顺序 xyzw。输出保留原值，不生成默认单位变换。
4. 动态样本按整数时间排序，支持 bag 接收顺序与 header 时间顺序不同；静态边只保留一致的固定几何。重复静态广播不扩展成假动态时间段。
5. 检查唯一 parent、静态/动态类型冲突、同一时刻不同值、静态值变化与环。对矛盾节点做保守阻断，不静默选最后一条；未涉及冲突的其他树仍可检查。
6. inspectChain 返回是否连通、路径、每边首末时间与整条链的共同可用区间。动态时间范围内部的大间隔必须显式切断，不能只用最早/最晚时间包络宣称全覆盖。
7. checkTime 在指定图像采样时刻检查各边：接受精确样本或间隔符合阈值的前后样本，不外推、不以最近帧/latest 填补。返回原始样本和来源，供 lookupPose 求解。
8. 静态链在该索引数据段内无动态时间限制；混合链取动态边区间交集。真实零采样时刻按零处理，没有 tf2 latest 哨兵歧义。
9. ROS 适配不读取 bag、不订阅在线 TF、不替换时间；真实 topic 和消息 ID 来自 builder，支持显式 remap。单条变换插入原子化，TFMessage 可部分成功且逐条报告，调用者必须检查报告。
10. 明确资源上限和只读查询的线程边界；新增目标、安装路径和同名分类文档。先测核心，再测真实 Humble/安装接口，最后通读框架及文档示例。

## 3. 与系统规划的边界

- 后续 builder 第一遍收集该数据段的静态/动态消息，再按 RGB/Depth 各自 stamp 调用查询；不能用 /tf 消息的 bag 接收时间代替 TransformStamped.header.stamp。
- map<-body、body<-camera_optical、map<-camera_optical 均是可配置的查询，不能硬编码必须有 odom 或某种相机名称。
- 链/覆盖及历史样本选择与数值位姿接口分开。lookupPose 复用 checkTime，数值用例与 tf2 对照。采样间隔符合阈值只表示可以按策略插值，不证明无重定位跳变或无运动误差。
- 动态换 parent、改变静态外参、时钟/地图重置应分 segment；跳变检测、磁盘索引、自动分段与 JSONL 保存留给后续 builder。
- 核心不依赖 ROS 或 vln_agent，也不改变 RGB/Depth/CameraInfo 的行为。

## 4. 输入、输出和接口

```cpp
TfIndex index(TfScope{"ros_clock_epoch_1", "map_segment_1"});
TfInsertResult insert(TfTransform transform);
TfChainReport inspectChain(target, source, TfCoverageOptions);
TfTimeCheck checkTime(target, source, SensorStamp, TfTimeOptions);
```

后三个是 TfIndex 的公开方法。TfTransform 持有 kind、parent_frame、child_frame、原始 stamp、translation_m、rotation_xyzw 和顶层来源字段 source_topic/source_message_id/source_transform_index。接收左值复制、接收右值可移动，索引不会借用源消息。

| 接口 | 实际用途 | 调用者必须检查 |
| --- | --- | --- |
| insert | 校验并索引一条变换，允许 header 时间乱序到达 | accepted() 和 code，不能忽略冲突 |
| inspectChain | 判断指定 source→target 的路径，计算各动态边覆盖交集 | chainExists()；随后独立查看 coverage_status/intervals |
| checkTime | 按指定采样时间检验链并选择精确/插值端点 | covered()；失败 frame 与具体 code |
| scope/edgeCount/sampleCount | 查看上下文与唯一数据规模 | ID 是调用者声明，不是从消息推断 |

TfChainReport 保存 clock_id/segment_id、target/source、失败 frame、source→target 的 frames、每条父子边的 kind/inverted/sample_count/dynamic_bounds、coverage_status、intervals。dynamic_bounds 是首末包络，不能代替 intervals。

TfTimeCheck 保存原始查询 stamp、chain_code、失败 frame、路径和 TfSelectedEdge。每条 selected edge 包含 inverted、before、可选 after：

- 静态/精确查询：before 是原始样本，after 为空；静态原 stamp 不会被替换成查询时间。
- 插值可行：before/after 是前后端点及完整来源，本函数没有计算中间位姿。
- 失败：edges 为空，不输出半条成功链；其他诊断字段仍可用于报告。

所有状态提供 tfInsertCodeName/tfChainCodeName/tfCoverageStatusName/tfTimeCodeName，报告应保存名称，避免依赖枚举序号。

## 5. 坐标链方向与有效性

对于消息 parent=map、child=base，保存的是 T_map_base，即 base 原点/姿态在 map 中的表达，应用时把 base 中的点变到 map。查询 inspectChain("map","camera_optical",...) 的方向是 camera_optical→...→map。

沿 child→parent 走时 inverted=false；沿 parent→child 走时 inverted=true。后续几何求解需要按这个顺序对每条边插值/求逆后组合，不能只把平移相加。中间框架没有硬编码 odom/base_link 等名称；任意已知 source/target 都通过最近共同祖先查链。

允许多个互不相连的树，但一个 child 在同一数据段只有一个 parent、只有一种动态/静态类型。frame 不自动剥前导 /，避免悄悄合并不同来源的命名错误。不存在的同名 source/target 仍返回 unknown_frame；只有已知 frame 可返回零边的 identity 链。

平移必须有限、约定米；四元数所有分量有限，norm 与 1 的差不大于 1e-3。这个检查验证数值可用，不认证单位设置、安装外参或光学轴。保留原始四元数，未来几何运算应在计算副本上归一化，不能改写证据。

## 6. 时间覆盖如何计算

1. 静态边不设置动态起止时间，只在当前数据段的上下文内复用。
2. 动态样本以 int64 纳秒有序索引；SensorStamp 的 int32 秒全范围及差值都可安全表示。输出仍用 {sec,nanosec}，没有浮点时间。
3. 对每条动态边，相邻样本的间隔 <= max_interpolation_gap_ns 才合并为连续闭区间；否则切成两个区间。单样本覆盖 [t,t]。
4. 链的 intervals 取所有动态边区间的交集，而不是首末包络的交集。
5. 全静态/identity 链的 coverage_status=static_only，intervals 为空；有动态边却无共同时间时为 no_common_time；有共同区间时为 available。
6. checkTime 用每边有序索引定位，不重新扫描整段区间。精确时刻优先；非精确必须有前后样本且间隔符合策略。不外推、不取最近、不补单位位姿。

max_interpolation_gap_ns 默认 0，只允许精确采样点；负值无效。项目不替设备选通用阈值，示例值仅便于演示，实际应结合 TF 发布周期、机器人运动、采集缺口和质量要求设置。

例：第一条动态边有 [1,3]、[10,12] 两段，另一条覆盖 [2,11]，静态相机边不再限制时间，则整条链是 [2,3] 与 [10,11]。5 秒虽然位于最早/最晚包络内，仍返回 gap_too_large。

scope.clock_id 和 scope.segment_id 必须显式给定；时间查询必须提供匹配的 ID。消息本身不带时钟类型，ID 是采集配置/builder 的声明，不能从数值接近推断。scope 不存数据段起止时间；builder 仍需保证查询图像属于该段和 manifest 范围。时间倒退可能是乱序，也可能是时钟重启，自动分段不由本索引猜测。

静态发布于 99 秒也可以用于同一静态配置段的 2 秒图像；这要求该外参确实在整段固定。动态边只有一个 99 秒样本，则只能证明 99 秒可用，不能当静态使用。零采样时间按真正的零处理，不映射到 tf2 的 latest。

## 7. 冲突、错误和恢复

| 情况 | 结果 |
| --- | --- |
| frame/stamp/数值/来源不合规则 | 返回具体错误，不插入样本 |
| 完全一致的动态同刻重复或静态重复广播 | duplicate，唯一数量不增加，保留首次样本来源 |
| q 与 -q、静态广播 stamp 不同 | 几何一致时仍是 duplicate，原始广播由 builder 留存 |
| child 换 parent、静动态混用 | parent_conflict/kind_conflict，阻断 child |
| 同刻动态值不同、静态外参变化 | duplicate_stamp_conflict/static_conflict，阻断 child |
| 新边会形成环 | cycle_detected，不插入该边，同时阻断闭环 child |
| 样本/边数量超限 | limit_exceeded，不插入，不剪掉早期数据 |

重复判定的平移绝对容差为 1e-9 米；比较归一化四元数分量时容差 1e-9，并识别正负等价。容差只消除重复广播的数值差异，不用来忽略真实运动。

查询任一祖先路径触及被阻断 frame，就返回 topology_conflict，并保留 failed_frame。这个策略偏保守：即使短路径未用到该矛盾上游边，也不会在不确定的树上下文中继续使用。未涉及矛盾的其他树可继续查询。没有 reset/覆盖 last 消息接口；修复来源配置或按合法变化分 segment 后重新建索引。

TFMessage 按条处理，允许部分成功；insert 的普通错误不追加样本，拓扑矛盾还会记录阻断状态。后续 builder 必须保留逐条错误和原始证据，不能忽略报错后宣称 bag 完全有效。系统内存分配异常应终止/丢弃本次索引构建，不能当普通坏帧捕获后继续查询。

## 8. 与 RGB、Depth、CameraInfo、builder 和 Agent 的衔接

```text
bag 读取元数据/反序列化
  -> ingestRosTf(/tf 或 /tf_static, message_id)
  -> TfIndex（同一时钟/地图/采集配置段）
  -> inspectChain(map, body / rgb_optical / depth_optical)
  -> checkTime(map, rgb.frame_id, rgb.stamp)
  -> checkTime(map, depth.frame_id, depth.stamp)
  -> 后续几何求解与 Observation 写入
```

RGB/Depth 都已保留实际 frame/stamp，直接传公开字段即可。CameraInfo 负责明确图像的光学 frame 与相机模型；TF 负责 frame 链与历史覆盖。已注册 Depth 使用声明的目标光学 frame，未注册 Depth 使用自己的光学 frame；不能盲目把它改成 RGB frame。

map<-body、body<-camera、map<-camera 可以分别检查。RGB 与 Depth 即使同一 frame，采样时间不同也要分别 checkTime。无需在 vln_agent 中增加订阅或缓存：离线 builder 将来组合四个准备模块，Agent 通过后续记忆查询插件使用结果。当前未修改 vln_agent。

Builder 使用 lookupPose 输出 map 位姿、原始端点及来源。covered() 表示端点可用，数值状态由 lookupPose 单独返回。参见 [TF 数值位姿](tf_pose.md)。

## 9. 完整核心示例

```cpp
#include <vln_memory/bag/tf/tf.hpp>
#include <iostream>

int main()
{
  using namespace vln_memory;
  const TfScope scope{"ros_clock_epoch_1", "map_segment_1"};
  TfIndex index(scope);
  TfTransform fixed;
  fixed.kind = TfKind::static_transform;
  fixed.parent_frame = "base";
  fixed.child_frame = "camera_optical";
  fixed.stamp = {99, 0};
  fixed.source_topic = "/tf_static";
  fixed.source_message_id = "bag:message:0";
  if (!index.insert(fixed).accepted()) {return 1;}

  TfTransform dynamic;
  dynamic.parent_frame = "map";
  dynamic.child_frame = "base";
  dynamic.stamp = {1, 0};
  dynamic.source_topic = "/tf";
  dynamic.source_message_id = "bag:message:1";
  if (!index.insert(dynamic).accepted()) {return 1;}
  dynamic.stamp = {3, 0};
  dynamic.source_message_id = "bag:message:2";
  if (!index.insert(dynamic).accepted()) {return 1;}

  // 2 秒只为演示；实际间隔阈值应结合 TF 发布周期与运动设置。
  const TfCoverageOptions policy{2000000000LL};
  const auto chain = index.inspectChain("map", "camera_optical", policy);
  if (!chain.chainExists() || chain.coverage_status != TfCoverageStatus::available) {return 1;}
  TfTimeOptions options;
  options.clock_id = scope.clock_id;
  options.segment_id = scope.segment_id;
  options.max_interpolation_gap_ns = policy.max_interpolation_gap_ns;
  const auto query = index.checkTime("map", "camera_optical", {2, 0}, options);
  if (!query.covered()) {std::cerr << query.message; return 1;}
  std::cout << chain.intervals[0].start.sec << ".." << chain.intervals[0].end.sec
            << "; edges=" << query.edges.size() << '\n';
  return 0;
}
```

find_package(vln_memory CONFIG REQUIRED)，链接 vln_memory::vln_memory_tf；ROS 入口链接 vln_memory::vln_memory_tf_ros，见 [tf_ros 说明](tf_ros.md)。

## 10. 资源、复杂度和复核边界

默认最多 4096 条边、1000000 个唯一样本，可通过 TfLimits 配置。每条动态样本都有完整来源；这不是固定字节预算，也不限制此前反序列化内存。当前是数据段内存索引，大 bag 应按窗口/段处理，未来 builder 可增加磁盘索引，不默认清掉历史。

插入通常按边/时间树查找；首次新增边需检查祖先路径。inspectChain 扫描路径各边样本生成覆盖区间，适合初始化/检查报告；每张图使用 checkTime，只按路径定位相邻样本。没有全图遍历或对整段样本的逐图扫描。

先完成插入，再允许多个只读查询；insert 不能与查询并发。返回结果拥有所选样本，独立于索引生命周期；scope() 返回只读引用，依赖索引生命周期。

2026-10-29 最终复核：严格编译（含 -Wconversion/-Wsign-conversion/-Werror）下，核心 5/5、真实 Humble 9/9 测试通过，零跳过；核心与 ROS 安装后的全部消费者通过；TF 核心与 ROS 文档完整示例编译运行通过；TF 程序的 ASan/UBSan/泄漏检查通过，133 个本地链接/章节及全部 C++ 同名分类说明检查通过。关闭 BUILD_TESTING 的 ROS 生产配置也已验证。验证使用构造消息，没有真实 bag，不能据此证明目标设备所需坐标链与时间覆盖实际完整。

测试说明：[核心与生成用例](tests/test_tf.md)、[真实 ROS/tf2 对照](tests/test_tf_ros.md)、[安装接口](tests/tf_consumer.md)、[ROS 安装接口](tests/tf_ros_consumer.md)。

## 11. 官方实现对照

- [geometry_msgs TransformStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TransformStamped.msg)：parent/child、header 时间、平移与旋转的消息语义。
- [Humble tf2 BufferCore](https://github.com/ros2/geometry2/blob/humble/tf2/src/buffer_core.cpp)：查询坐标链、插入校验及历史时间查询行为。
- [Humble tf2 TimeCache](https://github.com/ros2/geometry2/blob/humble/tf2/src/cache.cpp)：动态缓存、边界、插值与 time=0 的 latest 行为。
- [Humble tf2 StaticCache](https://github.com/ros2/geometry2/blob/humble/tf2/src/static_cache.cpp)：静态几何不受动态采样时段限制。本索引额外保留原始静态时间，并对互相矛盾的重复静态消息报告错误。


## 2026-10-29 修复与注释复核

来源话题改用完整公共校验；补明静态/同刻去重、闭区间交集、精确端点优先及 covered 与数值位姿的区别。 话题格式统一最长 247 字节。具体修复与验证记录见 [本次实施记录](../../project/sensor_validation_comments_plan.md)。

## 12. 2026-10-30 可读性重构与字段迁移

依据 AGENTS.md，来源、查询配置和报告上下文改为扁平字段，内部边历史移出类内定义。函数签名和错误枚举保持；下表属于公开 C++ 字段/布局调整，下游须迁移并重新编译，不提供重复兼容字段。

| 原写法 | 当前写法 |
| --- | --- |
| transform.source.topic | transform.source_topic |
| transform.source.message_id | transform.source_message_id |
| transform.source.transform_index | transform.source_transform_index |
| options.scope.clock_id / options.scope.segment_id | options.clock_id / options.segment_id |
| options.coverage.max_interpolation_gap_ns | options.max_interpolation_gap_ns |
| report.scope.clock_id / report.scope.segment_id | report.clock_id / report.segment_id |

TfSource 已移除；TfScope 保留用于构造索引及 scope()。SensorStamp、TfTimeInterval、optional 动态包络以及 before/after 原始样本继续表达各自含义，不展开为易失配的重复字段。

人工审阅可按 insert（输入/冲突/资源/写入）、findChain（两端/祖先/共同祖先/方向）、inspectChain（单边区间/交集）、checkTime（配置/链/端点）顺序阅读。辅助函数注明前置条件、所有权、单位和失败边界。实施与验证见 [TF 可读性整理](../../project/tf_readability_plan.md)。
