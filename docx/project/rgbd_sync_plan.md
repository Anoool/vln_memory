# RGB-D 自动时间配对方案（编码前设计记录）

实施更新：2026-10-30 第一版已落地，实际接口/支持边界见 [BagBuilder 手册](../interfaces/bag_builder_api.md) 和 [实施记录](bag_builder_v1_implementation.md)。以下保留原规划，不作为当前全部已实现能力声明。
日期：2026-10-29。本文是本次用户要求的“先给方案”产物，不是已经提供的 C++ 接口。本轮不新增功能源码、不改变现有配对/解码行为。

范围调整：当前最终交付以原设计第 1、2 部分为准，见 [范围对照](sections_1_2_scope.md) 与 [整体规划](system_plan.md)。本方案仅是 S1 的一个小模块；A/B/C 交付表示观测准备的内部步骤，不等于原设计第 1/2/3 部分。

## 1. 问题和目标

现在只有 checkSensorTimes(reference,candidate,options)，能检查给定两帧的时间差；没有从多帧 RGB/Depth 中自动寻找对应帧的功能。

本次目标是在第五个 bag/builder 模块内部增加独立、无 ROS 的 rgbd_sync 小模块：输入同一明确时钟/数据段的两个图像时间索引，输出可追溯的一对一配对、未配对条目及统计。保留现有时间检查函数，不修改原始时间，不生成虚假的同时采样结论。

时间匹配不执行空间注册、图像插值、标定选择、TF 位姿求解、关键帧或文件导出；配对后仍要经过这些相应步骤，才能形成完整 Observation。

## 2. 参考实现与本项目取舍

- [ROS ExactTime](https://github.com/ros2/message_filters/blob/humble/include/message_filters/sync_policies/exact_time.h)：按消息时间戳组织严格匹配的元组。
- [ROS ApproximateTime](https://github.com/ros2/message_filters/blob/humble/include/message_filters/sync_policies/approximate_time.h)：使用消息队列和候选策略做近似匹配，可设置最大时间区间。它不是对每张 RGB 独立取最近 Depth 的同名算法。
- [depth_image_proc RegisterNode](https://github.com/ros-perception/image_pipeline/blob/humble/depth_image_proc/src/register.cpp)：区分几何注册与输入同步；use_rgb_timestamp 配置可让输出深度采用 RGB CameraInfo 时间，说明相等 header 不等于原始曝光同时。

本项目的选择是离线索引配对：同一合法数据段内可先收集完整元数据，再排序、选择，保存失败原因。输入只保存消息引用和时间，不把整包像素加载进同步队列；不为第一版增加在线 Subscriber/Listener。算法行为按下文自行定义，不宣称结果等价于 ApproximateTime。

## 3. 放在整体架构哪里

```text
ROS 2 bag
  -> 后续 reader：检查话题类型、读取 header、建立稳定消息引用
  -> RGB 时间索引 / Depth 时间索引（同一 clock epoch、数据段）
  -> rgbd_sync：选候选 + 调用现有 checkSensorTimes
  -> 已配对引用 / 未配对记录 / 时间质量统计
  -> 按引用解码 RGB/Depth
  -> 分别关联 CameraInfo、检查注册声明、查询 TF
  -> 后续几何求解、Observation 组装与导出
```

这属于第五个 builder 模块，不把已确定的五模块架构变成六个顶层模块。前四模块不反向依赖 builder。解码失败不默认触发“再配别的帧”，应保留原始配对并形成质量问题，避免不同执行路径悄悄改变配对。

建议实施时创建以下文件；现在不创建占位源码：

```text
include/vln_memory/bag/builder/rgbd_sync.hpp
src/bag/builder/rgbd_sync.cpp
test/bag/builder/test_rgbd_sync.cpp
test/install_consumer/rgbd_sync_consumer.cpp
docx/modules/bag/rgbd_sync.md
docx/modules/bag/tests/test_rgbd_sync.md
docx/modules/bag/tests/rgbd_sync_consumer.md
```

头文件定义接口和规则；实现负责验证/排序/选择/统计；核心测试验证算法；安装消费者证明其他模块可直接引用。CMake 计划导出 vln_memory::vln_memory_rgbd_sync，复用 vln_memory_sensor_time，不依赖 ROS。

## 4. 第一步：先固定输入契约

以下是建议类型，签名在编码前定稿。

| 类型 | 建议字段 | 用途 |
| --- | --- | --- |
| RgbdFrameRef | string message_id；uint64_t message_order；SensorStamp stamp | 稳定来源、原始消息顺序、header 采样时间；不装像素 |
| RgbdStreamIndex | string topic,clock_id,segment_id；vector<RgbdFrameRef> frames | 一个已选图像流在一个合法段内的索引 |
| RgbdSyncOptions | SensorTimeOptions time；string clock_basis；输入帧数上限 | 显式时钟关系、纳秒容差、采集配置依据和资源边界 |

message_id 应由 bag 身份与稳定消息序号生成；message_order 使用原始读取序号，不是函数接收 vector 的排列位置。同一输入消息不能重复出现，message_id/顺序约束需校验，不能让同一帧以两个引用被多次使用。

两个流必须有明确且一致的 clock_id/segment_id，time.clock_relation 必须声明 same_clock，clock_basis 指向设备/采集配置依据。相同 ID 本身不证明硬件时钟同步，它们是 reader/配置的声明。话题通过已有 isResolvedTopicName 检查，RGB 与 Depth 的绑定明确且互不混淆。

使用 header.stamp；bag 接收时间只用于追溯。时间回退可能来自乱序，也可能来自时钟重启，不能仅靠全局排序掩盖重启。reader 负责识别/明确数据段；无法分辨的重置先报告，不能跨段配对。相同时间但不同消息身份保留，不能只凭 stamp 相等当作同一条消息去重。

输入上限是元数据条数上限，不等于进程字节预算或先前反序列化上限；超限返回错误，不截断后假装成功。具体默认数量随实现的内存预算说明一起确定。

## 5. 第二步：固定第一版匹配规则

1. RGB 是唯一基准流，输出按 RGB 时间顺序排列。
2. 按 (header_stamp, message_order) 对两个索引排序；输入 vector 顺序改变、稳定来源序号不变时结果必须相同。
3. 最大时间差采用非负整数纳秒；0 为严格相等，正数为近似。必须明确配置，不能因为匹配率低就自动放宽。
4. 一帧 RGB 最多选择一帧 Depth；一帧 Depth 最多使用一次。
5. Depth 选择顺序不能倒退：已选深度之前的未选条目不再分配给更晚 RGB，避免交叉配对。
6. 在满足以上条件的候选中，选择绝对时差最小的 Depth；若等距，优先较早采样的 Depth；同刻再按原始 message_order 选择。
7. 没有容差内的合法候选，RGB 保留为未配对；Depth 未被使用也保留清单，未使用不等于坏帧。
8. 选定后调用现有 checkSensorTimes(rgb_stamp,depth_stamp,time) 复核，不重写底层时间规则。delta_ns=depth-rgb，闭区间边界通过。
9. 保留原始 rgb_stamp/depth_stamp 和精确差值，不取平均、不覆盖、不用 bag 时间补齐。

这是确定的“按 RGB 顺序、单调、一对一最近候选贪心”策略，不保证全局最多配对或全局最小总误差。例：RGB=[4,6]、Depth=[0,5]、容差=6（同一演示单位），先把 RGB=4 配到 Depth=5，后续 RGB=6 未配对，结果少于另一种全局安排。第一版明确选择可审阅的局部时间接近原则，不隐藏这个取舍；实际数据若要求全局目标，应另开策略设计和验收，不能悄悄换算法。

Depth 频率低于 RGB 时会自然出现部分 RGB 无对应深度；当前不做深度重复复用、不合成新深度帧。未配对数量要如实报告。

## 6. 第三步：怎样实现候选查找

- 同段内只处理元数据；排序成本 O(R log R + D log D)。
- 维护下一次允许使用的 Depth 索引下界，选定后移到被选条目的下一位，实现不复用、不倒序。
- 每张 RGB 在合法后缀中二分定位时间前后候选，比较绝对时差；重复时间组选择仍可使用的最早原始序号。
- 再用 checkSensorTimes 确认容差；单帧查找约 O(log D)，避免逐张 RGB 扫描整包 Depth。
- 选择失败不推进到一个虚构深度位置；可保留原合法下界，下一张 RGB 继续检查。超过当前候选的旧 Depth 不回头复用。

这一查找形式属于本方案自身实现，不照搬在线 message_filters 的队列/pivot 策略。

## 7. 第四步：固定返回契约

建议主接口：

```cpp
RgbdSyncResult pairRgbDepth(
  const RgbdStreamIndex & rgb,
  const RgbdStreamIndex & depth,
  const RgbdSyncOptions & options);
```

| 返回对象 | 内容 |
| --- | --- |
| RgbdSyncResult | 类型化 code/message，成功时 optional<RgbdSyncReport> 有值；配置/输入错误无半份成功结果 |
| RgbdPair | 原始 RGB/Depth 引用、各自 stamp、delta_ns、exact/approximate 标记 |
| 未配对 RGB | 原始引用与原因：空 Depth 流、无容差内候选、窗口内候选已因单调/一对一策略不可用 |
| 未使用 Depth | 原始引用与 not_selected_by_policy 状态，不标成解码坏帧 |
| 统计 | 两流输入数、配对数、exact/approximate 数、两侧未配对数、RGB 匹配率和最大绝对时差；空输入时不做除零 |

配置正确但所有帧均未配对可以返回报告，ok 不代表配对率高；要看统计。纳秒字段沿用整数约定，绝对 stamp 以 sec/nanosec 表达，未来序列化不能损失 int64 精度。

一对一条件下必须满足：RGB输入数=配对数+未配对RGB数；Depth输入数=配对数+未使用Depth数。这里的统计只针对送入本次调用的两个合法索引。

示例：阈值 8ms，RGB 为 1000/1033/1066ms，Depth 为 1004/1037/1100ms。输出两对 +4ms、+4ms；RGB=1066ms 未配对，Depth=1100ms 未使用。显示单位为 ms，代码计算仍用纳秒。

## 8. 第五步：和已有模块对接

| 现有模块 | 使用方式 |
| --- | --- |
| common/sensor_time | 负责最终同域/容差/有符号差值校验；不改写 |
| common/sensor_names | 校验两条选定图像话题的绝对名称 |
| RGB/Depth | 从已解码帧或已反序列化消息提取 stamp；未来 reader 可以先读 header，配对后再解码/保存像素 |
| CameraInfo | 每张图分别选自己的标定候选，再调用 checkCameraInfoImage；静态标定不必逐帧强行同时 |
| TF | 分别用 rgb.frame_id/rgb_stamp、depth.frame_id/depth_stamp 调用 checkTime；插值间隔阈值与 RGB-D 容差不是同一个配置 |
| builder | 提供来源索引和时钟段上下文，消费配对结果、处理质量/pending，之后才保存 Observation/关键帧 |

时间配对通过后还要检查有效深度、空间注册、标定和 TF；不能只看 pair 的 matched/exact 标记就宣称完整观测可用。

若注册节点已改写 Depth header 采用 RGB 时间：保存实际消息 stamp 和上游处理依据，并检查能否取得原始深度话题。原始采样时间未记录时不能恢复，更不能用相等 header 声称实际曝光误差为零。本配对模块不推算隐藏的时间偏移。

## 9. 第六步：验证与人工审阅顺序

1. 先审头文件的输入/输出/所有权、错误与成功含义，再写实现。
2. 手写已知答案：严格相等、差值边界、超界 1ns、正负时差、等距、同刻不同消息、空流。
3. 一对一/单调检查：Depth 不重用、配对索引不倒退；包含上述贪心非全局最优示例，锁定真实语义。
4. 输入边界：乱序排列但稳定来源不变、重复来源 ID/顺序、坏 nanosec、负容差、未知/不同域、不同 segment、数量超限。
5. 时间边界：跨秒、负秒、真实零时刻、int32 秒范围，不能引入浮点秒精度损失或 abs 最小负数溢出。
6. 生成小规模用例，用简单穷举“当前可用候选”的参考实现核对每一步贪心选择；不拿全局最优算法当不同策略的预期结果。
7. 四模块组合/安装消费者：按返回引用取图像、标定和 TF，验证跨模块字段一致，仍明确未做位姿求解/导出。
8. reader 接入后再用真实短 bag 核对配对率、时差分布、未配对来源与重复运行一致性。构造数据通过不能代替这一步。

同名 Markdown 按 modules/bag 分类，解释每个文件和示例；实现完成后通读公开头/实现/测试/CMake，严格编译、安装后消费、Git 暂存供人工审阅。

## 10. 分次交付建议与自审

| 交付 | 内容 | 完成后能做什么 |
| --- | --- | --- |
| A：纯索引配对核心 | 上述最小类型、函数、统计、回归与文档 | 输入两个图像时间列表，输出确定性配对和未配对原因 |
| B：bag reader 接入 | 从真实 bag 读取 header/来源/段信息并调用 A | 实际从 bag 得到 RGB-D 配对引用与报告 |
| C：观测准备收口 | 标定选择、TF 数值求解、质量门控、关键帧、媒体/JSONL | 获得完整观测数据集；之后仍需原文实体/关系、MemoryEngine、纠错/发布与 Viewer |

完成 C 也只完成观测准备，不表示原文第 1、2 部分全部完成。建议从 A 开始逐个小模块落地；B 是让“给一个 bag 自动处理”成为现实的必要环节，不能在仅完成 A 时声称完整流程已修复。

自审结论：复用现有检查，不改前四模块；清晰区分采样/接收时间、同一时钟声明/物理同步、时间配对/空间注册；有确定平局和复用规则，错误可追溯，算法目标与限制明确；本轮仅形成待审方案。
