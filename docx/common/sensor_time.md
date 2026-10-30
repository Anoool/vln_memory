# sensor_time.hpp / sensor_time.cpp：跨模块时间检查与使用约定

## 1. 规划与职责

RGB/Depth 已保留整数 SensorStamp。此公共文件只补充相同时钟下的时间差检查，供 CameraInfo 和后续 builder 的候选消息比较使用，不实现消息队列、时钟校正、控制器或 bag 导出。TF 索引独立采用同一 SensorStamp 与显式时钟/段 ID。先定义时钟关系、符号和容差，再实现无浮点比较，自审 int32 秒全范围的溢出边界。

头文件：include/vln_memory/bag/common/sensor_time.hpp。实现：src/bag/common/sensor_time.cpp。目标：vln_memory::vln_memory_sensor_time；CameraInfo 核心 PUBLIC 链接它。

## 2. 接口与含义

```cpp
SensorTimeCheck checkSensorTimes(
  const SensorStamp & reference, const SensorStamp & candidate,
  const SensorTimeOptions & options);
```

options.clock_relation 必须显式为 same_clock；unknown 返回 clock_unknown，different_clocks 返回 clock_mismatch。消息里的 sec/nanosec 不带时钟标识，不能靠数值接近来认定同一时钟。驱动时间基准、硬件同步、时钟校准和 bag 分段由采集配置/builder 证明。

max_delta_ns 为非负整数纳秒，默认 0 表示严格相等。函数先验证输入与容差，再计算 delta_ns=candidate-reference；闭区间 [-max,+max] 为 matched，否则 outside_tolerance。超差仍保留 delta_ns；无效/未知时钟时 delta_ns 为空。sensorTimeCodeName 提供稳定状态名称。

sec 可负、零时刻合法，nanosec 必须 <1e9。先转换 int64 再乘 1e9，int32 秒全范围及其最大时差 4294967295999999999ns 均可表示，不经过 double。

## 3. 当前模块是否同步

现在 RGB、Depth、CameraInfo 都保存原始 header.stamp，并可调用本检查器检查候选消息的采样时差；它们不会自行对齐时间。没有实际 bag，不能判定设备同步情况。

后续 builder 应统计各流频率、重复/倒退时间、RGB-D 有符号时差分布、CameraInfo 变化点、TF 覆盖和未配对比例。相同时钟才能分析 bag 接收时间与采样时间的延迟；不能把不同域相减后的数字当成传输延迟。模拟时间重置/硬件时钟重启必须分 clock epoch，不能跨重置点配对。

ROS message_filters 的 ExactTime/ApproximateTime 按消息时间做队列同步；ApproximateTime 不会实现硬件同步或消除运动误差。离线 builder 会建立可重现的索引配对规则，当前这个函数仅核验已经选出的候选帧，不承担匹配队列。

## 4. 时间不同时，分别用哪一个

| 用途 | 应采用的时间 | 原因与处理 |
| --- | --- | --- |
| Observation 索引/关键帧排序 | RGB header.stamp | 视觉语义证据以 RGB 为主；同时保留 depth_stamp，不取平均 |
| RGB 对象/区域在图中的证据 | RGB header.stamp | 对应真正的 RGB 采样；模型返回时间不能替代它 |
| 深度点反投影与 map 变换 | Depth header.stamp | 深度属于该时刻的相机位姿，TF 查询在该时刻进行 |
| RGB 相机/底盘观测位姿 | RGB header.stamp | 单独查询该时刻 TF；不能拿来替代 Depth 位姿 |
| 逐帧 CameraInfo | 对应图像 header.stamp，按声明容差检查 CameraInfo stamp | 可以严格相等或显式近似；超限进入 pending |
| 固定 CameraInfo | 显式固定数据段/版本 | 可以复用原始旧时间标定，不改 stamp，记录静态策略和依据 |
| 动态 CameraInfo | 图像时刻所在的标定有效区间 | builder 选择；不能整包取最后一条、最近未来版本或未经声明无限向后复用 |
| 动态 TF | 被变换数据的采样时间 | 按采样时刻查历史链，允许何种插值由 TF 模块定义，不用 latest 填历史数据 |
| 静态 TF | 经声明的静态关系及其数据集有效范围 | 不要求 stamp 与图像相等，也不把动态 TF 误当静态 |
| bag 记录/接收时间 | bag 自己的时间字段 | 用于追溯、读取顺序与同域延迟分析；不能默认替代 header.stamp |
| 记忆构建/入库时间 | 实际处理系统时间，单独 stored_at/processed_at | 是软件处理时刻，不是物体被观察到的时间 |
| 在线规划和底层控制 | 当前控制周期的机器人状态/控制时钟 | 历史传感数据先按采样时间转到 map，再结合当前状态、数据新鲜度和地图版本规划 |

**时间容差通过不等于像素同时曝光，更不等于物理配准通过。** RGB-D 差异超限时应拒绝组合并保留 pending。容差通过但机器人运动明显时，仍有投影误差；降低容差、使用硬件同步或明确运动补偿。运动补偿涉及两个时刻的相机位姿和静态场景假设，当前没有实现，不能用改 stamp 或平均时间来代替。运动物体需要额外模型。

在线控制不能直接把 bag 的历史时间当“现在”，也不能把历史图像坐标直接当当前车体坐标。记忆中对象的 map 坐标还需核对 map/geometry revision、定位跳变和观测新鲜度；控制器由当前定位和路径结果闭环执行。当前模块不会发布任何控制指令。

## 5. 未来 JSONL 交接字段（规划，尚未写文件）

builder 应保留：rgb_stamp、depth_stamp、各 CameraInfo 原始 stamp、每条源消息 bag_time、clock_id/epoch、map_id、rgb/depth 各自 pose 查询时间、配对 delta/tolerance/status、标定策略/版本/依据、来源 topic 和消息 ID。示例字段不等于已经确定完整 Observation schema。

stamp 始终用 {sec,nanosec} 整数对象；若保存绝对 int64 纳秒，写成十进制字符串，避免 JS/Viewer 读取超出 2^53 的 JSON 数值而损失精度。没有检查的时差用 null 加策略说明，不能写 0 冒充同步。

header 时间缺失、时钟域不明或超差时默认进入 pending。确有驱动配置证明只能采用 bag 时间时，应作为显式替代策略保存原时间、替代依据、降级质量和受影响的 TF 语义，不在模块内静默回退。

零采样时间可表示，但 tf2 的 time=0 有 latest 哨兵含义。当前 [TfIndex](../modules/tf/tf.md) 直接按整数索引查询真实零时刻，不调用 latest；后续几何求解若接入 tf2 Buffer，仍必须处理这个差异，不能直接透传成 latest。

## 6. 示例

```cpp
#include <vln_memory/bag/common/sensor_time.hpp>
#include <iostream>

int main()
{
  using namespace vln_memory;
  // 这里只是演示 10ms 配置，不是项目默认推荐容差。
  SensorTimeOptions options{SensorClockRelation::same_clock, 10000000};
  const auto result = checkSensorTimes({10, 0}, {10, 5000000}, options);
  if (!result.matched()) {return 1;}
  std::cout << *result.delta_ns << '\n';  // depth 比 rgb 晚 5ms
  return 0;
}
```

## 7. 验证与参考

[test_sensor_time.cpp 说明](tests/test_sensor_time.md) 覆盖秒边界、正负差值、最大秒范围、最大容差、无效纳秒、负容差、未知/不同域、零时刻。真实 RGB/Depth/CameraInfo 的公开接口协作见 [安装消费者](../modules/camera_info/tests/camera_info_consumer.md)。测试没有真实设备采样，不提供实测同步误差结论。

- [CameraInfo 消息约定](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)：header 是采样时间。
- [Humble ApproximateTime 实现](https://github.com/ros2/message_filters/blob/humble/include/message_filters/sync_policies/approximate_time.h)：队列时间匹配，可明确限制时间区间。
- [Humble TF 时间教程](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.rst)：按时刻查询以及零时刻的 latest 含义。


## 2026-10-29 修复与注释复核

补明“先选候选、再确认同域、最后比较时差”的调用顺序；matched 不代表已完成同步队列或时钟校准。 具体修复与验证记录见 [本次实施记录](../project/sensor_validation_comments_plan.md)。
