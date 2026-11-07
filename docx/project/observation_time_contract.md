# Observation 参考时刻与字段关联约定

更新于 2026-11-08。本文说明已实现的 `observation/1` 时间语义及位姿、图像网格和来源之间的关联。

## 1. 交付状态与职责

一条 Observation 聚合一对已选定的 RGB-D、各自模型和位姿、底盘位姿、媒体及来源，使下游能够按一个观测 ID 读取相关信息。该记录采用一个参考时刻组织字段，同时保留各输入的真实采样时间与有效范围。

本契约已落实到 [正式记录](../modules/memory/observation.md) 和 [观测数据集](../modules/engine/observation_dataset.md)。组装器消费 Reader 已回读的 v5/v6/v7 preparation 配对，保留原时间和来源，再统一校验和序列化。

## 2. 参考时刻和来源

Observation 的 stamp 取 RGB 原始 header 时间，stamp 与 rgb_stamp 必须逐字段相等。排序采用该参考时间，同刻记录保留稳定的来源顺序。零时刻作为真实采样时间处理。

| 字段 | 约定 |
| --- | --- |
| id | 稳定的业务观测 ID，引用的 pair_id 及源消息身份可回查 |
| bag_id / pair_id | 来源 preparation 及已保存配对 |
| clock_id / segment_id | 继承来源 manifest 中经声明的共同时间基准和数据段 |
| map_id / geometry_revision | 明确地图身份及其几何版本，空间参考依据与数据一同保存 |
| stamp | 观测参考时刻，等于 rgb_stamp |
| rgb_stamp / depth_stamp | 两张原始图像各自的整数 sec/nanosec |
| delta_ns | Depth−RGB 的有符号整数纳秒，JSON 写十进制字符串 |
| sync_tolerance_ns | 实际使用的非负配对门限，JSON 写十进制字符串 |
| time_match | 完整有效配对中 delta 为 0 时为 exact，其余门限内结果为 approximate |
| capture_sync_status | 保留已有曝光同步状态及依据 |

时间字段、时差和配对策略来自已保存记录。读取入口核对同一 preparation 内的配对、图像、模型和位姿引用。跨数据集关联需要额外的时间关系及地图配准依据。30 ms 为显式试用门限。它在已有共同时间基准的前提下筛选采样差异，曝光同步另有采集证据。

## 3. 每个模型和位姿对应的时刻

| 字段或数据 | 关联时刻与坐标 |
| --- | --- |
| camera_pose_map | map ← RGB 原光学帧，查询时刻为 rgb_stamp |
| body_pose_map | map ← body，查询时刻为 stamp |
| depth_camera_pose_map | map ← Depth 原光学帧，查询时刻为 depth_stamp |
| rgb_model / depth_model | 分别属于各自的实际图像网格、光学帧和采样时刻 |
| CameraInfo 来源 | 保留原始标定时间、标定 ID、匹配策略和有效范围 |
| 固定 CameraInfo / 静态 TF | 按明确数据段和有效范围复用，原始广播时间保留 |
| bag 记录时间 / 处理时间 | 作为追溯与处理信息单独记录 |

位姿平移用米，旋转用 xyzw 单位四元数，含义为 T_map_source。动态 TF 可按当前策略选择历史样本并插值，保留端点来源、查询时间和求解状态。缺失或被拒绝的模型、位姿保留为空及原因。构建输入按具体操作需要的字段判断可用性。

物品区域投影使用 depth_stamp 的相机位姿，空间锚点使用 RGB 参考时刻的历史底盘位姿。Depth→map 在 depth_stamp 使用深度值、Depth 模型与 Depth 相机位姿。Depth→RGB 先得到该地图点，再在 rgb_stamp 使用 RGB 相机位姿与 RGB 模型投影。该路径按地图点在两帧间静止计算候选像素，动态物体、遮挡和可见性需要后续依据。

## 4. 单一 width、height 和 intrinsics 的使用条件

用户示例中的单一相机模型适用于已经确认的共同几何表示。图像域为明确的 rectified pinhole，RGB 与 Depth 位于同一目标像素网格，相机内参、深度光轴与该目标光学帧一致，并有相应注册依据。

采用各自原始网格的输入时，分别关联 RGB 和 Depth 的 width、height、frame 和有效模型。相同尺寸或同名 frame 只能作为字段一致性条件，空间注册仍需来源依据。

若后续生成面向参考时刻的派生深度图，保存目标时刻、目标模型、派生方法和有效值/覆盖规则，并继续引用原始深度与时间。只有原始 header 实际相等的配对使用 exact。单点重投影结果按候选像素使用，整图派生另需处理多点落到同一像素、空洞和遮挡。

## 5. 深度媒体与几何单位

深度媒体路径和文件的实际格式、单位、比例及无效值规则必须共同可读。复用 preparation 时继承每帧 details.storage。源 uint16 PNG 按保存的比例恢复为米，米制 NPY 按原值读取，运行时 DepthFrame 使用米制 optical-z。

若简化观测声明 depth_unit=m，depth_path 应指向对应的米制媒体。直接引用源 PNG 时，应同时保留其存储单位与 scale_to_m，读取后再使用米制几何值。两种媒体的来源和恢复规则均可由观测引用追溯。

## 6. 时间字段示例

以下是时间契约片段，演示同一时间基准内相差 20 ms 的一对图像。位姿、模型、媒体与来源按上表补齐。

```json
{
  "id": "obs_128",
  "clock_id": "bag_session_1",
  "segment_id": "segment_1",
  "stamp": {"sec": 100, "nanosec": 0},
  "rgb_stamp": {"sec": 100, "nanosec": 0},
  "depth_stamp": {"sec": 100, "nanosec": 20000000},
  "delta_ns": "20000000",
  "sync_tolerance_ns": "30000000",
  "time_match": "approximate",
  "capture_sync_status": "unknown"
}
```

严格时间相等的示例使用实际相等的两路原始时间、delta=0 和 time_match=exact。保留原始采样时间可以让历史位姿查询及几何计算复现。

## 7. 实现时的验收

1. 同一输入得到稳定的观测 ID 和来源引用，stamp 始终等于 rgb_stamp。
2. 原始采样时间和有符号 delta 可独立回查，门限内的非零差值保留 approximate。
3. 三个位姿分别核对自己的查询时刻、源帧和地图上下文，错误时刻返回明确原因。
4. 固定标定按数据段复用，各路模型与实际网格一致，原始标定时间保留。
5. PNG/NPY 的存储参数与恢复后的米制深度一致。
6. 共同模型视图的生成需满足网格、光轴、图像域和注册条件，派生数据可回查原始输入。
7. 缺项进入 pending，读取、选择和发布均保留原因及证据引用。

参考已有 [跨模块时间规则](../common/sensor_time.md)、[配对门限](../interfaces/bag_builder_api.md#rgb-d-时间基准与配对门限) 和 [Preparation 读取接口](../modules/bag/preparation_reader.md)。

2026-11-06：正式记录 schema 为 observation/1，公开类型、单位、引用和编解码约束见 [模块](../modules/memory/observation.md)；实施前方案与自查见 [方案](observation_implementation_plan.md)。
