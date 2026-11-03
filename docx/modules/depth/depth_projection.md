# depth_projection.hpp / depth_projection.cpp

[公开头](../../../include/vln_memory/bag/depth/depth_projection.hpp) 与 [实现](../../../src/bag/depth/depth_projection.cpp) 提供 projectDepthPixelToMap。使用 find_package(vln_memory CONFIG REQUIRED) 后链接 vln_memory::vln_memory_depth_projection。该库复用无 ROS 的 Depth、CameraModel 和 TF 核心。

输入是已解码的 DepthFrame、为该帧选出的有效 CameraModel、TfIndex::lookupPose 的结果、目标 map_frame，以及原深度网格的整数列 u 和行 v。调用方须确认模型对应当前深度帧、位姿来自同一时钟段，以及米制 optical-z 的轴属于 depth.frame_id 的原光学坐标系。对齐到 RGB 的像素网格声明本身不足以确定深度 Z 轴。

接口核对布局、像素边界、深度正值与有限性、模型网格与光学帧，并检查位姿查询时间与该帧的原始 sec/nanosec 完全相等、TF 链方向是 depth.frame_id → map_frame。随后取一条 z=1 射线，乘以深度得到相机点，再通过 transformTfPoint 计算旋转和平移后的地图点。

成功返回 ready 和独立持有的三元素 point_map_m，单位米。失败返回对应状态和空坐标。ray_unavailable 表示已有相机模型入口无法生成射线，pose_unavailable 表示历史查询未产出可用位姿，pose_mismatch 表示时刻或帧不符，numerical_failure 表示位姿参数或点变换数值无效。原始深度帧、时间、模型和 TF 证据留在调用方。接口借用输入，只访问一个深度采样，额外空间不随图像大小增长。

projectDepthPixelToRgb 在单点地图投影之后，用 RGB 位姿执行 p_rgb = R_map_from_rgb^T * (p_map - t_map_from_rgb)，再调用 cameraPointToPixel。

| 参数 | 含义 |
| --- | --- |
| depth / depth_model | 已解码的深度帧及该帧原始网格上的有效相机模型。 |
| depth_pose | 用 Depth 原始采样时刻查询的 map ← Depth 光学帧位姿结果。 |
| rgb / rgb_model | RGB 的 CameraImageReference（话题、网格、光学帧、原始采样时间）及该图的有效模型。只用 RGB 元数据与模型。 |
| rgb_pose | 用 RGB 原始采样时刻查询的 map ← RGB 光学帧位姿结果。 |
| map_frame | 两次查询共有的地图坐标系名称。调用方提供同一时钟段、同一地图上下文下已选中的配对。 |
| u / v | 原深度网格的整数列、行，零起始。 |

DepthToRgbResult::pixel_rgb 是 RGB 实际网格上的浮点列、行，成功时有效。code 区分 Depth 侧失败、RGB 模型或位姿不匹配、RGB 位姿缺失、数值失败和 RGB 正投影失败。depth_code 和 camera_code 分别保留对应子阶段的错误。两帧有时差时按地图点静止计算候选像素。可见性、遮挡、物理尺度与注册证据由后续验收处理。

验证入口见 [test_depth_projection](tests/test_depth_projection.md)。
