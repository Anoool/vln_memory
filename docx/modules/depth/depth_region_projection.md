# 深度区域投影

公开接口为 `include/vln_memory/bag/depth/depth_region_projection.hpp`，随 `vln_memory::vln_memory_depth_projection` 安装。输入为同一深度帧的米制 optical-z 像素、有效模型、原始深度时间的历史位姿和 map frame。

- `DepthBoundingBox{u_min, v_min, u_end, v_end}` 使用半开区间，end 可以等于原图宽高。选区越界或为空时拒绝。
- `DepthPolygon` 接收 3–64 个原图整数像素中心，隐式闭合，包含边界。支持凹多边形与两种绕序，拒绝重复顶点、零面积、自交和越界。
- 两种选区逐像素复用 `projectDepthPixelToMap`。结果分别报告区域像素数、无效深度、投影失败、成功地图点数。
- 成功投影点按原图四邻接和最大 optical-z 差分簇。默认相邻深度差上限 0.05 m，最小簇 4 点，至少 20% 的选区像素可投影，首簇至少占成功点的 60% 才选全局代表点。阈值为可调的几何规则。
- 每簇提供点数、深度范围、map 轴对齐范围和实际像素代表点。代表点取最靠近簇质心的实测点。需要完整成员时设 `include_points=true`。多簇缺少主导支持时返回 `ambiguous_region`，保留簇结果供调用方检查。
- 默认包围窗口预算为 1,048,576 像素。设窗口面积 A、顶点数 V、候选簇数 K，投影与连通搜索按 A 扫描，polygon 掩膜为 O(A × V)，簇排序为 O(K log K)，临时空间为 O(A)。

消费正式观测时使用 [观测区域投影](../engine/observation_depth_region.md) 进行来源绑定和回读。
