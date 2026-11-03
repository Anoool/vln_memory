# test_depth_projection.cpp

[测试源码](../../../../test/bag/depth/test_depth_projection.cpp) 使用构造的深度帧、有效 raw 模型和实际 TfIndex 查询，以离轴像素 (3, 2) 的 2 m 深度验证结果为 (9, 22, 32) m。使用非零旋转和平移核对方向及顺序。另检查越界、零/NaN 深度、截断布局、错误网格/相机帧、无效内参、错误时刻或方向、位姿缺失及坏四元数均无坐标。

CTest 入口为 depth_projection_core。独立安装工程的 depth_projection_consumer 复用同一源码，只链接安装后的公开目标。这是构造验证。

跨流构造用例检查同位姿离轴像素回投影、0.1 m 基线产生 25 px 左移、同名帧在不同时刻的位移，以及 RGB 逆旋转。错用 Depth 位姿时间、缺 RGB 位姿、无效深度、相机后方或投影越界均返回空像素，并保留失败阶段。
