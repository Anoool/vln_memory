# camera_model.hpp / camera_model.cpp

## 职责与接口

[公开头](../../../include/vln_memory/bag/camera_info/camera_model.hpp) 与 [实现](../../../src/bag/camera_info/camera_model.cpp) 为实际图像生成独立拥有的有效模型。模型库 vln_memory::vln_memory_camera_model 公开依赖既有 CameraInfo，内部链接 OpenCV core/calib3d，不依赖 ROS；无 ROS 构建也需要已安装的 OpenCV 开发库。

| 接口 | 输入 | 输出 |
| --- | --- | --- |
| buildCameraModel | CameraInfoData、实际 CameraImageReference、CameraInfoMatchOptions | CameraModelResult：code/message/可选 model；先匹配，再按统一策略规范化并调整实际网格 |
| pixelToCameraRay | CameraModel、原图实际网格浮点 u/v | CameraRayResult：code/message/可选三元素方向 |
| cameraPointToPixel | CameraModel、原始光学帧中的米制三维点 | CameraPixelResult：code/可选浮点列行。raw 应用有效 K/D，rectified 先应用 R 再用有效 P。 |
| cameraGeometryCodeName | CameraGeometryCode | 静态字符串，无需释放 |

模型拥有 width/height/frame_id、image_domain、distortion_model、D/K/R/P、normalization_applied。K/P 已应用 ROI 和 binning，不能再次缩放。原始模型及来源由 camera_info.jsonl 和每帧 calibration_id 保存，模型不借用像素或原消息。

## 计算与坐标

raw：依据有效 K/D，用 OpenCV 对单点去畸变。支持 plumb_bob(5)、rational_polynomial(8)、equidistant(4)。普通模型逆解最多 30 次，再按同一正向模型验证像素残差不超过 1e-4；非有限/不收敛返回 numerical_failure。

rectified：依据有效 P 左侧内参反投影，再乘 R 的转置回到原始 frame_id 光学坐标；除以原光学 z，输出 z=1 方向。不是单位长度向量。若深度定义确为此原光学 frame 的 optical-z，可再乘对应深度；校正坐标的 z 不能直接当原坐标 z。

ROI 使用完整未 binning 坐标：先扣主点偏移，再将 K/P 对应行按 bx/by 缩放，P 的平移项同样缩放。binning=0 按 1 解释。无隐式 resize、裁剪或图像重采样。

非零双目 Tx/Ty 的 rectified 投影返回 unsupported_projection；rectified + roi.do_rectify 仍由匹配返回明确不支持。原始参数可保存，不能将未支持几何静默当作单目。

## 错误与边界

ready、camera_mismatch、unsupported_projection、invalid_model、invalid_pixel、numerical_failure 是稳定码。失败无默认模型/射线。公开模型可变，因此每次射线入口复用严格数值/结构检查；u/v 必须有限且位于 [0,width)×[0,height)。

bad_alloc 和 OpenCV 内存分配失败继续抛出，不伪装成输入问题。其他 OpenCV 数值异常转换为 numerical_failure。每次模型调整 O(1)，单点求解有固定迭代上限和固定额外空间；当前不建立全图射线缓存。

有效模型不等于深度注册、物理标定精度、设备曝光同步或 map 位姿；这些状态由各自阶段负责。测试和独立安装入口见 [test_camera_model](tests/test_camera_model.md)。

正投影要求原光学 z 为正。rectified 还要求校正后的 z 为正。非正深度返回 point_behind_camera，超出实际网格返回 invalid_pixel。输出像素保留浮点精度。
