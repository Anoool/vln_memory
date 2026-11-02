# 真实 bag 相机兼容与有效模型实施

日期：2026-11-03。分支 codex/camera-model-compatibility，能力基线 24af181，进度文档基线 c1c7ab3。用户授权以真实录包实现并跑通；原始 ZIP、CDR、header 和标定字段保留。

## 范围与接口

1. normalizeCameraInfo(raw, compatibility) 统一生成严格校验后的派生标定；默认 strict；显式 plumb_bob_zero_tail_d8_v1 只接受有限且末三项严格为零的 D8，派生 D5。
2. BagCameraInfoRecord 保存原始参数、原始校验、派生参数/校验与是否转换。索引仍按原始版本和时间选择，最终匹配及模型使用派生结果；坏候选不回退。
3. buildCameraModel(info, image, options) 输出实际图像网格的 K/D/R/P、raw/rectified、光学 frame；pixelToCameraRay 输出原始光学 frame 中 z=1 的方向。raw 调用 OpenCV 去畸变；rectified 使用 P 的内参与 R 的逆旋转。非零双目 Tx/Ty 的校正投影和需要几何变换的 rectified ROI 首版明确拒绝。
4. builder 逐帧导出模型、绑定和来源；配对与汇总报告模型的实际成功状态，数值 TF、物理注册仍分别报告。当前不新增 Observation 类型；preparation 的读取/引用/筛选在后续 API 输入组织时处理。

## 依赖、错误与所有权

既有严格 CameraInfo 库保持无 ROS；新增模型库只使用普通 C++ 公开结构，内部复用现有 OpenCV core/calib3d。模型/标定结果独立拥有数据；索引选择借用记录，生命周期随不可变 scan。无效参数、未支持的投影、越界像素、数值逆解失败均有稳定码；不使用单位模型补缺失。

索引构造按话题排序 O(C log C)，单次选择 O(log C)；模型矩阵/ROI 运算 O(1)，单点逆畸变有固定迭代上限。继续流式读取图像，不持有全部像素；不新增无使用方的类/目录。

## 验收

- 严格/D8 有界兼容、原值保留、非零尾/NaN/未知策略拒绝；
- 索引、直接匹配和配置对同一策略一致，固定模型变化/同刻冲突保留；
- K != P 的 raw/rectified 分支、ROI/binning 数值、旋转方向和去畸变往返；
- 无 ROS、ROS 全套、安装消费者；
- 原 ZIP SHA/DB 核对后完整真实 bag 提取，独立回查原始/派生标定、图像模型、来源与原始时间。记录仍有的 TF/注册问题，不把提取成功写成完整空间记忆完成。
