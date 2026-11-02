# test_camera_model.cpp

[测试源码](../../../../test/bag/camera_info/test_camera_model.cpp) 通过 CTest camera_model_core 运行；安装消费者 camera_model_consumer 复用该源，仅链接安装后的 vln_memory::vln_memory_camera_model。

验证显式有限零尾 D8、严格默认、非零尾/NaN/未知策略拒绝，直接匹配/模型入口一致，原始数据独立保留。K 与 P 故意不同，分别核验 raw/rectified 射线；非单位 R 用已知旋转检查逆变换方向。ROI/binning 使用独立数值答案；验证越界、可变模型绕过、双目平移和校正 ROI 拒绝。

三种畸变以独立正向公式生成已知像素，再检查逆解恢复的方向，不仅比较同一实现自身的零畸变结果。以上为构造几何测试，不证明设备物理精度。真实全包验收见 [相机模型验收](../../../reviews/camera_model_verification.md)。

运行：ctest --test-dir build/camera-model-ros --output-on-failure；无 ROS 使用 build/camera-model-core。
