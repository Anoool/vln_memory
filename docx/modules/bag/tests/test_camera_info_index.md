# test_camera_info_index.cpp

[源码](../../../../test/bag/builder/test_camera_info_index.cpp) 对应 CTest camera_info_index_core。用完整模型检查等距偏早、静态模型变化、同刻冲突、坏最近候选不回退、非法时间阻断版本选择。测试仅调用无 ROS 公开接口，不代替真实物理标定验收。

同一测试源码也构建为包外 camera_info_index_consumer，仅链接安装目标 vln_memory::vln_memory_camera_info_index，避免复制测试实现，同时验证安装公开头与依赖传递。
