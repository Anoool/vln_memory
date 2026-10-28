# depth_consumer.cpp：安装后的 RGB/Depth 接口协作

源码：[depth_consumer.cpp](../../../../test/install_consumer/depth_consumer.cpp)。

## 规划与用途

独立下游只通过 find_package 和安装目标，分别处理 RGB 与 Depth。以 RGB 输出构造 ImageGeometry，检查毫米深度换算与注册声明是否匹配实际 RGB 网格。

链接 vln_memory::vln_memory_rgb 与 vln_memory::vln_memory_depth；不借用源码 include，不手动编译核心源文件。因而能检查公开共享类型、C++17 与导出库是否足以被后续 bag 组装层使用。

从 [安装消费者项目](../../../../test/install_consumer/CMakeLists.txt) 配置、构建并运行 depth_consumer。本轮无 ROS 安装与真实 ament 安装两种方式均已验证通过；见 [Depth 验证](../depth.md#验证结果)。
