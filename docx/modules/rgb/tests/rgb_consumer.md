# rgb_consumer.cpp：安装后的下游接口验证

源码位于 `test/install_consumer/rgb_consumer.cpp`，独立项目配置位于同目录 CMakeLists.txt。

## 规划与用途

只通过安装前缀中的 find_package(vln_memory CONFIG REQUIRED) 与导出目标连接；不添加源码 include，不直接编译 rgb.cpp。这样能够暴露安装头文件、C++17 传递、库导出和链接路径问题。

## 接口示例

构造一个 BGR 像素 {3,2,1}，调用公开 decodeRgb，确认得到 {1,2,3}。这不是额外的图像算法测试，而是检验其他模块确实可以使用安装产物。

## 使用方式

先按 [项目 README](../../../../README.md) 构建，再安装到一个临时前缀。另行配置本测试项目，将 CMAKE_PREFIX_PATH 指向该前缀，然后构建并运行 rgb_consumer。Windows 还需把安装目录 bin 加入本次进程 PATH；Linux 使用正常动态库解析环境。

本轮实际执行情况见 [RGB 模块验证记录](../rgb.md#实施与验证记录)。
