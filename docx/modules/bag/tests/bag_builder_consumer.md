# bag_builder_consumer.cpp

[源码](../../../../test/install_consumer/bag_builder_consumer.cpp) 只链接 vln_memory::vln_memory_bag_builder，不引用源码私有路径。无参数校验默认配置失败契约；带 YAML/可选新输出目录时运行真实安装库提取。

构建 test/install_consumer 时 CMAKE_PREFIX_PATH 指向安装前缀；加载 ROS 与安装环境后运行。它验证头文件/依赖/库安装完整，实际物理标定仍由数据验收负责。

## 压缩适配安装消费

当前程序还包含安装后的 image_decoder_ros.hpp，并直接调用 decodeCompressedImage 验证类型化错误返回；只链接 vln_memory_bag_builder，不借用源码 include。新适配器是同一目标的接口，不要求新增顶层模块。
