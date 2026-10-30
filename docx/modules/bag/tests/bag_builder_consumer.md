# bag_builder_consumer.cpp

[源码](../../../../test/install_consumer/bag_builder_consumer.cpp) 只链接 vln_memory::vln_memory_bag_builder，不引用源码私有路径。无参数校验默认配置失败契约；带 YAML/可选新输出目录时运行真实安装库提取。

构建 test/install_consumer 时 CMAKE_PREFIX_PATH 指向安装前缀；加载 ROS 与安装环境后运行。它验证头文件/依赖/库安装完整，实际物理标定仍由数据验收负责。
