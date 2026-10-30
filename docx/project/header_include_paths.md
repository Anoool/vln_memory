# 公共头文件位置、搜索根目录与 CMake 配置

日期：2026-10-31。本轮修正公共头引用的可读性和 CMake 自身头路径声明，不改变消息处理、零容差配对或公开函数签名。

## 编码前核查与规划

实际头文件均在 include/vln_memory/bag/ 下，src/bag/ 没有重复头文件。原 #include "vln_memory/bag/..." 在现有 include 搜索根下能够找到真实文件，并不是在 src/bag 下寻找同名头。已发现的可维护性问题是部分库通过依赖库间接获得本包 include 根目录，不便人工审阅其自身编译条件。

实施拆分：

1. 保持物理目录和安装路径，公共引用统一为 <vln_memory/...>；不增加 include/ 前缀或跨层 ../ 相对路径。
2. 对所有实际库目标直接设置 C++17、本包 BUILD_INTERFACE include 根与 INSTALL_INTERFACE include 根；集中使用现有 memory_export_targets 清单，避免逐模块漏配或新建中间库。
3. 真正调用其他模块实现的 target_link_libraries 保留；头文件搜索和链接依赖分别说明。
4. 验证原生 g++ -I（不依赖 CMake）、ROS 全量构建/测试、安装后的独立消费者以及无 ROS 核心构建。只使用小型构造测试，不重新解压真实 rosbag 或生成大型媒体。
5. 更新本文件、CMakeLists 同名说明与文档导航，使用独立 codex 分支保存。

自审：头文件路径应在源码和安装后都成立；每个库拥有自己的 include 属性；不通过复制头文件到 src、添加全局 include_directories 或硬编码机器绝对路径掩盖问题。

## 路径怎样对应

```text
仓库根目录/
├── include/                         编译时搜索根：-I<仓库>/include
│   └── vln_memory/bag/builder/bag_builder.hpp
└── src/bag/builder/bag_builder.cpp
```

源码中的 `#include <vln_memory/bag/builder/bag_builder.hpp>` 对应：

```text
<仓库>/include + vln_memory/bag/builder/bag_builder.hpp
```

安装后同一写法对应 `<安装前缀>/include/vln_memory/bag/builder/bag_builder.hpp`。`include` 是搜索根，`vln_memory/bag/...` 是相对该根的公共路径；bag 表示模块分类。尖括号不等于只能引用系统库，配合 -I 同样可以引用本项目公共头。

不能把代码改成 <include/vln_memory/...>：那会要求搜索根变成仓库根，且不符合已安装包的公开路径。也不应改成 ../../../include/...，那会把接口绑定到当前 cpp 的目录深度。

## CMake 输入和作用

实际设置集中在 CMakeLists.txt 的库目标列表之后。BUILD_INTERFACE 用于当前源码构建，INSTALL_INTERFACE 的 include 相对安装前缀；PUBLIC 使目标自身及下游都获得头文件位置。target_link_libraries 另外声明实现符号和模块依赖，不能将二者含义混为一谈。

已安装下游仍使用 find_package(vln_memory REQUIRED)，链接相应 vln_memory:: 目标，并包含 <vln_memory/bag/...>。不需要借用仓库 src 目录，也不需要手动添加源码绝对路径。

官方依据：[CMake target_include_directories](https://cmake.org/cmake/help/latest/command/target_include_directories.html)、[GCC 头文件搜索路径](https://gcc.gnu.org/onlinedocs/cpp/Search-Path.html)。

## 验证结果

已完成：

- 80 处公共 include 在 51 个 C++ 文件中统一为尖括号；逐文件与修改前比较，除该语法外正文完全一致。原函数签名、处理算法与零容差逻辑不变。
- 91 处公共头引用全部能映射到真实 include/vln_memory/... 文件；源码 src 下没有重复头。
- 实际编译数据库中，17 个库源码编译项都有仓库 include 根；10 个 ROS 安装消费者使用安装前缀/include，不含源码 include 根。
- 编译器实际依赖文件确认 bag_builder.cpp 包含的是 /home/dev/agent/vln_memory/include/vln_memory/bag/builder/bag_builder.hpp。
- 直接执行 g++ -I<仓库>/include 编译配对核心和现有测试，不通过 CMake，600 个参考算法案例及边界检查通过。
- ROS 构建与 15/15 CTest 通过；安装后的 10 个独立消费者全部编译并运行通过。
- 全新无 ROS 构建 8/8 CTest，安装后的 6 个独立消费者通过；临时构建/安装目录已自动清理。

证据位于 build/rgbd-strict-20261030/header_include_verification.json、header_ros_build.log、header_core_verification.log、header_consumer_build.log。ROS 编译数据库为 build/rgbd-strict-ros/compile_commands.json，编辑器可使用真实编译参数解析头文件。

本轮没有重新生成真实 bag 媒体，也没有恢复已清理的大型数据。使用 codex/explicit-header-paths 分支，原功能和清理提交保留，不自动合并。

## 对照示例

```cpp
#include <vln_memory/bag/builder/bag_builder.hpp>
```

```cmake
# 每个实际库都直接获得自己的根目录，既用于本库也传递给下游。
target_include_directories(your_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
```

上面的 your_target 是解释性示例；实际 CMake 使用 memory_export_targets 循环对每个库设置同样属性，未新增中间目标或更改安装树。
