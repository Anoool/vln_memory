# test_depth.cpp：Depth 核心测试

源码：[test_depth.cpp](../../../../test/bag/depth/test_depth.cpp)。

## 规划与自审

已知字节序使用手写 uint16/float32 字节作为独立依据；生成样本覆盖不同编码、端序、宽高与填充。普通检查不使用会被 NDEBUG 关闭的 assert。

## 11 组覆盖

1. uint16 毫米、奇数行填充、非对齐输入起点、元数据与统计。
2. 手写大/小端 uint16 和 IEEE float32，验证已知物理深度。
3. 单位和 convention 必填，非标准但显式单位组合按声明换算。
4. 零、负零、负数、NaN、正负 Inf、范围边界和过滤参数来源。
5. 全无效帧、float32 下溢及有限最大值。
6. 未知/未对齐/已声明对齐、实际 RGB 网格不匹配和矛盾声明。
7. 尺寸、时间、编码、端序、stride、空指针、截断和额外字节。
8. 精确限额、超限、零限额和整数溢出拒绝。
9. 源数据、字符串、注册依据与网格销毁后输出仍独立有效。
10. 稳定错误、单位、数值类型和对齐状态名称。
11. 两种编码、两种端序共 600 组生成样本，检查逐像素结果和计数恒等式。

## 执行

包根目录可直接运行：

```bash
mkdir -p build/depth-check
g++ -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror \
  -Iinclude src/bag/depth/depth.cpp test/bag/depth/test_depth.cpp -o build/depth-check/test_depth
./build/depth-check/test_depth
```

CMake/CTest 名称为 depth_core。添加 -fsanitize=address,undefined -fno-omit-frame-pointer -g 可运行地址和未定义行为检查；本轮已通过。完整结果见 [Depth 验证](../depth.md#验证结果)。
