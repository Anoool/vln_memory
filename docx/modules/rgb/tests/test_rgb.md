# test_rgb.cpp：RGB 核心验证

## 规划与自审

针对外部可见行为构造独立输入和期望像素，不依赖 ROS、相机、模型或 gtest。不使用可能被 NDEBUG 禁用的 assert。以已知颜色和行尾哨兵暴露通道错误及填充泄漏，以源消息销毁暴露借用生命周期错误。

## 覆盖范围

1. rgb8 已知颜色和源元数据保留。
2. RGB/BGR 行尾填充、红蓝交换与标准 stride。
3. mono8、零时间、空 frame_id、8 位大小端标记。
4. 源数据和字符串修改并销毁后，输出仍然有效。
5. 非法尺寸、时间戳、编码和字节序标记。
6. 截断、多余字节、空指针、无效 stride。
7. 正好等于限额、超限、零限额与极大尺寸。
8. 稳定错误码名称。
9. 三种编码合计 900 组确定性生成样本，覆盖不同宽高与填充。

## 执行

在包根目录运行 [项目 README](../../../../README.md) 中的 g++ 命令，或通过 CMake/CTest 的 rgb_core 测试。

地址与未定义行为检查：

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer -g \
  -Iinclude src/bag/rgb/rgb.cpp test/bag/rgb/test_rgb.cpp -o build/standalone/test_rgb_sanitized
ASAN_OPTIONS=detect_leaks=1 ./build/standalone/test_rgb_sanitized
```

测试返回 0 表示全部通过；失败会输出原因并返回 1。此测试不代替 ROS 适配或真实 bag 验收。
