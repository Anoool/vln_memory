# test_sensor_time.cpp：采样时差的边界检查

源码 test/bag/common/test_sensor_time.cpp，只链接 vln_memory_sensor_time，CTest 名 sensor_time_core。

规划先验证已知秒边界和符号，再验证 int32 秒极值与 int64 容差边界，最后验证无效输入和不同/未知时钟的拒绝。使用独立的常量预期值，不用被测函数生成期望值。

验证范围：闭区间边界与边界外 1ns、反向负差、负秒/零时间、完整 int32 秒区间、最大 int64 容差、负容差、nanosec=1e9、未知时钟不返回差值、不同域拒绝。

运行：ctest --test-dir build/camera-core -R sensor_time_core --output-on-failure。也随真实 ROS 模式的 CTest 执行。检查器只评估候选对，不验证硬件同步，详细契约见 [sensor_time](../sensor_time.md)。
