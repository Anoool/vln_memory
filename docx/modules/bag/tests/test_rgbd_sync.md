# test_rgbd_sync.cpp

[测试源码](../../../../test/bag/builder/test_rgbd_sync.cpp) 通过 CTest rgbd_sync_core 运行。覆盖严格/容差边界、等距与同刻、贪心非最优例、空流、坏引用、资源上限、完整 int32 时间跨度。600 组固定种子小数据用独立全扫描 oracle 核对二分选择、数量守恒及排列不变性。

这是无 ROS 算法测试，不代表真实图像硬件同步或录包验收。

## 零容差专项回归

新增 strictEquality：真实零时刻有效；同刻多来源仍一对一，不复用 Depth；未知/不同 clock 即使 stamp 相同也拒绝。BagBuilder 仅用零容差，通用核心 API 的既有近似策略回归保留兼容。
