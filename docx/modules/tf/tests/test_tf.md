# test_tf.cpp：坐标链与时间覆盖验证

源码 test/bag/tf/test_tf.cpp；CTest 名 tf_core，只链接无 ROS 的 vln_memory_tf。使用可运行的异常检查，不依赖可能被 NDEBUG 关闭的 assert。

## 规划与覆盖

1. 先验证明确的 map/odom/base/camera 路径、反向求逆标志、兄弟节点的最近共同祖先、已知 identity、未知与断开的树，以及数据所有权。
2. 手写有时间空洞的两条动态边，核对公共区间 [2,3]/[10,11]、精确端点、插值前后样本、超界与空洞、静态时间语义。
3. 验证 parent/kind/static/same-stamp 冲突、闭环与节点阻断，确保没有 first/last 消息自动覆盖，且独立树仍可查询；测试唯一样本/边上限及重复广播。
4. 校验坏 frame、stamp、平移、四元数、来源、scope/limits，核对真实零时刻、负时间和完整 int32 秒跨度，避免整数溢出。
5. 200 组固定种子的生成用例，每组 33 个时刻，通过独立的离散样本 oracle 核对逐时刻查询与区间交集，预期值不来自被测区间算法。

## 运行与边界

```bash
ctest --test-dir build/tf-core -R tf_core --output-on-failure
```

核心 5/5 回归中包含此项，真实 ROS 模式执行同一核心。它证明索引契约和覆盖运算，不证明真实机器人 TF 录制完整，也不验证尚未实现的位姿组合。接口见 [tf](../tf.md)。

## 2026-10-30 可读性迁移

已迁移扁平来源字段和查询配置，原断言预期保留；核心另核对报告上下文、来源所有权及首个错误顺序。当前验证结果统一记录在 [TF 可读性整理](../../../project/tf_readability_plan.md)。
