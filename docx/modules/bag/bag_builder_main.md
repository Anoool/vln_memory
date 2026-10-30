# bag_builder_main.cpp

[实现](../../../src/apps/bag_builder_main.cpp) 是薄命令入口，只加载 YAML、覆盖路径、调用 buildBag、显示结果，不复制业务算法，也不创建在线 ROS 节点。

## 运行

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vln_memory bag_builder --config /absolute/path/vln_memory.yaml \
  --bag /absolute/path/recording --output /absolute/path/new_dataset
```

也可运行构建目录中的 bag_builder。--config 必需；--bag/--output 可覆盖配置路径；--help 显示语法。输出需不存在。先编辑实际话题、单位、时钟依据和光学 frame，不能直接套用示例语义。

## 返回

显示 status/code/message/output_directory/work_directory、图像/配对/问题数量。成功（包含 completed_with_issues）退出 0；构建失败退出 1；参数/加载错误退出 2。不要只凭退出 0 推断配对全部可用，应检查 prepared_pairs 与报告。

命令存在 rosbag2 日志，不能把全部 stdout 当一份纯 JSON。C++/其他模块应直接调用 buildBag 或读取文件契约。
