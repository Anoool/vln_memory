# test_bag_builder.cpp

[源码](../../../../test/bag/builder/test_bag_builder.cpp) 对应 CTest bag_builder_ros。使用官方 rosbag2 Writer 和真实 ROS 类型支持，将 CDR 写入磁盘 SQLite；用未选择的大消息强制分卷，再调用实际 buildBag。

## 验证内容

RGB 行填充/色彩、uint16 毫米转换、NaN/全无效、RGB-D 正负 4ms 差值、未配对、重复记录时间、负 bag 时间、迟到静态/动态 TF、重复广播和空 TFMessage。检查所有原始 CDR 引用及不虚构位姿。

独立变体覆盖错误消息类型、静态模型变化、TF 冲突、图像时间重启、容量失败、旧结果覆盖拒绝、跨遍源文件改变、单文件范围标记和配置未知/重复/负容量。

## 产物与范围

每次在 build 下独立 test_artifacts/run_<计时值> 创建录包和数据集，latest.txt 指向最近成功用例。demo_config.yaml 可直接运行 CLI，输出应选新的目录。生成物不入 Git。

这些是磁盘格式/库的构造回归，不是实际设备录制；未验证物理同步、标定精度或 map 数值位姿。
