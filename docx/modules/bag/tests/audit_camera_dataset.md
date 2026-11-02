# audit_camera_dataset.py

[源码](../../../../test/bag/builder/audit_camera_dataset.py) 是当前真实录包的独立 Python 审计入口，不是构造测试。需要加载 ROS Humble Python 环境以及 OpenCV/NumPy。

输入 --db 原始单 SQLite DB、--dataset 完整提取目录、--report 新 JSON 报告路径。只读打开 DB 并核查 SQLite 完整性；按话题/记录时间/CDR SHA 和重复计数逐条比对原始消息。随后反序列化原始 CameraInfo/图像，核对全部原始时间、参数和派生 D5，以矩阵 A*K/A*P 独立计算有效内参，重新解码 JPEG/uint16 PNG 并逐像素比较 RGB PNG/米制 NPY。逐对重算整数纳秒差、一对一和模型引用。

该入口明确面向完整单 DB、plumb_bob 零尾 D8、raw 图像、毫米 16UC1 compressedDepth 的当前录包；不把它当所有设备/配置的通用审计器。任何断言失败均非零退出；不验证物理尺度、曝光同步或注册精度，不做 TF 数值求解。执行结果见 [验收](../../../reviews/camera_model_verification.md)。
