# test_bag_output.py

[源码](../../../../test/bag/builder/test_bag_output.py) 对应 CTest bag_output_numpy，依赖 bag_builder_ros fixture。Python 标准 JSON/hashlib 和 NumPy 独立读取 C++ 产物，不调用 C++ 写入代码。

检查 schema/来源/TF 端点引用、每个原始 CDR 的 SHA-256 与字节数，NPY 的二维 shape、小端 float32、1/2/3 米与 NaN。两个相同输入导出的全部文件逐字节比较，锁定确定性。依赖 python3-numpy，仅用于测试。

## 零容差输出复核

新增检查每条 pair 的 exact=true、delta_ns="0"、两 stamp 对象相同及策略/采集证据字段；summary 中 approximate_pairs=0，并分别核对有效 RGB/Depth = pairs + unmatched。两次构建仍要求全部产物字节一致。

## 最近配对独立回查

check_nearest_dataset 直接从 CDR Header 按封装端序读取 sec/nanosec，对照 frames/pair；独立计算 delta/exact，检查门限、唯一来源、策略和两流守恒。TF 的 stamp 必须等于各自图像原时间。覆盖原始/压缩夹具、相等对与正门限空结果，不证明硬件同步。
