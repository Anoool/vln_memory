# test_bag_output.py

[源码](../../../../test/bag/builder/test_bag_output.py) 对应 CTest bag_output_numpy，依赖 bag_builder_ros fixture。Python 标准 JSON/hashlib 和 NumPy 独立读取 C++ 产物，不调用 C++ 写入代码。

检查 schema/来源/TF 端点引用、每个原始 CDR 的 SHA-256 与字节数，NPY 的二维 shape、小端 float32、1/2/3 米与 NaN。两个相同输入导出的全部文件逐字节比较，锁定确定性。依赖 python3-numpy，仅用于测试。
