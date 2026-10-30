# media_writer.hpp / media_writer.cpp

## 职责

[头](../../../include/vln_memory/bag/builder/media_writer.hpp)、[实现](../../../src/bag/builder/media_writer.cpp) 提供 OutputFiles 共用字节预算和 RGB/Depth 文件编码。没有读 bag/配对逻辑。

## 接口

OutputFiles(fresh_work_directory, uint64 max_bytes) 要求已存在新建目录。write(relative_path,const uint8_t*,size_t) 同步借用字节、写临时文件并检查关闭后改名；拒绝覆盖、绝对路径、..。ensureLines/appendLine 复用固定 JSONL 流；close 显式检查延迟错误。bytesWritten 是文件内容累计字节，非内存/磁盘配额。

writeRgb(files,message_id,RgbFrame) → 相对 PNG 路径；输入必须紧密 RGB8。显式转换到 OpenCV BGR 再编码，不改原帧，避免红蓝颠倒。

writeDepth(files,message_id,DepthFrame) → 相对 NPY 路径；二维 height×width、C 顺序、小端 IEEE float32、米制 optical-z，NaN 原样保留；不量化毫米、不填洞。按 [NPY 规范](https://numpy.org/doc/stable/reference/generated/numpy.lib.format.html) 写最小单数组格式，不新增通用数组库。

失败抛 BagBuildError，分配/编码库异常由顶层捕获为失败；不能按坏帧忽略写入失败。大数组/编码缓冲仍是当前帧内存，预算不限制编码器内部所有分配。

## 验证

[磁盘回归](tests/test_bag_builder.md) 用 OpenCV 回读红/蓝像素；[独立 NumPy](tests/test_bag_output.md) 检查 dtype/shape、1m/2m/3m 和 NaN。另检查预算失败不发布。

OutputFiles 是串行写入接口，不可从多个线程并发调用；工作目录由本次 builder 独占。
