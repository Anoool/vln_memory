# Preparation 目录发布辅助

[preparation_storage.hpp](../../../src/bag/builder/preparation_storage.hpp) 和 [实现](../../../src/bag/builder/preparation_storage.cpp) 是 builder 内部辅助文件，供 BagBuilder 与 preparation_compactor 共用。

containsPath 按路径组件检查包含关系。createWorkDirectory 接收源和目标路径，拒绝路径重叠、既有目标及符号链接目标，创建与目标同文件系统的唯一 partial 目录。

publishDirectory 使用 Linux renameat2 的 RENAME_NOREPLACE 原子改名，拒绝覆盖并发出现的目录。文件写入和校验由调用方在发布前完成。其他平台返回 unsupported_platform，并保留调用方工作目录。

这组辅助随 preparation_reader 库编译。公开调用从 buildBag 或 compactPreparation 进入。
