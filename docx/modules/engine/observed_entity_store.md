# 从正式观测写入实体工作库

公开头为 [observed_entity_store.hpp](../../../include/vln_memory/engine/observed_entity_store.hpp)，链接 `vln_memory::vln_memory_observed_entity_store`。实体创建还需链接 `vln_memory::vln_memory_entity_builder`。

1. 用 `openObservationDataset` 打开正式观测目录。工作库写入使用具备文件摘要且校验通过的 v2 观测集。
2. 用 `openSourcePreparation` 核对来源。按观测 ID 回读，再用现有 [物品或空间创建接口](entity_builder.md) 生成实体。
3. 用 `createObservedEntityStore(db_path, *dataset.reader)` 创建工作库，或用 `openEntityStore(db_path)` 打开原库。
4. 调用 `insertObservedEntity(*opened.store, *dataset.reader, *built.entity)`。保存前按实体引用再次回读正式观测，比较观测 ID、修订、来源和地图上下文。
5. 关闭后可用 `openEntityStore` 和 `loadEntity(entity_id)` 读回实体。详细存储契约见 [实体工作库](../memory/entity_store.md)。

保持清单字节原样时，可移动整个观测目录。引用模式下可给 `openSourcePreparation` 传入新的 preparation 路径。重写或重新发布观测清单会改变绑定身份。

ROS 夹具使用已发布的便携观测集验证物品和经确认的空间写入及重开回读。测试程序的 `--published DATASET U V DB_PARENT` 模式可以对真实短包复验物品和未确认空间，`U,V` 是有效深度原图像素。数值位置仍以该录包的标定、尺度及历史轨迹为依据。
