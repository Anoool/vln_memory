# SQLite 实体工作库

公开头为 [entity_store.hpp](../../../include/vln_memory/memory/entity_store.hpp)，链接 `vln_memory::vln_memory_entity_store`。该库在 ROS 和无 ROS 构建中可用。同一连接由调用方串行访问，SQLite 写事务维护实体及证据的一致性。

| 接口 | 行为 |
| --- | --- |
| `createEntityStore(path, context)` | 在新路径创建数据库并写入观测集绑定，已有路径返回 `already_exists`。 |
| `openEntityStore(path)` | 检查已有数据库的应用标记、schema 版本和绑定。路径缺失时不会新建。 |
| `insertEntity(entity, context, observation)` | 比较已经验证的观测集绑定和正式观测，原子保存初始实体及其证据。常规写入走 [观测集接入](../engine/observed_entity_store.md)。 |
| `loadEntity(id)` | 读回独立持有的实体和证据，未知 ID 返回 `not_found`。查看原始图像时再打开正式观测数据集。 |

`store_meta` 保存观测清单 SHA-256、preparation 指纹、地图 ID、frame 和几何版本。`entities` 按永久实体 ID 保存类型、标签、修订、定位状态及待定位原因。`entity_observations` 以外键关联实体，保存正式观测 ID、完整无符号修订、来源指纹和类型对应的证据 JSON。源图像仍在 preparation 中。

当前 schema 为 1，写入接收 revision 为 1 且恰有一条观测引用的实体。已定位物品保存深度原图选区、投影参数、质量计数和实测代表点。已定位空间保存内部确认引用和原始底盘锚点。待定位记录保存原因和已有证据，缺失的点为空。工作库可在关闭后独立回读，实体更新和多视图证据追加属于下一阶段。

`dataset_mismatch` 表示观测集绑定不同，`invalid_reference` 表示引用、修订、来源或地图上下文对不上，`invalid_entity` 表示实体或证据结构不合法。读取遇到错误的 schema、断引用或损坏的证据会失败。重复实体 ID 不覆盖旧记录。单笔写入在两张业务表都成功后才提交。

核心回归注入证据写入故障，验证事务回滚后查不到部分写入的实体，并验证重试、重复 ID、错修订、错来源、错几何版本和重开回读。
