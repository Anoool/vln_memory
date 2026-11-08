# SQLite 实体工作库

公开头为 [entity_store.hpp](../../../include/vln_memory/memory/entity_store.hpp)，链接 `vln_memory::vln_memory_entity_store`。该库在 ROS 和无 ROS 构建中可用。同一连接由调用方串行访问，SQLite 写事务维护实体及证据的一致性。

| 接口 | 行为 |
| --- | --- |
| `createEntityStore(path, context)` | 在新路径创建数据库并写入观测集绑定，已有路径返回 `already_exists`。 |
| `openEntityStore(path)` | 检查已有数据库的应用标记、schema 版本和绑定，事务升级 v1 工作库。 |
| `insertEntity(entity, context, observation)` | 比较已经验证的观测集绑定和正式观测，原子保存初始实体及其证据。常规写入走 [观测集接入](../engine/observed_entity_store.md)。 |
| `appendEntityObservation(id, expected_revision, evidence, context, observation)` | 向已有实体追加一条核验后的证据，提交时递增实体 revision。 |
| `loadEntity(id)` | 按关联顺序读回独立持有的实体和全部证据，未知 ID 返回 `not_found`。查看原始图像时再打开正式观测数据集。 |

`store_meta` 保存观测清单 SHA-256、preparation 指纹、地图 ID、frame 和几何版本。`entities` 按永久实体 ID 保存类型、标签、修订、定位状态及待定位原因。`entity_observations` 以外键关联实体，保存正式观测 ID、完整无符号修订、来源指纹和类型对应的证据 JSON。源图像仍在 preparation 中。

当前 schema 为 2。打开 v1 库时在事务中新增 `linked_at_revision`，旧记录的首条观测记为 `1`。实体创建仍从 revision `1` 和一条观测开始，之后每次成功追加一条观测，实体 revision 加一。回读检查关联序号与实体修订一致。

物品证据可保存深度原图选区、投影参数、质量计数和实测代表点。空间证据可保存内部确认引用和原始底盘锚点。每条追加证据单独保存其几何结果，首条观测仍是实体当前定位状态的依据。待定位实体收到后续几何证据时，保留当前的待定位状态和原因。

`dataset_mismatch` 表示观测集绑定不同，`invalid_reference` 表示引用、修订、来源或地图上下文对不上，`invalid_entity` 表示实体或证据结构不合法。读取遇到错误的 schema、断引用或损坏的证据会失败。重复实体 ID 不覆盖旧记录。单笔写入在两张业务表都成功后才提交。

追加时先检查 `expected_revision`。过期返回 `revision_conflict`。版本匹配时，相同引用及相同证据返回成功的 `already_linked`，同一引用携带不同证据返回 `evidence_conflict`。这三种结果均保留现有证据和实体 revision。

核心回归覆盖 v1 迁移、创建和追加回滚、旧修订、重复和冲突证据、错来源、错几何版本和重开回读。追加回归在新关联写入后注入版本更新错误，验证旧修订及原有观测集合仍可读。
