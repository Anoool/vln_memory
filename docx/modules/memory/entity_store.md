# SQLite 实体工作库

公开头为 [entity_store.hpp](../../../include/vln_memory/memory/entity_store.hpp)，链接 `vln_memory::vln_memory_entity_store`。该库在 ROS 和无 ROS 构建中可用。同一连接由调用方串行访问，SQLite 写事务维护实体及证据的一致性。

| 接口 | 行为 |
| --- | --- |
| `createEntityStore(path, context)` | 在新路径创建数据库并写入观测集绑定，已有路径返回 `already_exists`。 |
| `openEntityStore(path)` | 检查已有数据库的应用标记、schema 版本和绑定，事务升级 v1/v2 工作库。 |
| `insertEntity(entity, context, observation)` | 比较已经验证的观测集绑定和正式观测，原子保存初始实体及其证据。常规写入走 [观测集接入](../engine/observed_entity_store.md)。 |
| `appendEntityObservation(id, expected_revision, evidence, context, observation)` | 向已有实体追加一条核验后的证据，提交时递增实体 revision。 |
| `resolveEntityLocation(id, expected_revision, selected, context, observation)` | 将已关联的有效几何证据选为待定位实体的定位依据，同一事务更新状态和实体修订。 |
| `loadEntity(id)` | 按关联顺序读回实体、全部证据与明确的定位观测引用。未知 ID 返回 `not_found`。查看原始图像时再打开正式观测数据集。 |

`store_meta` 保存观测清单 SHA-256、preparation 指纹、地图 ID、frame 和几何版本。`entities` 按永久实体 ID 保存类型、标签、修订、定位状态及待定位原因。`entity_observations` 以外键关联实体，保存正式观测 ID、完整无符号修订、来源指纹和类型对应的证据 JSON。源图像仍在 preparation 中。

当前 schema 为 3。`entities.location_linked_at_revision` 指向本实体的一条观测关联。打开旧库时，在一个事务中升级。v1 首先补齐关联序号，v1 和 v2 的已定位实体都指向原首条证据，待定位实体的定位引用为空。

创建时，已定位实体的引用指向创建所用的观测。旧调用方省略该字段时，工作库从经过验证的唯一证据得到该引用。追加观测保留已选定位依据。坐标、原始图像引用和 provenance 随原证据保存。

`loadEntity` 检查关联序号从 1 开始、严格递增且不超过实体 revision。序号允许有空档，因为补定位单独推进实体版本。已定位实体的引用必须解析到本实体的一条有效几何证据。物品取实测表面点，空间取历史底盘锚点。读取结果中的 `location_observation` 包含确切的观测 ID 和修订。

`dataset_mismatch` 表示观测集绑定不同，`invalid_reference` 表示引用、修订、来源或地图上下文对不上，`invalid_entity` 表示实体或证据结构不合法。读取遇到错误的 schema、断引用或损坏的证据会失败。重复实体 ID 不覆盖旧记录。单笔写入在两张业务表都成功后才提交。

追加时先检查 `expected_revision`。过期返回 `revision_conflict`。版本匹配时，相同引用及相同证据返回成功的 `already_linked`，同一引用携带不同证据返回 `evidence_conflict`。这三种结果均保留现有证据和实体 revision。

补定位先核对观测集绑定和选中观测的修订。取得 SQLite 写锁后检查 `expected_revision`，并按实体 ID 和观测键读回已保存的证据。物品使用有效的实测表面点，并要求正式观测保留深度媒体、存储契约、相机模型和深度相机历史位姿。空间使用内部确认和与正式观测一致的底盘锚点。提交时同时更新定位引用、状态、待定位原因和实体 revision，已关联的证据继续由原观测保存。

过期版本返回 `revision_conflict`。观测不存在于该实体或身份不匹配时返回 `invalid_reference`，几何条件不足时返回 `location_unavailable`，已定位实体返回 `already_located`。

核心回归覆盖旧库迁移、版本和引用错误、几何缺失、状态更新回滚，以及补定位后继续追加观测。真实短包验证了物品从第二帧补定位及关闭后回读。ROS 构造录包还验证了经内部确认的空间选用第二帧的历史底盘锚点。
