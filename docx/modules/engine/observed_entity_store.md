# 从正式观测写入实体工作库

公开头为 [observed_entity_store.hpp](../../../include/vln_memory/engine/observed_entity_store.hpp)，链接 `vln_memory::vln_memory_observed_entity_store`。实体创建还需链接 `vln_memory::vln_memory_entity_builder`。

1. 用 `openObservationDataset` 打开正式观测目录。工作库写入接受校验通过的 BagBuilder 直出目录或 v2 观测集。
2. 按观测 ID 回读。旧 v2 使用 `openSourcePreparation` 核对来源，直出目录从自身 `evidence/` 读取。再用 [物品或空间创建接口](entity_builder.md) 生成实体。
3. 用 `createObservedEntityStore(db_path, *dataset.reader)` 创建工作库，或用 `openEntityStore(db_path)` 打开原库。
4. 调用 `insertObservedEntity(*opened.store, *dataset.reader, *built.entity)`。保存前按实体引用再次回读正式观测，比较观测 ID、修订、来源和地图上下文。
5. 关闭后可用 `openEntityStore` 和 `loadEntity(entity_id)` 读回实体。详细存储契约见 [实体工作库](../memory/entity_store.md)。

给已有实体追加一帧时，先用 `loadEntity` 获取当前 revision，再从同一观测集取得下一条正式观测及该条证据。证据可以只包含观测 ID、观测 revision 和来源指纹，也可以包含该帧的实测深度选区或空间内部确认。用原实体 ID 提交：

```cpp
auto appended = vln_memory::appendObservedEvidence(
  *opened.store, *dataset.reader, current.entity->entity_id, current.entity->revision, evidence);
```

追加成功后，`loadEntity` 按加入顺序返回全部证据。读取实体当前位置时，用 `location_observation` 的观测 ID 和修订定位对应证据。若返回 `revision_conflict`，先重新读取实体再决定是否追加。

需要让待定位实体使用某条已关联证据时，从 `loadEntity` 返回的观测列表中选出候选，并传入当前实体 revision。成功后用 `loadEntity` 读取新的状态和定位引用。

```cpp
const auto & candidate = current.entity->observations.at(1);
auto resolved = vln_memory::resolveEntityLocation(
  *opened.store, *dataset.reader, current.entity->entity_id, current.entity->revision,
  {candidate.observation_id, candidate.observation_revision});
```

保持清单字节原样时，可移动整个观测目录。引用模式下可给 `openSourcePreparation` 传入新的 preparation 路径。重写或重新发布观测清单会改变绑定身份。

ROS 夹具使用已发布的便携观测集验证物品和空间从第二帧补定位。重开后保留原 ID 和两条观测，实体 revision 为 3，定位依据指向第二帧。测试程序的 `--published DATASET U V DB_PARENT` 模式可以对真实短包复验物品补定位，需要至少两条观测，`U,V` 是前两条观测中均有效的深度原图像素。数值位置仍以该录包的标定、尺度及历史轨迹为依据。
