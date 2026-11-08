# 从正式观测创建实体

公开头为 [entity_builder.hpp](../../../include/vln_memory/engine/entity_builder.hpp)，安装目标为 `vln_memory::vln_memory_entity_builder`。

```cpp
auto result = vln_memory::createObjectEntity(
  *opened.reader, *parsed.observation, "箱子",
  vln_memory::DepthBoundingBox{512, 316, 528, 332});
if (result.ok()) {
  const auto & entity = *result.entity;
  // 按 entity.status 区分已定位和待定位。
}
```

输入为已打开的来源 preparation、正式观测、语义标签和可选深度原图选区。选区使用 [现有投影规则](observation_depth_region.md)。来源、选区和几何核对后，直接取可信簇的实测代表点。返回的选区证据只留代表点和质量计数，因此物品创建把 `include_points` 设为 false。未进入投影的路径通过 Reader 元数据检查配对身份和深度原图网格。

| 情况 | 返回 |
| --- | --- |
| 可信表面簇 | `located`，带实测代表点和选区证据 |
| 未提供选区 | `pending_location` / `missing_selection`，保留观测引用 |
| 缺深度、模型或历史深度位姿 | `pending_location` / `missing_depth`、`missing_depth_model` 或 `missing_depth_pose`，保留已检查的选区 |
| 深度无效、簇支持不足或多簇歧义 | `pending_location`，保留投影状态和质量计数 |
| 标签、观测、来源或选区错误，或读回及投影计算失败 | 返回具体错误，无实体记录 |

## 空间创建

`createRegionEntity(reader, observation, semantic_label, confirmation)` 使用已发布的观测修订及其 preparation。上游提供 `RegionInteriorConfirmation`，声明底盘在目标空间内部，并给出可回查的确认引用，例如对应人工审阅记录的 ID。确认记录绑定目标标签、观测 ID 与修订、preparation 指纹、地图 ID、地图 frame 和几何版本。确认引用由上游解释和核实，创建器核对上述绑定字段。

入口复用 `validateObservation` 重算观测的历史 TF，通过 Reader 元数据模式核对保存配对、原图时刻和消息摘要，并与原始底盘 TF 链、采样端点及数值逐项比较。锚点取 RGB 参考时刻的底盘平移。图像像素无需解码。

| 情况 | 返回 |
| --- | --- |
| 确认匹配，且底盘位姿与来源一致 | `located`，保留确认引用、底盘 frame、原始时刻和地图代表点 |
| 缺确认 | `pending_location` / `missing_region_confirmation`，保留观测引用 |
| 有确认但观测缺底盘位姿 | `pending_location` / `missing_body_map_pose`，保留确认引用 |
| 确认字段不完整或跨目标、观测修订、来源、地图和几何版本 | `invalid_region_confirmation`，无实体记录 |
| 观测校验失败、来源不匹配或读回失败 | 返回对应错误，无实体记录 |

返回值拥有实体数据。可通过[观测集写入接入](observed_entity_store.md)事务保存。重复调用幂等和更新修订由后续业务流程处理。CTest 使用构造录包覆盖物品表面、空间锚点、缺确认或位姿、跨版本及改写位姿。测试程序还可用 `--published PREPARATION OBSERVATIONS` 对已有短包复验物品 bbox/polygon 和空间缺确认状态。
