#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <vln_memory/bag/depth/depth_region_projection.hpp>

namespace vln_memory
{

enum class EntityKind {object, region};
enum class EntityStatus {pending_location, located};

/** @brief 深度原图选区、求解参数及可回查的实测表面点；未求解时投影码为空。 */
struct ObjectSelectionEvidence
{
  DepthRegion selection;
  DepthRegionOptions options;
  std::optional<DepthRegionCode> projection_code;
  std::size_t region_pixels{0};
  std::size_t valid_points{0};
  std::size_t leading_cluster_points{0};
  std::optional<DepthRegionPoint> representative;
};

/** @brief 确认在区域内部的底盘原点，使用观测时刻和米制地图坐标。 */
struct RegionBodyAnchor
{
  std::string body_frame;
  SensorStamp stamp;
  std::array<double, 3> point_map_m{};
};

/** @brief 上游确认依据的引用；缺底盘位姿时仍保留引用。 */
struct RegionInteriorEvidence
{
  std::string confirmation_ref;
  std::optional<RegionBodyAnchor> body_anchor;
};

/** @brief 引用确切的观测修订和 preparation；几何证据按实体类型保存。 */
struct EntityObservationRef
{
  std::string observation_id;
  std::uint64_t observation_revision{0};
  std::string source_manifest_sha256;
  std::optional<ObjectSelectionEvidence> object_selection;
  std::optional<RegionInteriorEvidence> region_interior{};
  std::optional<std::string> requested_selector_json{};
};

/** @brief 选中作为定位依据的观测键，指向当前实体自己的证据列表。 */
struct EntityLocationRef
{
  std::string observation_id;
  std::uint64_t observation_revision{0};
};

/**
 * @brief 业务实体记录，ID 独立于语义、观测和地图版本，创建时 revision 为 1。
 * @note object 代表点来自深度选区；region 代表点来自确认在区域内部的历史底盘位姿。
 */
struct Entity
{
  std::string entity_id;
  EntityKind kind{EntityKind::object};
  std::string semantic_label;
  EntityStatus status{EntityStatus::pending_location};
  std::uint64_t revision{1};
  std::string map_id;
  std::string map_frame;
  std::string geometry_revision;
  // 工作库按加入顺序返回，首条为创建时的观测。
  std::vector<EntityObservationRef> observations;
  std::optional<std::string> location_pending_reason;
  std::optional<EntityLocationRef> location_observation{};
  std::optional<std::string> region_id{};  // 由已保存的 inside 关系读出。
  std::string details_json{"{}"};
};

/** @brief 从选中的证据取得代表点，未定位时返回空。 */
std::optional<std::array<double, 3>> selectedEntityPoint(const Entity & entity);

/** @brief 分配独立的 128 位随机实体键；随机源失败时返回空，不生成替代键。 */
std::optional<std::string> allocateEntityId();

}  // namespace vln_memory
