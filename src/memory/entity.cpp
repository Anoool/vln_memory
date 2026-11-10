#include <vln_memory/memory/entity.hpp>

#include <array>

#include <openssl/rand.h>

namespace vln_memory
{

/** @brief 生成可保存的 ASCII 实体 ID；后续修订沿用该值。 */
std::optional<std::string> allocateEntityId()
{
  std::array<unsigned char, 16> bytes{};
  if (RAND_bytes(bytes.data(), static_cast<int>(bytes.size())) != 1) {return std::nullopt;}
  constexpr char hex[] = "0123456789abcdef";
  std::string id = "ent_";
  id.reserve(id.size() + bytes.size() * 2);
  for (const auto byte : bytes) {
    id.push_back(hex[byte >> 4]);
    id.push_back(hex[byte & 0x0f]);
  }
  return id;
}

/** @brief 从工作库回读的已校验定位引用取得地图代表点。 */
std::optional<std::array<double, 3>> selectedEntityPoint(const Entity & entity)
{
  if (!entity.location_observation) {return std::nullopt;}
  for (const auto & ref : entity.observations) {
    if (ref.observation_id != entity.location_observation->observation_id ||
      ref.observation_revision != entity.location_observation->observation_revision) {continue;}
    if (ref.object_selection && ref.object_selection->representative) {
      return ref.object_selection->representative->point_map_m;
    }
    if (ref.region_interior && ref.region_interior->body_anchor) {return ref.region_interior->body_anchor->point_map_m;}
  }
  return std::nullopt;
}

}  // namespace vln_memory
