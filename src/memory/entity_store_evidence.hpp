#pragma once

#include <string>
#include <string_view>

#include <vln_memory/memory/entity.hpp>

namespace vln_memory::detail
{

/** @brief 编码当前实体类型的唯一证据，缺项为 JSON null。 */
std::string encodeEntityEvidence(const EntityObservationRef & ref);

/** @brief 按实体类型解码拥有型证据，损坏的字段抛出异常。 */
void decodeEntityEvidence(std::string_view payload, EntityKind kind, EntityObservationRef & ref);

/** @brief 判断此条证据是否携带地图定位点，调用方先验证证据。 */
bool hasEntityLocation(const EntityObservationRef & ref) noexcept;

/** @brief 校验单条观测证据，几何结果独立于实体当前的定位状态。 */
bool validEntityEvidence(EntityKind kind, const EntityObservationRef & ref, std::uint32_t width, std::uint32_t height);

}  // namespace vln_memory::detail
