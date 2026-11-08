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

/** @brief 检查证据形状、质量计数、坐标和状态的一致性。 */
bool validEntityEvidence(const Entity & entity, std::uint32_t width, std::uint32_t height);

}  // namespace vln_memory::detail
