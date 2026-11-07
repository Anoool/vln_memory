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

}  // namespace vln_memory
