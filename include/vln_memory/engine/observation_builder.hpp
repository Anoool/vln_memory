#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/memory/observation.hpp>

namespace vln_memory
{

/** @brief 调用方声明的地图归属、几何版本和依据；map_frame 必须与来源一致。 */
struct ObservationBuildContext
{
  std::string map_id;
  std::string geometry_revision;
  std::string map_frame;
  std::string map_basis;
  std::uint64_t revision{1};
  std::vector<std::string> additional_pending_reasons;
};

/** @brief 成功持有独立观测，失败只携带原读取错误或正式记录校验错误。 */
struct ObservationBuildResult
{
  std::optional<PreparationReadCode> preparation_error;
  std::optional<ObservationCode> observation_error;
  std::string message;
  std::optional<Observation> observation;

  /** @brief 合法 pending 也算成功；按实际用途检查观测能力。 */
  bool ok() const noexcept {return observation.has_value();}
};

/**
 * @brief 从已回读配对及显式业务上下文组装、校验单条正式观测。
 * @note 复制来源元数据和相对路径，结果不持有像素、配对或 Reader 引用。
 */
ObservationBuildResult buildObservation(const PreparedPair & pair, const ObservationBuildContext & context);

/**
 * @brief 通过已打开的索引读取一个保存的配对，再组装正式观测。
 * @note 传播 Reader 的具体错误码；调用方可以在多次调用间复用索引。
 */
ObservationBuildResult buildObservation(
  const PreparationReader & reader, std::string_view pair_id, const ObservationBuildContext & context);

}  // namespace vln_memory
