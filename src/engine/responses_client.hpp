#pragma once

#include "model_batches.hpp"

namespace vln_memory
{
/** @brief 向配置的 Responses 端点发送一次非流式请求。 */
nlohmann::json requestResponse(const ModelBuildOptions & options, const nlohmann::json & request);
}  // namespace vln_memory
