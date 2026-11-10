#pragma once

#include "entity_tools.hpp"

namespace vln_memory
{
/** @brief JSONL 与模型共用业务分发和 SQLite 调用回执。 */
class MemoryTools
{
public:
  MemoryTools(const ObservationDatasetReader & dataset, std::filesystem::path root, NearOptions near);

  /** @brief 请求可附 call_id，重放时返回首次提交的结果。 */
  nlohmann::json call(const nlohmann::json & request);
  /** @brief 首次使用时打开与观测集绑定的工作库。 */
  EntityStore & store();

private:
  /** @brief 将已经解析的参数交给观测或实体工具。 */
  nlohmann::json execute(std::string_view name, const nlohmann::json & arguments);
  const ObservationDatasetReader & dataset_;
  std::filesystem::path root_;
  NearOptions near_;
  std::unique_ptr<EntityStore> store_;
};

/** @brief Responses 函数定义与现有工具参数保持一致。 */
nlohmann::json memoryToolDefinitions();
}  // namespace vln_memory
