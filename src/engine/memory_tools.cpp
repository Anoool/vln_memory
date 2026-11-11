#include "memory_tools.hpp"

#include "observation_tools.hpp"

#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace vln_memory
{
namespace
{
using Json = nlohmann::json;

/** @brief 生成工具错误，供模型修正参数或由调用方判断是否重试。 */
Json errorResult(const char * status, std::string_view message)
{
  return {{"status", status}, {"message", message}};
}

/** @brief 这些故障应重试原调用，不能成为永久回执。 */
bool retryable(const Json & result)
{
  const auto & status = result.at("status");
  return status == "storage_error" || status == "tool_error" ||
    status == "observation_read_failed" || status == "depth_read_failed";
}

/** @brief 构造对象参数，保留工具中已有的可选字段。 */
Json objectSchema(Json properties, Json required)
{
  return {{"type", "object"}, {"properties", std::move(properties)},
    {"required", std::move(required)}, {"additionalProperties", false}};
}

/** @brief 定义一个可被 Responses 调用的函数。 */
Json functionTool(const char * name, const char * description, Json properties, Json required)
{
  return {{"type", "function"}, {"name", name}, {"description", description}, {"strict", false},
    {"parameters", objectSchema(std::move(properties), std::move(required))}};
}
}  // namespace

/** @brief 持有同一个只读观测索引和工作目录，数据库按需打开。 */
MemoryTools::MemoryTools(const ObservationDatasetReader & dataset, std::filesystem::path root, NearOptions near)
: dataset_(dataset), root_(std::move(root)), near_(std::move(near)) {}

EntityStore & MemoryTools::store()
{
  if (!store_) {
    const auto path = root_ / "entities.sqlite3";
    auto opened = std::filesystem::exists(path) ? openEntityStore(path) : createObservedEntityStore(path, dataset_);
    if (!opened.ok()) {throw std::runtime_error(opened.message);}
    if (opened.store->context().dataset_manifest_sha256 != dataset_.info().dataset_manifest_sha256) {
      throw std::runtime_error("Work store belongs to a different observation dataset");
    }
    store_ = std::move(opened.store);
  }
  return *store_;
}

Json MemoryTools::execute(std::string_view name, const Json & arguments)
{
  try {
    const auto args = arguments.is_string() ? Json::parse(arguments.get<std::string>()) : arguments;
    if (!args.is_object()) {throw std::invalid_argument("arguments must be an object");}
    if (name == "get_observation" || name == "project_depth") {
      return runObservationTool(dataset_, root_, name, args);
    }
    if (name != "search_entities" && name != "create_entity" && name != "update_entity" && name != "spatial_query") {
      return errorResult("unknown_tool", "Unknown memory tool");
    }
    auto result = runEntityTool(store(), dataset_, root_, near_, name, args);
    result["frame_id"] = dataset_.info().map_frame;
    result["map_id"] = dataset_.info().map_id;
    result["geometry_revision"] = dataset_.info().geometry_revision;
    result["memory_version"] = nullptr;
    return result;
  } catch (const Json::exception & error) {
    return errorResult("invalid_request", error.what());
  } catch (const std::invalid_argument & error) {
    return errorResult("invalid_request", error.what());
  } catch (const std::exception & error) {
    return errorResult("tool_error", error.what());
  }
}

Json MemoryTools::call(const Json & request)
{
  try {
    const auto name = request.at("name").get<std::string>();
    auto arguments = request.at("arguments");
    if (!request.contains("call_id")) {return execute(name, arguments);}
    const auto id = request.at("call_id").get<std::string>();
    if (id.find_first_not_of(' ') == std::string::npos || id.size() > 4096 ||
      id.find_first_of("\r\n\t") != std::string::npos)
    {
      return errorResult("invalid_request", "Invalid call_id");
    }
    if (arguments.is_string()) {
      auto parsed = Json::parse(arguments.get<std::string>(), nullptr, false);
      if (!parsed.is_discarded()) {arguments = std::move(parsed);}
    }
    const auto signature = Json{{"name", name}, {"arguments", arguments}}.dump();
    auto & database = store();
    Json output;
    const auto saved = database.writeAtomically([&]() -> EntityStoreResult {
      if (const auto previous = database.findToolCall(id)) {
        if (previous->request_json != signature) {
          output = errorResult("call_id_conflict", "Call ID already used with different arguments");
        } else {
          output = Json::parse(previous->result_json);
        }
        return {EntityStoreCode::ready, ""};
      }
      output = execute(name, arguments);
      if (retryable(output)) {
        return {EntityStoreCode::storage_error, output.value("message", "Retryable tool failure")};
      }
      database.insertToolCall(id, {signature, output.dump(-1, ' ', false, Json::error_handler_t::replace)});
      return {EntityStoreCode::ready, ""};
    });
    if (!saved.ok()) {return errorResult("storage_error", saved.message);}
    return output;
  } catch (const Json::exception & error) {
    return errorResult("invalid_request", error.what());
  } catch (const std::exception & error) {
    return errorResult("tool_error", error.what());
  }
}

Json modelWriteToolDefinitions()
{
  const Json text{{"type", "string"}};
  const auto selector = Json::parse(R"json({
    "type":"object",
    "description":"Coordinates in original RGB image unless image=depth. Use exactly the coordinate field matching type.",
    "properties":{
      "type":{"enum":["point","bbox","polygon"]},
      "image":{"enum":["rgb","depth"]},
      "pixel_xy":{"type":"array","items":{"type":"integer"},"minItems":2,"maxItems":2},
      "xyxy_px":{"type":"array","items":{"type":"integer"},"minItems":4,"maxItems":4},
      "vertices_xy_px":{"type":"array","items":{"type":"array","items":{"type":"integer"},"minItems":2,"maxItems":2}}
    },
    "required":["type"]
  })json");
  const Json strings{{"type", "array"}, {"items", text}};
  const auto view = objectSchema({{"observation_id", text}, {"selector", selector},
    {"confirmation_ref", {{"type", "string"}, {"description", "Evidence that the robot was inside a region"}}}},
    {"observation_id"});
  const Json observations{{"type", "array"}, {"items", view}};
  const auto relation = objectSchema({{"type", {{"enum", {"near", "on", "hanging_on", "inside", "connects_regions"}}}},
    {"entity_ids", strings}, {"evidence_observation_ids", strings}, {"recognition_state", text}},
    {"type", "entity_ids", "evidence_observation_ids"});
  const Json relations{{"type", "array"}, {"items", relation}};
  const Json fields{{"semantic_label", text}, {"description", text}, {"attributes", {{"type", "object"}}},
    {"recognition_state", {{"enum", {"tentative", "supported", "needs_review"}}}}, {"notes", strings}};
  auto create = fields;
  create.update({{"kind", {{"enum", {"object", "region"}}}}, {"add_observations", observations},
    {"relations", relations}});
  return Json::array({
    functionTool("create_entity", "Create an entity and project its selectors locally. Use $self for its relation endpoint.",
      create, {"kind", "semantic_label", "add_observations"}),
    functionTool("update_entity", "Reuse an existing entity ID and add evidence, fields or relations.",
      {{"entity_id", text}, {"expected_revision", {{"type", "integer"}, {"minimum", 1}}},
        {"set_fields", objectSchema(fields, Json::array())}, {"add_observations", observations}, {"relations", relations}},
      {"entity_id", "expected_revision"})
  });
}
}  // namespace vln_memory
