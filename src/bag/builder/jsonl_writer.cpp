#include <vln_memory/bag/builder/jsonl_writer.hpp>

#include <cmath>

#include "preparation_metadata.hpp"

namespace vln_memory
{
namespace
{
/** @brief 检查数组非有限值，原始位模式对照需要源消息。 */
template<typename Values>
bool nonfinite(const Values & values)
{
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      return true;
    }
  }
  return false;
}

/** @brief 端点引用保留查询所依赖的源消息和条目，不重复嵌入整张 TF 历史。 */
nlohmann::json endpointJson(const TfTransform & transform)
{
  return {{"message_id", transform.source_message_id}, {"transform_index", transform.source_transform_index},
    {"stamp", stampJson(transform.stamp)}, {"topic", transform.source_topic}};
}
}

/** @brief 序列化原始 int32 秒及 uint32 纳秒，JSON 数字可精确表示其范围。 */
nlohmann::json stampJson(const SensorStamp & stamp)
{
  return {{"sec", stamp.sec}, {"nanosec", stamp.nanosec}};
}

/** @brief 64 位时间和序号用十进制字符串，避免 JavaScript 消费丢精度。 */
nlohmann::json sourceJson(const BagMessageRef & source)
{
  nlohmann::json record{{"message_id", source.message_id}, {"topic", source.topic}, {"file_index", source.file_index},
    {"topic_ordinal", std::to_string(source.topic_ordinal)}, {"message_order", std::to_string(source.message_order)},
    {"bag_record_time_ns", std::to_string(source.bag_record_time_ns)}, {"serialized_bytes", source.serialized_bytes},
    {"payload_sha256", source.payload_sha256}, {"raw_path", nullptr}};
  if (source.raw_cdr_saved) {record["raw_path"] = "evidence/raw/" + source.message_id + ".cdr";}
  return record;
}

/** @brief 输出未经重写的标定字段，校验失败不会被默默删除。 */
nlohmann::json cameraInfoJson(const BagCameraInfoRecord & record)
{
  const auto & data = record.data;
  const auto & roi = data.roi;
  return {{"calibration_id", record.source.message_id}, {"source", sourceJson(record.source)},
    {"stamp", stampJson(data.stamp)}, {"frame_id", data.frame_id}, {"width", data.width}, {"height", data.height},
    {"distortion_model", data.distortion_model}, {"d", data.d}, {"k", data.k}, {"r", data.r}, {"p", data.p},
    {"binning_x", data.binning_x}, {"binning_y", data.binning_y},
    {"roi", {{"x_offset", roi.x_offset}, {"y_offset", roi.y_offset}, {"width", roi.width},
        {"height", roi.height}, {"do_rectify", roi.do_rectify}}},
    {"validation_code", cameraInfoErrorCodeName(record.validation_code)},
    {"validation_message", record.validation_message},
    {"compatibility", cameraInfoCompatibilityName(record.compatibility)},
    {"normalization_applied", record.normalization_applied},
    {"normalized_code", cameraInfoErrorCodeName(record.normalized_code)},
    {"effective_d", record.normalized ? nlohmann::json(record.normalized->d) : nlohmann::json(nullptr)},
    {"source_nonfinite", nonfinite(data.d) || nonfinite(data.k) || nonfinite(data.r) || nonfinite(data.p)}};
}

/** @brief 输出时间覆盖和原始端点引用。 */
nlohmann::json tfTimeJson(const TfTimeCheck & check)
{
  nlohmann::json edges = nlohmann::json::array();
  for (const auto & edge : check.edges) {
    nlohmann::json item{{"inverted", edge.inverted}, {"before", endpointJson(edge.before)}};
    item["after"] = nullptr;
    if (edge.after) {item["after"] = endpointJson(*edge.after);}
    edges.push_back(std::move(item));
  }
  return {{"code", tfTimeCodeName(check.code)}, {"stamp", stampJson(check.stamp)},
    {"chain_code", tfChainCodeName(check.chain_code)}, {"failed_frame", check.failed_frame},
    {"message", check.message}, {"frames", check.frames}, {"edges", edges}, {"pose_computed", false}};
}

/** @brief 复用时间诊断并输出独立的数值状态和可选位姿。 */
nlohmann::json tfPoseJson(const TfPoseResult & result)
{
  auto record = tfTimeJson(result.time);
  record["pose_code"] = tfPoseCodeName(result.code);
  record["pose_failed_frame"] = result.failed_frame;
  record["pose"] = nullptr;
  const auto & frames = result.time.frames;
  if (result.pose && !frames.empty()) {
    record["pose_computed"] = true;
    record["pose"] = {{"target_frame", frames.back()}, {"source_frame", frames.front()},
      {"translation_m", result.pose->translation_m}, {"rotation_xyzw", result.pose->rotation_xyzw}};
  }
  return record;
}

/** @brief 稳定字段映射，不要求调用方解析自由文本。 */
nlohmann::json issueJson(const BagIssue & issue)
{
  return {{"stage", issue.stage}, {"code", issue.code}, {"message", issue.message},
    {"message_id", issue.message_id}, {"pair_id", issue.pair_id}};
}

/** @brief 只接收对象型业务记录，补版本后写一行，dump 保证字符串转义。 */
void writeJsonLine(OutputFiles & files, const std::string & path, nlohmann::json record)
{
  if (!record.is_object()) {throw BagBuildError("invalid_output", "JSONL record must be an object");}
  record["schema_version"] = "bag_preparation/1";
  const auto line = record.dump();
  if (line.size() > detail::kMaxPreparationJsonBytes) {
    throw BagBuildError("limit_exceeded", "JSONL record exceeds preparation reader limit: " + path);
  }
  files.appendLine(path, line);
}

/** @brief 整体 JSON 先编码再写临时文件，失败不产生已发布文件。 */
void writeJsonFile(OutputFiles & files, const std::string & path, nlohmann::json record)
{
  if (!record.is_object()) {throw BagBuildError("invalid_output", "JSON document must be an object");}
  if (!record.contains("schema_version")) {record["schema_version"] = "bag_preparation/1";}
  const auto content = record.dump(2) + "\n";
  if (content.size() > detail::kMaxPreparationJsonBytes) {
    throw BagBuildError("limit_exceeded", "JSON document exceeds preparation reader limit: " + path);
  }
  files.write(path, reinterpret_cast<const std::uint8_t *>(content.data()), content.size());
}

}  // namespace vln_memory
