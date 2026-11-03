#pragma once

#include <nlohmann/json.hpp>

#include <vln_memory/bag/builder/camera_info_index.hpp>
#include <vln_memory/bag/builder/media_writer.hpp>
#include <vln_memory/bag/tf/tf.hpp>

namespace vln_memory
{

/** @brief 原始整数时间拆分为 sec/nanosec，不转成浮点秒。 */
nlohmann::json stampJson(const SensorStamp & stamp);
/** @brief 来源包含精确十进制 64 位值及原始 CDR 路径，不嵌入消息字节。 */
nlohmann::json sourceJson(const BagMessageRef & source);
/** @brief 完整 CameraInfo 原值与错误；非有限数会写 null 并标记，CDR 保留原证据。 */
nlohmann::json cameraInfoJson(const BagCameraInfoRecord & record);
/** @brief 显式序列化 TF 查询端点来源，无求解位姿字段。 */
nlohmann::json tfTimeJson(const TfTimeCheck & check);
/** @brief 在端点诊断上附加求解状态和可选位姿。 */
nlohmann::json tfPoseJson(const TfPoseResult & result);
/** @brief 稳定问题字段，可供下游按 stage/code 处理。 */
nlohmann::json issueJson(const BagIssue & issue);
/** @brief dump 后一次追加有效 JSON 行；无效 UTF-8 等序列化异常终止发布。 */
void writeJsonLine(OutputFiles & files, const std::string & path, nlohmann::json record);
/** @brief 写完整 JSON 文件；追加 schema_version，使用与媒体一致的字节预算。 */
void writeJsonFile(OutputFiles & files, const std::string & path, nlohmann::json record);

}  // namespace vln_memory
