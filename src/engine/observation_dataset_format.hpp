#pragma once

#include <array>
#include <utility>

namespace vln_memory::detail
{
inline constexpr char kObservationDatasetManifest[] = "observation_manifest.json";
inline constexpr char kBundledPreparation[] = "preparation";
inline constexpr char kObservationDatasetRows[] = "observations.jsonl";
inline constexpr char kObservationDatasetBatches[] = "batches.jsonl";
inline constexpr char kObservationDatasetVersion[] = "observation_dataset/2";
inline constexpr char kObservationDatasetLegacyVersion[] = "observation_dataset/1";
inline constexpr char kDirectObservationVersion[] = "observation_dataset/3";
inline constexpr char kDirectObservationManifest[] = "evidence/observation_manifest.json";
inline constexpr char kDirectObservationSource[] = "evidence/source.json";
inline constexpr char kDirectObservationMediaIndex[] = "evidence/media_sha256.jsonl";
// 清单的逻辑字段及其固定相对文件名；顺序与读取器快照一致。
inline constexpr std::array<std::pair<const char *, const char *>, 4> kObservationDatasetFiles{{
  {"observations", kObservationDatasetRows}, {"batches", kObservationDatasetBatches},
  {"selection", "build_input_manifest.json"}, {"issues", "reports/issues.jsonl"}
}};
}  // namespace vln_memory::detail
