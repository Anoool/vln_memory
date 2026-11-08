#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <vln_memory/bag/builder/preparation_reader.hpp>
#include <vln_memory/memory/observation.hpp>

namespace vln_memory
{

enum class ObservationDatasetReadCode
{
  ready, unsupported_dataset, invalid_manifest, invalid_observation, invalid_batch,
  integrity_mismatch, observation_not_found, batch_not_found, source_changed, io_error
};

/** @brief 打开时核对的来源、地图和观测版本；preparation_root 为清单中的路径。 */
struct ObservationDatasetInfo
{
  std::filesystem::path preparation_root;
  std::string source_manifest_sha256;
  std::string map_id;
  std::string geometry_revision;
  std::string map_frame;
  std::string map_basis;
  std::uint64_t revision{0};
  std::size_t observation_count{0};
  std::size_t pending_count{0};
  bool file_integrity_verified{false};
  bool preparation_bundled{false};
};

/** @brief 一组按发布顺序排列的正式观测 ID，批次从 0 开始。 */
struct ObservationDatasetBatch
{
  std::size_t batch_index{0};
  std::vector<std::string> observation_ids;
};

struct ObservationDatasetLoadResult
{
  ObservationDatasetReadCode code{ObservationDatasetReadCode::io_error};
  std::string message;
  std::optional<Observation> observation;
  /** @brief 成功包含独立持有的记录，合法 pending 也可成功。 */
  bool ok() const noexcept {return code == ObservationDatasetReadCode::ready && observation.has_value();}
};

struct ObservationDatasetBatchResult
{
  ObservationDatasetReadCode code{ObservationDatasetReadCode::io_error};
  std::string message;
  std::vector<Observation> observations;
  /** @brief 批次完整读回才成功，任一记录错误则返回空列表。 */
  bool ok() const noexcept {return code == ObservationDatasetReadCode::ready;}
};

struct ObservationDatasetReaderState;
struct ObservationDatasetOpenResult;

/** @brief 只读观测目录索引，使用独立文件流按 ID 或批次回读。 */
class ObservationDatasetReader
{
public:
  /** @brief 释放索引，已返回的观测继续独立持有。 */
  ~ObservationDatasetReader();

  /** @brief 返回已核对的清单信息，引用在 reader 存活期间有效。 */
  const ObservationDatasetInfo & info() const noexcept;

  /** @brief 返回批次及观测 ID 的发布顺序，引用在 reader 存活期间有效。 */
  const std::vector<ObservationDatasetBatch> & batches() const noexcept;

  /** @brief 按发布的 ID 回读一条观测，复用 observation/1 完整校验。 */
  ObservationDatasetLoadResult loadObservation(std::string_view observation_id) const;

  /** @brief 按批次序号完整回读并校验该批记录。 */
  ObservationDatasetBatchResult loadBatch(std::size_t batch_index) const;

  /**
   * @brief 按清单路径或显式迁移路径打开来源，核对 preparation 指纹、地图 frame 和发布观测引用的配对及原始时间。
   * @note 默认相对路径以观测目录解析，显式相对路径以调用进程的工作目录解析。
   *       绑定只检查索引，不解码媒体。返回的 PreparationReader 由调用方持有。
   */
  PreparationOpenResult openSourcePreparation(const std::filesystem::path & override_root = {}) const;

private:
  /** @brief 接管已完整建立的只读索引。 */
  explicit ObservationDatasetReader(std::unique_ptr<ObservationDatasetReaderState> state);
  std::unique_ptr<ObservationDatasetReaderState> state_;
  friend ObservationDatasetOpenResult openObservationDataset(const std::filesystem::path & root);
};

struct ObservationDatasetOpenResult
{
  ObservationDatasetReadCode code{ObservationDatasetReadCode::io_error};
  std::string message;
  std::unique_ptr<ObservationDatasetReader> reader;
  /** @brief 成功时持有完整索引，失败时 reader 为空。 */
  bool ok() const noexcept {return code == ObservationDatasetReadCode::ready && reader != nullptr;}
};

/**
 * @brief 打开已发布的 v1/v2 正式观测目录，核对摘要、计数、上下文和批次引用并索引观测行。
 * @note 不打开来源 preparation；详细观测语义在 loadObservation/loadBatch 时校验。
 */
ObservationDatasetOpenResult openObservationDataset(const std::filesystem::path & root);

}  // namespace vln_memory
