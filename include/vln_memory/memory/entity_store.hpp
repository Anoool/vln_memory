#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <vln_memory/memory/entity.hpp>

namespace vln_memory
{

struct Observation;

/** @brief 工作库绑定一份发布的正式观测及其地图上下文，路径不参与身份。 */
struct EntityStoreContext
{
  std::string dataset_manifest_sha256;
  std::string source_manifest_sha256;
  std::string map_id;
  std::string map_frame;
  std::string geometry_revision;
};

enum class EntityStoreCode
{
  ready, already_exists, not_found, invalid_store, invalid_entity, invalid_reference,
  dataset_mismatch, observation_read_failed, storage_error,
  already_linked, revision_conflict, evidence_conflict, location_unavailable, already_located
};

struct EntityStoreResult
{
  EntityStoreCode code{EntityStoreCode::storage_error};
  std::string message;
  /** @brief ready 表示操作成功，already_linked 表示相同证据已存在。 */
  bool ok() const noexcept {return code == EntityStoreCode::ready || code == EntityStoreCode::already_linked;}
};

struct NearRelationResult : EntityStoreResult
{
  double distance_m{0.0};
  bool related{false};
};

struct EntityStoreLoadResult : EntityStoreResult
{
  std::optional<Entity> entity;
};

/** @brief 关系端点按语义顺序保存，near 和 connects_regions 的对称端点统一排序。 */
struct EntityRelation
{
  std::string type;
  std::vector<std::string> entity_ids;
  std::vector<std::string> evidence_observation_ids;
  std::string basis{"model_observed"};
  std::string geometry_revision;
  std::string recognition_state{"supported"};
  std::uint64_t revision{1};
  std::optional<double> distance_m;
  std::optional<double> threshold_m;
  std::string dimension;
  std::string evidence_ref;
};

/** @brief 请求与结果一起保存，便于检查重放是否确实对应同一调用。 */
struct ToolCallRecord
{
  std::string request_json;
  std::string result_json;
};

struct EntityStoreOpenResult;
struct EntityStoreState;

/** @brief 单连接工作库。同一实例的访问由调用方串行化。 */
class EntityStore
{
public:
  /** @brief 关闭数据库连接，已返回的实体由调用方独立持有。 */
  ~EntityStore();

  /** @brief 返回数据库中保存的观测集绑定，引用在实例存活期间有效。 */
  const EntityStoreContext & context() const noexcept;

  /**
   * @brief 原子写入初始实体及其唯一观测引用，重复实体 ID 返回冲突。
   * @param context 由上游验证的观测集绑定。
   * @param observation 上游从该观测集校验后读出的记录。
   * @note 文件读取和几何计算属于上游。常规入口为 insertObservedEntity。
   */
  EntityStoreResult insertEntity(
    const Entity & entity, const EntityStoreContext & context, const Observation & observation);

  /**
   * @brief 给已有实体追加一条证据，在同一事务中递增实体 revision。
   * @param expected_revision 调用方读到的实体 revision，过期时返回冲突。
   * @param evidence 上游基于已校验观测构造的类型证据。
   * @param context 已校验的观测集绑定。
   * @param observation 从该观测集回读的正式观测。
   * @note 追加时保留已选定位依据。常规入口为 appendObservedEvidence。
   */
  EntityStoreResult appendEntityObservation(
    std::string_view entity_id, std::uint64_t expected_revision, const EntityObservationRef & evidence,
    const EntityStoreContext & context, const Observation & observation);

  /**
   * @brief 用已关联的几何证据补定位，原子更新状态、定位依据和实体修订。
   * @param expected_revision 调用方读到的实体修订，过期时返回冲突。
   * @param selected 本实体中作为定位依据的观测 ID 与修订。
   * @param context 已校验的观测集绑定。
   * @param observation 从该观测集回读的选中观测。
   */
  EntityStoreResult resolveEntityLocation(
    std::string_view entity_id, std::uint64_t expected_revision, const EntityLocationRef & selected,
    const EntityStoreContext & context, const Observation & observation);

  /** @brief 保存有确切观测证据的物品归属。重复归属可重试，已有不同归属返回冲突。 */
  EntityStoreResult recordInside(
    std::string_view object_id, std::string_view region_id,
    const EntityLocationRef & observation, std::string_view evidence_ref);

  /** @brief 用双方已选的地图点测量三维欧氏距离，按阈值更新一条对称关系。 */
  NearRelationResult recordNear(std::string_view first_id, std::string_view second_id, double threshold_m);

  /** @brief 原子执行多项实体与关系写入，回调任一步失败时回滚。 */
  EntityStoreResult writeAtomically(const std::function<EntityStoreResult()> & write);

  /** @brief 在指定修订上更新标签和 JSON 元数据。 */
  EntityStoreResult updateEntityFields(
    std::string_view entity_id, std::uint64_t expected_revision,
    std::string_view label, std::string_view details_json, bool relation_changed = false);

  /** @brief 按永久 ID 顺序列出工作库实体。 */
  std::vector<std::string> listEntityIds() const;

  /** @brief 写入有证据的语义关系，原子维护区域归属。 */
  EntityStoreResult recordRelation(EntityRelation relation);

  /** @brief 读取指定关系及其状态、证据和数值距离。 */
  std::optional<EntityRelation> findRelation(std::string_view type, std::vector<std::string> entity_ids) const;

  /** @brief 保存未定位的新实体请求及证据，供后续复核。 */
  EntityStoreResult savePending(std::string_view pending_id, std::string_view request_json);

  /** @brief 读取一条待定位请求，或按写入顺序倒序读取最近四条。 */
  std::optional<std::string> findPending(std::string_view id) const;
  std::vector<std::pair<std::string, std::string>> recentPending() const;

  /** @brief 正式实体写入后，在同一事务中移除待定位请求。 */
  EntityStoreResult removePending(std::string_view id);

  /** @brief 读取已经提交的工具调用。 */
  std::optional<ToolCallRecord> findToolCall(std::string_view call_id) const;

  /** @brief 在调用方写事务内记录调用结果，和业务修改一起提交。 */
  void insertToolCall(std::string_view call_id, const ToolCallRecord & record);

  /** @brief 回读整个批次恢复点，首次运行时为空。 */
  std::optional<std::string> loadBuildProgress() const;

  /** @brief 只在恢复点仍等于 expected 时更新，避免并行进程推进同一批次。 */
  EntityStoreResult saveBuildProgress(
    const std::optional<std::string> & expected, std::string_view progress_json);

  /** @brief 按原 ID 读取拥有型实体，证据按关联顺序排列，无需访问观测目录。 */
  EntityStoreLoadResult loadEntity(std::string_view entity_id) const;

private:
  /** @brief 接管已完成 schema 和上下文检查的 SQLite 连接。 */
  explicit EntityStore(std::unique_ptr<EntityStoreState> state);
  std::unique_ptr<EntityStoreState> state_;
  friend EntityStoreOpenResult createEntityStore(const std::filesystem::path &, const EntityStoreContext &);
  friend EntityStoreOpenResult openEntityStore(const std::filesystem::path &);
};

struct EntityStoreOpenResult : EntityStoreResult
{
  std::unique_ptr<EntityStore> store;
};

/** @brief 在新路径创建并绑定工作库，不覆盖已有文件。 */
EntityStoreOpenResult createEntityStore(const std::filesystem::path & path, const EntityStoreContext & context);

/** @brief 打开既有工作库，验证绑定并事务升级旧版 schema，缺失路径不创建文件。 */
EntityStoreOpenResult openEntityStore(const std::filesystem::path & path);

}  // namespace vln_memory
