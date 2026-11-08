#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

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
  already_linked, revision_conflict, evidence_conflict
};

struct EntityStoreResult
{
  EntityStoreCode code{EntityStoreCode::storage_error};
  std::string message;
  /** @brief ready 表示操作成功，already_linked 表示相同证据已存在。 */
  bool ok() const noexcept {return code == EntityStoreCode::ready || code == EntityStoreCode::already_linked;}
};

struct EntityStoreLoadResult : EntityStoreResult
{
  std::optional<Entity> entity;
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

/** @brief 打开既有工作库，验证绑定并事务升级 v1/v2，缺失路径不创建文件。 */
EntityStoreOpenResult openEntityStore(const std::filesystem::path & path);

}  // namespace vln_memory
