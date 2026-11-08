#include <vln_memory/memory/entity_store.hpp>

#include <algorithm>
#include <charconv>
#include <limits>
#include <stdexcept>
#include <utility>

#include <sqlite3.h>

#include <vln_memory/memory/observation.hpp>

#include "entity_store_evidence.hpp"

namespace vln_memory
{
namespace
{
using Code = EntityStoreCode;
using Db = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;

constexpr int kApplicationId = 0x564c4e45;
constexpr int kSchemaVersion = 3;

constexpr char kSchema[] = R"sql(
  PRAGMA application_id = 0x564c4e45;
  PRAGMA user_version = 3;
  CREATE TABLE store_meta(
    singleton INTEGER PRIMARY KEY CHECK(singleton = 1),
    dataset_manifest_sha256 TEXT NOT NULL, source_manifest_sha256 TEXT NOT NULL,
    map_id TEXT NOT NULL, map_frame TEXT NOT NULL, geometry_revision TEXT NOT NULL
  );
  CREATE TABLE entities(
    entity_id TEXT PRIMARY KEY NOT NULL,
    kind TEXT NOT NULL CHECK(kind IN ('object','region')),
    label TEXT NOT NULL, revision INTEGER NOT NULL CHECK(revision > 0),
    status TEXT NOT NULL CHECK(status IN ('located','pending_location')),
    pending_reason TEXT,
    location_linked_at_revision INTEGER CHECK(location_linked_at_revision > 0 AND location_linked_at_revision <= revision),
    CHECK((status = 'located') = (pending_reason IS NULL))
  );
  CREATE TABLE entity_observations(
    entity_id TEXT NOT NULL REFERENCES entities(entity_id),
    observation_id TEXT NOT NULL, observation_revision TEXT NOT NULL, source_manifest_sha256 TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
    linked_at_revision INTEGER NOT NULL DEFAULT 1 CHECK(linked_at_revision > 0),
    PRIMARY KEY(entity_id, observation_id, observation_revision)
  );
)sql";

struct SqlError : std::runtime_error
{
  int code;
  /** @brief 保留 SQLite 扩展错误码及连接上的诊断。 */
  SqlError(sqlite3 * db, int result)
  : std::runtime_error(db ? sqlite3_errmsg(db) : sqlite3_errstr(result)), code(result) {}
};

/** @brief 检查 SQLite 操作是否完成。 */
void check(sqlite3 * db, int result)
{
  if (result != SQLITE_OK) {throw SqlError(db, result);}
}

/** @brief 执行不含外部数据的固定 SQL。 */
void execute(sqlite3 * db, const char * sql)
{
  check(db, sqlite3_exec(db, sql, nullptr, nullptr, nullptr));
}

class Statement
{
public:
  /** @brief 编译固定 SQL，析构时自动释放。 */
  Statement(sqlite3 * db, const char * sql) : db_(db)
  {
    sqlite3_stmt * stmt = nullptr;
    const int result = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
    stmt_.reset(stmt);
    check(db, result);
  }

  /** @brief 按字节绑定字符串，SQLite 复制内容。 */
  void bind(int column, std::string_view value)
  {
    check(db_, sqlite3_bind_text64(stmt_.get(), column, value.empty() ? "" : value.data(),
      value.size(), SQLITE_TRANSIENT, SQLITE_UTF8));
  }

  /** @brief 绑定 SQLite 有符号整数，用于实体修订。 */
  void bindInteger(int column, std::int64_t value)
  {
    check(db_, sqlite3_bind_int64(stmt_.get(), column, value));
  }

  /** @brief 绑定可空字段，空指针含义由 SQL NULL 表达。 */
  void bindOptional(int column, const std::optional<std::string> & value)
  {
    if (value) {bind(column, *value);}
    else {check(db_, sqlite3_bind_null(stmt_.get(), column));}
  }

  /** @brief 前进一行或结束，保留 SQL 失败的真实错误码。 */
  bool step()
  {
    const auto result = sqlite3_step(stmt_.get());
    if (result == SQLITE_ROW) {return true;}
    if (result == SQLITE_DONE) {return false;}
    throw SqlError(db_, result);
  }

  /** @brief 写入一行，忽略写入的触发器也视为失败。 */
  void writeOne()
  {
    if (step() || sqlite3_changes(db_) != 1) {throw std::runtime_error("Expected exactly one written row");}
  }

  /** @brief 读取拥有型文本，拒绝 SQLite 的隐式数值转换。 */
  std::string text(int column) const
  {
    if (sqlite3_column_type(stmt_.get(), column) != SQLITE_TEXT) {
      throw std::invalid_argument("Expected SQLite text column");
    }
    const auto * value = sqlite3_column_text(stmt_.get(), column);
    if (!value) {throw SqlError(db_, SQLITE_NOMEM);}
    return {reinterpret_cast<const char *>(value), static_cast<std::size_t>(sqlite3_column_bytes(stmt_.get(), column))};
  }

  /** @brief 读出可空文本。 */
  std::optional<std::string> optionalText(int column) const
  {
    if (sqlite3_column_type(stmt_.get(), column) == SQLITE_NULL) {return std::nullopt;}
    return text(column);
  }

  /** @brief 读取有符号 SQLite 整数。 */
  std::int64_t integer(int column) const
  {
    if (sqlite3_column_type(stmt_.get(), column) != SQLITE_INTEGER) {
      throw std::invalid_argument("Expected SQLite integer column");
    }
    return sqlite3_column_int64(stmt_.get(), column);
  }

  /** @brief 读出可空的关联修订序号，NULL 对应尚无定位依据。 */
  std::optional<std::int64_t> optionalInteger(int column) const
  {
    if (sqlite3_column_type(stmt_.get(), column) == SQLITE_NULL) {return std::nullopt;}
    return integer(column);
  }

private:
  sqlite3 * db_;
  std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> stmt_{nullptr, sqlite3_finalize};
};

class Transaction
{
public:
  /** @brief 先取得写锁，在一个事务中维护实体和证据。 */
  explicit Transaction(sqlite3 * db) : db_(db) {execute(db_, "BEGIN IMMEDIATE");}

  /** @brief 任何未提交路径都回滚。 */
  ~Transaction() {if (!committed_) {sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);}}

  /** @brief 仅在 SQLite 确认提交后标记成功。 */
  void commit() {execute(db_, "COMMIT"); committed_ = true;}

private:
  sqlite3 * db_;
  bool committed_{false};
};

/** @brief 只在显式创建模式传入 CREATE，启用 SQLite 的外键约束。 */
Db connect(const std::filesystem::path & path, int flags)
{
  sqlite3 * handle = nullptr;
  const int result = sqlite3_open_v2(path.string().c_str(), &handle, flags, nullptr);
  Db db(handle, sqlite3_close);
  check(handle, result);
  check(handle, sqlite3_extended_result_codes(handle, 1));
  execute(handle, "PRAGMA foreign_keys = ON");
  return db;
}

/** @brief 校验规范十六进制身份，不接受空值或大写别名。 */
bool hexId(std::string_view id, std::size_t length)
{
  return id.size() == length && id.find_first_not_of("0123456789abcdef") == std::string_view::npos;
}

/** @brief 文本字段不允许空值或控制字符。 */
bool validText(std::string_view text)
{
  return !text.empty() && text.size() <= 4096 && text.find_first_not_of(' ') != std::string_view::npos &&
    std::none_of(text.begin(), text.end(), [](unsigned char byte) {return byte < ' ' || byte == 0x7f;});
}

/** @brief 检查可持久化的观测集绑定。 */
bool validContext(const EntityStoreContext & context)
{
  return hexId(context.dataset_manifest_sha256, 64) && hexId(context.source_manifest_sha256, 64) &&
    validText(context.map_id) && validText(context.map_frame) && validText(context.geometry_revision);
}

/** @brief 比对同一地图的三个独立字段。 */
template<typename Record>
bool sameMap(const Record & record, const EntityStoreContext & context)
{
  return record.map_id == context.map_id && record.map_frame == context.map_frame &&
    record.geometry_revision == context.geometry_revision;
}

/** @brief 比较写入来源与工作库绑定的发布版本及地图。 */
bool matchesDataset(const EntityStoreContext & dataset, const EntityStoreContext & bound)
{
  return dataset.dataset_manifest_sha256 == bound.dataset_manifest_sha256 &&
    dataset.source_manifest_sha256 == bound.source_manifest_sha256 && sameMap(dataset, bound);
}

/** @brief 校验实体自身字段，定位状态与具体证据单独核对。 */
bool validEntity(const Entity & entity)
{
  const auto & id = entity.entity_id;
  if (id.compare(0, 4, "ent_") != 0 || !hexId(std::string_view(id).substr(4), 32) ||
    !validText(entity.semantic_label) || entity.revision == 0 ||
    (entity.kind != EntityKind::object && entity.kind != EntityKind::region)) {return false;}
  if (entity.status == EntityStatus::pending_location) {
    return entity.location_pending_reason && validText(*entity.location_pending_reason);
  }
  return entity.status == EntityStatus::located && !entity.location_pending_reason;
}

/** @brief 创建时核对唯一观测和定位状态，旧调用方可省略显式引用。 */
bool validInitialLocation(const Entity & entity, const EntityObservationRef & evidence)
{
  const bool located = entity.status == EntityStatus::located;
  if (detail::hasEntityLocation(evidence) != located) {return false;}
  const auto & selected = entity.location_observation;
  return !selected || (located && selected->observation_id == evidence.observation_id &&
    selected->observation_revision == evidence.observation_revision);
}

/** @brief 解码工作库中的实体类型，遇到损坏记录返回错误。 */
EntityKind readKind(const std::string & kind)
{
  if (kind == "object") {return EntityKind::object;}
  if (kind == "region") {return EntityKind::region;}
  throw std::invalid_argument("Invalid entity kind");
}

/** @brief 已定位空间的锚点必须与正式观测的底盘位姿一致。 */
bool matchesBodyAnchor(const EntityObservationRef & ref, const Observation & observation)
{
  if (!ref.region_interior || !ref.region_interior->body_anchor) {return true;}
  const auto & a = *ref.region_interior->body_anchor;
  const auto & pose = observation.body_pose_map;
  return pose && pose->pose && a.body_frame == observation.body_frame &&
    a.stamp.sec == pose->time.stamp.sec && a.stamp.nanosec == pose->time.stamp.nanosec &&
    a.point_map_m == pose->pose->translation_m;
}

/** @brief 核对确切的正式观测及其空间锚点，供创建和追加共用。 */
bool matchesObservation(
  const EntityObservationRef & ref, const Observation & observation, const EntityStoreContext & bound)
{
  return sameMap(observation, bound) && !ref.observation_id.empty() && ref.observation_id == observation.id &&
    ref.observation_revision != 0 && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256 &&
    ref.source_manifest_sha256 == bound.source_manifest_sha256 && matchesBodyAnchor(ref, observation);
}

/** @brief 绑定一个实体与观测修订的联合键，保留完整无符号观测修订。 */
void bindAssociationKey(Statement & row, std::string_view entity_id, const EntityObservationRef & ref)
{
  row.bind(1, entity_id);
  row.bind(2, ref.observation_id);
  row.bind(3, std::to_string(ref.observation_revision));
}

/** @brief 在调用方的事务中写一条证据及加入时的实体修订。 */
void insertEvidence(
  sqlite3 * db, std::string_view entity_id, const EntityObservationRef & ref,
  std::string_view payload, std::int64_t linked_at_revision)
{
  Statement row(db, "INSERT INTO entity_observations"
    "(entity_id,observation_id,observation_revision,source_manifest_sha256,evidence_json,linked_at_revision) "
    "VALUES(?1,?2,?3,?4,?5,?6)");
  bindAssociationKey(row, entity_id, ref);
  row.bind(4, ref.source_manifest_sha256);
  row.bind(5, payload);
  row.bindInteger(6, linked_at_revision);
  row.writeOne();
}

/** @brief 用规范十进制 TEXT 保留无符号观测修订的完整范围。 */
std::uint64_t readRevision(const std::string & text)
{
  std::uint64_t revision = 0;
  const auto end = text.data() + text.size();
  const auto parsed = std::from_chars(text.data(), end, revision);
  if (parsed.ec != std::errc{} || parsed.ptr != end || revision == 0 || text.front() == '0') {
    throw std::invalid_argument("Invalid observation revision");
  }
  return revision;
}

/** @brief 读取一个整数 pragma。 */
std::int64_t pragma(sqlite3 * db, const char * sql)
{
  Statement query(db, sql);
  if (!query.step()) {throw std::invalid_argument("Missing SQLite pragma");}
  return query.integer(0);
}

/** @brief 读取并验证绑定，先释放查询再执行可能需要的 schema 迁移。 */
EntityStoreContext readContext(sqlite3 * db)
{
  Statement query(db, "SELECT dataset_manifest_sha256, source_manifest_sha256, map_id, map_frame, geometry_revision "
    "FROM store_meta WHERE singleton=1");
  if (!query.step()) {throw std::invalid_argument("Store binding is missing");}
  EntityStoreContext context{query.text(0), query.text(1), query.text(2), query.text(3), query.text(4)};
  if (!validContext(context)) {throw std::invalid_argument("Store binding is invalid");}
  return context;
}

/** @brief 在一个事务内升级 v1/v2，并保留历史首条定位证据。 */
void upgradeSchema(sqlite3 * db)
{
  Transaction transaction(db);
  const auto version = pragma(db, "PRAGMA user_version");
  if (version < 1 || version > kSchemaVersion) {
    throw std::invalid_argument("Unsupported entity store format");
  }
  if (version == 1) {
    execute(db, "ALTER TABLE entity_observations ADD COLUMN linked_at_revision "
      "INTEGER NOT NULL DEFAULT 1 CHECK(linked_at_revision > 0)");
  }
  if (version < 3) {
    execute(db, "ALTER TABLE entities ADD COLUMN location_linked_at_revision INTEGER "
      "CHECK(location_linked_at_revision > 0 AND location_linked_at_revision <= revision)");
    execute(db, "UPDATE entities SET location_linked_at_revision=1 WHERE status='located'");
    execute(db, "PRAGMA user_version = 3");
  }
  transaction.commit();
}

}  // namespace

struct EntityStoreState
{
  Db db{nullptr, sqlite3_close};
  EntityStoreContext context;
};

/** @brief 接管数据库连接和不可变来源绑定。 */
EntityStore::EntityStore(std::unique_ptr<EntityStoreState> state) : state_(std::move(state)) {}

/** @brief 释放 SQLite 连接。 */
EntityStore::~EntityStore() = default;

/** @brief 暴露已读取的绑定用于上游比对。 */
const EntityStoreContext & EntityStore::context() const noexcept {return state_->context;}

/** @brief 在新文件写入 schema 与绑定，不修改现存工作库。 */
EntityStoreOpenResult createEntityStore(const std::filesystem::path & path, const EntityStoreContext & context)
{
  if (path.empty()) {return {{Code::invalid_store, "Store path is empty"}, nullptr};}
  if (!validContext(context)) {return {{Code::invalid_reference, "Invalid observation dataset binding"}, nullptr};}
  try {
    if (std::filesystem::symlink_status(path).type() != std::filesystem::file_type::not_found) {
      return {{Code::already_exists, "Store path already exists"}, nullptr};
    }
    std::unique_ptr<EntityStore> store;
    auto state = std::make_unique<EntityStoreState>();
    state->context = context;
    state->db = connect(path, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
    auto * db = state->db.get();
    Transaction transaction(db);
    Statement contents(db, "SELECT 1 FROM sqlite_master LIMIT 1");
    if (contents.step() || pragma(db, "PRAGMA application_id") || pragma(db, "PRAGMA user_version")) {
      return {{Code::already_exists, "Store path was initialized by another writer"}, nullptr};
    }
    execute(db, kSchema);
    Statement insert(db, "INSERT INTO store_meta VALUES(1,?1,?2,?3,?4,?5)");
    insert.bind(1, context.dataset_manifest_sha256);
    insert.bind(2, context.source_manifest_sha256);
    insert.bind(3, context.map_id);
    insert.bind(4, context.map_frame);
    insert.bind(5, context.geometry_revision);
    insert.writeOne();
    store.reset(new EntityStore(std::move(state)));
    // 先提交，失败时连接仍须比事务和语句活得更久。
    transaction.commit();
    return {{Code::ready, ""}, std::move(store)};
  } catch (const std::exception & error) {
    return {{Code::storage_error, error.what()}, nullptr};
  }
}

/** @brief 打开已有数据库并验证 schema 版本和来源绑定。 */
EntityStoreOpenResult openEntityStore(const std::filesystem::path & path)
{
  try {
    auto state = std::make_unique<EntityStoreState>();
    state->db = connect(path, SQLITE_OPEN_READWRITE);
    auto * db = state->db.get();
    const auto version = pragma(db, "PRAGMA user_version");
    if (pragma(db, "PRAGMA application_id") != kApplicationId || version < 1 || version > kSchemaVersion) {
      return {{Code::invalid_store, "Unsupported entity store format"}, nullptr};
    }
    state->context = readContext(db);
    if (version < kSchemaVersion) {upgradeSchema(db);}
    return {{Code::ready, ""}, std::unique_ptr<EntityStore>(new EntityStore(std::move(state)))};
  } catch (const std::invalid_argument & error) {
    return {{Code::invalid_store, error.what()}, nullptr};
  } catch (const std::exception & error) {
    return {{Code::storage_error, error.what()}, nullptr};
  }
}

/** @brief 校验新实体及其观测身份，在单事务中落库。 */
EntityStoreResult EntityStore::insertEntity(
  const Entity & entity, const EntityStoreContext & dataset, const Observation & observation)
{
  const auto & bound = context();
  if (!matchesDataset(dataset, bound)) {
    return {Code::dataset_mismatch, "Dataset differs from the store binding"};
  }
  if (!validEntity(entity) || entity.revision != 1 || entity.observations.size() != 1) {
    return {Code::invalid_entity, "Expected one observation for a new entity"};
  }
  const auto & ref = entity.observations.front();
  if (!detail::validEntityEvidence(entity.kind, ref, observation.depth_image.width, observation.depth_image.height) ||
    !validInitialLocation(entity, ref)) {
    return {Code::invalid_entity, "Invalid new entity location evidence"};
  }
  if (!sameMap(entity, bound) || !matchesObservation(ref, observation, bound)) {
    return {Code::invalid_reference, "Entity does not reference the verified observation revision and map"};
  }

  try {
    const auto evidence = detail::encodeEntityEvidence(ref);
    auto * db = state_->db.get();
    Transaction transaction(db);
    Statement row(db, "INSERT INTO entities"
      "(entity_id,kind,label,revision,status,pending_reason,location_linked_at_revision) "
      "VALUES(?1,?2,?3,1,?4,?5,CASE WHEN ?4='located' THEN 1 END)");
    row.bind(1, entity.entity_id);
    row.bind(2, entity.kind == EntityKind::object ? "object" : "region");
    row.bind(3, entity.semantic_label);
    row.bind(4, entity.status == EntityStatus::located ? "located" : "pending_location");
    row.bindOptional(5, entity.location_pending_reason);
    try {
      row.writeOne();
    } catch (const SqlError & error) {
      if (error.code == SQLITE_CONSTRAINT_PRIMARYKEY) {return {Code::already_exists, "Entity ID already exists"};}
      throw;
    }
    insertEvidence(db, entity.entity_id, ref, evidence, 1);
    transaction.commit();
    return {Code::ready, ""};
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 在持有写锁时检查当前修订、追加证据并推进实体修订。 */
EntityStoreResult EntityStore::appendEntityObservation(
  std::string_view entity_id, std::uint64_t expected_revision, const EntityObservationRef & evidence,
  const EntityStoreContext & dataset, const Observation & observation)
{
  const auto & bound = context();
  if (!matchesDataset(dataset, bound)) {
    return {Code::dataset_mismatch, "Dataset differs from the store binding"};
  }
  if (!matchesObservation(evidence, observation, bound)) {
    return {Code::invalid_reference, "Entity evidence does not reference the verified observation revision and map"};
  }
  try {
    auto * db = state_->db.get();
    Transaction transaction(db);
    EntityKind kind;
    std::int64_t revision;
    {
      Statement current(db, "SELECT kind,revision FROM entities WHERE entity_id=?1");
      current.bind(1, entity_id);
      if (!current.step()) {return {Code::not_found, "Entity ID not found"};}
      kind = readKind(current.text(0));
      revision = current.integer(1);
      if (revision <= 0) {return {Code::invalid_store, "Invalid stored entity revision"};}
    }
    if (static_cast<std::uint64_t>(revision) != expected_revision) {
      return {Code::revision_conflict, "Entity revision has changed"};
    }
    const auto width = observation.depth_image.width;
    const auto height = observation.depth_image.height;
    if (!detail::validEntityEvidence(kind, evidence, width, height)) {
      return {Code::invalid_entity, "Invalid observation evidence for entity kind"};
    }
    const auto payload = detail::encodeEntityEvidence(evidence);
    {
      Statement existing(db, "SELECT source_manifest_sha256,evidence_json FROM entity_observations "
        "WHERE entity_id=?1 AND observation_id=?2 AND observation_revision=?3");
      bindAssociationKey(existing, entity_id, evidence);
      if (existing.step()) {
        EntityObservationRef saved;
        detail::decodeEntityEvidence(existing.text(1), kind, saved);
        if (existing.text(0) != bound.source_manifest_sha256 ||
          !detail::validEntityEvidence(kind, saved, width, height) || !matchesBodyAnchor(saved, observation)) {
          return {Code::invalid_store, "Invalid stored observation evidence"};
        }
        if (detail::encodeEntityEvidence(saved) == payload) {
          return {Code::already_linked, "Identical evidence is already linked"};
        }
        return {Code::evidence_conflict, "Observation revision is already linked with different evidence"};
      }
    }
    if (revision == std::numeric_limits<std::int64_t>::max()) {
      return {Code::invalid_entity, "Entity revision limit reached"};
    }
    const auto next_revision = revision + 1;
    insertEvidence(db, entity_id, evidence, payload, next_revision);
    Statement advance(db, "UPDATE entities SET revision=?1 WHERE entity_id=?2 AND revision=?3");
    advance.bindInteger(1, next_revision);
    advance.bind(2, entity_id);
    advance.bindInteger(3, revision);
    advance.writeOne();
    transaction.commit();
    return {Code::ready, ""};
  } catch (const std::invalid_argument & error) {
    return {Code::invalid_store, error.what()};
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 一次主键查询恢复实体和证据，断引用或损坏证据返回错误。 */
EntityStoreLoadResult EntityStore::loadEntity(std::string_view entity_id) const
{
  try {
    Statement query(state_->db.get(), "SELECT e.kind,e.label,e.revision,e.status,e.pending_reason,"
      "o.observation_id,o.observation_revision,o.source_manifest_sha256,o.evidence_json,o.linked_at_revision,"
      "e.location_linked_at_revision "
      "FROM entities e LEFT JOIN entity_observations o USING(entity_id) WHERE e.entity_id=?1 "
      "ORDER BY o.linked_at_revision");
    query.bind(1, entity_id);
    if (!query.step()) {return {{Code::not_found, "Entity ID not found"}, std::nullopt};}
    const auto kind = readKind(query.text(0));
    const auto status = query.text(3);
    const auto revision = query.integer(2);
    if ((status != "located" && status != "pending_location") || revision <= 0) {
      return {{Code::invalid_store, "Invalid entity status or revision"}, std::nullopt};
    }
    const auto & binding = context();
    Entity entity{std::string(entity_id), kind,
      query.text(1), status == "located" ? EntityStatus::located : EntityStatus::pending_location,
      static_cast<std::uint64_t>(revision), binding.map_id, binding.map_frame, binding.geometry_revision,
      {}, query.optionalText(4)};
    if (!validEntity(entity)) {return {{Code::invalid_store, "Invalid stored entity"}, std::nullopt};}
    const auto selected_revision = query.optionalInteger(10);
    if ((entity.status == EntityStatus::located) != selected_revision.has_value()) {
      return {{Code::invalid_store, "Entity status differs from its location reference"}, std::nullopt};
    }
    std::int64_t last_linked_at = 0;
    // 回读不打开图像，使用像素类型范围检查形状和数值。
    constexpr auto max_image_side = std::numeric_limits<std::uint32_t>::max();
    do {
      const auto linked_at = query.integer(9);
      if (linked_at <= last_linked_at || linked_at > revision || (last_linked_at == 0 && linked_at != 1)) {
        return {{Code::invalid_store, "Invalid observation association order"}, std::nullopt};
      }
      last_linked_at = linked_at;
      EntityObservationRef ref{query.text(5), readRevision(query.text(6)), query.text(7), std::nullopt, std::nullopt};
      detail::decodeEntityEvidence(query.text(8), entity.kind, ref);
      if (ref.observation_id.empty() || ref.source_manifest_sha256 != binding.source_manifest_sha256 ||
        !detail::validEntityEvidence(entity.kind, ref, max_image_side, max_image_side)) {
        return {{Code::invalid_store, "Invalid stored observation evidence"}, std::nullopt};
      }
      if (selected_revision && *selected_revision == linked_at) {
        if (!detail::hasEntityLocation(ref)) {
          return {{Code::invalid_store, "Selected observation lacks location evidence"}, std::nullopt};
        }
        entity.location_observation = EntityLocationRef{ref.observation_id, ref.observation_revision};
      }
      entity.observations.push_back(std::move(ref));
    } while (query.step());
    if (selected_revision && !entity.location_observation) {
      return {{Code::invalid_store, "Selected location observation is not linked to the entity"}, std::nullopt};
    }
    return {{Code::ready, ""}, std::move(entity)};
  } catch (const SqlError & error) {
    return {{Code::storage_error, error.what()}, std::nullopt};
  } catch (const std::exception & error) {
    return {{Code::invalid_store, error.what()}, std::nullopt};
  }
}

}  // namespace vln_memory
