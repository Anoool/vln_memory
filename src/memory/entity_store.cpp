#include <vln_memory/memory/entity_store.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <vln_memory/memory/observation.hpp>

#include "entity_store_evidence.hpp"

namespace vln_memory
{
namespace
{
using Code = EntityStoreCode;
using Json = nlohmann::json;
using Db = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;

constexpr int kApplicationId = 0x564c4e45;
constexpr int kSchemaVersion = 6;

constexpr char kSchema[] = R"sql(
  PRAGMA application_id = 0x564c4e45;
  PRAGMA user_version = 6;
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
    details_json TEXT NOT NULL DEFAULT '{}',
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

constexpr char kRelationSchema[] = R"sql(
  CREATE TABLE entity_relations(
    predicate TEXT NOT NULL CHECK(predicate IN ('inside','near','on','hanging_on','connects_regions')),
    subject_id TEXT NOT NULL REFERENCES entities(entity_id),
    target_id TEXT NOT NULL REFERENCES entities(entity_id),
    extra_id TEXT NOT NULL DEFAULT '',
    geometry_revision TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
    basis TEXT NOT NULL, evidence_ref TEXT NOT NULL,
    recognition_state TEXT NOT NULL DEFAULT 'supported',
    revision INTEGER NOT NULL DEFAULT 1,
    region_membership INTEGER NOT NULL DEFAULT 0,
    distance_m REAL, threshold_m REAL, dimension TEXT NOT NULL,
    PRIMARY KEY(predicate,subject_id,target_id,extra_id),
    CHECK(subject_id <> target_id)
  );
  CREATE UNIQUE INDEX entity_inside_region ON entity_relations(subject_id)
    WHERE region_membership=1 AND recognition_state='supported';
  CREATE TABLE pending_entities(pending_id TEXT PRIMARY KEY NOT NULL, request_json TEXT NOT NULL);
)sql";

constexpr char kBuildSchema[] = R"sql(
  CREATE TABLE tool_calls(
    call_id TEXT PRIMARY KEY NOT NULL, request_json TEXT NOT NULL, result_json TEXT NOT NULL
  );
  CREATE TABLE build_progress(
    singleton INTEGER PRIMARY KEY CHECK(singleton=1), progress_json TEXT NOT NULL
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

  /** @brief 保存可空的米制数值。 */
  void bindReal(int column, std::optional<double> value)
  {
    if (value) {check(db_, sqlite3_bind_double(stmt_.get(), column, *value));}
    else {check(db_, sqlite3_bind_null(stmt_.get(), column));}
  }

  /** @brief 读取数值距离，NULL 表示定性关系。 */
  std::optional<double> optionalReal(int column) const
  {
    if (sqlite3_column_type(stmt_.get(), column) == SQLITE_NULL) {return std::nullopt;}
    return sqlite3_column_double(stmt_.get(), column);
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
  explicit Transaction(sqlite3 * db) : db_(db), nested_(!sqlite3_get_autocommit(db))
  {
    execute(db_, nested_ ? "SAVEPOINT entity_write" : "BEGIN IMMEDIATE");
  }

  /** @brief 任何未提交路径都回滚。 */
  ~Transaction()
  {
    if (!committed_) {
      const auto * rollback = nested_ ? "ROLLBACK TO entity_write; RELEASE entity_write" : "ROLLBACK";
      sqlite3_exec(db_, rollback, nullptr, nullptr, nullptr);
    }
  }

  /** @brief 仅在 SQLite 确认提交后标记成功。 */
  void commit()
  {
    execute(db_, nested_ ? "RELEASE entity_write" : "COMMIT");
    committed_ = true;
  }

private:
  sqlite3 * db_;
  bool nested_;
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
void bindAssociationKey(
  Statement & row, std::string_view entity_id, std::string_view observation_id, std::uint64_t observation_revision)
{
  row.bind(1, entity_id);
  row.bind(2, observation_id);
  row.bind(3, std::to_string(observation_revision));
}

/** @brief 在调用方的事务中写一条证据及加入时的实体修订。 */
void insertEvidence(
  sqlite3 * db, std::string_view entity_id, const EntityObservationRef & ref,
  std::string_view payload, std::int64_t linked_at_revision)
{
  Statement row(db, "INSERT INTO entity_observations"
    "(entity_id,observation_id,observation_revision,source_manifest_sha256,evidence_json,linked_at_revision) "
    "VALUES(?1,?2,?3,?4,?5,?6)");
  bindAssociationKey(row, entity_id, ref.observation_id, ref.observation_revision);
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

/** @brief 在一个事务内升级旧库，补齐定位引用和关系表。 */
void upgradeSchema(sqlite3 * db)
{
  Transaction transaction(db);
  const auto version = pragma(db, "PRAGMA user_version");
  if (version < 1 || version > kSchemaVersion) {
    throw std::invalid_argument("Unsupported entity store format");
  }
  if (version == kSchemaVersion) {
    transaction.commit();
    return;
  }
  if (version < 5) {
    if (version == 1) {
      execute(db, "ALTER TABLE entity_observations ADD COLUMN linked_at_revision "
        "INTEGER NOT NULL DEFAULT 1 CHECK(linked_at_revision > 0)");
    }
    if (version < 3) {
      execute(db, "ALTER TABLE entities ADD COLUMN location_linked_at_revision INTEGER "
        "CHECK(location_linked_at_revision > 0 AND location_linked_at_revision <= revision)");
      execute(db, "UPDATE entities SET location_linked_at_revision=1 WHERE status='located'");
    }
    execute(db, "ALTER TABLE entities ADD COLUMN details_json TEXT NOT NULL DEFAULT '{}'");
    if (version == 4) {
      execute(db, "DROP INDEX entity_inside_region; ALTER TABLE entity_relations RENAME TO relations_v4");
    }
    execute(db, kRelationSchema);
    if (version == 4) {
      execute(db, R"sql(
        INSERT INTO entity_relations
        (predicate,subject_id,target_id,geometry_revision,evidence_json,basis,evidence_ref,region_membership,
         distance_m,threshold_m,dimension)
        SELECT predicate,subject_id,target_id,geometry_revision,
          CASE WHEN observation_id IS NULL THEN '[]' ELSE json_array(observation_id) END,
          CASE WHEN predicate='near' THEN 'geometry_computed' ELSE 'human_confirmed' END,
          coalesce(evidence_ref,''),predicate='inside',distance_m,threshold_m,
          CASE WHEN predicate='near' THEN 'xyz' ELSE '' END FROM relations_v4;
        DROP TABLE relations_v4;
      )sql");
    }
  }
  execute(db, kBuildSchema);
  execute(db, "PRAGMA user_version = 6");
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
    execute(db, kRelationSchema);
    execute(db, kBuildSchema);
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
    if (!Json::parse(entity.details_json).is_object()) {
      return {Code::invalid_entity, "Entity details must be an object"};
    }
    auto * db = state_->db.get();
    Transaction transaction(db);
    Statement row(db, "INSERT INTO entities"
      "(entity_id,kind,label,revision,status,pending_reason,location_linked_at_revision,details_json) "
      "VALUES(?1,?2,?3,1,?4,?5,CASE WHEN ?4='located' THEN 1 END,?6)");
    row.bind(1, entity.entity_id);
    row.bind(2, entity.kind == EntityKind::object ? "object" : "region");
    row.bind(3, entity.semantic_label);
    row.bind(4, entity.status == EntityStatus::located ? "located" : "pending_location");
    row.bindOptional(5, entity.location_pending_reason);
    row.bind(6, entity.details_json);
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
      bindAssociationKey(existing, entity_id, evidence.observation_id, evidence.observation_revision);
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

/** @brief 选择本实体已保存的证据，在事务中把待定位实体更新为已定位。 */
EntityStoreResult EntityStore::resolveEntityLocation(
  std::string_view entity_id, std::uint64_t expected_revision, const EntityLocationRef & selected,
  const EntityStoreContext & dataset, const Observation & observation)
{
  const auto & bound = context();
  if (!matchesDataset(dataset, bound)) {
    return {Code::dataset_mismatch, "Dataset differs from the store binding"};
  }
  if (selected.observation_id.empty() || selected.observation_id != observation.id ||
    selected.observation_revision == 0 || selected.observation_revision != observation.revision) {
    return {Code::invalid_reference, "Selected observation revision differs from the verified observation"};
  }

  try {
    auto * db = state_->db.get();
    Transaction transaction(db);
    EntityKind kind;
    std::int64_t revision;
    {
      Statement current(db, "SELECT kind,revision,status,pending_reason,location_linked_at_revision "
        "FROM entities WHERE entity_id=?1");
      current.bind(1, entity_id);
      if (!current.step()) {return {Code::not_found, "Entity ID not found"};}
      kind = readKind(current.text(0));
      revision = current.integer(1);
      if (revision <= 0) {return {Code::invalid_store, "Invalid stored entity revision"};}
      if (static_cast<std::uint64_t>(revision) != expected_revision) {
        return {Code::revision_conflict, "Entity revision has changed"};
      }
      const auto status = current.text(2);
      const auto pending_reason = current.optionalText(3);
      const auto current_location = current.optionalInteger(4);
      if (status == "located" && !pending_reason && current_location &&
        *current_location > 0 && *current_location <= revision) {
        return {Code::already_located, "Entity already has a selected location"};
      }
      if (status != "pending_location" || !pending_reason || !validText(*pending_reason) || current_location) {
        return {Code::invalid_store, "Invalid stored entity location state"};
      }
    }

    std::int64_t linked_at_revision;
    {
      Statement linked(db, "SELECT linked_at_revision,source_manifest_sha256,evidence_json FROM entity_observations "
        "WHERE entity_id=?1 AND observation_id=?2 AND observation_revision=?3");
      bindAssociationKey(linked, entity_id, selected.observation_id, selected.observation_revision);
      if (!linked.step()) {return {Code::invalid_reference, "Selected observation is not linked to the entity"};}
      linked_at_revision = linked.integer(0);
      EntityObservationRef saved{selected.observation_id, selected.observation_revision, linked.text(1),
        std::nullopt, std::nullopt};
      try {
        detail::decodeEntityEvidence(linked.text(2), kind, saved);
      } catch (const std::exception & error) {
        return {Code::invalid_store, error.what()};
      }
      if (linked_at_revision <= 0 || linked_at_revision > revision ||
        !detail::validEntityEvidence(kind, saved, observation.depth_image.width, observation.depth_image.height)) {
        return {Code::invalid_store, "Invalid stored location evidence"};
      }
      if (!matchesObservation(saved, observation, bound)) {
        return {Code::invalid_reference, "Saved evidence does not match the verified observation and map"};
      }
      if (!detail::hasEntityLocation(saved)) {
        return {Code::location_unavailable, "Selected evidence has no valid location"};
      }
      if (kind == EntityKind::object &&
        (!observation.depth_media || !observation.depth_storage || !observation.depth_model ||
         !observation.depth_camera_pose_map || !observation.depth_camera_pose_map->ok())) {
        return {Code::location_unavailable, "Selected observation lacks depth projection inputs"};
      }
    }
    if (revision == std::numeric_limits<std::int64_t>::max()) {
      return {Code::invalid_entity, "Entity revision limit reached"};
    }

    Statement resolve(db, "UPDATE entities SET revision=?1,status='located',pending_reason=NULL,"
      "location_linked_at_revision=?2 WHERE entity_id=?3 AND revision=?4 AND status='pending_location'");
    resolve.bindInteger(1, revision + 1);
    resolve.bindInteger(2, linked_at_revision);
    resolve.bind(3, entity_id);
    resolve.bindInteger(4, revision);
    resolve.writeOne();
    Statement review(db, "UPDATE entity_relations SET recognition_state='needs_review', revision=revision+1 "
      "WHERE (subject_id=?1 OR target_id=?1 OR extra_id=?1) AND recognition_state='supported'");
    review.bind(1, entity_id);
    review.step();
    transaction.commit();
    return {Code::ready, ""};
  } catch (const std::invalid_argument & error) {
    return {Code::invalid_store, error.what()};
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 保存显式归属，实体的 region_id 读取自该关系。 */
EntityStoreResult EntityStore::recordInside(
  std::string_view object_id, std::string_view region_id,
  const EntityLocationRef & observation, std::string_view evidence_ref)
{
  if (object_id == region_id || !validText(evidence_ref)) {
    return {Code::invalid_reference, "Inside requires distinct endpoints and an evidence reference"};
  }
  try {
    auto * db = state_->db.get();
    Transaction transaction(db);
    const auto subject = loadEntity(object_id);
    if (!subject.ok()) {return {subject.code, subject.message};}
    const auto target = loadEntity(region_id);
    if (!target.ok()) {return {target.code, target.message};}
    if (subject.entity->kind != EntityKind::object || target.entity->kind != EntityKind::region) {
      return {Code::invalid_entity, "Inside requires an object and a region"};
    }
    // 证据必须确切指向已经关联到物品的观测修订。
    const auto & observations = subject.entity->observations;
    const auto ref = std::find_if(observations.begin(), observations.end(), [&](const auto & candidate) {
        return candidate.observation_id == observation.observation_id &&
          candidate.observation_revision == observation.observation_revision;
      });
    if (ref == observations.end()) {return {Code::invalid_reference, "Inside evidence is not linked to the object"};}
    if (subject.entity->region_id) {
      if (*subject.entity->region_id == region_id) {return {Code::already_linked, "Object is already inside this region"};}
      return {Code::evidence_conflict, "Object already belongs to a different region"};
    }
    EntityRelation relation;
    relation.type = "inside";
    relation.entity_ids = {std::string(object_id), std::string(region_id)};
    relation.evidence_observation_ids = {observation.observation_id};
    relation.basis = "human_confirmed";
    relation.evidence_ref = evidence_ref;
    const auto recorded = recordRelation(std::move(relation));
    if (!recorded.ok()) {return recorded;}
    transaction.commit();
    return {Code::ready, ""};
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 按无向端点顺序维护当前阈值对应的 near 结果。 */
NearRelationResult EntityStore::recordNear(
  std::string_view first_id, std::string_view second_id, double threshold_m)
{
  if (first_id == second_id || !std::isfinite(threshold_m) || threshold_m <= 0) {
    return {{Code::invalid_reference, "Near requires distinct endpoints and a positive finite threshold"}};
  }
  try {
    auto * db = state_->db.get();
    Transaction transaction(db);
    const auto first = loadEntity(first_id);
    if (!first.ok()) {return {{first.code, first.message}};}
    const auto second = loadEntity(second_id);
    if (!second.ok()) {return {{second.code, second.message}};}
    const auto first_point = selectedEntityPoint(*first.entity);
    const auto second_point = selectedEntityPoint(*second.entity);
    if (!first_point || !second_point) {return {{Code::location_unavailable, "Both entities need a selected location"}};}
    const auto & a = *first_point;
    const auto & b = *second_point;
    const auto distance = std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    if (!std::isfinite(distance)) {return {{Code::invalid_entity, "Non-finite map distance"}};}
    if (second_id < first_id) {std::swap(first_id, second_id);}
    const bool related = distance <= threshold_m;
    const auto existing = findRelation("near", {std::string(first_id), std::string(second_id)});
    if (existing && existing->basis != "geometry_computed") {
      transaction.commit();
      return {{Code::ready, ""}, distance, related};
    }
    if (related) {
      EntityRelation relation;
      relation.type = "near";
      relation.entity_ids = {std::string(first_id), std::string(second_id)};
      relation.basis = "geometry_computed";
      relation.distance_m = distance;
      relation.threshold_m = threshold_m;
      relation.dimension = "xyz";
      const auto recorded = recordRelation(std::move(relation));
      if (!recorded.ok()) {return {{recorded.code, recorded.message}};}
    } else {
      Statement remove(db, "DELETE FROM entity_relations WHERE predicate='near' AND subject_id=?1 AND target_id=?2 "
        "AND basis='geometry_computed'");
      remove.bind(1, first_id);
      remove.bind(2, second_id);
      remove.step();
    }
    transaction.commit();
    return {{Code::ready, ""}, distance, related};
  } catch (const std::exception & error) {
    return {{Code::storage_error, error.what()}};
  }
}

/** @brief 在同一连接上组合既有写入，底层方法以 savepoint 隔离自己的失败。 */
EntityStoreResult EntityStore::writeAtomically(const std::function<EntityStoreResult()> & write)
{
  try {
    Transaction transaction(state_->db.get());
    auto result = write();
    if (result.ok()) {transaction.commit();}
    return result;
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 乐观校验并保存实体的语义字段，字段相同时保持原修订。 */
EntityStoreResult EntityStore::updateEntityFields(
  std::string_view entity_id, std::uint64_t expected_revision,
  std::string_view label, std::string_view details_json, bool relation_changed)
{
  if (!validText(label) || !Json::parse(details_json, nullptr, false).is_object()) {
    return {Code::invalid_entity, "Invalid entity label or details"};
  }
  return writeAtomically([&]() -> EntityStoreResult {
    const auto current = loadEntity(entity_id);
    if (!current.ok()) {return current;}
    if (current.entity->revision != expected_revision) {return {Code::revision_conflict, "Entity revision has changed"};}
    if (!relation_changed && current.entity->semantic_label == label && current.entity->details_json == details_json) {
      return {Code::already_linked, ""};
    }
    if (expected_revision >= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      return {Code::invalid_entity, "Entity revision limit reached"};
    }
    Statement row(state_->db.get(), "UPDATE entities SET label=?2,details_json=?3,revision=revision+1 WHERE entity_id=?1");
    row.bind(1, entity_id);
    row.bind(2, label);
    row.bind(3, details_json);
    row.writeOne();
    return {Code::ready, ""};
  });
}

/** @brief 使用现有主键索引列出实体，搜索逻辑由工具决定。 */
std::vector<std::string> EntityStore::listEntityIds() const
{
  Statement row(state_->db.get(), "SELECT entity_id FROM entities ORDER BY entity_id");
  std::vector<std::string> ids;
  while (row.step()) {ids.push_back(row.text(0));}
  return ids;
}

namespace
{
/** @brief 校验关系类型和端点个数，将对称端点排成统一顺序。 */
void orderEndpoints(std::string_view type, std::vector<std::string> & ids)
{
  const bool connects = type == "connects_regions";
  if ((type != "near" && type != "inside" && type != "on" && type != "hanging_on" && !connects) ||
    ids.size() != (connects ? 3U : 2U) || ids[0] == ids[1] ||
    (connects && (ids[0] == ids[2] || ids[1] == ids[2])))
  {
    throw std::invalid_argument("Invalid relation type or endpoints");
  }
  if (type == "near" && ids[1] < ids[0]) {std::swap(ids[0], ids[1]);}
  if (connects && ids[2] < ids[1]) {std::swap(ids[1], ids[2]);}
}

/** @brief 将规范化的关系键绑定到固定 SQL。 */
void bindRelationKey(Statement & row, std::string_view type, const std::vector<std::string> & ids)
{
  row.bind(1, type);
  row.bind(2, ids[0]);
  row.bind(3, ids[1]);
  row.bind(4, ids.size() == 3 ? std::string_view(ids[2]) : std::string_view{});
}
}  // namespace

/** @brief 通过复合主键恢复可回查的关系。 */
std::optional<EntityRelation> EntityStore::findRelation(std::string_view type, std::vector<std::string> ids) const
{
  orderEndpoints(type, ids);
  Statement row(state_->db.get(), "SELECT geometry_revision,evidence_json,basis,evidence_ref,"
    "recognition_state,revision,distance_m,threshold_m,dimension FROM entity_relations "
    "WHERE predicate=?1 AND subject_id=?2 AND target_id=?3 AND extra_id=?4");
  bindRelationKey(row, type, ids);
  if (!row.step()) {return std::nullopt;}
  EntityRelation relation;
  relation.type = type;
  relation.entity_ids = std::move(ids);
  relation.geometry_revision = row.text(0);
  relation.evidence_observation_ids = Json::parse(row.text(1)).get<std::vector<std::string>>();
  relation.basis = row.text(2);
  relation.evidence_ref = row.text(3);
  relation.recognition_state = row.text(4);
  relation.revision = static_cast<std::uint64_t>(row.integer(5));
  relation.distance_m = row.optionalReal(6);
  relation.threshold_m = row.optionalReal(7);
  relation.dimension = row.text(8);
  return relation;
}

/** @brief 保存已核验来源的关系，保留修订并让已确认的 inside 成为空间归属来源。 */
EntityStoreResult EntityStore::recordRelation(EntityRelation relation)
{
  try {
    orderEndpoints(relation.type, relation.entity_ids);
  } catch (const std::invalid_argument & error) {
    return {Code::invalid_reference, error.what()};
  }
  const bool measured = relation.basis == "geometry_computed";
  if (measured) {
    if (relation.type != "near" || !relation.distance_m || !relation.threshold_m ||
      !std::isfinite(*relation.distance_m) || !std::isfinite(*relation.threshold_m) ||
      *relation.distance_m < 0 || *relation.threshold_m <= 0 || *relation.distance_m > *relation.threshold_m ||
      (relation.dimension != "xy" && relation.dimension != "xyz"))
    {
      return {Code::invalid_reference, "Invalid measured near relation"};
    }
  } else if ((relation.basis != "model_observed" && relation.basis != "human_confirmed") ||
    relation.evidence_observation_ids.empty() || relation.distance_m || relation.threshold_m)
  {
    return {Code::invalid_reference, "Semantic relations require observation evidence"};
  }
  if (relation.recognition_state != "tentative" && relation.recognition_state != "supported" &&
    relation.recognition_state != "needs_review")
  {
    return {Code::invalid_reference, "Invalid relation recognition state"};
  }
  std::sort(relation.evidence_observation_ids.begin(), relation.evidence_observation_ids.end());
  relation.evidence_observation_ids.erase(
    std::unique(relation.evidence_observation_ids.begin(), relation.evidence_observation_ids.end()),
    relation.evidence_observation_ids.end());
  return writeAtomically([&]() -> EntityStoreResult {
    std::vector<EntityKind> kinds;
    for (const auto & id : relation.entity_ids) {
      const auto entity = loadEntity(id);
      if (!entity.ok()) {return entity;}
      kinds.push_back(entity.entity->kind);
    }
    const bool region_membership = relation.type == "inside" && kinds[1] == EntityKind::region;
    if (relation.type == "connects_regions" &&
      (kinds[0] != EntityKind::object || kinds[1] != EntityKind::region || kinds[2] != EntityKind::region))
    {
      return {Code::invalid_entity, "connects_regions requires an object and two regions"};
    }
    if (region_membership && relation.recognition_state == "supported") {
      const auto subject = loadEntity(relation.entity_ids[0]);
      if (subject.entity->region_id && *subject.entity->region_id != relation.entity_ids[1]) {
        return {Code::evidence_conflict, "Entity already belongs to a different region"};
      }
    }
    const auto previous = findRelation(relation.type, relation.entity_ids);
    if (previous && previous->geometry_revision == context().geometry_revision &&
      previous->evidence_observation_ids == relation.evidence_observation_ids && previous->basis == relation.basis &&
      previous->evidence_ref == relation.evidence_ref && previous->recognition_state == relation.recognition_state &&
      previous->distance_m == relation.distance_m && previous->threshold_m == relation.threshold_m &&
      previous->dimension == relation.dimension)
    {
      return {Code::already_linked, ""};
    }
    Statement row(state_->db.get(), R"sql(
      INSERT INTO entity_relations(predicate,subject_id,target_id,extra_id,geometry_revision,evidence_json,
        basis,evidence_ref,recognition_state,region_membership,distance_m,threshold_m,dimension)
      VALUES(?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12,?13)
      ON CONFLICT(predicate,subject_id,target_id,extra_id) DO UPDATE SET
        geometry_revision=excluded.geometry_revision,evidence_json=excluded.evidence_json,basis=excluded.basis,
        evidence_ref=excluded.evidence_ref,recognition_state=excluded.recognition_state,
        region_membership=excluded.region_membership,distance_m=excluded.distance_m,
        threshold_m=excluded.threshold_m,dimension=excluded.dimension,revision=entity_relations.revision+1
    )sql");
    bindRelationKey(row, relation.type, relation.entity_ids);
    row.bind(5, context().geometry_revision);
    row.bind(6, Json(relation.evidence_observation_ids).dump());
    row.bind(7, relation.basis);
    row.bind(8, relation.evidence_ref);
    row.bind(9, relation.recognition_state);
    row.bindInteger(10, region_membership);
    row.bindReal(11, relation.distance_m);
    row.bindReal(12, relation.threshold_m);
    row.bind(13, relation.dimension);
    row.writeOne();
    return {Code::ready, ""};
  });
}

/** @brief 将待处理请求和原始证据保存到工作库，供之后定位或发布报告。 */
EntityStoreResult EntityStore::savePending(std::string_view pending_id, std::string_view request_json)
{
  return writeAtomically([&]() -> EntityStoreResult {
    Statement row(state_->db.get(), "INSERT INTO pending_entities VALUES(?1,?2)");
    row.bind(1, pending_id);
    row.bind(2, request_json);
    row.writeOne();
    return {Code::ready, ""};
  });
}

std::optional<std::string> EntityStore::findPending(std::string_view id) const
{
  Statement row(state_->db.get(), "SELECT request_json FROM pending_entities WHERE pending_id=?1");
  row.bind(1, id);
  if (!row.step()) {return std::nullopt;}
  return row.text(0);
}

std::vector<std::pair<std::string, std::string>> EntityStore::recentPending() const
{
  Statement row(state_->db.get(), "SELECT pending_id,request_json FROM pending_entities ORDER BY rowid DESC LIMIT 4");
  std::vector<std::pair<std::string, std::string>> requests;
  while (row.step()) {requests.emplace_back(row.text(0), row.text(1));}
  return requests;
}

EntityStoreResult EntityStore::removePending(std::string_view id)
{
  return writeAtomically([&]() -> EntityStoreResult {
    Statement row(state_->db.get(), "DELETE FROM pending_entities WHERE pending_id=?1");
    row.bind(1, id);
    row.writeOne();
    return {Code::ready, ""};
  });
}

/** @brief 返回已提交工具调用的原参数和输出。 */
std::optional<ToolCallRecord> EntityStore::findToolCall(std::string_view call_id) const
{
  Statement row(state_->db.get(), "SELECT request_json,result_json FROM tool_calls WHERE call_id=?1");
  row.bind(1, call_id);
  if (!row.step()) {return std::nullopt;}
  return ToolCallRecord{row.text(0), row.text(1)};
}

/** @brief 必须与业务写入处于同一事务，唯一键保护并行重放。 */
void EntityStore::insertToolCall(std::string_view call_id, const ToolCallRecord & record)
{
  auto * db = state_->db.get();
  if (sqlite3_get_autocommit(db) || !validText(call_id)) {
    throw std::invalid_argument("Tool calls require a transaction and a nonempty call ID");
  }
  Statement row(db, "INSERT INTO tool_calls VALUES(?1,?2,?3)");
  row.bind(1, call_id);
  row.bind(2, record.request_json);
  row.bind(3, record.result_json);
  row.writeOne();
}

/** @brief 读取单个构建恢复点。 */
std::optional<std::string> EntityStore::loadBuildProgress() const
{
  Statement row(state_->db.get(), "SELECT progress_json FROM build_progress WHERE singleton=1");
  if (!row.step()) {return std::nullopt;}
  return row.text(0);
}

/** @brief 比对已读取的状态后推进恢复点，失败时保留已提交的进度。 */
EntityStoreResult EntityStore::saveBuildProgress(
  const std::optional<std::string> & expected, std::string_view progress_json)
{
  return writeAtomically([&]() -> EntityStoreResult {
    if (loadBuildProgress() != expected) {
      return {Code::revision_conflict, "Build progress changed in another process"};
    }
    Statement row(state_->db.get(), "INSERT INTO build_progress VALUES(1,?1) "
      "ON CONFLICT(singleton) DO UPDATE SET progress_json=excluded.progress_json");
    row.bind(1, progress_json);
    row.writeOne();
    return {Code::ready, ""};
  });
}

/** @brief 一次主键查询恢复实体和证据，断引用或损坏证据返回错误。 */
EntityStoreLoadResult EntityStore::loadEntity(std::string_view entity_id) const
{
  try {
    Statement query(state_->db.get(), "SELECT e.kind,e.label,e.revision,e.status,e.pending_reason,"
      "o.observation_id,o.observation_revision,o.source_manifest_sha256,o.evidence_json,o.linked_at_revision,"
      "e.location_linked_at_revision,(SELECT target_id FROM entity_relations "
      "WHERE region_membership=1 AND recognition_state='supported' AND subject_id=e.entity_id),e.details_json "
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
    entity.region_id = query.optionalText(11);
    entity.details_json = query.text(12);
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
