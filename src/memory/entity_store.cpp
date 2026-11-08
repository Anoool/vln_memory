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
constexpr int kSchemaVersion = 1;

constexpr char kSchema[] = R"sql(
  PRAGMA application_id = 0x564c4e45;
  PRAGMA user_version = 1;
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
    CHECK((status = 'located') = (pending_reason IS NULL))
  );
  CREATE TABLE entity_observations(
    entity_id TEXT NOT NULL REFERENCES entities(entity_id),
    observation_id TEXT NOT NULL, observation_revision TEXT NOT NULL, source_manifest_sha256 TEXT NOT NULL,
    evidence_json TEXT NOT NULL,
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

/** @brief 校验实体记录及第一版的唯一观测约束。 */
bool validEntity(const Entity & entity, std::uint32_t width, std::uint32_t height)
{
  const auto & id = entity.entity_id;
  if (id.compare(0, 4, "ent_") != 0 || !hexId(std::string_view(id).substr(4), 32) ||
    !validText(entity.semantic_label) || entity.revision != 1 || entity.observations.size() != 1 ||
    (entity.kind != EntityKind::object && entity.kind != EntityKind::region)) {return false;}
  if (entity.status == EntityStatus::pending_location) {
    if (!entity.location_pending_reason || !validText(*entity.location_pending_reason)) {return false;}
  } else if (entity.status != EntityStatus::located || entity.location_pending_reason) {return false;}
  return detail::validEntityEvidence(entity, width, height);
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
    if (pragma(db, "PRAGMA application_id") != kApplicationId || pragma(db, "PRAGMA user_version") != kSchemaVersion) {
      return {{Code::invalid_store, "Unsupported entity store format"}, nullptr};
    }
    Statement query(db, "SELECT dataset_manifest_sha256, source_manifest_sha256, map_id, map_frame, geometry_revision "
      "FROM store_meta WHERE singleton=1");
    if (!query.step()) {return {{Code::invalid_store, "Store binding is missing"}, nullptr};}
    state->context = {query.text(0), query.text(1), query.text(2), query.text(3), query.text(4)};
    if (!validContext(state->context)) {return {{Code::invalid_store, "Store binding is invalid"}, nullptr};}
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
  if (dataset.dataset_manifest_sha256 != bound.dataset_manifest_sha256 ||
    dataset.source_manifest_sha256 != bound.source_manifest_sha256 || !sameMap(dataset, bound)) {
    return {Code::dataset_mismatch, "Dataset differs from the store binding"};
  }
  if (!validEntity(entity, observation.depth_image.width, observation.depth_image.height)) {
    return {Code::invalid_entity, "Invalid new entity or location evidence"};
  }
  const auto & ref = entity.observations.front();
  if (!sameMap(entity, bound) || !sameMap(observation, bound) ||
    ref.observation_id.empty() || ref.observation_id != observation.id ||
    ref.observation_revision == 0 || ref.observation_revision != observation.revision ||
    ref.source_manifest_sha256 != observation.source_manifest_sha256 ||
    ref.source_manifest_sha256 != bound.source_manifest_sha256 || !matchesBodyAnchor(ref, observation)) {
    return {Code::invalid_reference, "Entity does not reference the verified observation revision and map"};
  }

  try {
    const auto evidence = detail::encodeEntityEvidence(ref);
    auto * db = state_->db.get();
    Transaction transaction(db);
    Statement row(db, "INSERT INTO entities VALUES(?1,?2,?3,1,?4,?5)");
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
    Statement evidence_row(db, "INSERT INTO entity_observations VALUES(?1,?2,?3,?4,?5)");
    evidence_row.bind(1, entity.entity_id);
    evidence_row.bind(2, ref.observation_id);
    evidence_row.bind(3, std::to_string(ref.observation_revision));
    evidence_row.bind(4, ref.source_manifest_sha256);
    evidence_row.bind(5, evidence);
    evidence_row.writeOne();
    transaction.commit();
    return {Code::ready, ""};
  } catch (const std::exception & error) {
    return {Code::storage_error, error.what()};
  }
}

/** @brief 一次主键查询恢复实体和证据，断引用或损坏证据返回错误。 */
EntityStoreLoadResult EntityStore::loadEntity(std::string_view entity_id) const
{
  try {
    Statement query(state_->db.get(), "SELECT e.kind,e.label,e.revision,e.status,e.pending_reason,"
      "o.observation_id,o.observation_revision,o.source_manifest_sha256,o.evidence_json "
      "FROM entities e LEFT JOIN entity_observations o USING(entity_id) WHERE e.entity_id=?1");
    query.bind(1, entity_id);
    if (!query.step()) {return {{Code::not_found, "Entity ID not found"}, std::nullopt};}
    const auto kind = query.text(0);
    const auto status = query.text(3);
    const auto revision = query.integer(2);
    if ((kind != "object" && kind != "region") || (status != "located" && status != "pending_location") || revision != 1) {
      return {{Code::invalid_store, "Invalid entity kind, status or revision"}, std::nullopt};
    }
    const auto & binding = context();
    Entity entity{std::string(entity_id), kind == "object" ? EntityKind::object : EntityKind::region,
      query.text(1), status == "located" ? EntityStatus::located : EntityStatus::pending_location,
      1, binding.map_id, binding.map_frame, binding.geometry_revision, {}, query.optionalText(4)};
    EntityObservationRef ref{query.text(5), readRevision(query.text(6)), query.text(7), std::nullopt, std::nullopt};
    detail::decodeEntityEvidence(query.text(8), entity.kind, ref);
    entity.observations.push_back(std::move(ref));
    // 回读不打开图像，使用像素类型范围检查形状和数值。
    constexpr auto max_image_side = std::numeric_limits<std::uint32_t>::max();
    if (query.step() || entity.observations.front().observation_id.empty() ||
      entity.observations.front().source_manifest_sha256 != binding.source_manifest_sha256 ||
      !validEntity(entity, max_image_side, max_image_side)) {
      return {{Code::invalid_store, "Invalid stored entity or observation evidence"}, std::nullopt};
    }
    return {{Code::ready, ""}, std::move(entity)};
  } catch (const SqlError & error) {
    return {{Code::storage_error, error.what()}, std::nullopt};
  } catch (const std::exception & error) {
    return {{Code::invalid_store, error.what()}, std::nullopt};
  }
}

}  // namespace vln_memory
