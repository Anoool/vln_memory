#include <vln_memory/engine/entity_builder.hpp>
#include <vln_memory/engine/observation_builder.hpp>
#include <vln_memory/engine/observation_depth_region.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace vln_memory;
namespace fs = std::filesystem;

/** @brief Release 构建也执行契约检查。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 独立创建的身份不可由相同语义或观测推导。 */
void identity()
{
  const auto first = allocateEntityId();
  const auto second = allocateEntityId();
  expect(first && second && *first != *second, "Entity IDs are not independent");
  expect(first->size() == 36 && first->compare(0, 4, "ent_") == 0 &&
    first->find_first_not_of("0123456789abcdef", 4) == std::string::npos, "Invalid entity ID format");
}

/** @brief 从已保存配对构造一条正式观测，revision 刻意与实体不同。 */
Observation fixtureObservation(const PreparationReader & reader)
{
  expect(!reader.pairs().empty(), "Preparation fixture has no pairs");
  const ObservationBuildContext context{"fixture_map", "geometry_fixture_v2", std::string(reader.mapFrame()),
    "Recorded synthetic transforms", 3, {}};
  const auto built = buildObservation(reader, reader.pairs().front().pair_id, context);
  expect(built.ok(), built.message);
  return *built.observation;
}

/** @brief 检查对象身份、地图版本和可追溯的观测引用。 */
void checkReference(const Entity & entity, const Observation & observation)
{
  expect(entity.kind == EntityKind::object && entity.semantic_label == "箱子" &&
    entity.revision == 1 && !entity.entity_id.empty(), "Wrong object identity, label or revision");
  expect(entity.map_id == observation.map_id && entity.map_frame == observation.map_frame &&
    entity.geometry_revision == observation.geometry_revision && entity.observations.size() == 1,
    "Entity map context or observation count differs");
  const auto & ref = entity.observations.front();
  expect(ref.observation_id == observation.id && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256, "Observation reference was not preserved");
}

/** @brief 核查物品点确实来自所选深度像素簇，返回记录供身份独立性检查。 */
Entity checkLocated(
  const PreparationReader & reader, const Observation & observation,
  const DepthRegion & selection, const DepthRegionOptions & options = {})
{
  const auto projected = projectObservationDepthRegion(reader, observation, selection, options);
  expect(projected.ok(), projected.message);
  const auto built = createObjectEntity(reader, observation, "箱子", selection, options);
  expect(built.ok(), built.message);
  const auto & entity = *built.entity;
  checkReference(entity, observation);
  const auto & evidence = entity.observations.front().object_selection;
  expect(entity.status == EntityStatus::located && !entity.location_pending_reason &&
    evidence && evidence->projection_code == DepthRegionCode::ready && evidence->representative,
    "Located entity lost its projection evidence");
  const auto & actual = *evidence->representative;
  const auto & expected = *projected.projection->representative;
  expect(actual.pixel.u == expected.pixel.u && actual.pixel.v == expected.pixel.v &&
    actual.optical_z_m == expected.optical_z_m && actual.point_map_m == expected.point_map_m &&
    evidence->region_pixels == projected.projection->region_pixels &&
    evidence->valid_points == projected.projection->valid_points &&
    evidence->leading_cluster_points == projected.projection->clusters.front().point_count &&
    evidence->options.min_cluster_points == options.min_cluster_points,
    "Entity representative or quality evidence differs from the measured surface");
  return entity;
}

/** @brief 合法待定位记录保留来源和选区，缺失的测量保持为空。 */
void checkPending(const EntityBuildResult & built, const Observation & observation, const char * reason,
  bool has_selection, std::optional<DepthRegionCode> projection_code = std::nullopt)
{
  expect(built.ok(), built.message);
  const auto & entity = *built.entity;
  checkReference(entity, observation);
  const auto & evidence = entity.observations.front().object_selection;
  expect(entity.status == EntityStatus::pending_location && entity.location_pending_reason == reason &&
    evidence.has_value() == has_selection && (!evidence || (!evidence->representative &&
    evidence->projection_code == projection_code)), "Pending entity fabricated or lost location evidence");
}

constexpr char kRegionLabel[] = "测试房间";

/** @brief 构造测试用的外部确认，内容只适用于这个观测修订与地图。 */
RegionInteriorConfirmation confirmFor(const Observation & observation)
{
  return {kRegionLabel, observation.id, observation.revision, observation.source_manifest_sha256,
    observation.map_id, observation.map_frame, observation.geometry_revision, "fixture/interior-confirmation"};
}

/** @brief 待定位空间保留来源和可选确认，代表点保持空。 */
void checkPendingRegion(
  const EntityBuildResult & built, const Observation & observation, const char * reason,
  const RegionInteriorConfirmation * confirmation = nullptr)
{
  expect(built.ok(), built.message);
  const auto & entity = *built.entity;
  expect(entity.kind == EntityKind::region && entity.status == EntityStatus::pending_location &&
    entity.location_pending_reason == reason && entity.observations.size() == 1,
    "Region did not retain the expected pending state");
  const auto & ref = entity.observations.front();
  expect(ref.observation_id == observation.id && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256 && !ref.object_selection,
    "Pending region lost its observation reference");
  if (confirmation) {
    expect(ref.region_interior && ref.region_interior->confirmation_ref == confirmation->evidence_ref &&
      !ref.region_interior->body_anchor, "Pending region lost confirmation or gained an unsupported anchor");
  } else {
    expect(!ref.region_interior, "Pending region invented interior evidence");
  }
}

/** @brief 用带独立 RGB/Depth 时刻的构造录包验证空间定位、缺项和拒绝路径。 */
void regionCases(const fs::path & root, const PreparationReader & foreign_reader)
{
  const auto opened = openPreparation(root / "near_raw_1_out");
  expect(opened.ok(), opened.message);
  const auto & reader = *opened.reader;
  const auto observation = fixtureObservation(reader);
  expect(observation.delta_ns == -1 && observation.body_pose_map, "Fixture lacks a distinct depth time or body pose");
  const auto confirmation = confirmFor(observation);
  const auto built = createRegionEntity(reader, observation, kRegionLabel, confirmation);
  expect(built.ok(), built.message);
  const auto & entity = *built.entity;
  expect(entity.kind == EntityKind::region && entity.semantic_label == kRegionLabel && entity.revision == 1 &&
    !entity.entity_id.empty() && entity.status == EntityStatus::located && !entity.location_pending_reason &&
    entity.map_id == observation.map_id && entity.map_frame == observation.map_frame &&
    entity.geometry_revision == observation.geometry_revision && entity.observations.size() == 1,
    "Located region lost identity, context or status");
  const auto & ref = entity.observations.front();
  expect(ref.observation_id == observation.id && ref.observation_revision == observation.revision &&
    ref.source_manifest_sha256 == observation.source_manifest_sha256 && !ref.object_selection &&
    ref.region_interior && ref.region_interior->confirmation_ref == confirmation.evidence_ref &&
    ref.region_interior->body_anchor, "Located region lost interior evidence");
  const auto & anchor = *ref.region_interior->body_anchor;
  expect(anchor.body_frame == "base" && anchor.stamp.sec == 10 && anchor.stamp.nanosec == 0 &&
    anchor.point_map_m == std::array<double, 3>{2.0, 0.0, 0.0}, "Wrong historical body anchor");

  checkPendingRegion(createRegionEntity(reader, observation, kRegionLabel), observation, "missing_region_confirmation");
  auto without_pose = observation;
  without_pose.body_pose_map.reset();
  ++without_pose.revision;
  without_pose.pending_reasons = collectObservationPendingReasons(without_pose);
  const auto no_pose_confirmation = confirmFor(without_pose);
  checkPendingRegion(createRegionEntity(reader, without_pose, kRegionLabel, no_pose_confirmation),
    without_pose, "missing_body_map_pose", &no_pose_confirmation);

  for (const auto field : {&RegionInteriorConfirmation::map_id, &RegionInteriorConfirmation::map_frame,
    &RegionInteriorConfirmation::geometry_revision, &RegionInteriorConfirmation::semantic_label,
    &RegionInteriorConfirmation::observation_id, &RegionInteriorConfirmation::source_manifest_sha256}) {
    auto changed = confirmation;
    changed.*field = "different";
    const auto rejected = createRegionEntity(reader, observation, kRegionLabel, changed);
    expect(rejected.code == EntityBuildCode::invalid_region_confirmation && !rejected.entity,
      "Cross-context confirmation was accepted");
  }
  const auto stale = createRegionEntity(reader, without_pose, kRegionLabel, confirmation);
  expect(stale.code == EntityBuildCode::invalid_region_confirmation && !stale.entity,
    "Confirmation of a different observation revision was accepted");
  auto empty = confirmation;
  empty.evidence_ref = " ";
  const auto empty_ref = createRegionEntity(reader, observation, kRegionLabel, empty);
  expect(empty_ref.code == EntityBuildCode::invalid_region_confirmation && !empty_ref.entity,
    "Empty confirmation reference was accepted");
  const auto foreign = createRegionEntity(foreign_reader, observation, kRegionLabel);
  expect(foreign.code == EntityBuildCode::source_mismatch && !foreign.entity,
    "Foreign preparation was accepted without confirmation");

  auto altered = observation;
  auto & body = *altered.body_pose_map;
  expect(body.time.edges.size() == 1 && body.time.edges.front().after, "Expected interpolated body fixture");
  body.time.edges.front().before.translation_m[0] += 5;
  body.time.edges.front().after->translation_m[0] += 5;
  body.pose->translation_m[0] += 5;
  expect(validateObservation(altered).ok(), "Altered fixture must pass internal TF recomputation");
  const auto tampered = createRegionEntity(reader, altered, kRegionLabel, confirmation);
  expect(tampered.code == EntityBuildCode::source_mismatch && !tampered.entity,
    "Self-consistent rewrite of body trajectory was accepted");
}

/** @brief 使用 BagBuilder 实际发布的构造夹具覆盖成功、缺位姿、无有效深度和坏输入。 */
void fixture(const fs::path & pointer)
{
  std::ifstream latest(pointer / "latest.txt");
  std::string path;
  expect(static_cast<bool>(std::getline(latest, path)), "Cannot locate BagBuilder fixture");
  const auto root = fs::path(path);
  const DepthBoundingBox box{0, 0, 1, 1};
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const auto normal = openPreparation(root / "normal_no_cdr_out");
  expect(normal.ok(), normal.message);
  const auto observation = fixtureObservation(*normal.reader);
  regionCases(root, *normal.reader);
  const auto located = checkLocated(*normal.reader, observation, box, options);
  const auto & saved = std::get<DepthBoundingBox>(located.observations.front().object_selection->selection);
  expect(saved.u_min == box.u_min && saved.v_min == box.v_min &&
    saved.u_end == box.u_end && saved.v_end == box.v_end, "Original bbox not preserved");
  const auto independent = checkLocated(*normal.reader, observation, box, options);
  expect(located.entity_id != independent.entity_id, "Separate creates reused the same entity ID");
  checkPending(createObjectEntity(*normal.reader, observation, "箱子"), observation, "missing_selection", false);
  expect(createObjectEntity(*normal.reader, observation, " ").code == EntityBuildCode::invalid_label,
    "Blank label accepted");

  const auto conflict = openPreparation(root / "tf_conflict_out");
  expect(conflict.ok(), conflict.message);
  const auto without_pose = fixtureObservation(*conflict.reader);
  expect(!without_pose.depth_camera_pose_map, "Missing pose fixture has depth pose");
  checkPending(createObjectEntity(*conflict.reader, without_pose, "箱子", box, options),
    without_pose, "missing_depth_pose", true);
  const DepthPolygon crossed{{{0, 0}, {1, 1}, {0, 1}, {1, 0}}};
  const auto rejected = createObjectEntity(*conflict.reader, without_pose, "箱子", crossed, options);
  expect(rejected.code == EntityBuildCode::invalid_selection && !rejected.entity,
    "Invalid polygon became pending when depth pose was missing");
  const auto wrong_source = createObjectEntity(*normal.reader, without_pose, "箱子");
  expect(wrong_source.code == EntityBuildCode::source_mismatch && !wrong_source.entity,
    "Foreign observation accepted without selection");

  const auto empty = openPreparation(root / "invalid_depth_out");
  expect(empty.ok(), empty.message);
  const auto invalid_depth = fixtureObservation(*empty.reader);
  checkPending(createObjectEntity(*empty.reader, invalid_depth, "箱子", box, options),
    invalid_depth, "no_valid_depth", true, DepthRegionCode::no_valid_depth);
}

/** @brief 用已有短包产物复验 bbox 与 polygon，不创建新的 preparation 副本。 */
void published(const fs::path & source, const fs::path & observations)
{
  const auto opened = openPreparation(source);
  expect(opened.ok(), opened.message);
  std::ifstream file(observations);
  std::string line;
  expect(static_cast<bool>(std::getline(file, line)), "Cannot read published observation");
  const auto parsed = deserializeObservation(line);
  expect(parsed.ok(), parsed.message);
  checkPendingRegion(createRegionEntity(*opened.reader, *parsed.observation, kRegionLabel),
    *parsed.observation, "missing_region_confirmation");
  const auto box = checkLocated(*opened.reader, *parsed.observation, DepthBoundingBox{512, 316, 528, 332});
  const auto polygon = checkLocated(*opened.reader, *parsed.observation,
    DepthPolygon{{{512, 316}, {527, 316}, {512, 331}}});
  std::cout << "bbox_valid=" << box.observations.front().object_selection->valid_points <<
    " polygon_valid=" << polygon.observations.front().object_selection->valid_points << '\n';
}
}  // namespace

/** @brief 运行核心身份检查，可选验证构造夹具或已有短包。 */
int main(int argc, char ** argv)
{
  try {
    identity();
    if (argc == 3 && std::string_view(argv[1]) == "--fixture-pointer") {
      fixture(argv[2]);
    } else if (argc == 4 && std::string_view(argv[1]) == "--published") {
      published(argv[2], argv[3]);
    } else {
      expect(argc == 1, "Expected --fixture-pointer DIR or --published PREPARATION OBSERVATIONS");
    }
    std::cout << "Entity checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
