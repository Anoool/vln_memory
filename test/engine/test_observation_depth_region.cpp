#include <vln_memory/engine/observation_depth_region.hpp>
#include <vln_memory/engine/observation_builder.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
namespace fs = std::filesystem;
using namespace vln_memory;

/** @brief Release 模式也报告失败，附上下文。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 确认已发布观测的代表点能从同一真实深度像素重算。 */
void checkProjection(
  const PreparationReader & reader, const Observation & observation, const DepthRegion & shape,
  const DepthRegionOptions & options)
{
  const auto result = projectObservationDepthRegion(reader, observation, shape, options);
  expect(result.projection.has_value(), result.message);
  const auto & projection = *result.projection;
  expect(result.ok(), depthRegionCodeName(projection.code));
  const auto & representative = *projection.representative;
  const auto loaded = reader.loadPair(observation.pair_id);
  expect(loaded.ok() && loaded.pair->depth, loaded.message);
  const auto point = projectDepthPixelToMap(*loaded.pair->depth, *loaded.pair->depth_model,
    *loaded.pair->depth_pose, observation.map_frame, representative.pixel.u, representative.pixel.v);
  expect(point.ok() && *point.point_map_m == representative.point_map_m, "Representative is not an observed point");
  expect(projection.region_pixels == projection.valid_points + projection.invalid_depth_pixels +
    projection.projection_failures, "Region counts do not reconcile");
  std::cout << "pixels=" << projection.region_pixels << " valid=" << projection.valid_points
            << " clusters=" << projection.clusters.size() << " top_points=" << projection.clusters.front().point_count
            << " representative_pixel=[" << representative.pixel.u << ',' << representative.pixel.v
            << "] optical_z_m=" << representative.optical_z_m << " representative_map_m=["
            << representative.point_map_m[0] << ',' << representative.point_map_m[1] << ','
            << representative.point_map_m[2] << "]\n";
}

/** @brief 改动来源指纹、消息摘要或历史时刻时，禁止输出地图点。 */
void checkRejections(const PreparationReader & reader, const Observation & observation, const DepthRegion & shape)
{
  auto changed = observation;
  changed.source_manifest_sha256.assign(64, '0');
  auto result = projectObservationDepthRegion(reader, changed, shape);
  expect(result.code == ObservationDepthRegionCode::source_mismatch && !result.projection,
    "Wrong source fingerprint was accepted");
  changed = observation;
  auto & hash = changed.evidence.front().payload_sha256;
  hash.front() = hash.front() == '0' ? '1' : '0';
  result = projectObservationDepthRegion(reader, changed, shape);
  expect(result.code == ObservationDepthRegionCode::source_mismatch && !result.projection,
    "Wrong source evidence was accepted");
  changed = observation;
  changed.depth_calibration->calibration_id += "_other";
  result = projectObservationDepthRegion(reader, changed, shape);
  expect(result.code == ObservationDepthRegionCode::source_mismatch && !result.projection,
    "Wrong calibration identity was accepted");
  changed = observation;
  auto & time = changed.depth_camera_pose_map->time.stamp;
  time.nanosec = (time.nanosec + 1) % 1000000000U;
  result = projectObservationDepthRegion(reader, changed, shape);
  expect(result.code == ObservationDepthRegionCode::invalid_observation && !result.projection,
    "Wrong historical query time was accepted");
  changed = observation;
  changed.depth_camera_pose_map.reset();
  changed.pending_reasons = collectObservationPendingReasons(changed);
  result = projectObservationDepthRegion(reader, changed, shape);
  expect(result.code == ObservationDepthRegionCode::missing_depth_pose && !result.projection,
    "Missing historical pose was accepted");
}

/** @brief 从 ROS 录包夹具生成正式观测，过 JSON 契约后验证读回。 */
void fixture(const fs::path & parent)
{
  std::ifstream latest(parent / "latest.txt");
  std::string path;
  expect(static_cast<bool>(std::getline(latest, path)), "Cannot locate preparation fixture");
  auto opened = openPreparation(fs::path(path) / "normal_no_cdr_out");
  expect(opened.ok() && !opened.reader->pairs().empty(), opened.message);
  const ObservationBuildContext context{"fixture_map", "fixture_revision", std::string(opened.reader->mapFrame()),
    "Synthetic recorded TF", 1, {}};
  const auto built = buildObservation(*opened.reader, opened.reader->pairs().front().pair_id, context);
  expect(built.ok(), built.message);
  const auto json = serializeObservation(*built.observation);
  expect(json.ok(), json.message);
  const auto parsed = deserializeObservation(json.json);
  expect(parsed.ok(), parsed.message);
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const DepthBoundingBox one_pixel{0, 0, 1, 1};
  checkProjection(*opened.reader, *parsed.observation, one_pixel, options);
  checkRejections(*opened.reader, *parsed.observation, one_pixel);
}

/** @brief 用已有观测文件与指定深度像素窗口复现真实录包链路。 */
void published(const fs::path & source, const fs::path & observations, std::int64_t u, std::int64_t v)
{
  auto opened = openPreparation(source);
  expect(opened.ok(), opened.message);
  std::ifstream file(observations);
  std::string line;
  expect(static_cast<bool>(std::getline(file, line)), "Cannot read first published observation");
  const auto parsed = deserializeObservation(line);
  expect(parsed.ok(), parsed.message);
  const auto & observation = *parsed.observation;
  expect(!observation.pending_reasons.empty(), "Real fixture should retain source pending");
  std::cout << "pair=" << observation.pair_id << " depth_stamp=" << observation.depth_image.stamp.sec << ':'
            << observation.depth_image.stamp.nanosec << " map=" << observation.map_frame << '\n';
  const DepthBoundingBox box{u, v, u + 16, v + 16};
  const DepthPolygon polygon{{{u, v}, {u + 15, v}, {u, v + 15}}};
  checkProjection(*opened.reader, observation, box, {});
  checkProjection(*opened.reader, observation, polygon, {});
  checkRejections(*opened.reader, observation, box);
}
}  // namespace

/** @brief 运行录包夹具或真实数据集验收。 */
int main(int argc, char ** argv)
{
  try {
    if (argc == 3 && std::string_view(argv[1]) == "--fixture-pointer") {
      fixture(argv[2]);
    } else if (argc == 5) {
      const auto u = std::stoll(argv[3]);
      const auto v = std::stoll(argv[4]);
      expect(u >= 0 && v >= 0 && u <= UINT32_MAX && v <= UINT32_MAX, "Invalid diagnostic origin");
      published(argv[1], argv[2], u, v);
    } else {
      throw std::invalid_argument("Expected --fixture-pointer DIR or PREPARATION OBSERVATIONS U V");
    }
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
