#include <vln_memory/engine/observation_dataset_reader.hpp>
#include <vln_memory/engine/entity_builder.hpp>
#include <vln_memory/engine/observation_depth_region.hpp>

#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace
{
using namespace vln_memory;
namespace fs = std::filesystem;
using Code = ObservationDatasetReadCode;

/** @brief Release 构建也保留验证。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 从发布目录通过正常 API 回读顺序、身份、来源及所有观测。 */
void verify(const fs::path & root, std::size_t expected, bool sealed, const fs::path & override_root = {})
{
  const auto opened = openObservationDataset(root);
  expect(opened.ok(), opened.message);
  const auto & reader = *opened.reader;
  const auto & info = reader.info();
  expect(info.file_integrity_verified == sealed && info.observation_count == expected,
    "Unexpected dataset count or integrity mode");
  if (!override_root.empty()) {expect(!reader.openSourcePreparation().ok(), "Missing original path accepted");}
  const auto source = reader.openSourcePreparation(override_root);
  expect(source.ok(), source.message);
  expect(source.reader->manifestSha256() == info.source_manifest_sha256 &&
    source.reader->mapFrame() == info.map_frame, "Source binding lost");
  if (info.preparation_bundled) {
    expect(source.reader->root() == fs::canonical(root) / "preparation", "Bundle did not resolve its own preparation");
  }
  expect(reader.loadObservation("absent").code == Code::observation_not_found, "Unknown ID was accepted");
  expect(reader.loadBatch(reader.batches().size()).code == Code::batch_not_found, "Unknown batch was accepted");
  std::size_t records = 0;
  std::size_t pending = 0;
  for (const auto & entry : reader.batches()) {
    const auto batch = reader.loadBatch(entry.batch_index);
    expect(batch.ok() && batch.observations.size() == entry.observation_ids.size(), batch.message);
    for (std::size_t i = 0; i < batch.observations.size(); ++i) {
      const auto & observation = batch.observations[i];
      const auto by_id = reader.loadObservation(entry.observation_ids[i]);
      expect(by_id.ok() && by_id.observation->id == observation.id &&
        observation.id == entry.observation_ids[i] && observation.map_id == info.map_id &&
        observation.revision == info.revision, "By-ID and batch results differ");
      if (!observation.pending_reasons.empty()) {++pending;}
      ++records;
    }
  }
  expect(records == expected && pending == info.pending_count, "Readback count differs");
}

/** @brief 改写夹具的一个字节并恢复修改时间，保留文件大小和 JSON 结构。 */
void changeFixtureText(const fs::path & path)
{
  const auto modified = fs::last_write_time(path);
  std::fstream output(path, std::ios::binary | std::ios::in | std::ios::out);
  const std::string content{std::istreambuf_iterator<char>(output), std::istreambuf_iterator<char>()};
  const auto location = content.find("Fixture recorded TF");
  expect(location != std::string::npos, "Missing test text");
  output.seekp(static_cast<std::streamoff>(location));
  output.put('f');
  output.close();
  expect(static_cast<bool>(output), "Cannot edit test row");
  fs::last_write_time(path, modified);
}

/** @brief 观测行和清单均须识别同大小、同修改时间的改写。 */
void changedAfterOpen(const fs::path & root, bool change_manifest)
{
  const auto opened = openObservationDataset(root);
  expect(opened.ok() && !opened.reader->batches().empty(), opened.message);
  const auto file = change_manifest ? "observation_manifest.json" : "observations.jsonl";
  changeFixtureText(root / file);
  const auto batch = opened.reader->loadBatch(0);
  expect(!batch.ok() && batch.code == Code::source_changed && batch.observations.empty(),
    "Same-size dataset replacement was not detected by batch read");
  const auto single = opened.reader->loadObservation(opened.reader->batches().front().observation_ids.front());
  expect(single.code == Code::source_changed && !single.observation, "Same-size replacement reached the by-ID read");
  if (change_manifest) {
    const auto source = opened.reader->openSourcePreparation();
    expect(source.code == PreparationReadCode::source_changed && !source.reader,
      "An old manifest binding was reused after its bytes changed");
  }
}

/** @brief 检查有问题的目录在打开边界被拒绝。 */
void rejectOpen(const fs::path & root, std::string_view expected)
{
  const auto opened = openObservationDataset(root);
  expect(!opened.ok() && !opened.reader, "Unexpectedly opened invalid dataset");
  if (expected == "batch") {expect(opened.code == Code::invalid_batch, opened.message);}
  else if (expected == "record") {expect(opened.code == Code::invalid_observation, opened.message);}
  else if (expected == "digest") {expect(opened.code == Code::integrity_mismatch, opened.message);}
  else {expect(expected == "manifest" && opened.code == Code::invalid_manifest, opened.message);}
}
/** @brief 使用读取器返回的同源记录直接投影和创建物品。 */
void projectBundled(const fs::path & root, std::int64_t u, std::int64_t v)
{
  const auto opened = openObservationDataset(root);
  expect(opened.ok() && opened.reader->info().preparation_bundled, opened.message);
  const auto source = opened.reader->openSourcePreparation();
  expect(source.ok(), source.message);
  const auto batch = opened.reader->loadBatch(0);
  expect(batch.ok() && !batch.observations.empty(), batch.message);
  const auto & observation = batch.observations.front();
  const DepthBoundingBox selection{u, v, u + 1, v + 1};
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const auto projected = projectObservationDepthRegion(*source.reader, observation, selection, options);
  expect(projected.ok(), projected.message);
  const auto entity = createObjectEntity(*source.reader, observation, "测试物品", selection, options);
  expect(entity.ok() && entity.entity->status == EntityStatus::located, entity.message);
  const auto region = createRegionEntity(*source.reader, observation, "测试区域");
  expect(region.ok() && region.entity->status == EntityStatus::pending_location &&
    region.entity->location_pending_reason == "missing_region_confirmation", region.message);
}

/** @brief 使用搬移后的直出目录读媒体、投影，或验证篡改拒绝。 */
void verifyDirect(const fs::path & root, bool corrupt, std::int64_t u = 0, std::int64_t v = 0)
{
  const auto opened = openObservationDataset(root);
  expect(opened.ok() && opened.reader->info().direct_evidence, opened.message);
  const auto & reader = *opened.reader;
  expect(!reader.openSourcePreparation().ok(), "Direct observations required preparation");
  const auto batch = reader.loadBatch(0);
  expect(batch.ok() && !batch.observations.empty(), batch.message);
  const auto & id = batch.observations.front().id;
  const auto rgb = reader.loadRgb(id);
  const auto depth = reader.loadDepth(id);
  if (corrupt) {
    expect(rgb.code == Code::integrity_mismatch && depth.code == Code::integrity_mismatch,
      "Tampered image or depth media was accepted");
    return;
  }
  expect(rgb.ok() && rgb.png.size() > 8 && rgb.png[0] == 137 && rgb.png[1] == 'P', rgb.message);
  expect(depth.ok(), depth.message);
  DepthRegionOptions options;
  options.min_cluster_points = 1;
  const DepthBoundingBox selection{u, v, u + 1, v + 1};
  const auto projected = projectObservationDepthRegion(reader, id, selection, options);
  expect(projected.ok(), projected.message);
  const auto entity = createObjectEntity(reader, id, "测试物品", selection, options);
  expect(entity.ok() && entity.entity->status == EntityStatus::located, entity.message);
}

}  // namespace

/** @brief 与 Python 发布夹具配合，也可独立验证任意已发布目录。 */
int main(int argc, char ** argv)
{
  try {
    expect(argc >= 3, "Expected MODE DATASET [OPTIONS]");
    const std::string_view mode(argv[1]);
    if (mode == "--verify" && (argc == 5 || argc == 6)) {
      verify(argv[2], std::stoull(argv[3]), std::string_view(argv[4]) == "v2", argc == 6 ? fs::path(argv[5]) : fs::path{});
    } else if (mode == "--direct" && argc == 5) {
      verifyDirect(argv[2], false, std::stoll(argv[3]), std::stoll(argv[4]));
    } else if (mode == "--direct-corrupt" && argc == 3) {
      verifyDirect(argv[2], true);
    } else if (mode == "--project" && argc == 5) {
      projectBundled(argv[2], std::stoll(argv[3]), std::stoll(argv[4]));
    } else if (mode == "--reject" && argc == 4) {
      rejectOpen(argv[2], argv[3]);
    } else if ((mode == "--changed" || mode == "--changed-manifest") && argc == 3) {
      changedAfterOpen(argv[2], mode == "--changed-manifest");
    } else if (mode == "--wrong-source" && (argc == 4 || argc == 5)) {
      const auto opened = openObservationDataset(argv[2]);
      expect(opened.ok(), opened.message);
      const auto wrong = opened.reader->openSourcePreparation(argv[3]);
      expect(!wrong.ok() && wrong.code == PreparationReadCode::invalid_reference, "Wrong source accepted");
      if (argc == 5) {expect(wrong.message.find(argv[4]) != std::string::npos, wrong.message);}
    } else if (mode == "--reject-on-read" && argc == 3) {
      const auto opened = openObservationDataset(argv[2]);
      expect(opened.ok(), opened.message);
      const auto batch = opened.reader->loadBatch(0);
      expect(batch.code == Code::invalid_observation && batch.observations.empty(), "Malformed observation accepted");
    } else {
      throw std::invalid_argument("Unsupported dataset reader test invocation");
    }
    std::cout << "Observation dataset reader checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
