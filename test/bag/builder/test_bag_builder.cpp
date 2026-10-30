#include "vln_memory/bag/builder/bag_builder.hpp"
#include "vln_memory/bag/builder/bag_reader_ros.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
namespace fs = std::filesystem;
using namespace vln_memory;
using Json = nlohmann::json;
enum class FixtureKind {normal, wrong_type, tf_conflict, changed_calibration, time_reset, invalid_depth};

/** @brief Release 同样检查测试条件，失败描述实际不符合的契约。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 构造明确同钟同段、双流标定及注册声明，时间容差独立。 */
BagBuildOptions fixtureOptions(const fs::path & input, const fs::path & output)
{
  BagBuildOptions options;
  options.bag_uri = input.string();
  options.output_directory = output.string();
  options.clock_id = "fixture_clock";
  options.clock_relation = SensorClockRelation::same_clock;
  options.clock_basis = "Generated messages use the same declared sensor clock";
  options.segment_id = "fixture_segment";
  options.segment_basis = "Single fixture epoch, no resets";
  options.sync_tolerance_ns = 5000000;
  options.tf_max_gap_ns = 6000000000LL;
  options.body_frame = "base";
  options.rgb_camera.image_topic = "/rgb";
  options.rgb_camera.camera_info_topic = "/rgb_info";
  options.rgb_camera.optical_frame = "camera";
  options.rgb_camera.binding_basis = "Synthetic static pinhole model";
  options.rgb_camera.image_domain = CameraImageDomain::raw;
  options.rgb_camera.time_policy = CameraInfoTimePolicy::static_calibration;
  options.rgb_camera.clock_relation = SensorClockRelation::same_clock;
  options.depth_camera = options.rgb_camera;
  options.depth_camera.image_topic = "/depth";
  options.depth_camera.camera_info_topic = "/depth_info";
  options.depth.input_unit = DepthUnit::millimeters;
  options.depth.convention = DepthConvention::optical_z;
  options.depth.alignment = DepthAlignment::aligned_to_rgb;
  options.depth.alignment_basis = "Synthetic depth on the declared RGB grid";
  options.depth.rgb_grid = {2, 1, "camera"};
  options.min_valid_depth_ratio = 0.2;
  return options;
}

/** @brief 构造有行填充的 BGR 两像素，标准输出应是红色和蓝色。 */
sensor_msgs::msg::Image rgb(std::int32_t sec)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera";
  image.header.stamp.sec = sec;
  image.width = 2;
  image.height = 1;
  image.encoding = "bgr8";
  image.step = 8;
  image.data = {0, 0, 255, 255, 0, 0, 77, 88};
  return image;
}

/** @brief 深度是小端 uint16 毫米，允许 0 验证 NaN 保留和质量统计。 */
sensor_msgs::msg::Image depth(std::int32_t sec, std::uint32_t nanosec,
  std::uint16_t first, std::uint16_t second)
{
  auto image = rgb(sec);
  image.header.stamp.nanosec = nanosec;
  image.encoding = "16UC1";
  image.step = 4;
  image.data = {static_cast<std::uint8_t>(first & 255U), static_cast<std::uint8_t>(first >> 8U),
    static_cast<std::uint8_t>(second & 255U), static_cast<std::uint8_t>(second >> 8U)};
  return image;
}

/** @brief 合法完整 CameraInfo，不因为是 raw 图就省略 R/P。 */
sensor_msgs::msg::CameraInfo calibration()
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera";
  info.width = 2;
  info.height = 1;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {1, 0, 1, 0, 1, 0, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  return info;
}

/** @brief 原始父子变换，动态样本和静态外参都使用单位四元数。 */
geometry_msgs::msg::TransformStamped transform(
  const std::string & parent, const std::string & child, std::int32_t sec, double x)
{
  geometry_msgs::msg::TransformStamped value;
  value.header.frame_id = parent;
  value.child_frame_id = child;
  value.header.stamp.sec = sec;
  value.transform.translation.x = x;
  value.transform.rotation.w = 1;
  return value;
}

/** @brief 将独立字节交给官方 Writer，支持负记录时间及坏 CDR 边界测试。 */
void writeBytes(rosbag2_cpp::Writer & writer, const std::string & topic, std::int64_t record_ns,
  const std::uint8_t * data, std::size_t bytes)
{
  auto buffer = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t(
    rcutils_get_zero_initialized_uint8_array()), [](auto * value) {
      const auto status = rcutils_uint8_array_fini(value);
      (void)status;
      delete value;
    });
  auto allocator = rcutils_get_default_allocator();
  expect(rcutils_uint8_array_init(buffer.get(), bytes, &allocator) == RCUTILS_RET_OK, "Fixture buffer allocation failed");
  if (bytes > 0) {std::memcpy(buffer->buffer, data, bytes);}
  buffer->buffer_length = bytes;
  auto raw = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  raw->serialized_data = std::move(buffer);
  raw->topic_name = topic;
  raw->time_stamp = record_ns;
  writer.write(raw);
}

/** @brief 使用真实 ROS 类型支持生成 CDR；测试不手工伪造有效消息序列化。 */
template<typename Message>
void writeMessage(rosbag2_cpp::Writer & writer, const std::string & topic,
  std::int64_t record_ns, const Message & value)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<Message> encoder;
  encoder.serialize_message(&value, &serialized);
  const auto & bytes = serialized.get_rcl_serialized_message();
  writeBytes(writer, topic, record_ns, bytes.buffer, bytes.buffer_length);
}

/** @brief 创建固定话题清单，未选择的大消息只用于强制 SQLite 分卷和 inventory 核查。 */
void createTopics(rosbag2_cpp::Writer & writer, FixtureKind kind)
{
  std::string rgb_type = "sensor_msgs/msg/Image";
  if (kind == FixtureKind::wrong_type) {rgb_type = "sensor_msgs/msg/CompressedImage";}
  writer.create_topic({"/rgb", rgb_type, "cdr", ""});
  writer.create_topic({"/depth", "sensor_msgs/msg/Image", "cdr", ""});
  writer.create_topic({"/rgb_info", "sensor_msgs/msg/CameraInfo", "cdr", ""});
  writer.create_topic({"/depth_info", "sensor_msgs/msg/CameraInfo", "cdr", ""});
  writer.create_topic({"/tf", "tf2_msgs/msg/TFMessage", "cdr", ""});
  writer.create_topic({"/tf_static", "tf2_msgs/msg/TFMessage", "cdr", ""});
  writer.create_topic({"/not_selected", "sensor_msgs/msg/Image", "cdr", ""});
}

/** @brief 构造五类消息、分卷、重复记录时间、负记录时间、坏 CDR 和迟到 TF。 */
void createFixture(const fs::path & path, FixtureKind kind)
{
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage;
  storage.uri = path.string();
  storage.storage_id = "sqlite3";
  storage.max_bagfile_size = 200000;
  writer.open(storage);
  createTopics(writer, kind);
  writeMessage(writer, "/rgb", -1000000000LL, rgb(10));
  auto noise = rgb(10);
  noise.data.resize(1000000, 0);
  writeMessage(writer, "/not_selected", -999999999LL, noise);
  writeMessage(writer, "/rgb_info", 0, calibration());
  writeMessage(writer, "/depth_info", 0, calibration());
  std::uint16_t first = 1000;
  if (kind == FixtureKind::invalid_depth) {first = 0;}
  writeMessage(writer, "/depth", 1000000000LL, depth(10, 4000000, first, 0));
  writeMessage(writer, "/rgb", 2000000000LL, rgb(11));
  if (kind == FixtureKind::invalid_depth) {writeMessage(writer, "/depth", 2000000000LL, depth(10, 996000000, 0, 0));}
  else {writeMessage(writer, "/depth", 2000000000LL, depth(10, 996000000, 2000, 3000));}
  writeMessage(writer, "/rgb", 3000000000LL, rgb(12));
  writeMessage(writer, "/depth", 4000000000LL, depth(14, 0, 0, 0));
  const std::array<std::uint8_t, 4> bad_cdr{0xff, 0xff, 0xff, 0xff};
  writeBytes(writer, "/rgb", 5000000000LL, bad_cdr.data(), bad_cdr.size());
  tf2_msgs::msg::TFMessage fixed;
  fixed.transforms.push_back(transform("base", "camera", 0, 0.1));
  writeMessage(writer, "/tf_static", 6000000000LL, fixed);
  tf2_msgs::msg::TFMessage dynamic;
  dynamic.transforms = {transform("map", "base", 9, 1), transform("map", "base", 15, 7)};
  writeMessage(writer, "/tf", 7000000000LL, dynamic);
  if (kind == FixtureKind::tf_conflict) {fixed.transforms[0].transform.translation.x = 2;}
  writeMessage(writer, "/tf_static", 8000000000LL, fixed);
  writeMessage(writer, "/tf", 9000000000LL, tf2_msgs::msg::TFMessage{});
  if (kind == FixtureKind::changed_calibration) {
    auto changed = calibration();
    changed.k[0] = 2;
    writeMessage(writer, "/rgb_info", 10000000000LL, changed);
  }
  if (kind == FixtureKind::time_reset) {writeMessage(writer, "/rgb", 10000000000LL, rgb(1));}
  writer.close();
}

/** @brief 读取完整 JSON 对象，损坏数据直接使测试失败。 */
Json readJson(const fs::path & path)
{
  std::ifstream input(path);
  expect(static_cast<bool>(input), "Missing JSON: " + path.string());
  Json value;
  input >> value;
  return value;
}

/** @brief 逐行解析真实产物，空行或非对象记录都拒绝。 */
std::vector<Json> readLines(const fs::path & path)
{
  std::ifstream input(path);
  expect(static_cast<bool>(input), "Missing JSONL: " + path.string());
  std::vector<Json> records;
  std::string line;
  while (std::getline(input, line)) {
    auto record = Json::parse(line);
    expect(record.is_object(), "JSONL item is not object");
    records.push_back(std::move(record));
  }
  return records;
}

/** @brief 验证来源/配对/质量状态和 PNG 色彩，保留产物供独立 NumPy 测试。 */
void checkNormal(const fs::path & output, const BagBuildResult & result)
{
  expect(result.ok(), "Normal fixture build failed: " + result.code + " " + result.message);
  expect(result.rgb_frames == 4 && result.depth_frames == 3 && result.decoded_frames == 6,
    "Frame/decode counts incorrect");
  expect(result.pairs == 2 && result.prepared_pairs == 2, "Valid sensor preparation pairs missing");
  const auto manifest = readJson(output / "bag_manifest.json");
  expect(manifest.at("source_files").size() >= 2, "Split fixture did not create multiple files");
  expect(!manifest.at("pose_computed").get<bool>(), "Uncomputed pose was claimed");
  bool negative_record = false;
  for (const auto & message : readLines(output / "messages.jsonl")) {
    if (message.at("bag_record_time_ns") == "-1000000000") {negative_record = true;}
    expect(fs::is_regular_file(output / message.at("raw_path").get<std::string>()), "Missing raw evidence");
  }
  expect(negative_record, "Negative bag record time was silently skipped");
  for (const auto & frame : readLines(output / "frames.jsonl")) {
    if (frame.at("kind") == "rgb" && frame.at("decode_code") == "none") {
      const auto image = cv::imread((output / frame.at("media_path").get<std::string>()).string());
      expect(image.rows == 1 && image.cols == 2, "PNG dimensions wrong");
      expect(image.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255) &&
        image.at<cv::Vec3b>(0, 1) == cv::Vec3b(255, 0, 0), "PNG red/blue channels swapped");
    }
  }
  const auto summary = readJson(output / "reports/summary.json");
  expect(summary.at("tf_duplicates") == 1 && summary.at("tf_unique_samples") == 3, "TF duplicate provenance wrong");
  const auto pairs = readLines(output / "rgbd_pairs.jsonl");
  expect(pairs[0].at("delta_ns") == "4000000" && pairs[1].at("delta_ns") == "-4000000",
    "Header time pairing used record time or wrong sign");
}

/** @brief 生成可直接运行 CLI 的示例配置，不把生成 bag 或证据加入 Git。 */
void writeFixtureConfig(const fs::path & path, const BagBuildOptions & options)
{
  YAML::Node root;
  auto bag = root["bag"];
  bag["input"] = options.bag_uri;
  bag["output"] = options.output_directory;
  bag["clock_id"] = options.clock_id;
  bag["clock_relation"] = "same_clock";
  bag["clock_basis"] = options.clock_basis;
  bag["segment_id"] = options.segment_id;
  bag["segment_basis"] = options.segment_basis;
  bag["body_frame"] = "base";
  bag["sync_tolerance_ns"] = 5000000;
  bag["tf_max_gap_ns"] = 6000000000LL;
  for (const auto * name : {"rgb", "depth"}) {
    auto camera = bag[std::string(name) + "_camera"];
    camera["image_topic"] = std::string("/") + name;
    camera["camera_info_topic"] = std::string("/") + name + "_info";
    camera["optical_frame"] = "camera";
    camera["binding_basis"] = "Synthetic static fixture";
    camera["image_domain"] = "raw";
    camera["time_policy"] = "static_calibration";
  }
  auto depth_options = bag["depth"];
  depth_options["input_unit"] = "millimeters";
  depth_options["convention"] = "optical_z";
  depth_options["alignment"] = "aligned_to_rgb";
  depth_options["alignment_basis"] = options.depth.alignment_basis;
  depth_options["rgb_width"] = 2;
  depth_options["rgb_height"] = 1;
  depth_options["rgb_frame"] = "camera";
  std::ofstream output(path);
  output << root;
  output.close();
  expect(static_cast<bool>(output), "Fixture config write failed");
}

/** @brief 配置加载错误必须稳定拒绝，不能把拼错/负容量当默认值。 */
void configCases(const fs::path & root, const fs::path & config)
{
  auto options = loadBagBuildOptions(config.string());
  expect(validateBagBuildOptions(options).empty(), "Fixture YAML not usable");
  for (const auto * variant : {"unknown", "negative", "duplicate"}) {
    const auto path = root / (std::string(variant) + ".yaml");
    auto yaml = YAML::LoadFile(config.string());
    if (std::string(variant) == "unknown") {yaml["bag"]["typo"] = true;}
    else if (std::string(variant) == "negative") {yaml["bag"]["limits"]["max_frames"] = -1;}
    std::ofstream output(path);
    if (std::string(variant) == "duplicate") {output << "bag:\n  input: a\n  input: b\n";}
    else {output << yaml;}
    output.close();
    bool rejected = false;
    try {loadBagBuildOptions(path.string());}
    catch (const BagBuildError & error) {rejected = error.code() == "invalid_config";}
    expect(rejected, "Invalid YAML was not rejected: " + std::string(variant));
  }
}

/** @brief 失败不发布，旧结果不覆盖；静态标定/TF 变化不被最后一条掩盖。 */
void failureAndQualityCases(const fs::path & root)
{
  const std::vector<std::pair<FixtureKind, std::string>> variants{
    {FixtureKind::wrong_type, "wrong_type"}, {FixtureKind::tf_conflict, "tf_conflict"},
    {FixtureKind::changed_calibration, "calibration_change"}, {FixtureKind::time_reset, "time_reset"},
    {FixtureKind::invalid_depth, "invalid_depth"}};
  for (const auto & variant : variants) {
    const auto input = root / (variant.second + "_bag");
    const auto output = root / (variant.second + "_out");
    createFixture(input, variant.first);
    const auto result = buildBag(fixtureOptions(input, output));
    if (variant.first == FixtureKind::wrong_type || variant.first == FixtureKind::time_reset) {
      expect(!result.ok() && !fs::exists(output), "Fatal failure published output");
      if (variant.first == FixtureKind::wrong_type) {expect(result.code == "topic_type_mismatch", "Wrong type misclassified");}
      else {expect(result.code == "ambiguous_time_segment", "Clock reset was silently sorted away");}
    } else {
      expect(result.ok() && result.prepared_pairs == 0 && result.pairs == 2, "Quality/conflict did not gate pairs");
      const auto pairs = readLines(output / "rgbd_pairs.jsonl");
      expect(pairs[0].at("status") == "pending", "Bad pair marked prepared");
    }
  }
}

/** @brief 不改动原 fixture，复制输入后改变文件，第二遍必须拒绝跨遍来源变化。 */
void sourceChangeCase(const fs::path & root)
{
  const auto input = root / "changed_source_bag";
  fs::copy(root / "normal_bag", input, fs::copy_options::recursive);
  const auto work = root / "scan_work";
  fs::create_directory(work);
  const auto options = fixtureOptions(input, root / "unused_scan_output");
  OutputFiles files(work, options.max_output_bytes);
  const auto scan = scanBag(options, files);
  files.close();
  std::ofstream altered(scan.source_files.front().path, std::ios::binary | std::ios::app);
  altered.put('x');
  altered.close();
  bool rejected = false;
  try {readBagImages(options, scan, [](const auto &, const auto *) {});}
  catch (const BagBuildError & error) {rejected = error.code() == "source_changed";}
  expect(rejected, "Changed source accepted by second pass");
}

/** @brief 同输入重复构建、覆盖/预算失败、单文件范围和所有特殊路径回归。 */
void integrationCases(const fs::path & root)
{
  const auto input = root / "normal_bag";
  createFixture(input, FixtureKind::normal);
  auto options = fixtureOptions(input, root / "normal_out");
  checkNormal(root / "normal_out", buildBag(options));
  const auto duplicate = buildBag(options);
  expect(!duplicate.ok() && duplicate.code == "output_exists", "Existing output overwritten");
  options.output_directory = (root / "repeat_out").string();
  checkNormal(root / "repeat_out", buildBag(options));
  options.output_directory = (root / "budget_fail").string();
  options.max_output_bytes = 1;
  const auto budget = buildBag(options);
  expect(!budget.ok() && budget.code == "limit_exceeded" && !fs::exists(root / "budget_fail"),
    "Budget failure published complete output");
  const auto manifest = readJson(root / "normal_out/bag_manifest.json");
  const auto single_file = manifest.at("source_files")[0].at("path").get<std::string>();
  auto single = fixtureOptions(single_file, root / "single_out");
  expect(buildBag(single).ok(), "Single SQLite file extraction failed");
  expect(readJson(root / "single_out/bag_manifest.json").at("input_scope") == "single_storage_file",
    "Single file was falsely described as complete directory bag");
  auto demo = fixtureOptions(input, root / "cli_out");
  writeFixtureConfig(root / "demo_config.yaml", demo);
  configCases(root, root / "demo_config.yaml");
  failureAndQualityCases(root);
  sourceChangeCase(root);
}
}

/** @brief 创建独立临时产物并写 latest.txt 供 NumPy/人工检查，生成物不入 Git。 */
int main(int argc, char ** argv)
{
  try {
    fs::path parent = fs::temp_directory_path() / "vln_memory_bag_tests";
    if (argc > 1) {parent = argv[1];}
    fs::create_directories(parent);
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto root = parent / ("run_" + std::to_string(tick));
    expect(fs::create_directory(root), "Test directory not fresh");
    integrationCases(root);
    std::ofstream latest(parent / "latest.txt");
    latest << root.string() << '\n';
    latest.close();
    expect(static_cast<bool>(latest), "Cannot write artifact pointer");
    std::cout << "BagBuilder integration passed. Artifacts: " << root << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "BagBuilder integration failed: " << error.what() << '\n';
    return 1;
  }
}
