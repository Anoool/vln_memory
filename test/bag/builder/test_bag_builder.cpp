#include <vln_memory/bag/builder/bag_builder.hpp>
#include <vln_memory/bag/builder/bag_reader_ros.hpp>

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
#include <cv_bridge/cv_bridge.h>
#include <compressed_depth_image_transport/codec.h>
#include <rclcpp/serialization.hpp>
#include <rcutils/error_handling.h>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
namespace fs = std::filesystem;
using namespace vln_memory;
using Json = nlohmann::json;
enum class FixtureKind {
  normal, d8, wrong_type, tf_conflict, changed_calibration, time_reset, invalid_depth,
  compressed, mixed_compressed, bad_compressed, offset_depth, near_depth, near_compressed, far_depth
};

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
  options.sync_tolerance_ns = 0;
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
sensor_msgs::msg::Image depth(
  std::int32_t sec, std::uint32_t nanosec,
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
void writeBytes(
  rosbag2_cpp::Writer & writer, const std::string & topic, std::int64_t record_ns,
  const std::uint8_t * data, std::size_t bytes)
{
  auto buffer = std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t(
      rcutils_get_zero_initialized_uint8_array()), [](auto * value) {
      const auto status = rcutils_uint8_array_fini(value);
      (void)status;
      delete value;
    });
  auto allocator = rcutils_get_default_allocator();
  expect(
    rcutils_uint8_array_init(buffer.get(), bytes, &allocator) == RCUTILS_RET_OK,
    "Fixture buffer allocation failed");
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
void writeMessage(
  rosbag2_cpp::Writer & writer, const std::string & topic,
  std::int64_t record_ns, const Message & value)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<Message> encoder;
  encoder.serialize_message(&value, &serialized);
  const auto & bytes = serialized.get_rcl_serialized_message();
  writeBytes(writer, topic, record_ns, bytes.buffer, bytes.buffer_length);
}

/** @brief 通过标准编码器写压缩或原 Image，损坏负载仍保留可读的 ROS header。 */
void writeFixtureImage(
  rosbag2_cpp::Writer & writer, const std::string & topic, std::int64_t record_ns,
  const sensor_msgs::msg::Image & image, FixtureKind kind)
{
  bool compressed = kind == FixtureKind::compressed || kind == FixtureKind::bad_compressed ||
    kind == FixtureKind::near_compressed;
  if (kind == FixtureKind::mixed_compressed && topic == "/depth") {compressed = true;}
  if (!compressed) {writeMessage(writer, topic, record_ns, image); return;}
  sensor_msgs::msg::CompressedImage message;
  if (topic == "/depth") {
    const auto encoded = compressed_depth_image_transport::encodeCompressedDepthImage(image, 10, 100, 1);
    expect(encoded != nullptr, "Fixture depth compression failed");
    message = *encoded;
  } else {
    message.header = image.header;
    message.format = "bgr8; jpeg compressed bgr8";
    const auto pixels = cv_bridge::toCvCopy(image, "bgr8");
    expect(cv::imencode(".jpg", pixels->image, message.data), "Fixture JPEG compression failed");
    if (kind == FixtureKind::bad_compressed && image.header.stamp.sec == 10) {message.data.resize(3);}
  }
  writeMessage(writer, topic, record_ns, message);
}

/** @brief 创建固定话题清单，未选择的大消息只用于强制 SQLite 分卷和 inventory 核查。 */
void createTopics(rosbag2_cpp::Writer & writer, FixtureKind kind)
{
  std::string rgb_type = "sensor_msgs/msg/Image";
  if (kind == FixtureKind::wrong_type) {rgb_type = "std_msgs/msg/String";}
  if (kind == FixtureKind::compressed || kind == FixtureKind::bad_compressed ||
    kind == FixtureKind::near_compressed)
  {
    rgb_type = "sensor_msgs/msg/CompressedImage";
  }
  std::string depth_type = "sensor_msgs/msg/Image";
  if (kind == FixtureKind::compressed || kind == FixtureKind::mixed_compressed ||
    kind == FixtureKind::bad_compressed || kind == FixtureKind::near_compressed)
  {
    depth_type = "sensor_msgs/msg/CompressedImage";
  }
  writer.create_topic({"/rgb", rgb_type, "cdr", ""});
  writer.create_topic({"/depth", depth_type, "cdr", ""});
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
  writeFixtureImage(writer, "/rgb", -1000000000LL, rgb(10), kind);
  auto noise = rgb(10);
  noise.data.resize(1000000, 0);
  writeMessage(writer, "/not_selected", -999999999LL, noise);
  auto camera_info = calibration();
  if (kind == FixtureKind::d8) {camera_info.d.resize(8, 0);}
  writeMessage(writer, "/rgb_info", 0, camera_info);
  writeMessage(writer, "/depth_info", 0, camera_info);
  std::uint16_t first = 1000;
  if (kind == FixtureKind::invalid_depth) {first = 0;}
  std::uint32_t offset_ns = 0;
  if (kind == FixtureKind::offset_depth) {offset_ns = 1;}
  if (kind == FixtureKind::far_depth) {offset_ns = 3;}
  auto first_depth = depth(10, offset_ns, first, 0);
  // 负 1 ns 跨秒边界，确保实现没有用无符号相减或覆盖成 RGB 时间。
  if (kind == FixtureKind::near_depth || kind == FixtureKind::near_compressed) {
    first_depth.header.stamp.sec = 9;
    first_depth.header.stamp.nanosec = 999999999;
  }
  writeFixtureImage(writer, "/depth", 1000000000LL, first_depth, kind);
  writeFixtureImage(writer, "/rgb", 2000000000LL, rgb(11), kind);
  if (kind == FixtureKind::invalid_depth) {
    writeFixtureImage(writer, "/depth", 2000000000LL, depth(11, offset_ns, 0, 0), kind);
  } else {
    if (kind == FixtureKind::near_depth || kind == FixtureKind::near_compressed) {offset_ns = 2;}
    writeFixtureImage(writer, "/depth", 2000000000LL, depth(11, offset_ns, 2000, 3000), kind);
  }
  writeFixtureImage(writer, "/rgb", 3000000000LL, rgb(12), kind);
  writeFixtureImage(writer, "/depth", 4000000000LL, depth(14, 0, 0, 0), kind);
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
  expect(
    result.rgb_frames == 4 && result.depth_frames == 3 && result.decoded_frames == 6,
    "Frame/decode counts incorrect");
  expect(result.pairs == 2 && result.prepared_pairs == 2, "Valid sensor preparation pairs missing");
  const auto manifest = readJson(output / "bag_manifest.json");
  expect(manifest.at("source_files").size() >= 2, "Split fixture did not create multiple files");
  expect(manifest.at("pose_computed") == true && result.posed_pairs == 2, "Numeric poses not exported");
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
      expect(
        image.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255) &&
        image.at<cv::Vec3b>(0, 1) == cv::Vec3b(255, 0, 0), "PNG red/blue channels swapped");
    }
  }
  const auto summary = readJson(output / "reports/summary.json");
  expect(summary.at("tf_duplicates") == 1 && summary.at("tf_unique_samples") == 3, "TF duplicate provenance wrong");
  const auto pairs = readLines(output / "rgbd_pairs.jsonl");
  expect(
    pairs[0].at("delta_ns") == "0" && pairs[1].at("delta_ns") == "0",
    "Strict header pairing used record time or accepted unequal stamps");
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
  bag["sync_tolerance_ns"] = options.sync_tolerance_ns;
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
  auto negative = options;
  negative.sync_tolerance_ns = -1;
  const auto refused = buildBag(negative);
  expect(!refused.ok() && refused.code == "invalid_config", "Negative tolerance accepted through C++ API");
  auto nonzero = options;
  nonzero.sync_tolerance_ns = 1;
  expect(validateBagBuildOptions(nonzero).empty(), "Explicit positive tolerance rejected");
  writeFixtureConfig(root / "positive_sync.yaml", nonzero);
  expect(loadBagBuildOptions((root / "positive_sync.yaml").string()).sync_tolerance_ns == 1,
    "YAML did not preserve positive integer nanoseconds");
  auto conflicting = options;
  conflicting.depth.rgb_grid.frame_id = "different_optical_frame";
  expect(!validateBagBuildOptions(conflicting).empty(), "Conflicting registration configuration accepted");
  conflicting = options;
  conflicting.depth_camera.camera_info_topic = conflicting.rgb_camera.camera_info_topic;
  conflicting.depth.alignment = DepthAlignment::unknown;
  conflicting.depth.rgb_grid = {};
  expect(!validateBagBuildOptions(conflicting).empty(), "Unregistered depth silently shared RGB calibration");
  for (const auto * variant : {"unknown", "negative", "negative_grid", "duplicate", "negative_sync", "overflow_sync", "fractional_sync"}) {
    const auto path = root / (std::string(variant) + ".yaml");
    auto yaml = YAML::LoadFile(config.string());
    if (std::string(variant) == "unknown") {yaml["bag"]["typo"] = true;} else if (std::string(variant) == "negative") {
      yaml["bag"]["limits"]["max_frames"] = -1;
    } else if (std::string(variant) == "negative_grid") {yaml["bag"]["depth"]["rgb_width"] = -1;}
    else if (std::string(variant) == "negative_sync") {yaml["bag"]["sync_tolerance_ns"] = -1;}
    else if (std::string(variant) == "overflow_sync") {yaml["bag"]["sync_tolerance_ns"] = "9223372036854775808";}
    else if (std::string(variant) == "fractional_sync") {yaml["bag"]["sync_tolerance_ns"] = 1.5;}
    std::ofstream output(path);
    if (std::string(variant) == "duplicate") {output << "bag:\n  input: a\n  input: b\n";} else {output << yaml;}
    output.close();
    bool rejected = false;
    try {
      loadBagBuildOptions(path.string());
    } catch (const BagBuildError & error) {
      rejected = error.code() == "invalid_config";
    }
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
      if (variant.first == FixtureKind::wrong_type) {
        expect(result.code == "topic_type_mismatch", "Wrong type misclassified");
      } else {expect(result.code == "ambiguous_time_segment", "Clock reset was silently sorted away");}
    } else {
      expect(result.ok() && result.prepared_pairs == 0 && result.pairs == 2, "Quality/conflict did not gate pairs");
      const auto pairs = readLines(output / "rgbd_pairs.jsonl");
      expect(pairs[0].at("status") == "pending", "Bad pair marked prepared");
    }
  }
}

/** @brief 精确/相差 1ns、压缩/混合输入和坏压缩体都保持同一零容差契约。 */
void strictCompressedCases(const fs::path & root)
{
  const std::vector<std::pair<FixtureKind, std::string>> variants{
    {FixtureKind::compressed, "compressed"}, {FixtureKind::mixed_compressed, "mixed"},
    {FixtureKind::bad_compressed, "bad_compressed"}, {FixtureKind::offset_depth, "offset_1ns"}};
  for (const auto & variant : variants) {
    const auto input = root / (variant.second + "_bag");
    const auto output = root / (variant.second + "_out");
    createFixture(input, variant.first);
    const auto result = buildBag(fixtureOptions(input, output));
    expect(result.ok(), "Strict/compressed fixture failed: " + result.message);
    const auto summary = readJson(output / "reports/summary.json");
    expect(summary.at("approximate_pairs") == 0, "Approximate pair escaped strict builder");
    expect(summary.at("capture_sync_status") == "unknown", "Unproven exposure sync claimed");
    if (variant.first == FixtureKind::offset_depth) {
      expect(result.pairs == 0 && result.decoded_frames == 6, "1ns mismatch passed or lost unpaired media");
      expect(result.timestamp_match_status == "no_exact_pairs", "No-pair status missing");
    } else {
      expect(result.pairs == 2, "Strict compressed pairs missing");
      for (const auto & pair : readLines(output / "rgbd_pairs.jsonl")) {
        expect(pair.at("delta_ns") == "0" && pair.at("exact") == true, "Non-exact pair published");
      }
    }
    if (variant.first == FixtureKind::bad_compressed) {
      expect(result.decoded_frames == 5 && result.prepared_pairs == 1, "Bad payload was silently replaced");
      const auto frames = readLines(output / "frames.jsonl");
      expect(frames[0].at("metadata_available") == true && frames[0].at("stamp").at("sec") == 10,
        "Bad compressed body erased original header");
      expect(frames[0].at("decode_code") == "invalid_compressed_payload", "Payload error confused with CDR");
    }
  }
}

/** @brief 核查近似报告/来源/真实差值，不用准备质量计数替代时间匹配结果。 */
void checkNearestOutput(const fs::path & output, std::int64_t tolerance_ns, std::size_t expected_pairs)
{
  const auto summary = readJson(output / "reports/summary.json");
  const auto manifest = readJson(output / "bag_manifest.json");
  const auto policy = "rgb_reference_monotonic_nearest_one_to_one_v1";
  expect(summary.at("timestamp_match_policy") == policy, "Summary mislabeled nearest policy");
  expect(manifest.at("timestamp_match_policy") == policy && manifest.at("builder_version") == 5,
    "Manifest did not declare current behavior");
  expect(summary.at("sync_tolerance_ns") == std::to_string(tolerance_ns), "Missing summary tolerance");
  expect(manifest.at("sync_tolerance_ns") == std::to_string(tolerance_ns), "Missing manifest tolerance");
  expect(summary.at("pairs") == expected_pairs && summary.at("approximate_pairs") == expected_pairs,
    "Nearest pair counts incorrect");
  const auto pairs = readLines(output / "rgbd_pairs.jsonl");
  expect(pairs.size() == expected_pairs, "Pair export count differs");
  std::size_t index = 0;
  for (const auto & pair : pairs) {
    expect(pair.at("policy") == policy && pair.at("exact") == false, "Approximate pair marked exact");
    expect(pair.at("sync_tolerance_ns") == std::to_string(tolerance_ns), "Pair threshold differs");
    expect(pair.at("capture_sync_status") == "unknown", "Exposure sync invented");
    if (index == 0) {
      expect(pair.at("delta_ns") == "-1" && pair.at("depth_stamp").at("sec") == 9 &&
        pair.at("depth_stamp").at("nanosec") == 999999999, "Negative delta or original stamp changed");
    } else {
      expect(pair.at("delta_ns") == "2" && pair.at("depth_stamp").at("nanosec") == 2,
        "Positive delta or original stamp changed");
    }
    ++index;
  }
}

/** @brief 验证近似模式中的相等对、空结果与资源预算，不假定正门限一定有近似对。 */
void nearestSpecialCases(const fs::path & root)
{
  // 正门限描述选择策略；如果真实 stamp 恰好相等，exact 仍必须为 true。
  auto exact = fixtureOptions(root / "normal_bag", root / "nearest_exact_out");
  exact.sync_tolerance_ns = 1;
  expect(buildBag(exact).pairs == 2, "Exact stamps lost in nearest mode");
  for (const auto & pair : readLines(root / "nearest_exact_out/rgbd_pairs.jsonl")) {
    expect(pair.at("exact") == true && pair.at("delta_ns") == "0", "Exact pair forced approximate");
  }
  createFixture(root / "far_bag", FixtureKind::far_depth);
  auto empty = fixtureOptions(root / "far_bag", root / "nearest_empty_out");
  empty.sync_tolerance_ns = 2;
  const auto none = buildBag(empty);
  expect(none.ok() && none.pairs == 0 && none.timestamp_match_status == "no_pairs",
    "Positive threshold with no pairs mislabeled exact");
  bool no_pairs_issue = false;
  for (const auto & issue : readLines(root / "nearest_empty_out/reports/issues.jsonl")) {
    if (issue.at("code") == "no_rgbd_pairs") {no_pairs_issue = true;}
    expect(issue.at("code") != "no_exact_rgbd_pairs", "Approximate mode emitted strict-only issue");
  }
  expect(no_pairs_issue, "No-pair aggregate issue missing");
  auto budget = fixtureOptions(root / "far_bag", root / "nearest_budget_out");
  budget.sync_tolerance_ns = 2;
  budget.max_frames = 1;
  expect(buildBag(budget).code == "limit_exceeded", "Nearest mode bypassed frame budget");
}

/** @brief 原始和压缩输入均验收零、边界及超差；输出保留给独立 CDR/JSON 回查。 */
void nearestCases(const fs::path & root)
{
  for (const auto kind : {FixtureKind::near_depth, FixtureKind::near_compressed}) {
    std::string prefix = "near_raw";
    if (kind == FixtureKind::near_compressed) {prefix = "near_compressed";}
    const auto input = root / (prefix + "_bag");
    createFixture(input, kind);
    for (const std::int64_t tolerance_ns : {0, 1, 2}) {
      const auto output = root / (prefix + "_" + std::to_string(tolerance_ns) + "_out");
      auto options = fixtureOptions(input, output);
      options.sync_tolerance_ns = tolerance_ns;
      const auto result = buildBag(options);
      expect(result.ok() && result.decoded_frames == 6, "Nearest fixture failed or lost media");
      if (tolerance_ns == 0) {
        expect(result.pairs == 0 && result.timestamp_match_status == "no_exact_pairs", "Zero compatibility lost");
      } else {
        checkNearestOutput(output, tolerance_ns, static_cast<std::size_t>(tolerance_ns));
      }
    }
  }
  nearestSpecialCases(root);
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
  try {
    readBagImages(
      options, scan, [](const auto &, const auto *) {
      });
  } catch (const BagBuildError & error) {
    rejected = error.code() == "source_changed";
  }
  expect(rejected, "Changed source accepted by second pass");
}

/** @brief 真实序列化 D8 贯穿 reader/索引/匹配/模型/JSON，严格模式仍明确拒绝。 */
void cameraCompatibilityCases(const fs::path & root)
{
  const auto input = root / "d8_bag";
  createFixture(input, FixtureKind::d8);
  auto options = fixtureOptions(input, root / "d8_strict_out");
  const auto strict = buildBag(options);
  expect(strict.ok() && strict.camera_models == 0, "Strict D8 bypassed");
  options.output_directory = (root / "d8_compatible_out").string();
  options.rgb_camera.compatibility = CameraInfoCompatibility::plumb_bob_zero_tail_d8;
  options.depth_camera.compatibility = options.rgb_camera.compatibility;
  const auto compatible = buildBag(options);
  expect(compatible.ok() && compatible.camera_models == 6 && compatible.prepared_pairs == 2,
    "D8 did not reach prepared pairs");
  for (const auto & info : readLines(root / "d8_compatible_out/camera_info.jsonl")) {
    expect(info.at("d").size() == 8 && info.at("effective_d").size() == 5 &&
      info.at("normalization_applied") == true && info.at("validation_code") == "invalid_distortion" &&
      info.at("normalized_code") == "none", "Raw/derived calibration or statuses lost");
  }
  for (const auto & frame : readLines(root / "d8_compatible_out/frames.jsonl")) {
    if (frame.at("media_path").is_null()) {continue;}
    expect(frame.at("effective_model_computed") == true &&
      frame.at("camera_model").at("image_domain") == "raw" &&
      frame.at("camera_model").at("normalization_applied") == true,
      "Frame lost model or normalization provenance");
  }
  for (const auto & issue : readLines(root / "d8_compatible_out/reports/issues.jsonl")) {
    expect(issue.at("code") != "invalid_distortion", "Recovered calibration reported as unusable");
  }
  auto yaml = YAML::LoadFile((root / "demo_config.yaml").string());
  yaml["bag"]["rgb_camera"]["compatibility"] = "plumb_bob_zero_tail_d8_v1";
  const auto config = root / "compatible_config.yaml";
  {std::ofstream output(config); output << yaml;}
  expect(loadBagBuildOptions(config.string()).rgb_camera.compatibility ==
    CameraInfoCompatibility::plumb_bob_zero_tail_d8, "YAML compatibility lost");
  yaml["bag"]["rgb_camera"]["compatibility"] = "guess";
  {std::ofstream output(config); output << yaml;}
  bool rejected = false;
  try {loadBagBuildOptions(config.string());} catch (const BagBuildError &) {rejected = true;}
  expect(rejected, "Unknown YAML compatibility accepted");
}

/** @brief 同输入重复构建、覆盖/预算失败、单文件范围和所有特殊路径回归。 */
void integrationCases(const fs::path & root)
{
  const auto input = root / "normal_bag";
  createFixture(input, FixtureKind::normal);
  auto options = fixtureOptions(input, root / "normal_out");
  const auto normal = buildBag(options);
  expect(!rcutils_error_is_set(), "Message decoding left stale global ROS error state");
  checkNormal(root / "normal_out", normal);
  const auto duplicate = buildBag(options);
  expect(!duplicate.ok() && duplicate.code == "output_exists", "Existing output overwritten");
  options.output_directory = (root / "repeat_out").string();
  checkNormal(root / "repeat_out", buildBag(options));
  options.output_directory = (root / "budget_fail").string();
  options.max_output_bytes = 1;
  const auto budget = buildBag(options);
  expect(
    !budget.ok() && budget.code == "limit_exceeded" && !fs::exists(root / "budget_fail"),
    "Budget failure published complete output");
  const auto manifest = readJson(root / "normal_out/bag_manifest.json");
  const auto single_file = manifest.at("source_files")[0].at("path").get<std::string>();
  auto single = fixtureOptions(single_file, root / "single_out");
  expect(buildBag(single).ok(), "Single SQLite file extraction failed");
  expect(
    readJson(root / "single_out/bag_manifest.json").at("input_scope") == "single_storage_file",
    "Single file was falsely described as complete directory bag");
  auto demo = fixtureOptions(input, root / "cli_out");
  writeFixtureConfig(root / "demo_config.yaml", demo);
  configCases(root, root / "demo_config.yaml");
  failureAndQualityCases(root);
  sourceChangeCase(root);
  strictCompressedCases(root);
  nearestCases(root);
  cameraCompatibilityCases(root);
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
