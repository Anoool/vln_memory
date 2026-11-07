#include <vln_memory/bag/builder/media_writer.hpp>

#include <array>
#include <cstring>
#include <limits>
#include <opencv2/imgcodecs.hpp>

namespace vln_memory
{
namespace
{
/** @brief 编码器收到的大小必须符合已验证拥有型帧，公开字段被改坏时拒绝。 */
void checkRgbFrame(const RgbFrame & frame)
{
  if (frame.width == 0 || frame.height == 0 || frame.encoding != "rgb8" ||
    frame.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    frame.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    static_cast<std::uint64_t>(frame.width) * frame.height * 3 != frame.pixels.size() ||
    static_cast<std::uint64_t>(frame.width) * 3 != frame.step)
  {
    throw BagBuildError("invalid_frame", "Invalid tight RGB8 frame passed to writer");
  }
}

/** @brief 只允许 reader 生成的文件 ID 字符，防止来源 topic 被误当文件路径。 */
void checkMessageId(const std::string & message_id)
{
  if (message_id.empty()) {throw BagBuildError("invalid_reference", "Empty message ID");}
  for (const char character : message_id) {
    if ((character < 'a' || character > 'z') && (character < 'A' || character > 'Z') &&
      (character < '0' || character > '9') && character != '_' && character != '-')
    {
      throw BagBuildError("invalid_reference", "Unsafe message ID for output filename");
    }
  }
}

/** @brief 创建符合 NPY 1.0 的二维 float32 header，长度对齐并固定小端表示。 */
std::vector<std::uint8_t> depthNpyHeader(std::uint32_t width, std::uint32_t height)
{
  std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" +
    std::to_string(height) + ", " + std::to_string(width) + "), }";
  const auto padding = (64 - ((10 + header.size() + 1) % 64)) % 64;
  header.append(padding, ' ');
  header.push_back('\n');
  if (header.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw BagBuildError("invalid_frame", "NPY header is too large");
  }
  std::vector<std::uint8_t> result{0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0};
  result.push_back(static_cast<std::uint8_t>(header.size() & 0xffU));
  result.push_back(static_cast<std::uint8_t>((header.size() >> 8U) & 0xffU));
  result.insert(result.end(), header.begin(), header.end());
  return result;
}

}  // namespace

/** @brief 保存工作目录和预算；调用方负责使用新目录，输入目录不被修改。 */
OutputFiles::OutputFiles(std::filesystem::path root, std::uint64_t max_bytes)
: root_(std::move(root)), max_bytes_(max_bytes)
{
  if (!std::filesystem::is_directory(root_) || max_bytes_ == 0) {
    throw BagBuildError("invalid_output", "Fresh output directory and positive budget required");
  }
}

/** @brief 检查路径片段，再由 root 解析；本 writer 只在私有新目录内创建文件。 */
std::filesystem::path OutputFiles::targetPath(const std::string & relative_path) const
{
  const std::filesystem::path relative(relative_path);
  if (relative.empty() || relative.is_absolute() || relative_path.find('\0') != std::string::npos) {
    throw BagBuildError("invalid_output", "Relative path required");
  }
  for (const auto & component : relative) {
    if (component == "..") {throw BagBuildError("invalid_output", "Parent path is forbidden");}
  }
  return root_ / relative;
}

/** @brief 在预算内累计实际字节；先减后比较避免加法溢出。 */
void OutputFiles::reserveBytes(std::size_t bytes)
{
  if (bytes > max_bytes_ - written_bytes_) {throw BagBuildError("limit_exceeded", "Output byte budget exceeded");}
  written_bytes_ += bytes;
}

/** @brief 写完整字节文件，不覆盖旧路径；错误使当前数据集不能发布。 */
void OutputFiles::write(const std::string & relative_path, const std::uint8_t * data, std::size_t bytes)
{
  const auto target = targetPath(relative_path);
  if (std::filesystem::exists(target)) {throw BagBuildError("output_exists", "Refusing to overwrite " + relative_path);}
  if (bytes > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()) || (bytes > 0 && data == nullptr)) {
    throw BagBuildError("invalid_output", "Invalid output byte span");
  }
  reserveBytes(bytes);
  std::filesystem::create_directories(target.parent_path());
  const auto temporary = target.string() + ".tmp";
  std::ofstream output(temporary, std::ios::binary);
  if (!output) {throw BagBuildError("io_error", "Cannot create " + temporary);}
  if (bytes > 0) {output.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(bytes));}
  output.close();
  if (!output) {throw BagBuildError("io_error", "Cannot finish " + temporary);}
  std::filesystem::rename(temporary, target);
}

/** @brief 为固定种类 JSONL 保持一个流；源消息文件仍逐帧关闭，不累积文件描述符。 */
void OutputFiles::ensureLines(const std::string & relative_path)
{
  if (line_streams_.count(relative_path) != 0) {return;}
  const auto target = targetPath(relative_path);
  if (std::filesystem::exists(target)) {throw BagBuildError("output_exists", "Refusing to overwrite " + relative_path);}
  std::filesystem::create_directories(target.parent_path());
  auto & stream = line_streams_[relative_path];
  stream.open(target, std::ios::binary);
  if (!stream) {throw BagBuildError("io_error", "Cannot create " + relative_path);}
}

/** @brief 只写完整文本行；不能包含换行分隔符，避免一条记录变成多条。 */
void OutputFiles::appendLine(const std::string & relative_path, const std::string & line)
{
  if (line.find('\n') != std::string::npos || line.find('\r') != std::string::npos ||
    line.size() >= static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
  {
    throw BagBuildError("invalid_output", "JSONL line contains raw newline or exceeds stream size");
  }
  reserveBytes(line.size() + 1);
  ensureLines(relative_path);
  auto & stream = line_streams_.at(relative_path);
  stream.write(line.data(), static_cast<std::streamsize>(line.size()));
  stream.put('\n');
  if (!stream) {throw BagBuildError("io_error", "Cannot append " + relative_path);}
}

/** @brief 显式关闭并校验延迟写失败；成功后清空流集合。 */
void OutputFiles::close()
{
  for (auto & entry : line_streams_) {
    entry.second.close();
    if (!entry.second) {throw BagBuildError("io_error", "Cannot close " + entry.first);}
  }
  line_streams_.clear();
}

/** @brief 转成 OpenCV 所需 BGR 后编码，避免保存后红蓝通道互换。 */
std::string writeRgb(OutputFiles & files, const std::string & message_id, const RgbFrame & frame)
{
  checkMessageId(message_id);
  checkRgbFrame(frame);
  cv::Mat bgr(static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3);
  for (std::size_t row = 0; row < frame.height; ++row) {
    auto * output_row = bgr.ptr<std::uint8_t>(static_cast<int>(row));
    for (std::size_t column = 0; column < frame.width; ++column) {
      const auto offset = row * frame.step + column * 3;
      output_row[column * 3] = frame.pixels[offset + 2];
      output_row[column * 3 + 1] = frame.pixels[offset + 1];
      output_row[column * 3 + 2] = frame.pixels[offset];
    }
  }
  std::vector<std::uint8_t> png;
  if (!cv::imencode(".png", bgr, png)) {throw BagBuildError("io_error", "PNG encoding failed");}
  const auto path = "evidence/rgb/" + message_id + ".png";
  files.write(path, png.data(), png.size());
  return path;
}

/** @brief 显式按 IEEE-754 位模式写小端 NPY，避免依赖主机字节序。 */
std::string writeDepth(OutputFiles & files, const std::string & message_id, const DepthFrame & frame)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "IEEE float32 required");
  checkMessageId(message_id);
  const auto pixels = static_cast<std::uint64_t>(frame.width) * frame.height;
  if (frame.width == 0 || frame.height == 0 || pixels != frame.depth_m.size() ||
    frame.unit != DepthUnit::meters || frame.convention != DepthConvention::optical_z ||
    frame.encoding != "32FC1" || pixels > (std::numeric_limits<std::size_t>::max() - 256) / 4)
  {
    throw BagBuildError("invalid_frame", "Invalid float32 meter optical-z frame");
  }
  auto npy = depthNpyHeader(frame.width, frame.height);
  npy.reserve(npy.size() + frame.depth_m.size() * 4);
  for (const auto value : frame.depth_m) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned int byte = 0; byte < 4; ++byte) {
      npy.push_back(static_cast<std::uint8_t>((bits >> (byte * 8U)) & 0xffU));
    }
  }
  const auto path = "evidence/depth/" + message_id + ".npy";
  files.write(path, npy.data(), npy.size());
  return path;
}

/** @brief 按源步长和字节序读整数，经共用预算写独立 PNG，失败时不发布。 */
std::string writeDepthPng16(
  OutputFiles & files, const std::string & message_id, const sensor_msgs::msg::Image & source)
{
  checkMessageId(message_id);
  if (source.encoding != "16UC1" || source.width == 0 || source.height == 0 ||
    source.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    source.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    source.step < static_cast<std::uint64_t>(source.width) * 2 ||
    static_cast<std::uint64_t>(source.step) * source.height != source.data.size() ||
    source.is_bigendian > 1)
  {
    throw BagBuildError("invalid_frame", "Invalid source layout for 16UC1 PNG");
  }
  cv::Mat pixels(static_cast<int>(source.height), static_cast<int>(source.width), CV_16UC1);
  const std::size_t high_byte = source.is_bigendian ? 0U : 1U;
  for (std::size_t row = 0; row < source.height; ++row) {
    const auto * source_row = source.data.data() + row * source.step;
    auto * output_row = pixels.ptr<std::uint16_t>(static_cast<int>(row));
    for (std::size_t column = 0; column < source.width; ++column) {
      const auto * sample = source_row + column * 2;
      output_row[column] = static_cast<std::uint16_t>(
        (static_cast<std::uint32_t>(sample[high_byte]) << 8U) | sample[1U - high_byte]);
    }
  }

  std::vector<std::uint8_t> png;
  if (!cv::imencode(".png", pixels, png, {cv::IMWRITE_PNG_COMPRESSION, 3})) {
    throw BagBuildError("io_error", "Depth PNG encoding failed");
  }
  const auto path = "evidence/depth/" + message_id + ".png";
  files.write(path, png.data(), png.size());
  return path;
}

}  // namespace vln_memory
