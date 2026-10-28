#include "vln_memory/bag/rgb/rgb.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Bytes = std::vector<std::uint8_t>;
using vln_memory::RgbErrorCode;

// 不依赖 assert，Release/NDEBUG 构建也实际执行检查。
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// 测试输入借用 Bytes；调用方需保持它存活，便于复现正式接口的所有权约定。
vln_memory::RgbImageView view(
  const Bytes & data, std::uint32_t width = 2, std::uint32_t height = 2,
  std::string_view encoding = "rgb8", std::uint32_t step = 6)
{
  vln_memory::RgbImageView input;
  input.stamp = {123, 456};
  input.frame_id = "camera_color_optical_frame";
  input.width = width;
  input.height = height;
  input.encoding = encoding;
  input.step = step;
  input.data = data.data();
  input.data_size = data.size();
  return input;
}

void expectError(
  const vln_memory::RgbImageView & input, RgbErrorCode code,
  const vln_memory::RgbLimits & limits = {})
{
  const auto result = vln_memory::decodeRgb(input, limits);
  expect(!result.ok(), "Invalid input was accepted");
  expect(result.code == code, std::string("Unexpected error: ") + result.message);
  expect(!result.frame.has_value(), "Failed decode exposed a partial frame");
  expect(!result.message.empty(), "Failure omitted its reason");
}

void testRgbAndMetadata()
{
  const Bytes source{255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
  auto input = view(source);
  input.stamp = {-1, 500000000};
  input.is_bigendian = 1;
  const auto result = vln_memory::decodeRgb(input);
  expect(result.ok(), result.message);
  const auto & frame = result.frame.value();
  expect(frame.pixels == source, "rgb8 channel values changed");
  expect(frame.encoding == "rgb8" && frame.step == 6, "Invalid packed output layout");
  expect(frame.width == 2 && frame.height == 2, "Dimensions changed");
  expect(frame.stamp.sec == -1 && frame.stamp.nanosec == 500000000, "Stamp changed");
  expect(frame.frame_id == input.frame_id, "frame_id changed");
  expect(frame.source_encoding == "rgb8" && frame.source_step == 6, "Source metadata lost");
  expect(frame.source_is_bigendian, "8-bit source endianness flag was lost");
}

// 行尾放置与图像颜色不同的哨兵值，暴露把填充误当像素的实现。
void testPaddedColorRows()
{
  const Bytes expected{255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
  const Bytes rgb{255, 0, 0, 0, 255, 0, 17, 19, 0, 0, 255, 255, 255, 255, 23, 29};
  const Bytes bgr{0, 0, 255, 0, 255, 0, 31, 37, 255, 0, 0, 255, 255, 255, 41, 43};
  for (const auto encoding : {"rgb8", "bgr8"}) {
    const auto & data = std::string_view(encoding) == "rgb8" ? rgb : bgr;
    const auto result = vln_memory::decodeRgb(view(data, 2, 2, encoding, 8));
    expect(result.ok(), result.message);
    expect(result.frame->pixels == expected, "Padding or color channels were exported as pixels");
    expect(result.frame->step == 6 && result.frame->source_step == 8, "Stride metadata wrong");
  }
}

void testMonoAndEmptyFrame()
{
  const Bytes source{0, 17, 201, 127, 255, 202};
  auto input = view(source, 2, 2, "mono8", 3);
  input.stamp = {0, 0};
  input.frame_id = {};
  input.is_bigendian = 1;
  const auto result = vln_memory::decodeRgb(input);
  const Bytes expected{0, 0, 0, 17, 17, 17, 127, 127, 127, 255, 255, 255};
  expect(result.ok(), result.message);
  expect(result.frame->pixels == expected, "mono8 intensities changed");
  expect(result.frame->frame_id.empty(), "RGB module invented a frame_id");
  expect(result.frame->stamp.sec == 0 && result.frame->stamp.nanosec == 0, "Zero stamp rejected");
}

// 同时修改并销毁源像素和字符串，检查输出没有遗漏任何借用引用。
void testOutputOwnership()
{
  const Bytes expected{1, 2, 3};
  vln_memory::RgbResult result;
  {
    Bytes source{3, 2, 1};
    std::string frame_id = "original_frame";
    std::string encoding = "bgr8";
    auto input = view(source, 1, 1, encoding, 3);
    input.frame_id = frame_id;
    result = vln_memory::decodeRgb(input);
    expect(result.ok(), result.message);
    source.assign(3, 0);
    frame_id.assign("changed_frame");
    encoding.assign("changed_encoding");
  }
  expect(result.frame->pixels == expected, "Output borrows released input pixels");
  expect(result.frame->frame_id == "original_frame", "Output borrows frame_id");
  expect(result.frame->source_encoding == "bgr8", "Output borrows encoding");
}

void testInvalidMetadata()
{
  const Bytes source(12, 0);
  auto input = view(source);
  input.width = 0;
  expectError(input, RgbErrorCode::invalid_dimensions);
  input = view(source);
  input.height = 0;
  expectError(input, RgbErrorCode::invalid_dimensions);
  input = view(source);
  input.stamp.nanosec = 1000000000U;
  expectError(input, RgbErrorCode::invalid_stamp);
  for (const auto encoding : {"", "RGB8", "rgba8", "16UC1", "jpeg"}) {
    input = view(source);
    input.encoding = encoding;
    expectError(input, RgbErrorCode::unsupported_encoding);
  }
  input = view(source);
  input.is_bigendian = 2;
  expectError(input, RgbErrorCode::invalid_endianness);
}

void testInvalidBuffers()
{
  const Bytes source(12, 0);
  auto input = view(source);
  input.step = 5;
  expectError(input, RgbErrorCode::invalid_step);
  input.step = 0;
  expectError(input, RgbErrorCode::invalid_step);
  input = view(source);
  input.data = nullptr;
  expectError(input, RgbErrorCode::invalid_data);
  input = view(source);
  input.data_size = 11;
  expectError(input, RgbErrorCode::invalid_data);
  input.data_size = 13;
  expectError(input, RgbErrorCode::invalid_data);
  input.data_size = 0;
  expectError(input, RgbErrorCode::invalid_data);
}

// 覆盖刚好等于上限的成功情况，避免只检查超限却遗漏边界误拒绝。
void testLimitsAndOverflow()
{
  const Bytes source(16, 0);
  const auto input = view(source, 2, 2, "rgb8", 8);
  vln_memory::RgbLimits limits{4, 16};
  expect(vln_memory::decodeRgb(input, limits).ok(), "Exact limit should succeed");
  limits.max_pixels = 3;
  expectError(input, RgbErrorCode::size_limit_exceeded, limits);
  limits = {4, 15};
  expectError(input, RgbErrorCode::size_limit_exceeded, limits);
  limits = {0, 16};
  expectError(input, RgbErrorCode::invalid_limits, limits);
  limits = {4, 0};
  expectError(input, RgbErrorCode::invalid_limits, limits);

  // 极大尺寸必须在访问这段很小的源缓冲区之前被拒绝。
  auto huge = view(source);
  huge.width = std::numeric_limits<std::uint32_t>::max();
  huge.height = std::numeric_limits<std::uint32_t>::max();
  huge.step = std::numeric_limits<std::uint32_t>::max();
  expectError(huge, RgbErrorCode::size_overflow);
  huge = view(source, 4096, 4097, "rgb8", 12288);
  expectError(huge, RgbErrorCode::size_limit_exceeded);
}

void testStableErrorNames()
{
  const std::vector<RgbErrorCode> codes{
    RgbErrorCode::none, RgbErrorCode::invalid_limits, RgbErrorCode::invalid_dimensions,
    RgbErrorCode::invalid_stamp, RgbErrorCode::unsupported_encoding,
    RgbErrorCode::invalid_endianness, RgbErrorCode::invalid_step,
    RgbErrorCode::size_overflow, RgbErrorCode::size_limit_exceeded, RgbErrorCode::invalid_data};
  for (const auto code : codes) {
    expect(std::string(vln_memory::rgbErrorCodeName(code)) != "unknown", "Missing error name");
  }
  expect(std::string(vln_memory::rgbErrorCodeName(RgbErrorCode::invalid_data)) ==
    "invalid_data", "Report error name changed");
}

void testRandomColorSamples()
{
  // 从期望 RGB 像素构造三种不同的输入布局，独立验证输出颜色与逐行边界。
  std::mt19937 random(20261028U);
  for (const auto encoding : {"rgb8", "bgr8", "mono8"}) {
    for (std::uint32_t sample = 0; sample < 300; ++sample) {
      const auto width = 1U + static_cast<std::uint32_t>(random() % 17U);
      const auto height = 1U + static_cast<std::uint32_t>(random() % 9U);
      const auto padding = static_cast<std::uint32_t>(random() % 7U);
      const bool mono = std::string_view(encoding) == "mono8";
      const auto channels = mono ? 1U : 3U;
      const auto step = width * channels + padding;
      Bytes source(static_cast<std::size_t>(step) * height, 0xDA);
      Bytes expected;
      expected.reserve(static_cast<std::size_t>(width) * height * 3);
      for (std::size_t row = 0; row < height; ++row) {
        for (std::size_t column = 0; column < width; ++column) {
          const auto red = static_cast<std::uint8_t>(random() % 256U);
          const auto green = mono ? red : static_cast<std::uint8_t>(random() % 256U);
          const auto blue = mono ? red : static_cast<std::uint8_t>(random() % 256U);
          expected.insert(expected.end(), {red, green, blue});
          const auto offset = row * step + column * channels;
          if (mono) {
            source[offset] = red;
          } else if (std::string_view(encoding) == "rgb8") {
            source[offset] = red; source[offset + 1] = green; source[offset + 2] = blue;
          } else {
            source[offset] = blue; source[offset + 1] = green; source[offset + 2] = red;
          }
        }
      }
      const auto result = vln_memory::decodeRgb(view(source, width, height, encoding, step));
      expect(result.ok(), result.message);
      expect(result.frame->pixels == expected, "Random color/padding sample failed");
    }
  }
}

}  // namespace

int main()
{
  try {
    testRgbAndMetadata();
    testPaddedColorRows();
    testMonoAndEmptyFrame();
    testOutputOwnership();
    testInvalidMetadata();
    testInvalidBuffers();
    testLimitsAndOverflow();
    testStableErrorNames();
    testRandomColorSamples();
    std::cout << "RGB core: 9 test groups passed (including 900 generated samples)\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "RGB core failed: " << error.what() << '\n';
    return 1;
  }
}
