#include "vln_memory/bag/depth/depth.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Bytes = std::vector<std::uint8_t>;
using vln_memory::DepthErrorCode;
using vln_memory::DepthUnit;

void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

vln_memory::DepthOptions options(DepthUnit unit = DepthUnit::millimeters)
{
  vln_memory::DepthOptions result;
  result.input_unit = unit;
  result.convention = vln_memory::DepthConvention::optical_z;
  return result;
}

vln_memory::DepthImageView view(
  const Bytes & data, std::uint32_t width = 2, std::uint32_t height = 2,
  std::string_view encoding = "16UC1", std::uint32_t step = 4, bool big_endian = false)
{
  vln_memory::DepthImageView result;
  result.stamp = {-1, 500000000};
  result.frame_id = "depth_optical";
  result.width = width;
  result.height = height;
  result.encoding = encoding;
  result.step = step;
  result.is_bigendian = big_endian ? 1 : 0;
  result.data = data.data();
  result.data_size = data.size();
  return result;
}

void expectError(
  const vln_memory::DepthImageView & input, DepthErrorCode code,
  const vln_memory::DepthOptions & interpretation = options(),
  const vln_memory::DepthLimits & limits = {})
{
  const auto result = vln_memory::decodeDepth(input, interpretation, limits);
  expect(!result.ok() && !result.frame.has_value(), "Bad depth returned a partial frame");
  expect(result.code == code, std::string("Unexpected depth error: ") + result.message);
  expect(!result.message.empty(), "Failure lacks a reason");
}

void expectValue(float actual, float expected)
{
  if (std::isnan(expected)) {
    expect(std::isnan(actual), "Invalid depth was not NaN");
  } else {
    expect(std::isfinite(actual) && std::abs(actual - expected) <= 0.00002F,
      "Depth numeric value changed");
  }
}

// 仅用于生成样本；关键端序测试另用手写字节，避免测试与实现共享同一种错误。
void storeBits(Bytes & data, std::size_t offset, std::uint32_t bits, std::size_t bytes, bool big)
{
  for (std::size_t index = 0; index < bytes; ++index) {
    const auto shift = static_cast<unsigned int>((big ? bytes - 1 - index : index) * 8);
    data[offset + index] = static_cast<std::uint8_t>((bits >> shift) & 0xFFU);
  }
}

Bytes floatBytes(const std::vector<float> & values)
{
  Bytes result(values.size() * 4);
  for (std::size_t index = 0; index < values.size(); ++index) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &values[index], sizeof(bits));
    storeBits(result, index * 4, bits, 4, false);
  }
  return result;
}

void checkStatistics(const vln_memory::DepthFrame & frame)
{
  const auto & s = frame.statistics;
  expect(s.invalid_pixels == s.zero_pixels + s.negative_pixels + s.nonfinite_pixels +
    s.out_of_range_pixels + s.unrepresentable_pixels, "Invalid counters overlap or omit pixels");
  expect(s.valid_pixels + s.invalid_pixels == frame.depth_m.size(), "Total pixel count changed");
}

void testUint16LayoutAndMetadata()
{
  // 两行分别为 [1000,2000]、[0,65535]，每行附一字节填充，且输入起点故意不对齐。
  const Bytes storage{0xAB, 0xE8, 0x03, 0xD0, 0x07, 0xA1, 0x00, 0x00, 0xFF, 0xFF, 0xB2};
  auto input = view(storage, 2, 2, "16UC1", 5);
  input.data = storage.data() + 1;
  input.data_size = storage.size() - 1;
  const auto result = vln_memory::decodeDepth(input, options());
  expect(result.ok(), result.message);
  const auto & frame = result.frame.value();
  expectValue(frame.depth_m[0], 1.0F);
  expectValue(frame.depth_m[1], 2.0F);
  expectValue(frame.depth_m[2], std::numeric_limits<float>::quiet_NaN());
  expectValue(frame.depth_m[3], 65.535F);
  expect(frame.encoding == "32FC1" && frame.step == 8, "Output layout is not packed float32");
  expect(frame.unit == DepthUnit::meters && frame.source_unit == DepthUnit::millimeters &&
    frame.source_scale_to_m == 0.001, "Unit provenance lost");
  expect(frame.source_numeric_type == vln_memory::DepthNumericType::uint16 &&
    frame.source_step == 5 && !frame.source_is_bigendian, "Source layout lost");
  expect(frame.stamp.sec == -1 && frame.stamp.nanosec == 500000000 &&
    frame.frame_id == "depth_optical", "Source identity changed");
  expect(frame.statistics.valid_pixels == 3 && frame.statistics.zero_pixels == 1 &&
    frame.statistics.valid_ratio == 0.75, "Depth statistics wrong");
  checkStatistics(frame);
}

void testKnownEndianValues()
{
  const Bytes little16{0xE8, 0x03, 0xD0, 0x07};
  const Bytes big16{0x03, 0xE8, 0x07, 0xD0};
  const Bytes little32{0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x20, 0x40};
  const Bytes big32{0x3F, 0x80, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00};
  for (const bool big : {false, true}) {
    auto result = vln_memory::decodeDepth(view(big ? big16 : little16, 2, 1, "16UC1", 4, big), options());
    expect(result.ok(), result.message);
    expectValue(result.frame->depth_m[0], 1.0F);
    expectValue(result.frame->depth_m[1], 2.0F);
    result = vln_memory::decodeDepth(
      view(big ? big32 : little32, 2, 1, "32FC1", 8, big), options(DepthUnit::meters));
    expect(result.ok(), result.message);
    expectValue(result.frame->depth_m[0], 1.0F);
    expectValue(result.frame->depth_m[1], 2.5F);
  }
}

void testExplicitSemantics()
{
  const Bytes source{0x01, 0x00};
  const auto input = view(source, 1, 1, "16UC1", 2);
  auto interpretation = options();
  interpretation.input_unit = DepthUnit::unspecified;
  expectError(input, DepthErrorCode::missing_unit, interpretation);
  interpretation.input_unit = static_cast<DepthUnit>(99);
  expectError(input, DepthErrorCode::unsupported_unit, interpretation);
  interpretation = options();
  interpretation.convention = vln_memory::DepthConvention::unspecified;
  expectError(input, DepthErrorCode::missing_convention, interpretation);
  interpretation.convention = vln_memory::DepthConvention::radial_range;
  expectError(input, DepthErrorCode::unsupported_convention, interpretation);

  // 非标准但显式声明的单位也应按配置处理，不能由编码悄悄覆盖。
  auto result = vln_memory::decodeDepth(input, options(DepthUnit::meters));
  expect(result.ok(), result.message);
  expectValue(result.frame->depth_m[0], 1.0F);
  const Bytes floating = floatBytes({1000.0F});
  result = vln_memory::decodeDepth(view(floating, 1, 1, "32FC1", 4), options());
  expect(result.ok(), result.message);
  expectValue(result.frame->depth_m[0], 1.0F);
}

void testInvalidPixelsAndRange()
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto inf = std::numeric_limits<float>::infinity();
  const auto source = floatBytes({0.0F, -0.0F, -1.0F, nan, inf, -inf, 0.5F, 1.0F, 2.0F, 3.0F});
  auto interpretation = options(DepthUnit::meters);
  interpretation.min_depth_m = 1.0;
  interpretation.max_depth_m = 2.0;
  const auto result = vln_memory::decodeDepth(view(source, 10, 1, "32FC1", 40), interpretation);
  expect(result.ok(), result.message);
  const auto & frame = result.frame.value();
  for (std::size_t i = 0; i < frame.depth_m.size(); ++i) {
    expectValue(frame.depth_m[i], i == 7 ? 1.0F : (i == 8 ? 2.0F : nan));
  }
  expect(frame.min_depth_m == 1.0 && frame.max_depth_m == 2.0, "Filter provenance lost");
  expect(frame.statistics.valid_pixels == 2 && frame.statistics.zero_pixels == 2 &&
    frame.statistics.negative_pixels == 1 && frame.statistics.nonfinite_pixels == 3 &&
    frame.statistics.out_of_range_pixels == 2, "Invalid depth reason counts wrong");
  checkStatistics(frame);
  interpretation.max_depth_m = interpretation.min_depth_m;
  expectError(view(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
  interpretation = options();
  interpretation.min_depth_m = -1;
  expectError(view(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
  interpretation = options();
  interpretation.max_depth_m = std::numeric_limits<double>::quiet_NaN();
  expectError(view(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
}

void testAllInvalidAndUnderflow()
{
  const Bytes zeros(8, 0);
  const auto empty = vln_memory::decodeDepth(view(zeros), options());
  expect(empty.ok() && empty.frame->statistics.valid_ratio == 0.0 &&
    empty.frame->statistics.invalid_pixels == 4, "All-invalid image should retain its quality result");
  for (const auto value : empty.frame->depth_m) {expect(std::isnan(value), "Zero depth escaped filtering");}
  const auto tiny = floatBytes({std::numeric_limits<float>::denorm_min()});
  auto result = vln_memory::decodeDepth(view(tiny, 1, 1, "32FC1", 4), options());
  expect(result.ok() && result.frame->statistics.unrepresentable_pixels == 1,
    "Float32 underflow was accepted as valid zero");
  const auto large = floatBytes({std::numeric_limits<float>::max()});
  result = vln_memory::decodeDepth(view(large, 1, 1, "32FC1", 4), options(DepthUnit::meters));
  expect(result.ok() && result.frame->statistics.valid_pixels == 1, "Finite maximum rejected by default");
}

void testRgbAlignment()
{
  const Bytes source{0xE8, 0x03, 0xD0, 0x07};
  const auto input = view(source, 2, 1, "16UC1", 4);
  const vln_memory::ImageGeometry rgb{2, 1, "depth_optical"};
  auto interpretation = options();
  auto result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::unknown, "Matching dimensions invented RGB registration");
  interpretation.alignment.state = vln_memory::DepthAlignment::not_aligned;
  result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::not_aligned, "Unaligned declaration was ignored");
  interpretation.alignment = {vln_memory::DepthAlignment::aligned_to_rgb, rgb, "registered_stream_config"};
  result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).alignedToRgb(), "Valid declaration rejected");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {3, 1, rgb.frame_id}).status ==
    vln_memory::DepthAlignmentStatus::size_mismatch, "RGB size mismatch missed");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {2, 1, "other"}).status ==
    vln_memory::DepthAlignmentStatus::frame_mismatch, "RGB optical frame mismatch missed");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {}).status ==
    vln_memory::DepthAlignmentStatus::invalid_reference, "Empty actual RGB accepted");

  interpretation.alignment.basis.clear();
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.alignment.basis = "registration";
  interpretation.alignment.rgb_grid.width = 3;
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.alignment.rgb_grid = {2, 1, "other"};
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.alignment.state = vln_memory::DepthAlignment::unknown;
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  result.frame->alignment.basis.clear();
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::inconsistent_declaration, "Mutated declaration accepted");
}

void testInvalidMetadataAndBuffer()
{
  const Bytes source(8, 0);
  auto input = view(source);
  input.width = 0;
  expectError(input, DepthErrorCode::invalid_dimensions);
  input = view(source);
  input.height = 0;
  expectError(input, DepthErrorCode::invalid_dimensions);
  input = view(source);
  input.stamp.nanosec = 1000000000U;
  expectError(input, DepthErrorCode::invalid_stamp);
  for (const auto encoding : {"16uc1", "mono16", "16SC1", "64FC1", "rgb8"}) {
    input = view(source);
    input.encoding = encoding;
    expectError(input, DepthErrorCode::unsupported_encoding);
  }
  input = view(source);
  input.is_bigendian = 2;
  expectError(input, DepthErrorCode::invalid_endianness);
  input = view(source);
  input.step = 3;
  expectError(input, DepthErrorCode::invalid_step);
  input = view(source);
  input.data = nullptr;
  expectError(input, DepthErrorCode::invalid_data);
  input = view(source);
  input.data_size = 7;
  expectError(input, DepthErrorCode::invalid_data);
  input.data_size = 9;
  expectError(input, DepthErrorCode::invalid_data);
}

void testLimitsAndOverflow()
{
  const Bytes source(8, 0);
  const auto input = view(source);
  expect(vln_memory::decodeDepth(input, options(), {4, 8}).ok(), "Exact limit should succeed");
  expectError(input, DepthErrorCode::size_limit_exceeded, options(), {3, 8});
  expectError(input, DepthErrorCode::size_limit_exceeded, options(), {4, 7});
  expectError(input, DepthErrorCode::invalid_limits, options(), {0, 8});
  expectError(input, DepthErrorCode::invalid_limits, options(), {4, 0});
  auto huge = view(source);
  huge.width = std::numeric_limits<std::uint32_t>::max();
  huge.height = std::numeric_limits<std::uint32_t>::max();
  huge.step = std::numeric_limits<std::uint32_t>::max();
  expectError(huge, DepthErrorCode::size_overflow);
  huge = view(source, 4096, 4097, "16UC1", 8192);
  expectError(huge, DepthErrorCode::size_limit_exceeded);
}

void testOutputOwnership()
{
  vln_memory::DepthResult result;
  {
    Bytes source{0xE8, 0x03};
    std::string frame_id = "rgb_optical";
    std::string encoding = "16UC1";
    auto input = view(source, 1, 1, encoding, 2);
    input.frame_id = frame_id;
    auto interpretation = options();
    interpretation.alignment = {vln_memory::DepthAlignment::aligned_to_rgb,
      {1, 1, frame_id}, "original_registration"};
    result = vln_memory::decodeDepth(input, interpretation);
    expect(result.ok(), result.message);
    source.assign(2, 0);
    frame_id = "changed";
    encoding = "changed";
    interpretation.alignment.basis = "changed";
    interpretation.alignment.rgb_grid.frame_id = "changed";
  }
  expectValue(result.frame->depth_m[0], 1.0F);
  expect(result.frame->frame_id == "rgb_optical" && result.frame->source_encoding == "16UC1" &&
    result.frame->alignment.basis == "original_registration" &&
    result.frame->alignment.rgb_grid.frame_id == "rgb_optical", "Output retained borrowed metadata");
}

void testStableNames()
{
  for (const auto code : {DepthErrorCode::none, DepthErrorCode::invalid_limits,
      DepthErrorCode::invalid_dimensions, DepthErrorCode::invalid_stamp,
      DepthErrorCode::unsupported_encoding, DepthErrorCode::invalid_endianness,
      DepthErrorCode::invalid_step, DepthErrorCode::size_overflow,
      DepthErrorCode::size_limit_exceeded, DepthErrorCode::invalid_data,
      DepthErrorCode::missing_unit, DepthErrorCode::unsupported_unit,
      DepthErrorCode::missing_convention, DepthErrorCode::unsupported_convention,
      DepthErrorCode::invalid_range, DepthErrorCode::invalid_alignment})
  {
    expect(std::string(vln_memory::depthErrorCodeName(code)) != "unknown", "Unmapped depth error code");
  }
  expect(std::string(vln_memory::depthUnitName(DepthUnit::meters)) == "m" &&
    std::string(vln_memory::depthUnitName(DepthUnit::millimeters)) == "mm", "Unit names changed");
  expect(std::string(vln_memory::depthNumericTypeName(vln_memory::DepthNumericType::float32)) ==
    "float32", "Numeric type name wrong");
  expect(std::string(vln_memory::depthAlignmentStatusName(vln_memory::DepthAlignmentStatus::aligned)) ==
    "aligned", "Alignment status name wrong");
}

void testGeneratedSamples()
{
  std::mt19937 random(20261028U);
  for (const bool floating : {false, true}) {
    for (const bool big : {false, true}) {
      for (std::uint32_t sample = 0; sample < 150; ++sample) {
        const auto width = 1U + static_cast<std::uint32_t>(random() % 13U);
        const auto height = 1U + static_cast<std::uint32_t>(random() % 7U);
        const auto bytes = floating ? 4U : 2U;
        const auto step = width * bytes + static_cast<std::uint32_t>(random() % 5U);
        Bytes source(static_cast<std::size_t>(step) * height, 0xDA);
        std::vector<float> expected;
        for (std::size_t row = 0; row < height; ++row) {
          for (std::size_t column = 0; column < width; ++column) {
            const auto raw = static_cast<std::uint32_t>(random() % 65536U);
            std::uint32_t bits = raw;
            float value = static_cast<float>(static_cast<double>(raw) * 0.001);
            if (floating) {
              value = static_cast<float>(raw) / 4.0F;
              std::memcpy(&bits, &value, sizeof(bits));
            }
            expected.push_back(raw == 0 ? std::numeric_limits<float>::quiet_NaN() : value);
            storeBits(source, row * step + column * bytes, bits, bytes, big);
          }
        }
        const auto result = vln_memory::decodeDepth(
          view(source, width, height, floating ? "32FC1" : "16UC1", step, big),
          options(floating ? DepthUnit::meters : DepthUnit::millimeters));
        expect(result.ok(), result.message);
        for (std::size_t i = 0; i < expected.size(); ++i) {expectValue(result.frame->depth_m[i], expected[i]);}
        checkStatistics(result.frame.value());
      }
    }
  }
}

}  // namespace

int main()
{
  try {
    testUint16LayoutAndMetadata();
    testKnownEndianValues();
    testExplicitSemantics();
    testInvalidPixelsAndRange();
    testAllInvalidAndUnderflow();
    testRgbAlignment();
    testInvalidMetadataAndBuffer();
    testLimitsAndOverflow();
    testOutputOwnership();
    testStableNames();
    testGeneratedSamples();
    std::cout << "Depth core: 11 test groups passed (including 600 generated samples)\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Depth core failed: " << error.what() << '\n';
    return 1;
  }
}
