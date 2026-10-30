#include <vln_memory/bag/depth/depth.hpp>

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

/** @brief 检查测试断言；失败抛异常，Release 构建同样有效。 */
void expect(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/** @brief 构造显式单位与 optical-z 的测试配置，返回独立值。 */
vln_memory::DepthOptions makeDepthOptions(DepthUnit unit = DepthUnit::millimeters)
{
  vln_memory::DepthOptions result;
  result.input_unit = unit;
  result.convention = vln_memory::DepthConvention::optical_z;
  return result;
}

/** @brief 构造仅借用给定字节的图像视图；调用者保持源缓冲区存活。 */
vln_memory::DepthImageView makeDepthView(
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

/** @brief 核对失败码、原因和空帧，保证坏输入没有部分成功输出。 */
void expectError(
  const vln_memory::DepthImageView & input, DepthErrorCode code,
  const vln_memory::DepthOptions & interpretation = makeDepthOptions(),
  const vln_memory::DepthLimits & limits = {})
{
  const auto result = vln_memory::decodeDepth(input, interpretation, limits);
  expect(!result.ok() && !result.frame.has_value(), "Bad depth returned a partial frame");
  expect(result.code == code, std::string("Unexpected depth error: ") + result.message);
  expect(!result.message.empty(), "Failure lacks a reason");
}

/** @brief 比较已知米制结果；无效期望必须得到 NaN。 */
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
/** @brief 将测试位模式写成指定字节序；关键端序测试另外使用手写字节。 */
void storeBits(Bytes & data, std::size_t offset, std::uint32_t bits, std::size_t bytes, bool is_big_endian)
{
  for (std::size_t index = 0; index < bytes; ++index) {
    const auto shift = static_cast<unsigned int>((is_big_endian ? bytes - 1 - index : index) * 8);
    data[offset + index] = static_cast<std::uint8_t>((bits >> shift) & 0xFFU);
  }
}

/** @brief 将给定 float32 测试值编码为小端字节，返回拥有型缓冲区。 */
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

/** @brief 检查扁平质量计数互斥，且有效/无效之和等于像素数。 */
void checkStatistics(const vln_memory::DepthFrame & frame)
{
  expect(frame.invalid_pixels == frame.zero_pixels + frame.negative_pixels + frame.nonfinite_pixels +
    frame.out_of_range_pixels + frame.unrepresentable_pixels, "Invalid counters overlap or omit pixels");
  expect(frame.valid_pixels + frame.invalid_pixels == frame.depth_m.size(), "Total pixel count changed");
}

/** @brief 验证毫米换算、非对齐地址、行填充和完整源身份。 */
void testUint16LayoutAndMetadata()
{
  // 两行分别为 [1000,2000]、[0,65535]，每行附一字节填充，且输入起点故意不对齐。
  const Bytes storage{0xAB, 0xE8, 0x03, 0xD0, 0x07, 0xA1, 0x00, 0x00, 0xFF, 0xFF, 0xB2};
  auto input = makeDepthView(storage, 2, 2, "16UC1", 5);
  input.data = storage.data() + 1;
  input.data_size = storage.size() - 1;
  const auto result = vln_memory::decodeDepth(input, makeDepthOptions());
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
  expect(frame.valid_pixels == 3 && frame.zero_pixels == 1 &&
    frame.valid_ratio == 0.75, "Depth statistics wrong");
  checkStatistics(frame);
}

/** @brief 以手写字节核对两种数值编码的大/小端结果。 */
void testKnownEndianValues()
{
  const Bytes little16{0xE8, 0x03, 0xD0, 0x07};
  const Bytes big16{0x03, 0xE8, 0x07, 0xD0};
  const Bytes little32{0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x20, 0x40};
  const Bytes big32{0x3F, 0x80, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00};
  for (const bool is_big_endian : {false, true}) {
    auto result = vln_memory::decodeDepth(makeDepthView(is_big_endian ? big16 : little16, 2, 1, "16UC1", 4, is_big_endian), makeDepthOptions());
    expect(result.ok(), result.message);
    expectValue(result.frame->depth_m[0], 1.0F);
    expectValue(result.frame->depth_m[1], 2.0F);
    result = vln_memory::decodeDepth(
      makeDepthView(is_big_endian ? big32 : little32, 2, 1, "32FC1", 8, is_big_endian), makeDepthOptions(DepthUnit::meters));
    expect(result.ok(), result.message);
    expectValue(result.frame->depth_m[0], 1.0F);
    expectValue(result.frame->depth_m[1], 2.5F);
  }
}

/** @brief 验证单位/含义必须显式声明，不由编码改写来源。 */
void testExplicitSemantics()
{
  const Bytes source{0x01, 0x00};
  const auto input = makeDepthView(source, 1, 1, "16UC1", 2);
  auto interpretation = makeDepthOptions();
  interpretation.input_unit = DepthUnit::unspecified;
  expectError(input, DepthErrorCode::missing_unit, interpretation);
  interpretation.input_unit = static_cast<DepthUnit>(99);
  expectError(input, DepthErrorCode::unsupported_unit, interpretation);
  interpretation = makeDepthOptions();
  interpretation.convention = vln_memory::DepthConvention::unspecified;
  expectError(input, DepthErrorCode::missing_convention, interpretation);
  interpretation.convention = vln_memory::DepthConvention::radial_range;
  expectError(input, DepthErrorCode::unsupported_convention, interpretation);

  // 非标准但显式声明的单位也应按配置处理，不能由编码悄悄覆盖。
  auto result = vln_memory::decodeDepth(input, makeDepthOptions(DepthUnit::meters));
  expect(result.ok(), result.message);
  expectValue(result.frame->depth_m[0], 1.0F);
  const Bytes floating = floatBytes({1000.0F});
  result = vln_memory::decodeDepth(makeDepthView(floating, 1, 1, "32FC1", 4), makeDepthOptions());
  expect(result.ok(), result.message);
  expectValue(result.frame->depth_m[0], 1.0F);
}

/** @brief 核对各类无效值、闭区间过滤及范围参数错误。 */
void testInvalidPixelsAndRange()
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto inf = std::numeric_limits<float>::infinity();
  const auto source = floatBytes({0.0F, -0.0F, -1.0F, nan, inf, -inf, 0.5F, 1.0F, 2.0F, 3.0F});
  auto interpretation = makeDepthOptions(DepthUnit::meters);
  interpretation.min_depth_m = 1.0;
  interpretation.max_depth_m = 2.0;
  const auto result = vln_memory::decodeDepth(makeDepthView(source, 10, 1, "32FC1", 40), interpretation);
  expect(result.ok(), result.message);
  const auto & frame = result.frame.value();
  for (std::size_t index = 0; index < frame.depth_m.size(); ++index) {
    float expected_depth = nan;
    if (index == 7) {
      expected_depth = 1.0F;
    } else if (index == 8) {
      expected_depth = 2.0F;
    }
    expectValue(frame.depth_m[index], expected_depth);
  }
  expect(frame.min_depth_m == 1.0 && frame.max_depth_m == 2.0, "Filter provenance lost");
  expect(frame.valid_pixels == 2 && frame.zero_pixels == 2 &&
    frame.negative_pixels == 1 && frame.nonfinite_pixels == 3 &&
    frame.out_of_range_pixels == 2, "Invalid depth reason counts wrong");
  checkStatistics(frame);
  interpretation.max_depth_m = interpretation.min_depth_m;
  expectError(makeDepthView(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
  interpretation = makeDepthOptions();
  interpretation.min_depth_m = -1;
  expectError(makeDepthView(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
  interpretation = makeDepthOptions();
  interpretation.max_depth_m = std::numeric_limits<double>::quiet_NaN();
  expectError(makeDepthView(source, 10, 1, "32FC1", 40), DepthErrorCode::invalid_range, interpretation);
}

/** @brief 核对全无效成功、极小值下溢与有限最大值。 */
void testAllInvalidAndUnderflow()
{
  const Bytes zeros(8, 0);
  const auto empty = vln_memory::decodeDepth(makeDepthView(zeros), makeDepthOptions());
  expect(empty.ok() && empty.frame->valid_ratio == 0.0 &&
    empty.frame->invalid_pixels == 4, "All-invalid image should retain its quality result");
  for (const auto value : empty.frame->depth_m) {expect(std::isnan(value), "Zero depth escaped filtering");}
  const auto tiny = floatBytes({std::numeric_limits<float>::denorm_min()});
  auto result = vln_memory::decodeDepth(makeDepthView(tiny, 1, 1, "32FC1", 4), makeDepthOptions());
  expect(result.ok() && result.frame->unrepresentable_pixels == 1,
    "Float32 underflow was accepted as valid zero");
  const auto large = floatBytes({std::numeric_limits<float>::max()});
  result = vln_memory::decodeDepth(makeDepthView(large, 1, 1, "32FC1", 4), makeDepthOptions(DepthUnit::meters));
  expect(result.ok() && result.frame->valid_pixels == 1, "Finite maximum rejected by default");
}

/** @brief 核对扁平注册字段、实际网格及修改后声明的一致性。 */
void testRgbAlignment()
{
  const Bytes source{0xE8, 0x03, 0xD0, 0x07};
  const auto input = makeDepthView(source, 2, 1, "16UC1", 4);
  const vln_memory::ImageGeometry rgb{2, 1, "depth_optical"};
  auto interpretation = makeDepthOptions();
  auto result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::unknown, "Matching dimensions invented RGB registration");
  interpretation.alignment = vln_memory::DepthAlignment::not_aligned;
  result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::not_aligned, "Unaligned declaration was ignored");
  interpretation.alignment = vln_memory::DepthAlignment::aligned_to_rgb;
  interpretation.rgb_grid = rgb;
  interpretation.alignment_basis = "registered_stream_config";
  result = vln_memory::decodeDepth(input, interpretation);
  expect(result.ok(), result.message);
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).alignedToRgb(), "Valid declaration rejected");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {3, 1, rgb.frame_id}).status ==
    vln_memory::DepthAlignmentStatus::size_mismatch, "RGB size mismatch missed");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {2, 1, "other"}).status ==
    vln_memory::DepthAlignmentStatus::frame_mismatch, "RGB optical frame mismatch missed");
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), {}).status ==
    vln_memory::DepthAlignmentStatus::invalid_reference, "Empty actual RGB accepted");

  interpretation.alignment_basis.clear();
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.alignment_basis = "registration";
  interpretation.rgb_grid.width = 3;
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.rgb_grid = {2, 1, "other"};
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.alignment = vln_memory::DepthAlignment::unknown;
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  result.frame->alignment_basis.clear();
  expect(vln_memory::checkDepthRgbAlignment(result.frame.value(), rgb).status ==
    vln_memory::DepthAlignmentStatus::inconsistent_declaration, "Mutated declaration accepted");
}

/** @brief 拒绝坏布局、采样时间、编码和不完整字节缓冲区。 */
void testInvalidMetadataAndBuffer()
{
  const Bytes source(8, 0);
  auto input = makeDepthView(source);
  input.width = 0;
  expectError(input, DepthErrorCode::invalid_dimensions);
  input = makeDepthView(source);
  input.height = 0;
  expectError(input, DepthErrorCode::invalid_dimensions);
  input = makeDepthView(source);
  input.stamp.nanosec = 1000000000U;
  expectError(input, DepthErrorCode::invalid_stamp);
  for (const auto encoding : {"16uc1", "mono16", "16SC1", "64FC1", "rgb8"}) {
    input = makeDepthView(source);
    input.encoding = encoding;
    expectError(input, DepthErrorCode::unsupported_encoding);
  }
  input = makeDepthView(source);
  input.is_bigendian = 2;
  expectError(input, DepthErrorCode::invalid_endianness);
  input = makeDepthView(source);
  input.step = 3;
  expectError(input, DepthErrorCode::invalid_step);
  input = makeDepthView(source);
  input.data = nullptr;
  expectError(input, DepthErrorCode::invalid_data);
  input = makeDepthView(source);
  input.data_size = 7;
  expectError(input, DepthErrorCode::invalid_data);
  input.data_size = 9;
  expectError(input, DepthErrorCode::invalid_data);
}

/** @brief 核对精确限额及容量运算边界，避免读取越界。 */
void testLimitsAndOverflow()
{
  const Bytes source(8, 0);
  const auto input = makeDepthView(source);
  expect(vln_memory::decodeDepth(input, makeDepthOptions(), {4, 8}).ok(), "Exact limit should succeed");
  expectError(input, DepthErrorCode::size_limit_exceeded, makeDepthOptions(), {3, 8});
  expectError(input, DepthErrorCode::size_limit_exceeded, makeDepthOptions(), {4, 7});
  expectError(input, DepthErrorCode::invalid_limits, makeDepthOptions(), {0, 8});
  expectError(input, DepthErrorCode::invalid_limits, makeDepthOptions(), {4, 0});
  auto huge = makeDepthView(source);
  huge.width = std::numeric_limits<std::uint32_t>::max();
  huge.height = std::numeric_limits<std::uint32_t>::max();
  huge.step = std::numeric_limits<std::uint32_t>::max();
  expectError(huge, DepthErrorCode::size_overflow);
  huge = makeDepthView(source, 4096, 4097, "16UC1", 8192);
  expectError(huge, DepthErrorCode::size_limit_exceeded);
}

/** @brief 同时制造多项错误，保证重构前后的首个错误优先级一致。 */
void testValidationOrder()
{
  const Bytes source{0xE8, 0x03};
  auto input = makeDepthView(source, 1, 1, "16UC1", 2);
  input.data_size = 0;
  vln_memory::DepthOptions interpretation;
  expectError(input, DepthErrorCode::invalid_limits, interpretation, {0, 0});
  input.stamp.nanosec = 1000000000U;
  expectError(input, DepthErrorCode::invalid_stamp, interpretation);
  input.stamp.nanosec = 0;
  expectError(input, DepthErrorCode::missing_unit, interpretation);
  interpretation.input_unit = DepthUnit::millimeters;
  expectError(input, DepthErrorCode::missing_convention, interpretation);
  interpretation.convention = vln_memory::DepthConvention::optical_z;
  interpretation.max_depth_m = 0.0;
  expectError(input, DepthErrorCode::invalid_range, interpretation);
  interpretation.max_depth_m = 2.0;
  interpretation.alignment = vln_memory::DepthAlignment::aligned_to_rgb;
  expectError(input, DepthErrorCode::invalid_alignment, interpretation);
  interpretation.rgb_grid = {1, 1, "depth_optical"};
  interpretation.alignment_basis = "registered_stream_config";
  expectError(input, DepthErrorCode::invalid_data, interpretation);
}

/** @brief 销毁/修改输入和配置后，输出仍独立持有像素、网格和依据。 */
void testOutputOwnership()
{
  vln_memory::DepthResult result;
  {
    Bytes source{0xE8, 0x03};
    std::string frame_id = "rgb_optical";
    std::string encoding = "16UC1";
    auto input = makeDepthView(source, 1, 1, encoding, 2);
    input.frame_id = frame_id;
    auto interpretation = makeDepthOptions();
    interpretation.alignment = vln_memory::DepthAlignment::aligned_to_rgb;
    interpretation.rgb_grid = {1, 1, frame_id};
    interpretation.alignment_basis = "original_registration";
    result = vln_memory::decodeDepth(input, interpretation);
    expect(result.ok(), result.message);
    source.assign(2, 0);
    frame_id = "changed";
    encoding = "changed";
    interpretation.alignment_basis = "changed";
    interpretation.rgb_grid.frame_id = "changed";
  }
  expectValue(result.frame->depth_m[0], 1.0F);
  expect(result.frame->frame_id == "rgb_optical" && result.frame->source_encoding == "16UC1" &&
    result.frame->alignment_basis == "original_registration" &&
    result.frame->rgb_grid.frame_id == "rgb_optical", "Output retained borrowed metadata");
}

/** @brief 核对持久报告使用的错误、单位、类型和状态名称。 */
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

/** @brief 用固定种子的 600 组不同编码/布局样本检查数值与计数。 */
void testGeneratedSamples()
{
  std::mt19937 random(20261028U);
  for (const bool floating : {false, true}) {
    for (const bool is_big_endian : {false, true}) {
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
            storeBits(source, row * step + column * bytes, bits, bytes, is_big_endian);
          }
        }
        const auto result = vln_memory::decodeDepth(
          makeDepthView(source, width, height, floating ? "32FC1" : "16UC1", step, is_big_endian),
          makeDepthOptions(floating ? DepthUnit::meters : DepthUnit::millimeters));
        expect(result.ok(), result.message);
        for (std::size_t i = 0; i < expected.size(); ++i) {expectValue(result.frame->depth_m[i], expected[i]);}
        checkStatistics(result.frame.value());
      }
    }
  }
}

}  // namespace

/** @brief 运行全部 Depth 核心回归；任一失败返回非零并输出原因。 */
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
    testValidationOrder();
    testOutputOwnership();
    testStableNames();
    testGeneratedSamples();
    std::cout << "Depth core: 12 test groups passed (including 600 generated samples)\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Depth core failed: " << error.what() << '\n';
    return 1;
  }
}
