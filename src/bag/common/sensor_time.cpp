#include <vln_memory/bag/common/sensor_time.hpp>

/**
 * @file
 * 实现路径：合法 stamp/容差 -> 明确同一时钟 -> 整数差值 -> 闭区间判断。
 * 不把未知时钟当同域，也不把超过容差的差值丢掉，便于后续质量统计。
 */
namespace vln_memory
{

const char * sensorTimeCodeName(SensorTimeCode code) noexcept
{
  switch (code) {
    case SensorTimeCode::matched: return "matched";
    case SensorTimeCode::outside_tolerance: return "outside_tolerance";
    case SensorTimeCode::invalid_stamp: return "invalid_stamp";
    case SensorTimeCode::invalid_tolerance: return "invalid_tolerance";
    case SensorTimeCode::clock_unknown: return "clock_unknown";
    case SensorTimeCode::clock_mismatch: return "clock_mismatch";
  }
  return "unknown";
}

SensorTimeCheck checkSensorTimes(
  const SensorStamp & reference, const SensorStamp & candidate, const SensorTimeOptions & options)
{
  if (reference.nanosec >= 1000000000U || candidate.nanosec >= 1000000000U) {
    return {SensorTimeCode::invalid_stamp, std::nullopt};
  }
  if (options.max_delta_ns < 0) {
    return {SensorTimeCode::invalid_tolerance, std::nullopt};
  }
  if (options.clock_relation == SensorClockRelation::different_clocks) {
    return {SensorTimeCode::clock_mismatch, std::nullopt};
  }
  if (options.clock_relation != SensorClockRelation::same_clock) {
    return {SensorTimeCode::clock_unknown, std::nullopt};
  }
  // 先提升整数再乘，避免 int32 溢出；不经过会丢精度的 double 秒。
  const auto reference_ns = static_cast<std::int64_t>(reference.sec) * 1000000000LL +
    reference.nanosec;
  const auto candidate_ns = static_cast<std::int64_t>(candidate.sec) * 1000000000LL +
    candidate.nanosec;
  const std::int64_t delta = candidate_ns - reference_ns;
  const bool within = delta >= -options.max_delta_ns && delta <= options.max_delta_ns;
  return {within ? SensorTimeCode::matched : SensorTimeCode::outside_tolerance, delta};
}

}  // namespace vln_memory
