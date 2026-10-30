#pragma once

#include <cstdint>
#include <optional>

#include "vln_memory/bag/common/sensor_types.hpp"

/**
 * @file
 * 候选时间检查契约：调用者先选出两条消息并确认时钟关系，再检查整数时差。
 * 返回 matched 只表示容差通过，不建立同步队列、不改变任一原始 stamp。
 */
namespace vln_memory
{

/** @brief 由采集配置/时钟校准证明的关系，不能从两个时间数值相近推断。 */
enum class SensorClockRelation {unknown, same_clock, different_clocks};

struct SensorTimeOptions
{
  SensorClockRelation clock_relation{SensorClockRelation::unknown};
  // 非负整数纳秒，0 表示严格相等。项目不替调用者选择通用同步容差。
  std::int64_t max_delta_ns{0};
};

enum class SensorTimeCode
{
  matched, outside_tolerance, invalid_stamp, invalid_tolerance, clock_unknown, clock_mismatch
};

/** @brief 仅能在确认同一时钟且输入有效时得到 delta_ns，符号为 candidate-reference。 */
struct SensorTimeCheck
{
  SensorTimeCode code{SensorTimeCode::clock_unknown};
  std::optional<std::int64_t> delta_ns;
  bool matched() const noexcept {return code == SensorTimeCode::matched && delta_ns.has_value();}
};

const char * sensorTimeCodeName(SensorTimeCode code) noexcept;

/**
 * @brief 检查采样时差，不修改时间、不选择消息、不证明硬件同步。
 * @note sec 为 int32，转为 int64 纳秒后以及两时间之差均可安全表示。
 *       零时刻/负秒合法；bag 接收时间与采样时间必须在上层分开传递。
 */
SensorTimeCheck checkSensorTimes(
  const SensorStamp & reference, const SensorStamp & candidate, const SensorTimeOptions & options);

}  // namespace vln_memory
