#include "vln_memory/bag/common/sensor_time.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
void expect(bool value, const char * reason)
{
  if (!value) {throw std::runtime_error(reason);}
}
}

int main()
{
  using namespace vln_memory;
  try {
    SensorTimeOptions options{SensorClockRelation::same_clock, 10000000};
    const auto edge = checkSensorTimes({5, 995000000}, {6, 5000000}, options);
    expect(edge.matched() && edge.delta_ns == 10000000, "Cross-second inclusive boundary failed");
    expect(checkSensorTimes({6, 5000000}, {5, 995000000}, options).delta_ns == -10000000,
      "Delta sign must be candidate-reference");
    const auto late = checkSensorTimes({5, 995000000}, {6, 5000001}, options);
    expect(!late.matched() && late.code == SensorTimeCode::outside_tolerance &&
      late.delta_ns == 10000001, "Out-of-tolerance delta must remain available for reports");
    options.max_delta_ns = 0;
    expect(checkSensorTimes({0, 0}, {0, 0}, options).matched(), "Zero stamp must remain valid");
    expect(checkSensorTimes({-1, 999999999}, {0, 0}, options).delta_ns == 1,
      "Negative seconds lost precision");
    const auto extremes = checkSensorTimes({std::numeric_limits<std::int32_t>::min(), 0},
      {std::numeric_limits<std::int32_t>::max(), 999999999}, options);
    expect(extremes.delta_ns == 4294967295999999999LL, "Full int32 sec range overflowed");
    options.max_delta_ns = std::numeric_limits<std::int64_t>::max();
    expect(checkSensorTimes({0, 0}, {-1, 0}, options).matched(), "Largest tolerance overflowed");
    options.max_delta_ns = -1;
    expect(checkSensorTimes({}, {}, options).code == SensorTimeCode::invalid_tolerance,
      "Negative tolerance was accepted");
    options.max_delta_ns = 1;
    expect(checkSensorTimes({0, 1000000000}, {}, options).code == SensorTimeCode::invalid_stamp,
      "Invalid nanosec was accepted");
    options.clock_relation = SensorClockRelation::unknown;
    const auto unknown = checkSensorTimes({}, {}, options);
    expect(!unknown.matched() && !unknown.delta_ns && unknown.code == SensorTimeCode::clock_unknown,
      "Equal numbers cannot prove a shared clock");
    options.clock_relation = SensorClockRelation::different_clocks;
    expect(checkSensorTimes({}, {}, options).code == SensorTimeCode::clock_mismatch,
      "Different clocks were compared");
    std::cout << "Sensor time boundary and clock checks: passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
