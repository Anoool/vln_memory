#pragma once

#include <cstdint>

namespace vln_memory
{

/**
 * @brief 传感器采样时刻，保留 ROS sec/nanosec 表示与原始时钟域。
 * @note sec 可为负数，零时间也合法；nanosec 必须小于 1,000,000,000。
 *       本类型不代表 UTC；跨 bag/时钟的关联由 bag 模块负责。
 *       保留整数分量，避免转为浮点秒后影响图像配对精度。
 */
struct SensorStamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

}  // namespace vln_memory
