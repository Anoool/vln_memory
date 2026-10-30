#pragma once

#include <cstdint>
#include <string>

/**
 * @file
 * 共享数据契约：只描述采样时刻和像素网格，不执行校验。
 * clock/map/来源属于后续组装上下文，不能从这两个结构自行推断。
 */
namespace vln_memory
{

/**
 * @brief 传感器采样时刻，原样保留 ROS sec/nanosec；结构自身不存储时钟域 ID。
 * @note sec 可为负数，零时间也合法；nanosec 必须小于 1,000,000,000。
 *       本类型不代表 UTC；跨 bag/时钟的关联由 bag 模块负责。
 *       保留整数分量，避免转为浮点秒后影响图像配对精度。
 */
struct SensorStamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

/**
 * @brief 图像像素网格的公开元数据，用于模块间检查尺寸与光学帧是否一致。
 * @note 不包含内参、注册证明或时间；相同网格元数据不代表实际几何对齐。
 */
struct ImageGeometry
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string frame_id;
};

}  // namespace vln_memory
