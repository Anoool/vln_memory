#pragma once

#include <vln_memory/bag/depth/depth_projection.hpp>

namespace vln_memory::detail
{
/** @brief 检查整帧布局及历史投影上下文，独立于选区内是否有有效深度。 */
DepthProjectionCode checkDepthProjectionContext(
  const DepthFrame & depth, const CameraModel & model, const TfPoseResult & pose, std::string_view map_frame);
}  // namespace vln_memory::detail
