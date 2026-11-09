#pragma once

#include <vln_memory/bag/builder/bag_config.hpp>
#include <vln_memory/engine/build_input.hpp>
#include <vln_memory/engine/observation_builder.hpp>

namespace vln_memory
{

/**
 * @brief 两遍读取并发布 bag_preparation/1 数据集；不访问模型、不生成虚构位姿。
 * @param options 一个明确 clock/segment 的输入、绑定、语义和资源配置。
 * @return 完整成功/带问题成功，或结构化失败；失败工作目录保留，已有输出不覆盖。
 * @note 输入只读；输出成功前只写新建 .partial 工作目录。不是正式 Observation 构建器。
 */
BagBuildResult buildBag(const BagBuildOptions & options);

/** @brief 直接发布 observations.jsonl 和 evidence；地图上下文由调用方显式声明。 */
BagBuildResult buildBagObservations(
  const BagBuildOptions & options, const ObservationBuildContext & context,
  const BuildInputOptions & selection = {});

}  // namespace vln_memory
