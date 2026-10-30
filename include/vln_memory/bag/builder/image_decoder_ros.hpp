#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vln_memory
{

enum class BagImageRole {rgb, depth};

/** @brief 压缩头预检结果，拥有错误文本和解码后布局；不持有输入或像素。 */
struct CompressedImageInfo
{
  std::string code{"invalid_compressed_payload"};
  std::string message;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::size_t payload_offset{0};
  /** @brief 仅表示格式/布局/预算可解码，不证明压缩数据或物理标定有效。 */
  bool ok() const noexcept {return code == "none";}
};

/** @brief 拥有型 Image 结果；失败 image 为空，原始 header 不被修改。 */
struct CompressedImageResult
{
  sensor_msgs::msg::Image::SharedPtr image;
  std::string code;
  std::string message;
  /** @brief 确认解码成功和结果存在；分配失败由调用者处理。 */
  bool ok() const noexcept {return image != nullptr && code == "none";}
};

/**
 * @brief 无像素解压的头部预检；max_pixels 与 max_input_bytes 同时约束解码后布局。
 * @note RGB 支持 RGB8/BGR8 的 JPEG；Depth 支持 16UC1 compressedDepth PNG。
 *       原始压缩字节也不得超过 max_input_bytes；不支持的格式返回类型化字符串错误。
 */
CompressedImageInfo inspectCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes);

/**
 * @brief 先预检再逐帧解码，结果拥有字节；深度复用官方 codec，RGB 交给现有核心转 RGB8。
 * @note 不改 header，不推断深度单位/注册；坏负载返回失败，bad_alloc 继续向外传播。
 */
CompressedImageResult decodeCompressedImage(
  const sensor_msgs::msg::CompressedImage & message, BagImageRole role,
  std::size_t max_pixels, std::size_t max_input_bytes);

}  // namespace vln_memory
