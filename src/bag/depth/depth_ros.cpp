#include "vln_memory/bag/depth/depth_ros.hpp"

/**
 * @file
 * 此文件整理图像信息头、时间戳和像素布局，再交给 Depth 核心。
 * 不参与消息同步；源消息只在同步调用期间借用，输出独立持有。
 */
namespace vln_memory
{

/**
 * @brief 借用真实 Image 的布局和采样身份，使用调用方 options/limits 同步调用核心。
 * @return 核心的拥有型结果或结构化错误；不保存消息引用，也不改写原始时间。
 */
DepthResult decodeRosDepth(
  const sensor_msgs::msg::Image & image, const DepthOptions & options, const DepthLimits & limits)
{
  DepthImageView input;
  // 整理图像信息头与时间戳：保留真正的采样身份，不替换成 bag 接收时间。
  // 单位/注册依据来自 options；不能从话题、编码或宽高猜测。
  input.stamp = {image.header.stamp.sec, image.header.stamp.nanosec};
  input.frame_id = image.header.frame_id;
  // encoding 决定数值类型；step/is_bigendian 决定怎样跨行和解释每个数值的字节。
  input.width = image.width;
  input.height = image.height;
  input.encoding = image.encoding;
  input.step = image.step;
  input.is_bigendian = image.is_bigendian;
  // 借用只持续到同步解码结束；成功输出的像素、字符串与注册声明均独立持有。
  input.data = image.data.data();
  input.data_size = image.data.size();
  // decodeDepth 在同步调用中消耗借用视图；不保存 input.data 等指向消息的引用。
  return decodeDepth(input, options, limits);
}

}  // namespace vln_memory
