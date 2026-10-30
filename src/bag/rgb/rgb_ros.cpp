#include <vln_memory/bag/rgb/rgb_ros.hpp>

/**
 * @file
 * 此文件只做字段映射：Image -> 临时借用视图 -> 同步核心调用。
 * 适配返回前核心已经复制像素/字符串，因此不会把消息内部指针泄露给下游。
 */
namespace vln_memory
{

/** 借用已反序列化 Image 的字段并同步调用核心；返回的成功帧独立持有数据。 */
RgbResult decodeRosRgb(const sensor_msgs::msg::Image & image, const RgbLimits & limits)
{
  // bag 模块完成反序列化后可直接调用；此处不再复制完整输入像素。
  RgbImageView input;

  // 保留原始采样身份；bag 接收时间和 clock_id 由后续 bag 模块单独记录。
  input.stamp = {image.header.stamp.sec, image.header.stamp.nanosec};
  input.frame_id = image.header.frame_id;

  // 网格与布局原样传递：step 含填充，不能用 width*channels 代替消息实际跨度。
  input.width = image.width;
  input.height = image.height;
  input.encoding = image.encoding;
  input.step = image.step;
  input.is_bigendian = image.is_bigendian;

  // 仅在同步调用 decodeRgb 期间借用；空数据也交由核心统一返回错误。
  input.data = image.data.data();
  input.data_size = image.data.size();
  return decodeRgb(input, limits);
}

}  // namespace vln_memory
