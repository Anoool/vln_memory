#include <vln_memory/bag/rgb/rgb.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

int main()
{
  // 此项目只链接安装产物；一个已知像素足以检查公开接口和动态链接是否可用。
  const std::vector<std::uint8_t> source{3, 2, 1};
  vln_memory::RgbImageView image;
  image.width = 1;
  image.height = 1;
  image.encoding = "bgr8";
  image.step = 3;
  image.data = source.data();
  image.data_size = source.size();
  const auto result = vln_memory::decodeRgb(image);
  if (!result.ok() || result.frame->pixels != std::vector<std::uint8_t>{1, 2, 3}) {
    std::cerr << "Installed RGB API failed\n";
    return 1;
  }
  std::cout << "Installed RGB API: passed\n";
  return 0;
}
