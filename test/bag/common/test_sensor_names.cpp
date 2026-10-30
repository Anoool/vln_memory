#include "vln_memory/bag/common/sensor_names.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main()
{
  try {
    const std::vector<std::string> valid{
      "/tf", "/tf_static", "/robot_1/camera2/info", "/_hidden/__stream", "/A0/_1",
      "/" + std::string(246, 'a')};
    const std::vector<std::string> invalid{
      "", "/", "tf", "~/tf", "rostopic:///tf", "/bad topic", "/bad\ttopic", "/bad\ntopic",
      "/camera//info", "/tf/", "/1tf", "/robot/2tf", "/camera-info", "/{camera}/info",
      std::string("/bad\0topic", 10), "/" + std::string(247, 'a'), "/相机/info"};
    for (const auto & topic : valid) {
      if (!vln_memory::isResolvedTopicName(topic)) {throw std::runtime_error("Valid topic rejected: " + topic);}
    }
    for (const auto & topic : invalid) {
      if (vln_memory::isResolvedTopicName(topic)) {throw std::runtime_error("Invalid topic accepted: " + topic);}
    }
    std::cout << "Resolved topic syntax/length regression: passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
