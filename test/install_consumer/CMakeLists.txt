cmake_minimum_required(VERSION 3.16)
project(vln_memory_install_consumer LANGUAGES CXX)

# 这是独立下游项目，只读取安装后的包，不借用源码 include 路径。
find_package(vln_memory CONFIG REQUIRED)
add_executable(rgb_consumer rgb_consumer.cpp)
target_link_libraries(rgb_consumer PRIVATE vln_memory::vln_memory_rgb)

add_executable(depth_consumer depth_consumer.cpp)
target_link_libraries(depth_consumer PRIVATE
  vln_memory::vln_memory_rgb vln_memory::vln_memory_depth)

if(TARGET vln_memory::vln_memory_depth_ros)
  add_executable(depth_ros_consumer depth_ros_consumer.cpp)
  target_link_libraries(depth_ros_consumer PRIVATE vln_memory::vln_memory_depth_ros)
endif()
