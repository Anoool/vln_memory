cmake_minimum_required(VERSION 3.16)
project(vln_memory VERSION 0.1.0 LANGUAGES CXX)

# 默认作为 ROS 2 包构建；关闭后可独立验证 RGB/Depth 核心。
option(VLN_MEMORY_ENABLE_ROS "Build the ROS 2 Image adapters and ament package" ON)

# 核心库公开 C++17 和头文件路径，使下游只链接目标即可使用接口。
add_library(vln_memory_rgb SHARED src/bag/rgb/rgb.cpp)
add_library(vln_memory::vln_memory_rgb ALIAS vln_memory_rgb)
target_compile_features(vln_memory_rgb PUBLIC cxx_std_17)
target_include_directories(vln_memory_rgb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_rgb PRIVATE -Wall -Wextra -Wpedantic)
endif()
# Depth 只依赖共享数据类型，不链接 RGB 实现；bag 层以后组合两种输出。
add_library(vln_memory_depth SHARED src/bag/depth/depth.cpp)
add_library(vln_memory::vln_memory_depth ALIAS vln_memory_depth)
target_compile_features(vln_memory_depth PUBLIC cxx_std_17)
target_include_directories(vln_memory_depth PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_depth PRIVATE -Wall -Wextra -Wpedantic)
endif()
set(memory_export_targets vln_memory_rgb vln_memory_depth)

if(VLN_MEMORY_ENABLE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  add_library(vln_memory_rgb_ros SHARED src/bag/rgb/rgb_ros.cpp)
  add_library(vln_memory::vln_memory_rgb_ros ALIAS vln_memory_rgb_ros)
  target_link_libraries(vln_memory_rgb_ros PUBLIC vln_memory_rgb)
  ament_target_dependencies(vln_memory_rgb_ros PUBLIC sensor_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vln_memory_rgb_ros PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_library(vln_memory_depth_ros SHARED src/bag/depth/depth_ros.cpp)
  add_library(vln_memory::vln_memory_depth_ros ALIAS vln_memory_depth_ros)
  target_link_libraries(vln_memory_depth_ros PUBLIC vln_memory_depth)
  ament_target_dependencies(vln_memory_depth_ros PUBLIC sensor_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vln_memory_depth_ros PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  list(APPEND memory_export_targets vln_memory_rgb_ros vln_memory_depth_ros)
endif()

# 两种构建模式执行同一份核心测试；真实消息适配测试只在 ROS 模式启用。
include(CTest)
if(BUILD_TESTING)
  add_executable(test_rgb test/bag/rgb/test_rgb.cpp)
  target_link_libraries(test_rgb PRIVATE vln_memory_rgb)
  add_test(NAME rgb_core COMMAND test_rgb)
  add_executable(test_depth test/bag/depth/test_depth.cpp)
  target_link_libraries(test_depth PRIVATE vln_memory_depth)
  add_test(NAME depth_core COMMAND test_depth)
  if(VLN_MEMORY_ENABLE_ROS)
    add_executable(test_rgb_ros test/bag/rgb/test_rgb_ros.cpp)
    target_link_libraries(test_rgb_ros PRIVATE vln_memory_rgb_ros)
    add_test(NAME rgb_ros_adapter COMMAND test_rgb_ros)
    add_executable(test_depth_ros test/bag/depth/test_depth_ros.cpp)
    target_link_libraries(test_depth_ros PRIVATE vln_memory_depth_ros)
    add_test(NAME depth_ros_adapter COMMAND test_depth_ros)
  endif()
endif()

install(TARGETS ${memory_export_targets}
  EXPORT export_vln_memory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# 安装时保留层/模块路径，下游 include 与源码中的公开路径保持一致。
install(FILES include/vln_memory/bag/common/sensor_types.hpp
  DESTINATION include/vln_memory/bag/common)
install(FILES include/vln_memory/bag/rgb/rgb.hpp
  DESTINATION include/vln_memory/bag/rgb)
install(FILES include/vln_memory/bag/depth/depth.hpp
  DESTINATION include/vln_memory/bag/depth)
if(VLN_MEMORY_ENABLE_ROS)
  install(FILES include/vln_memory/bag/rgb/rgb_ros.hpp
    DESTINATION include/vln_memory/bag/rgb)
  install(FILES include/vln_memory/bag/depth/depth_ros.hpp
    DESTINATION include/vln_memory/bag/depth)
endif()
# 整体安装分类后的文档目录，保留导航所依赖的相对路径。
install(DIRECTORY docx DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(VLN_MEMORY_ENABLE_ROS)
  ament_export_targets(export_vln_memory HAS_LIBRARY_TARGET)
  ament_export_dependencies(sensor_msgs)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/vln_memoryConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/vln_memoryConfig.cmake
    INSTALL_DESTINATION lib/cmake/vln_memory)
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/vln_memoryConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(EXPORT export_vln_memory FILE vln_memoryTargets.cmake
    NAMESPACE vln_memory:: DESTINATION lib/cmake/vln_memory)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vln_memoryConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/vln_memoryConfigVersion.cmake
    DESTINATION lib/cmake/vln_memory)
endif()
