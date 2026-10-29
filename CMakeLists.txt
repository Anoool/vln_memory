cmake_minimum_required(VERSION 3.16)
project(vln_memory VERSION 0.1.0 LANGUAGES CXX)

# 默认作为 ROS 2 包构建；关闭后可独立验证 RGB/Depth/CameraInfo/TF 核心。
option(VLN_MEMORY_ENABLE_ROS "Build the ROS 2 sensor adapters and ament package" ON)

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
# 时间检查可独立给 RGB-D/TF 配对调用，CameraInfo 公开传递这个依赖。
add_library(vln_memory_sensor_time SHARED src/bag/common/sensor_time.cpp)
add_library(vln_memory::vln_memory_sensor_time ALIAS vln_memory_sensor_time)
target_compile_features(vln_memory_sensor_time PUBLIC cxx_std_17)
target_include_directories(vln_memory_sensor_time PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(vln_memory_camera_info SHARED src/bag/camera_info/camera_info.cpp)
add_library(vln_memory::vln_memory_camera_info ALIAS vln_memory_camera_info)
target_link_libraries(vln_memory_camera_info PUBLIC vln_memory_sensor_time)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_sensor_time PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(vln_memory_camera_info PRIVATE -Wall -Wextra -Wpedantic)
endif()
# TF 索引只使用公共数据类型，保持无 ROS、无几何求解依赖。
add_library(vln_memory_tf SHARED src/bag/tf/tf.cpp)
add_library(vln_memory::vln_memory_tf ALIAS vln_memory_tf)
target_compile_features(vln_memory_tf PUBLIC cxx_std_17)
target_include_directories(vln_memory_tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_tf PRIVATE -Wall -Wextra -Wpedantic)
endif()
set(memory_export_targets vln_memory_rgb vln_memory_depth
  vln_memory_sensor_time vln_memory_camera_info vln_memory_tf)

if(VLN_MEMORY_ENABLE_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
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
  add_library(vln_memory_camera_info_ros SHARED src/bag/camera_info/camera_info_ros.cpp)
  add_library(vln_memory::vln_memory_camera_info_ros ALIAS vln_memory_camera_info_ros)
  target_link_libraries(vln_memory_camera_info_ros PUBLIC vln_memory_camera_info)
  ament_target_dependencies(vln_memory_camera_info_ros PUBLIC sensor_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vln_memory_camera_info_ros PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_library(vln_memory_tf_ros SHARED src/bag/tf/tf_ros.cpp)
  add_library(vln_memory::vln_memory_tf_ros ALIAS vln_memory_tf_ros)
  target_link_libraries(vln_memory_tf_ros PUBLIC vln_memory_tf)
  ament_target_dependencies(vln_memory_tf_ros PUBLIC tf2_msgs geometry_msgs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vln_memory_tf_ros PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  list(APPEND memory_export_targets vln_memory_rgb_ros vln_memory_depth_ros
    vln_memory_camera_info_ros vln_memory_tf_ros)
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
  add_executable(test_sensor_time test/bag/common/test_sensor_time.cpp)
  target_link_libraries(test_sensor_time PRIVATE vln_memory_sensor_time)
  add_test(NAME sensor_time_core COMMAND test_sensor_time)
  add_executable(test_camera_info test/bag/camera_info/test_camera_info.cpp)
  target_link_libraries(test_camera_info PRIVATE vln_memory_camera_info)
  add_test(NAME camera_info_core COMMAND test_camera_info)
  add_executable(test_tf test/bag/tf/test_tf.cpp)
  target_link_libraries(test_tf PRIVATE vln_memory_tf)
  add_test(NAME tf_core COMMAND test_tf)
  if(VLN_MEMORY_ENABLE_ROS)
    # tf2 只用于真实 ROS 对照测试，当前索引/适配库不依赖其缓存。
    find_package(tf2 REQUIRED)
    add_executable(test_tf_ros test/bag/tf/test_tf_ros.cpp)
    target_link_libraries(test_tf_ros PRIVATE vln_memory_tf_ros)
    ament_target_dependencies(test_tf_ros PUBLIC tf2)
    add_test(NAME tf_ros_adapter COMMAND test_tf_ros)
    add_executable(test_camera_info_ros test/bag/camera_info/test_camera_info_ros.cpp)
    target_link_libraries(test_camera_info_ros PRIVATE vln_memory_camera_info_ros)
    add_test(NAME camera_info_ros_adapter COMMAND test_camera_info_ros)
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
  include/vln_memory/bag/common/sensor_time.hpp
  DESTINATION include/vln_memory/bag/common)
install(FILES include/vln_memory/bag/rgb/rgb.hpp
  DESTINATION include/vln_memory/bag/rgb)
install(FILES include/vln_memory/bag/depth/depth.hpp
  DESTINATION include/vln_memory/bag/depth)
install(FILES include/vln_memory/bag/camera_info/camera_info.hpp
  DESTINATION include/vln_memory/bag/camera_info)
install(FILES include/vln_memory/bag/tf/tf.hpp
  DESTINATION include/vln_memory/bag/tf)
if(VLN_MEMORY_ENABLE_ROS)
  install(FILES include/vln_memory/bag/tf/tf_ros.hpp
    DESTINATION include/vln_memory/bag/tf)
  install(FILES include/vln_memory/bag/camera_info/camera_info_ros.hpp
    DESTINATION include/vln_memory/bag/camera_info)
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
  ament_export_dependencies(sensor_msgs tf2_msgs geometry_msgs)
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
