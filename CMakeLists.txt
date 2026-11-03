cmake_minimum_required(VERSION 3.16)
project(vln_memory VERSION 0.1.0 LANGUAGES CXX)

# 默认作为 ROS 2 包构建；关闭后可独立验证 RGB/Depth/CameraInfo/TF 核心。
option(VLN_MEMORY_ENABLE_ROS "Build the ROS 2 sensor adapters and ament package" ON)

# 核心库和实际实现依赖；公共 C++17/头搜索根在全部库定义后统一直接设置。
add_library(vln_memory_rgb SHARED src/bag/rgb/rgb.cpp)
add_library(vln_memory::vln_memory_rgb ALIAS vln_memory_rgb)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_rgb PRIVATE -Wall -Wextra -Wpedantic)
endif()
# Depth 只依赖共享数据类型，不链接 RGB 实现；bag 层以后组合两种输出。
add_library(vln_memory_depth SHARED src/bag/depth/depth.cpp)
add_library(vln_memory::vln_memory_depth ALIAS vln_memory_depth)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_depth PRIVATE -Wall -Wextra -Wpedantic)
endif()
# 时间检查可独立给 RGB-D/TF 配对调用，CameraInfo 公开传递这个依赖。
add_library(vln_memory_sensor_time SHARED src/bag/common/sensor_time.cpp)
add_library(vln_memory::vln_memory_sensor_time ALIAS vln_memory_sensor_time)
add_library(vln_memory_camera_info SHARED src/bag/camera_info/camera_info.cpp)
add_library(vln_memory::vln_memory_camera_info ALIAS vln_memory_camera_info)
target_link_libraries(vln_memory_camera_info PUBLIC vln_memory_sensor_time)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_sensor_time PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(vln_memory_camera_info PRIVATE -Wall -Wextra -Wpedantic)
endif()
# TF 索引只使用公共数据类型，保持无 ROS、无几何求解依赖。
add_library(vln_memory_tf SHARED src/bag/tf/tf.cpp src/bag/tf/tf_pose.cpp)
add_library(vln_memory::vln_memory_tf ALIAS vln_memory_tf)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_tf PRIVATE -Wall -Wextra -Wpedantic)
endif()
# 无 ROS 的配对核心，前四传感器模块不反向依赖 builder。
add_library(vln_memory_rgbd_sync SHARED src/bag/builder/rgbd_sync.cpp)
add_library(vln_memory::vln_memory_rgbd_sync ALIAS vln_memory_rgbd_sync)
target_link_libraries(vln_memory_rgbd_sync PUBLIC vln_memory_sensor_time)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_rgbd_sync PRIVATE -Wall -Wextra -Wpedantic)
endif()
# 标定候选检索复用 CameraInfo，不依赖 ROS、YAML 或 JSON。
add_library(vln_memory_camera_info_index SHARED src/bag/builder/camera_info_index.cpp)
add_library(vln_memory::vln_memory_camera_info_index ALIAS vln_memory_camera_info_index)
target_link_libraries(vln_memory_camera_info_index PUBLIC vln_memory_camera_info)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_camera_info_index PRIVATE -Wall -Wextra -Wpedantic)
endif()
# 模型库独立于 ROS，内部复用已安装的 OpenCV 几何运算。
find_package(OpenCV REQUIRED COMPONENTS core calib3d)
add_library(vln_memory_camera_model SHARED src/bag/camera_info/camera_model.cpp)
add_library(vln_memory::vln_memory_camera_model ALIAS vln_memory_camera_model)
target_link_libraries(vln_memory_camera_model PUBLIC vln_memory_camera_info PRIVATE ${OpenCV_LIBS})
target_include_directories(vln_memory_camera_model SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vln_memory_camera_model PRIVATE -Wall -Wextra -Wpedantic)
endif()
set(memory_export_targets vln_memory_camera_model vln_memory_rgb vln_memory_depth
  vln_memory_sensor_time vln_memory_camera_info vln_memory_tf vln_memory_rgbd_sync vln_memory_camera_info_index)

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
  # BagBuilder 才依赖磁盘 bag、配置和编码；现有传感器核心仍可无 ROS 构建。
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(nlohmann_json REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(OpenSSL REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
  find_package(cv_bridge REQUIRED)
  find_package(compressed_depth_image_transport REQUIRED)
  # Humble 2.5.5 未导出可链接 target；从包前缀定位，避免写死发行版路径。
  get_filename_component(depth_codec_prefix "${compressed_depth_image_transport_DIR}/../../.." ABSOLUTE)
  find_library(memory_depth_codec NAMES compressed_depth_image_transport
    HINTS "${depth_codec_prefix}/lib" REQUIRED)
  find_path(memory_depth_codec_include compressed_depth_image_transport/codec.h
    HINTS "${depth_codec_prefix}/include" REQUIRED)
  add_library(vln_memory_bag_builder SHARED
    src/bag/builder/bag_config.cpp
    src/bag/builder/image_decoder_ros.cpp
    src/bag/builder/bag_reader_ros.cpp
    src/bag/builder/media_writer.cpp
    src/bag/builder/jsonl_writer.cpp
    src/bag/builder/bag_builder.cpp)
  add_library(vln_memory::vln_memory_bag_builder ALIAS vln_memory_bag_builder)
  target_link_libraries(vln_memory_bag_builder PUBLIC
    vln_memory_rgb_ros vln_memory_depth_ros vln_memory_camera_info_ros vln_memory_tf_ros
    vln_memory_rgbd_sync vln_memory_camera_info_index vln_memory_camera_model nlohmann_json::nlohmann_json
    PRIVATE yaml-cpp OpenSSL::Crypto ${OpenCV_LIBS} ${memory_depth_codec})
  target_include_directories(vln_memory_bag_builder SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${memory_depth_codec_include})
  ament_target_dependencies(vln_memory_bag_builder PUBLIC rosbag2_cpp rosbag2_storage rclcpp rcutils cv_bridge)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(vln_memory_bag_builder PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_executable(bag_builder src/apps/bag_builder_main.cpp)
  target_link_libraries(bag_builder PRIVATE vln_memory_bag_builder)
  install(TARGETS bag_builder DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  list(APPEND memory_export_targets vln_memory_bag_builder)
  list(APPEND memory_export_targets vln_memory_rgb_ros vln_memory_depth_ros
    vln_memory_camera_info_ros vln_memory_tf_ros)
endif()

# 每个实际库直接声明自己的公共头搜索根，不依赖其他库间接提供本包 include。
# 例如 <vln_memory/bag/rgb/rgb.hpp> 对应源码 include/vln_memory/bag/rgb/rgb.hpp。
# BUILD_INTERFACE 是当前编译的 -I；INSTALL_INTERFACE 相对安装前缀，供下游使用。
# 此处只声明头路径/C++ 标准，真正的实现依赖仍由各 target_link_libraries 保留。
foreach(memory_target IN LISTS memory_export_targets)
  target_compile_features(${memory_target} PUBLIC cxx_std_17)
  target_include_directories(${memory_target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()

# 两种构建模式执行同一份核心测试；真实消息适配测试只在 ROS 模式启用。
include(CTest)
if(BUILD_TESTING)
  add_executable(test_camera_model test/bag/camera_info/test_camera_model.cpp)
  target_link_libraries(test_camera_model PRIVATE vln_memory_camera_model)
  add_test(NAME camera_model_core COMMAND test_camera_model)
  add_executable(test_camera_info_index test/bag/builder/test_camera_info_index.cpp)
  target_link_libraries(test_camera_info_index PRIVATE vln_memory_camera_info_index)
  add_test(NAME camera_info_index_core COMMAND test_camera_info_index)
  add_executable(test_rgbd_sync test/bag/builder/test_rgbd_sync.cpp)
  target_link_libraries(test_rgbd_sync PRIVATE vln_memory_rgbd_sync)
  add_test(NAME rgbd_sync_core COMMAND test_rgbd_sync)
  add_executable(test_sensor_names test/bag/common/test_sensor_names.cpp)
  target_compile_features(test_sensor_names PRIVATE cxx_std_17)
  target_include_directories(test_sensor_names PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_test(NAME sensor_names_core COMMAND test_sensor_names)
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
    find_package(rmw REQUIRED)
    add_executable(test_image_decoder_ros test/bag/builder/test_image_decoder_ros.cpp)
    target_link_libraries(test_image_decoder_ros PRIVATE vln_memory_bag_builder ${OpenCV_LIBS} ${memory_depth_codec})
    target_include_directories(test_image_decoder_ros SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${memory_depth_codec_include})
    add_test(NAME image_decoder_ros COMMAND test_image_decoder_ros)
    add_executable(test_bag_builder test/bag/builder/test_bag_builder.cpp)
    target_link_libraries(test_bag_builder PRIVATE vln_memory_bag_builder yaml-cpp ${OpenCV_LIBS} ${memory_depth_codec})
    target_include_directories(test_bag_builder SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS} ${memory_depth_codec_include})
    add_test(NAME bag_builder_ros COMMAND test_bag_builder ${CMAKE_CURRENT_BINARY_DIR}/test_artifacts)
    set_tests_properties(bag_builder_ros PROPERTIES FIXTURES_SETUP bag_builder_dataset)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME bag_output_numpy COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/bag/builder/test_bag_output.py ${CMAKE_CURRENT_BINARY_DIR}/test_artifacts)
    set_tests_properties(bag_output_numpy PROPERTIES FIXTURES_REQUIRED bag_builder_dataset)
    add_executable(test_tf_ros test/bag/tf/test_tf_ros.cpp)
    target_link_libraries(test_tf_ros PRIVATE vln_memory_tf_ros)
    ament_target_dependencies(test_tf_ros PUBLIC tf2 rmw)
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

# Linux 安装后共享库按自身目录寻找兄弟依赖，包外消费者无需为本包手动补 LD_LIBRARY_PATH。
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set_target_properties(${memory_export_targets} PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

install(TARGETS ${memory_export_targets}
  EXPORT export_vln_memory
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# 安装时保留层/模块路径，下游 include 与源码中的公开路径保持一致。
install(FILES include/vln_memory/bag/common/sensor_types.hpp
  include/vln_memory/bag/common/sensor_time.hpp
  include/vln_memory/bag/common/sensor_names.hpp
  DESTINATION include/vln_memory/bag/common)
install(FILES include/vln_memory/bag/builder/rgbd_sync.hpp
  include/vln_memory/bag/builder/bag_types.hpp
  include/vln_memory/bag/builder/camera_info_index.hpp
  DESTINATION include/vln_memory/bag/builder)
install(FILES include/vln_memory/bag/rgb/rgb.hpp
  DESTINATION include/vln_memory/bag/rgb)
install(FILES include/vln_memory/bag/depth/depth.hpp
  DESTINATION include/vln_memory/bag/depth)
install(FILES include/vln_memory/bag/camera_info/camera_info.hpp
  include/vln_memory/bag/camera_info/camera_model.hpp
  DESTINATION include/vln_memory/bag/camera_info)
install(FILES include/vln_memory/bag/tf/tf.hpp
  DESTINATION include/vln_memory/bag/tf)
if(VLN_MEMORY_ENABLE_ROS)
  install(FILES
    include/vln_memory/bag/builder/bag_config.hpp
    include/vln_memory/bag/builder/bag_reader_ros.hpp
    include/vln_memory/bag/builder/image_decoder_ros.hpp
    include/vln_memory/bag/builder/media_writer.hpp
    include/vln_memory/bag/builder/jsonl_writer.hpp
    include/vln_memory/bag/builder/bag_builder.hpp
    DESTINATION include/vln_memory/bag/builder)
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
  ament_export_dependencies(sensor_msgs tf2_msgs geometry_msgs rosbag2_cpp rosbag2_storage rclcpp rcutils nlohmann_json cv_bridge)
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
