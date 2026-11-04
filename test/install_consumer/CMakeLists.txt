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

add_executable(camera_info_consumer camera_info_consumer.cpp)
target_link_libraries(camera_info_consumer PRIVATE vln_memory::vln_memory_camera_info
  vln_memory::vln_memory_rgb vln_memory::vln_memory_depth)

# 用同一真实消息检查程序验证安装后的 ROS 目标和 sensor_msgs 传递依赖。
if(TARGET vln_memory::vln_memory_camera_info_ros)
  add_executable(camera_info_ros_consumer ../bag/camera_info/test_camera_info_ros.cpp)
  target_link_libraries(camera_info_ros_consumer PRIVATE vln_memory::vln_memory_camera_info_ros)
endif()

add_executable(tf_consumer tf_consumer.cpp)
target_link_libraries(tf_consumer PRIVATE vln_memory::vln_memory_tf
  vln_memory::vln_memory_rgb vln_memory::vln_memory_depth)
if(TARGET vln_memory::vln_memory_tf_ros)
  add_executable(tf_ros_consumer tf_ros_consumer.cpp)
  target_link_libraries(tf_ros_consumer PRIVATE vln_memory::vln_memory_tf_ros)
endif()

add_executable(rgbd_sync_consumer rgbd_sync_consumer.cpp)
target_link_libraries(rgbd_sync_consumer PRIVATE vln_memory::vln_memory_rgbd_sync)

if(TARGET vln_memory::vln_memory_bag_builder)
  add_executable(bag_builder_consumer bag_builder_consumer.cpp)
  target_link_libraries(bag_builder_consumer PRIVATE vln_memory::vln_memory_bag_builder)
endif()

# 复用行为测试源，但只通过安装导出目标解析公开头和依赖。
add_executable(camera_info_index_consumer ../bag/builder/test_camera_info_index.cpp)
target_link_libraries(camera_info_index_consumer PRIVATE vln_memory::vln_memory_camera_info_index)

# 下游只链接已安装模型库，验证普通 C++ API 和 OpenCV 私有依赖可解析。
add_executable(camera_model_consumer ../bag/camera_info/test_camera_model.cpp)
target_link_libraries(camera_model_consumer PRIVATE vln_memory::vln_memory_camera_model)

add_executable(depth_projection_consumer ../bag/depth/test_depth_projection.cpp)
target_link_libraries(depth_projection_consumer PRIVATE vln_memory::vln_memory_depth_projection)

# 独立链接安装后的读取和投影入口，复用构造夹具或真实短包核查。
find_package(nlohmann_json REQUIRED)
add_executable(preparation_reader_consumer ../bag/builder/test_preparation_reader.cpp)
target_link_libraries(preparation_reader_consumer PRIVATE vln_memory::vln_memory_preparation_reader
  vln_memory::vln_memory_depth_projection nlohmann_json::nlohmann_json)
