cmake_minimum_required(VERSION 3.8)
project(point_cloud_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(cloud_filter_chain SHARED src/cloud_filter_chain.cpp)
target_include_directories(cloud_filter_chain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cloud_filter_chain
  filters
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
)

rclcpp_components_register_node(
    cloud_filter_chain
    PLUGIN "point_cloud_filters::CloudFilterChain"
    EXECUTABLE "cloud_filter_chain_node"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_targets(export_cloud_filter_chain)
install(TARGETS cloud_filter_chain
        EXPORT export_cloud_filter_chain
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_package()
