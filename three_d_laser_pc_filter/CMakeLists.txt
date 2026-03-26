cmake_minimum_required(VERSION 3.8)
project(three_d_laser_pc_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(angles REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(the_filters src/ring_filter.cpp)
target_include_directories(the_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(the_filters
  angles
  filters
  pluginlib
  rclcpp
  sensor_msgs
)
ament_export_targets(export_the_filters)

install(DIRECTORY include/
        DESTINATION include
)
install(FILES filters_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES config/example.yaml
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES launch/example.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)
install(TARGETS the_filters
        EXPORT export_the_filters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(filters filters_plugins.xml)

ament_package()
