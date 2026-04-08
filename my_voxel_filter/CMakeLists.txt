cmake_minimum_required(VERSION 3.8)
project(my_voxel_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(the_filters src/my_voxel_filter.cpp)
target_include_directories(the_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(the_filters
  filters
  pluginlib
  sensor_msgs
)
ament_export_targets(export_the_filters)
install(TARGETS the_filters
        EXPORT export_the_filters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(filters filters_plugins.xml)

ament_package()
