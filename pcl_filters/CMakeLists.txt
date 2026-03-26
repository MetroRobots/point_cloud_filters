cmake_minimum_required(VERSION 3.8)
project(pcl_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(the_filters
  src/voxel_filter.cpp
)
target_include_directories(the_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(the_filters
  filters
  pcl_conversions
  sensor_msgs
  pluginlib
)
ament_export_targets(export_pcl_filters)

install(DIRECTORY include/
        DESTINATION include
)
install(TARGETS the_filters
        EXPORT export_pcl_filters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(FILES config/example.yaml
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES launch/example.launch.py
        DESTINATION share/${PROJECT_NAME}/launch
)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(filters filters_plugins.xml)

ament_package()
