cmake_minimum_required(VERSION 3.8)
project(sensor_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(sensor_sim_node src/SensorSim.cpp)

ament_target_dependencies(sensor_sim_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  tf2
  tf2_ros 
)

install(TARGETS sensor_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

add_library(sensor_layer SHARED plugin/sensor_layer.cpp)
target_include_directories(sensor_layer PUBLIC include)

ament_target_dependencies(sensor_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_costmap_2d sensor_layer.xml)

install(TARGETS sensor_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES sensor_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()