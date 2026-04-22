cmake_minimum_required(VERSION 3.8)
project(special_diff_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(special_diff_controller SHARED
  src/special_diff_controller.cpp
)

ament_target_dependencies(special_diff_controller
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
)

target_include_directories(special_diff_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 關鍵修正：加入這行讓 nav2_core 正確找到 plugin description
pluginlib_export_plugin_description_file(nav2_core special_diff_controller_plugin.xml)

install(TARGETS special_diff_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES special_diff_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()