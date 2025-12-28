cmake_minimum_required(VERSION 3.8)
project(sima-localization-sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

# add the include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(global_sim_node src/global-sim_node.cpp)
ament_target_dependencies(global_sim_node ${THIS_PACKAGE_DEPS})
install(TARGETS global_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(odom_sim_node src/odom-sim_node.cpp)
ament_target_dependencies(odom_sim_node ${THIS_PACKAGE_DEPS})
install(TARGETS odom_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(robot_pose_node src/robot-pose_node.cpp)
ament_target_dependencies(robot_pose_node ${THIS_PACKAGE_DEPS})
install(TARGETS robot_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
