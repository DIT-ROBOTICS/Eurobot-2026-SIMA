cmake_minimum_required(VERSION 3.8)
project(sima-localization-real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sima-localization-odom-bridge_node src/sima-localization-odom-bridge_node.cpp)
ament_target_dependencies(sima-localization-odom-bridge_node ${THIS_PACKAGE_DEPS})
install(TARGETS sima-localization-odom-bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# add launch files into install folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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
