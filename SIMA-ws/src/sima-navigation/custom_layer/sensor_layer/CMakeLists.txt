# cmake_minimum_required(VERSION 3.8)
# project(sensor_layer)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)


# find_package(rclcpp REQUIRED)
# find_package(nav2_costmap_2d REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)

# include_directories(include)

# # 1. 編譯 SensorSim 模擬節點
# add_executable(sensor_sim_node src/SensorSim.cpp)
# ament_target_dependencies(sensor_sim_node rclcpp sensor_msgs)
# install(TARGETS sensor_sim_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # 2. 編譯 Custom Layer Library
# add_library(sensor_layer SHARED plugin/sensor_layer.cpp)
# target_include_directories(sensor_layer PUBLIC include)
# ament_target_dependencies(sensor_layer
#   rclcpp
#   nav2_costmap_2d
#   pluginlib
#   sensor_msgs
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   tf2
# )

# # 3. 註冊 Plugin (非常重要！)
# pluginlib_export_plugin_description_file(nav2_costmap_2d sensor_layer.xml)

# # 4. 安裝設定
# install(TARGETS sensor_layer
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

# install(FILES sensor_layer.xml
#   DESTINATION share/${PROJECT_NAME}
# )




# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()




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

# === [關鍵 1] 必須先找到這些套件 ===
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# =================================

include_directories(include)

# 1. 編譯 SensorSim 模擬節點
add_executable(sensor_sim_node src/SensorSim.cpp)

# === [關鍵 2] 將找到的套件連結到執行檔 ===
ament_target_dependencies(sensor_sim_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  tf2
  tf2_ros 
)
# ======================================

install(TARGETS sensor_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 編譯 Custom Layer Library
add_library(sensor_layer SHARED plugin/sensor_layer.cpp)
target_include_directories(sensor_layer PUBLIC include)

# === [關鍵 3] Layer 本身也需要這些依賴 ===
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
# =======================================

# 3. 註冊 Plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d sensor_layer.xml)

# 4. 安裝設定
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