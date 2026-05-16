cmake_minimum_required(VERSION 3.8)
project(rrg_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(rrg_planner SHARED
  src/rrg_planner.cpp
)
add_executable(rrg_auto_request src/rrg_auto_request.cpp)

ament_target_dependencies(rrg_auto_request
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
)
ament_target_dependencies(rrg_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS
  rrg_planner
  rrg_auto_request
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
