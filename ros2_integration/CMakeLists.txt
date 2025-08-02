cmake_minimum_required(VERSION 3.8)
project(path_smoothing_turtlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install scripts
install(PROGRAMS
  scripts/path_smoothing_node.py
  scripts/trajectory_controller_node.py
  scripts/waypoint_publisher_node.py
  scripts/obstacle_avoidance_node.py
  scripts/fake_robot_node.py
  scripts/simple_path_smoothing_node.py
  scripts/simple_trajectory_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# Install world files (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(DIRECTORY
    worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
