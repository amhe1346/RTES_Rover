cmake_minimum_required(VERSION 3.8)
project(trashbot_rtes)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# ════════════════════════════════════════════════════════════════════
# CORE C LIBRARY (Service Logic - Platform Independent)
# ════════════════════════════════════════════════════════════════════
add_library(trashbot_safety STATIC
  src/emergency_stop.c
  src/path_planner.c
  src/motor_control.c
)

target_include_directories(trashbot_safety PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(trashbot_safety m)  # Math library for C

# ════════════════════════════════════════════════════════════════════
# ROS2 DEPENDENCIES
# ════════════════════════════════════════════════════════════════════
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# ════════════════════════════════════════════════════════════════════
# PRODUCTION NODE (Full Integrated System with RT Features)
# ════════════════════════════════════════════════════════════════════
add_executable(realtime_system_node src/realtime_system_node.cpp)
target_link_libraries(realtime_system_node trashbot_safety pthread)
ament_target_dependencies(realtime_system_node ${ROS_DEPENDENCIES})

# ════════════════════════════════════════════════════════════════════
# DEVELOPMENT NODES (Individual Service Testing)
# ════════════════════════════════════════════════════════════════════
# Service 1: Emergency Stop
add_executable(emergency_stop_node src/emergency_stop_node.cpp)
target_link_libraries(emergency_stop_node trashbot_safety)
ament_target_dependencies(emergency_stop_node ${ROS_DEPENDENCIES})

# Service 2: Path Planner
add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node trashbot_safety)
ament_target_dependencies(path_planner_node ${ROS_DEPENDENCIES})

# Service 3: Motor Control
add_executable(motor_control_node src/motor_control_node.cpp)
target_link_libraries(motor_control_node trashbot_safety)
ament_target_dependencies(motor_control_node ${ROS_DEPENDENCIES})

# ════════════════════════════════════════════════════════════════════
# TESTING
# ════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Unit test for C library
  add_executable(test_emergency test/test_emergency.c)
  target_link_libraries(test_emergency trashbot_safety)
  target_include_directories(test_emergency PRIVATE include)
endif()

# ════════════════════════════════════════════════════════════════════
# INSTALLATION
# ════════════════════════════════════════════════════════════════════
# Install executables
install(TARGETS
  realtime_system_node
  emergency_stop_node
  path_planner_node
  motor_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install library
install(TARGETS
  trashbot_safety
  EXPORT export_trashbot_safety
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_trashbot_safety HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS_DEPENDENCIES})

ament_package()
