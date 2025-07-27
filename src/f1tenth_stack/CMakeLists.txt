cmake_minimum_required(VERSION 3.5)
project(f1tenth_stack)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(interfaces_pkg REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)


# Create the node executables
add_executable(tf_publisher_node 
  src/main.cpp
  src/tf_publisher_node.cpp
)

add_executable(tf_odom_node 
  src/tf_odom_node.cpp
)

add_executable(tf_imu_node 
  src/tf_imu_node.cpp
)


# Link dependencies for tf_publisher_node
ament_target_dependencies(tf_publisher_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  interfaces_pkg
)
ament_target_dependencies(tf_odom_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  interfaces_pkg
  vesc_msgs
)
ament_target_dependencies(tf_imu_node 
  rclcpp 
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  interfaces_pkg
  vesc_msgs
)

# Install targets
install(TARGETS 
  tf_publisher_node
  tf_odom_node
  tf_imu_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install include directories
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY 
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
install(PROGRAMS
  f1tenth_stack/test_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()