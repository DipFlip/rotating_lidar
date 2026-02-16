cmake_minimum_required(VERSION 3.8)
project(rotating_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# C++ pointcloud rotator node
add_executable(pointcloud_rotator_node src/pointcloud_rotator_node.cpp)
ament_target_dependencies(pointcloud_rotator_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS pointcloud_rotator_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python nodes
install(PROGRAMS
  rotating_lidar/motor_controller_node.py
  rotating_lidar/hall_sensor_node.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
