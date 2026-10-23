cmake_minimum_required(VERSION 3.8)
project(lio_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lio_bridge_node src/lio_bridge_node.cpp)
ament_target_dependencies(
  lio_bridge_node builtin_interfaces geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS lio_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
