cmake_minimum_required(VERSION 3.5)
project(cmd_vel_sport_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)

add_executable(cmd_vel_sport_bridge src/cmd_vel_sport_bridge.cpp)
ament_target_dependencies(cmd_vel_sport_bridge
  geometry_msgs
  rclcpp
  unitree_api)

install(TARGETS cmd_vel_sport_bridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
