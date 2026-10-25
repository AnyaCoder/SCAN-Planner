cmake_minimum_required(VERSION 3.8)
project(map_relocalizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree search)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_relocalizer_node src/map_relocalizer_node.cpp)
target_include_directories(map_relocalizer_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(
  map_relocalizer_node
  Eigen3 geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs std_msgs tf2 tf2_ros)
target_link_libraries(map_relocalizer_node ${PCL_LIBRARIES})

install(TARGETS map_relocalizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
