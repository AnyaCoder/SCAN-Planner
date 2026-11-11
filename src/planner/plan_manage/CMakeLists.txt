cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_AUTOMOC ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(CURL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

set(scan_planner_dependencies
  ament_index_cpp bspline_opt geometry_msgs nav_msgs path_searching plan_env
  rclcpp scan_planner_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
  traj_utils visualization_msgs)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node Eigen3::Eigen)

add_executable(yolo_overlay_node src/yolo_overlay_node.cpp)
ament_target_dependencies(yolo_overlay_node rclcpp sensor_msgs)
target_include_directories(yolo_overlay_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolo_overlay_node
  CURL::libcurl jsoncpp_lib ${OpenCV_LIBRARIES})

add_library(vlm_approval_panel SHARED
  src/vlm_approval_panel.cpp
  include/plan_manage/vlm_approval_panel.hpp)
target_include_directories(vlm_approval_panel PRIVATE include)
ament_target_dependencies(vlm_approval_panel
  rclcpp rviz_common std_msgs jsoncpp)
target_link_libraries(vlm_approval_panel
  Qt5::Widgets jsoncpp_lib)
pluginlib_export_plugin_description_file(rviz_common
  plugins_description.xml)

foreach(target open_loop_controller closed_loop_controller go2_kinematic_sim go2_gait_publisher)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

install(TARGETS
  scan_planner_node open_loop_controller closed_loop_controller
  go2_kinematic_sim go2_gait_publisher yolo_overlay_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS vlm_approval_panel
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz launch/default_real.rviz
  DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS
  scripts/clicked_waypoint_recorder.py
  scripts/vlm_common.py
  scripts/vlm_pixel_shadow.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
endif()

ament_package()
