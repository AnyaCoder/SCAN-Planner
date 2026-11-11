cmake_minimum_required(VERSION 3.8)
project(go2_camera_calibration)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY validation DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
