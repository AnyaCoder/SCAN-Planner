cmake_minimum_required(VERSION 3.10)
project(go2_yolo_trt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(CUDA REQUIRED)

add_executable(yolo_trt yolo_trt.cpp)
target_include_directories(yolo_trt PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu)
target_link_directories(yolo_trt PRIVATE
  ${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/lib)
target_link_libraries(yolo_trt PRIVATE
  ${OpenCV_LIBS}
  CURL::libcurl
  ${CUDA_LIBRARIES}
  nvinfer
  cudart)
