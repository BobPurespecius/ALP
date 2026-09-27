cmake_minimum_required(VERSION 2.6)

project(yolov5_trt)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    sensor_msgs
    cv_bridge
    object_detection_msgs
    message_filters
)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolov5_trt
  CATKIN_DEPENDS roscpp sensor_msgs object_detection_msgs
  DEPENDS OpenCV CUDA
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# tensorrt
# include_directories(/home/xun/TensorRT-7.2.1.6/include/)
# clink_directories(/home/xun/TensorRT-7.2.1.6/lib/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/src/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

add_executable(yolov5 ${PROJECT_SOURCE_DIR}/src/calibrator.cpp ${PROJECT_SOURCE_DIR}/src/yolov5.cpp)
add_executable(playvideo ${PROJECT_SOURCE_DIR}/src/playvideo.cpp)
target_link_libraries(yolov5 nvinfer)
target_link_libraries(yolov5 cudart)
target_link_libraries(yolov5 myplugins)
target_link_libraries(yolov5 ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(playvideo ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_definitions(-O2 -pthread)

