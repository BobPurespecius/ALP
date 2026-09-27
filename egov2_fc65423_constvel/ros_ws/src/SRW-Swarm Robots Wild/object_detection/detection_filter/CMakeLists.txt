cmake_minimum_required(VERSION 3.0.2)
project(detection_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w -g -fPIC")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rosbag
  std_msgs
  image_transport
  cv_bridge
  object_detection_msgs
  pcl_conversions
  message_filters
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  #CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


 add_executable(detection_filter_node 
  src/detection_filter_node.cpp
  src/detection_filter.cpp
  )

 target_link_libraries(detection_filter_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )
