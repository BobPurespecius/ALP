cmake_minimum_required(VERSION 3.0.2)
project(multi_uav_formation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs rospy std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
