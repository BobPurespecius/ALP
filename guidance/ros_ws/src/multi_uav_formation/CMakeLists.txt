cmake_minimum_required(VERSION 3.0.2)
project(multi_uav_formation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs quadrotor_msgs rospy std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/TargetTrack.py
  scripts/native_egov2_gazebo_bridge.py
  scripts/native_egov2_rviz_scene.py
  scripts/target_state_coordinator.py
  scripts/verify_egov2_exec.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
