cmake_minimum_required(VERSION 3.0.2)
project(target_tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    rospy
    std_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(fov_tracking_planner_node
  src/fov_tracking_planner_node.cpp
)
add_dependencies(fov_tracking_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fov_tracking_planner_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/fov_tracking_planner.py
  scripts/rao_2025_protocol.py
  scripts/rao_scene_model.py
  scripts/rao_scene_node.py
  scripts/rao_target_model.py
  scripts/rao_target_node.py
  scripts/rao_2025_reproduction.py
  scripts/rao_2025_full_reproduction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fov_tracking_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rao_2025_protocol.py)
  catkin_add_nosetests(test/test_rao_phase2_models.py)
endif()
