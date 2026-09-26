cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(osqp REQUIRED CONFIG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
  path_searching
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_opt
 CATKIN_DEPENDS plan_env
)

include_directories( 
    SYSTEM 
    include 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../multi_uav_formation/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( traj_opt 
    src/poly_traj_optimizer.cpp 
    src/scp_optimizer.cpp
    )
target_link_libraries( traj_opt
    ${catkin_LIBRARIES} 
    osqp::osqp
    )  

# Feedback126: the time-only dormant helpers were removed from production
# (§9); their contract tests were orphaned and are retired with them.
