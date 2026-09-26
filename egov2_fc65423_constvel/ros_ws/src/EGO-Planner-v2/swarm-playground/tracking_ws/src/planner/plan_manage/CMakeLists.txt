cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../multi_uav_formation/include
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  )
add_library(ego_planner_manager src/planner_manager.cpp)
add_dependencies(ego_planner_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_planner_manager ${catkin_LIBRARIES})
target_link_libraries(ego_planner_node ego_planner_manager
  ${catkin_LIBRARIES}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  add_executable(trajectory_lifecycle_contract_test test/trajectory_lifecycle_contract_test.cpp)
  target_link_libraries(trajectory_lifecycle_contract_test ${catkin_LIBRARIES})
  add_test(NAME trajectory_lifecycle_contract_test COMMAND trajectory_lifecycle_contract_test)
endif()

if(CATKIN_ENABLE_TESTING)
  add_executable(local_execution_contract_test test/local_execution_contract_test.cpp)
  target_link_libraries(local_execution_contract_test ${catkin_LIBRARIES})
  target_compile_options(local_execution_contract_test PRIVATE -UNDEBUG)
  add_test(NAME local_execution_contract_test COMMAND local_execution_contract_test)
endif()

if(CATKIN_ENABLE_TESTING)
  add_executable(fresh_moving_initializer_contract_test
    test/fresh_moving_initializer_contract_test.cpp)
  target_link_libraries(fresh_moving_initializer_contract_test
    ego_planner_manager ${catkin_LIBRARIES})
  add_dependencies(fresh_moving_initializer_contract_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME fresh_moving_initializer_contract_test
    COMMAND fresh_moving_initializer_contract_test)

  add_executable(visibility_topology_production_test
    test/visibility_topology_production_test.cpp)
  target_link_libraries(visibility_topology_production_test
    ego_planner_manager ${catkin_LIBRARIES})
  add_dependencies(visibility_topology_production_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME visibility_topology_production_test
    COMMAND visibility_topology_production_test)

endif()
