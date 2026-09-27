cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(ENABLE_PRECOMPILED_HEADERS "OFF")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED) #to help find Eigen

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  MINCOTraj.msg
  TopologyCandidate.msg
  VisibilityForecastTrace.msg
  TopologyCandidateBundle.msg
  TopologyCoordination.msg
  TeamTrajectorySolution.msg
  TeamTrajectoryAck.msg
  TeamReferenceSchedule.msg
  CooperativeTaskReference.msg
  TaskReferenceAdjustment.msg
  TopologyEscalation.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils 
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

# link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  

add_executable(ablation_config_cli src/ablation_config_cli.cpp)
target_link_libraries(ablation_config_cli ${catkin_LIBRARIES})
add_dependencies(ablation_config_cli ${catkin_EXPORTED_TARGETS})

add_executable(ablation_config_contract_test
    test/ablation_config_contract_test.cpp)
target_link_libraries(ablation_config_contract_test ${catkin_LIBRARIES})
add_dependencies(ablation_config_contract_test ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  add_test(NAME ablation_config_contract_test
      COMMAND ablation_config_contract_test)
endif()
