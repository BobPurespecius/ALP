cmake_minimum_required(VERSION 3.0.2)
project(multi_uav_formation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  plan_env
  roscpp
  rospy
  std_msgs
  traj_opt
  traj_utils
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs plan_env roscpp rospy std_msgs traj_opt traj_utils visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cooperative_viewpoint_manager src/cooperative_viewpoint_manager.cpp)
target_link_libraries(cooperative_viewpoint_manager ${catkin_LIBRARIES})
add_dependencies(cooperative_viewpoint_manager ${catkin_EXPORTED_TARGETS})

add_executable(cooperative_viewpoint_contract_test test/cooperative_viewpoint_contract_test.cpp)
target_link_libraries(cooperative_viewpoint_contract_test ${catkin_LIBRARIES})
add_dependencies(cooperative_viewpoint_contract_test ${catkin_EXPORTED_TARGETS})

add_library(topology_coordinator_core src/topology_coordinator_core.cpp)
target_link_libraries(topology_coordinator_core ${catkin_LIBRARIES})
add_dependencies(topology_coordinator_core ${catkin_EXPORTED_TARGETS})

add_library(team_target_centered_optimizer
  src/team_target_centered_optimizer.cpp)
target_link_libraries(team_target_centered_optimizer
  ${catkin_LIBRARIES})
add_dependencies(team_target_centered_optimizer ${catkin_EXPORTED_TARGETS})

# 阶段 F：RealizedTeamValidator —— 只评价、不修改轨迹的团队复评 authority。
add_library(realized_team_validator src/realized_team_validator.cpp)
target_link_libraries(realized_team_validator ${catkin_LIBRARIES})
add_dependencies(realized_team_validator ${catkin_EXPORTED_TARGETS})

add_executable(topology_coordinator_contract_test test/topology_coordinator_contract_test.cpp)
target_link_libraries(topology_coordinator_contract_test topology_coordinator_core ${catkin_LIBRARIES})
add_dependencies(topology_coordinator_contract_test ${catkin_EXPORTED_TARGETS})

add_executable(team_target_centered_optimizer_contract_test
  test/team_target_centered_optimizer_contract_test.cpp)
target_link_libraries(team_target_centered_optimizer_contract_test
  team_target_centered_optimizer ${catkin_LIBRARIES})
add_dependencies(team_target_centered_optimizer_contract_test
  ${catkin_EXPORTED_TARGETS})

add_executable(team_solution_commit_contract_test
  test/team_solution_commit_contract_test.cpp)
target_link_libraries(team_solution_commit_contract_test ${catkin_LIBRARIES})
add_dependencies(team_solution_commit_contract_test ${catkin_EXPORTED_TARGETS})

add_executable(early_joint_primary_contract_test
  test/early_joint_primary_contract_test.cpp)
target_link_libraries(early_joint_primary_contract_test ${catkin_LIBRARIES})
add_dependencies(early_joint_primary_contract_test ${catkin_EXPORTED_TARGETS})

foreach(committed_prefix_contract committed_prefix_joint_contract_test
    joint_failure_local_noop_contract_test feedback58_barrier_regression_test
    stale_joint_cas_contract_test joint_future_tail_activation_contract_test
    prediction_epoch_semantics_contract_test)
  add_executable(${committed_prefix_contract}
    test/committed_prefix_joint_contract_test.cpp)
  target_link_libraries(${committed_prefix_contract} ${catkin_LIBRARIES})
  add_dependencies(${committed_prefix_contract} ${catkin_EXPORTED_TARGETS})
endforeach()

add_executable(static_los_wall_contract_test test/static_los_wall_contract_test.cpp)
target_link_libraries(static_los_wall_contract_test ${catkin_LIBRARIES})
add_dependencies(static_los_wall_contract_test ${catkin_EXPORTED_TARGETS})

add_executable(multi_uav_topology_coordinator src/multi_uav_topology_coordinator.cpp)
target_link_libraries(multi_uav_topology_coordinator topology_coordinator_core
  team_target_centered_optimizer
  realized_team_validator ${catkin_LIBRARIES})
add_dependencies(multi_uav_topology_coordinator ${catkin_EXPORTED_TARGETS})

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

if(CATKIN_ENABLE_TESTING)
  foreach(contract cooperative_viewpoint_contract_test topology_coordinator_contract_test
      team_solution_commit_contract_test
      team_target_centered_optimizer_contract_test
      static_los_wall_contract_test
      early_joint_primary_contract_test committed_prefix_joint_contract_test
      joint_failure_local_noop_contract_test feedback58_barrier_regression_test
      stale_joint_cas_contract_test joint_future_tail_activation_contract_test
      prediction_epoch_semantics_contract_test)
    add_test(NAME ${contract} COMMAND $<TARGET_FILE:${contract}>)
  endforeach()
endif()

# This package historically had an empty CMAKE_BUILD_TYPE. Inline Eigen and
# geometry callbacks in these executables must be optimized too; optimizing
# only the shared library leaves interposed inline implementations at -O0.
# Keep IEEE behavior (no -ffast-math); safety/visibility contracts stay identical.
foreach(runtime cooperative_viewpoint_manager multi_uav_topology_coordinator
    topology_coordinator_core
    team_target_centered_optimizer team_target_centered_optimizer_contract_test)
  target_compile_options(${runtime} PRIVATE -O2)
endforeach()
