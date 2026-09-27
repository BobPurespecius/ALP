# ALP Target-Facing Yaw Implementation Report

## Source audit

The production EGOv2 chain is:

`traj_server` publishes `quadrotor_msgs/PositionCommand` → `SO3ControlNodelet`
consumes `yaw/yaw_dot` and publishes `SO3Command` → the SO3 simulator consumes
`aux.current_yaw` when `aux.use_external_yaw` is enabled → odometry publishes the
executed quaternion.  The previous ALP `traj_server` calculated a yaw but then
overwrote the published command with `cmd.yaw = 0.0`.  The SO3/simulator launch
also used misspelled parameter names with trailing spaces and disabled external
yaw.

The visibility collector already evaluated the odometry quaternion, so its
geometric predicate was retained and extended with explicit yaw/bearing fields.

## Implementation

Modified files:

- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/launch/run_in_sim.launch`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/launch/simulator.xml`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/CMakeLists.txt`
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/package.xml`
- `ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch`
- `ros_ws/src/multi_uav_formation/launch/native_egov2_gazebo.launch`
- `ros_ws/src/multi_uav_formation/config/native_egov2.rviz`
- `ros_ws/src/multi_uav_formation/scripts/native_egov2_rviz_scene.py`
- `run_constvel_gradient_rviz.sh`

Added parameter `traj_server/enable_target_facing_yaw`, default `true`,
transmitted through launch files.  When disabled, the legacy command behavior
is preserved (`yaw=0`, `yaw_dot=0`).  When enabled, the position trajectory is
unchanged and only yaw is generated from the current `/object_odom` target.
Missing/invalid target state holds the previous yaw rather than jumping to
zero.

## Yaw policy

Desired yaw is `atan2(target_y-uav_y, target_x-uav_x)`.  The implementation uses
shortest-angle wrapping, the existing EGOv2 limits (6.283 rad/s yaw rate and
15.708 rad/s² yaw acceleration), and bounded per-update motion.  No camera-body
extrinsic is present in this stack; the camera forward axis is therefore the
body +X axis (`extrinsic_yaw_rad=0`).  The command yaw and executed odometry yaw
are kept distinct.

## RViz camera/FOV visualization

The collector publishes a fixed-ID `visualization_msgs/MarkerArray` on
`/native_egov2/camera_fov`.  Each UAV has an independent namespace
`alp_camera_fov/uavN`.  The marker is a horizontal wireframe wedge containing
the forward ray, ±42.5° boundary rays, and a 16-segment 8 m arc.  Its origin is
the current UAV odometry position and its orientation is rebuilt from actual
odometry yaw every 30 Hz update.  It is visualization only and is not read by
the planner.

## Visibility metric update

The existing range (0.2–8.0 m), horizontal FOV (±42.5°), and static/dynamic LOS
tests are unchanged.  FOV now uses the executed odometry yaw (the same yaw used
for the RViz marker), with wrapped relative bearing.  The visibility CSV keeps
all legacy columns and appends, for each UAV:

- actual yaw (rad)
- target bearing (rad)
- relative bearing (rad)
- FOV margin (deg)
- commanded yaw (rad)
- commanded yaw rate (rad/s)

Trajectory CSVs also append `actual_yaw_rad`.

## Regression and smoke tests

Static checks:

- Python `py_compile`: PASS
- launch XML validation (`xmllint`): PASS
- `catkin build traj_opt ego_planner path_searching -j2` with the project OSQP
  path: PASS (all requested packages succeeded)

ON smoke (`long_cylinder_forest.json`, RViz OFF, 18 s):

- 412 visibility rows, 23 CSV columns
- command yaw ranges: UAV1 [0, 0.622], UAV2 [-0.191, 1.080], UAV3 [-0.664, 0.244] rad
- maximum commanded yaw-rate magnitude: 2.38 rad/s (below 6.283 limit)
- actual odometry yaw changed with the target (UAV1 reached 0.620 rad; UAV2
  reached 1.059 rad; UAV3 reached −0.659 rad)
- command-to-target bearing mean absolute error was 1.55°/0.74°/2.62° for
  UAV1/UAV2/UAV3; actual-odometry-to-target mean error was 2.22°/1.76°/3.62°
- visibility bucket counts: 3-visible=365, 2-visible=22, 1-visible=25,
  0-visible=0
- rosout topic list confirms `/native_egov2/camera_fov` is advertised
- no `ValueError`, closed-topic exception, segmentation fault, or double-free
  from the yaw/collector changes

OFF compatibility smoke (`long_cylinder_forest.json`, RViz OFF, 12 s):

- command yaw and yaw_dot were exactly zero for all three UAVs
- actual odometry yaw stayed near zero (maximum absolute values: UAV1 0.0205,
  UAV2 0.0075, UAV3 0.0035 rad)
- visibility CSV schema remained valid

The repeated `boost::lock_error` lines during shutdown are pre-existing planner
shutdown hygiene warnings; they were present before this change and no longer
produce collector file-close or closed-topic exceptions.

## Position-planner isolation

No planner, SIDE, A*, Local SFC, MINCO position objective, SCP, trust-region,
OSQP, dynamics limits, obstacle, target trajectory, or controller gain code was
changed.  Yaw is consumed only by the orientation execution path and the
visualization/visibility collector.

## Final status

```text
TARGET_FACING_YAW_IMPLEMENTED: YES
DEFAULT_ENABLED: YES
YAW_OFF_RESTORES_LEGACY_BEHAVIOR: YES
POSITION_PLANNER_UNCHANGED: YES
PATH_GEOMETRY_UNCHANGED_BY_DESIGN: YES
ACTUAL_YAW_TRACKS_TARGET: YES
YAW_RATE_LIMIT_ACTIVE: YES
RVIZ_CAMERA_FOV_REALTIME: YES
VISIBILITY_USES_ACTUAL_CAMERA_YAW: YES
VISIBILITY_IMPROVED: MIXED (short smoke only; no claim of long-run improvement)
DYNAMIC_AVOIDANCE_REGRESSION: NO EVIDENCE
TRACKING_REGRESSION: NO EVIDENCE
BUILD: PASS
RUNTIME: PASS
SHUTDOWN: PASS (SIGINT path; legacy boost warning remains)
```
