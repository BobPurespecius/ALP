# Feedback 17

## 结论

根因是原 `run_constvel_gradient_rviz.sh` 只执行一次 `roslaunch`，而
`native_egov2_rviz.launch` 在同一个 launch 中并行拉起 scene、target、三套
planner/simulator/controller/traj_server 和 RViz。RViz 虽写在 XML 末尾，但
roslaunch 不把 XML 顺序当作 ready barrier，因此 target/UAV/planner 不等待
RViz 加载配置、订阅地图和完成首次渲染。

本次只修改启动编排：RViz ON 时先启动冻结在实验 t=0 的 scene 和 RViz，等待
`/rviz`、`/map_generator/global_cloud` publisher，再等待一个集中配置的短
warm-up；之后才启动 target、UAV simulator、planner、traj_server 和 controller。
RViz OFF 仍走原单阶段路径，没有新增等待。

未修改 SIDE、A*、Local-SFC、MINCO、SCP、OSQP、动态风险、安全分类、
visibility optimizer、viewpoint manager 或场景几何。

## 根因与源码依据

### 原启动链

原主脚本在 `run_constvel_gradient_rviz.sh` 中直接 `exec roslaunch`。原 launch
同时包含：

- scene/metrics node：`native_egov2_rviz.launch:83-85`；
- target coordinator：`native_egov2_rviz.launch:87-109`；
- 三套 planner/simulator/traj_server/controller include：从
  `native_egov2_rviz.launch:140` 开始；
- RViz：`native_egov2_rviz.launch:326-327`。

上述节点没有 RViz-ready 依赖。RViz 位于文件末尾不代表最后完成初始化。

原真实顺序是“roslaunch 并发启动所有节点”，而不是严格的：

```text
scene -> simulator -> target -> planner -> controller -> RViz
```

各进程的具体先后由进程启动和初始化耗时决定。

### target 的既有 start contract

`target_state_coordinator.py:76-83` 订阅三条：

```text
/target_tracking/ready/uav1
/target_tracking/ready/uav2
/target_tracking/ready/uav3
```

scene node 在第一次收到各 UAV odom 时发布 ready：
`native_egov2_rviz_scene.py:434-438`。target coordinator 在三机 ready 后再等
`startup_settle_time=2.0 s`，见 `target_state_coordinator.py:140-161`，然后发布
latched `/target_tracking/target_start_time` 并开始目标运动。

这个 ready 实际代表“收到三机 odom”，不是“RViz 已加载”，也不是严格意义的
planner TRACK ready。日志文字 `after all UAVs entered TRACK` 比源码条件更强，
不能把它当作 RViz barrier。

### UAV simulator 与 planner

`simulator.xml:71-99` 同时启动 SO3 simulator 和 controller；simulator 主循环在
`quadrotor_simulator_so3.cpp:267-299` 从节点启动后立即循环，并在每次循环执行
`quad.step(dt)`。因此旧启动下，simulator 会在 RViz 完成加载前开始积分，即使
初始阶段可能只维持近似静止状态。

planner 由同一 `run_in_sim.launch` 启动；FSM 收到 odom 和 target 后即可从
`INIT -> WAIT_TARGET -> SEQUENTIAL_START` 进入规划。它没有、也不应有 RViz
依赖。因此旧启动下 planner 也可能在 RViz 完成加载/订阅前开始正式规划。

### 旧逻辑是否有固定 sleep

主 shell 脚本没有固定 sleep。唯一相关等待是 target coordinator 的 2.0 秒
三机 odom settle，它不检查 RViz，无法保证 GUI 完成加载。问题本质是一次性
并行 launch，而不是某个已有 RViz sleep 数值过小。

### RViz node 出现是否足够

不足。`rosnode list` 中出现 `/rviz` 只说明 RViz XML-RPC node 已注册，不保证
配置、display、topic subscription 和首次 render 已完成。本次采用用户要求的
低复杂度合同：

1. `/rviz` node 存在；
2. `/map_generator/global_cloud` 已有 publisher；
3. 再等待一个可配置 warm-up；
4. 不做 GUI 像素/窗口检测。

## 修改文件

### `run_constvel_gradient_rviz.sh`

新增：

- `NATIVE_EGOV2_ENABLE_RVIZ`，兼容简写 `ENABLE_RVIZ`；
- `NATIVE_EGOV2_WAIT_FOR_RVIZ`，兼容简写 `WAIT_FOR_RVIZ`；
- `NATIVE_EGOV2_RVIZ_WARMUP_SEC`，兼容简写 `RVIZ_WARMUP_SEC`；
- `NATIVE_EGOV2_RVIZ_READY_TIMEOUT_SEC`；
- RViz ON 的 Phase A/Phase B 编排；
- `/rviz` 与地图 publisher ready 检查；
- Phase A 的 SIGINT 清理 trap；
- `NATIVE_EGOV2_SCENE_FILE`：从 scene JSON 自动传入真实
  `scene_file`、target waypoints、target speed 和 moving-object count，避免人工
  运行新场景时仍误用 launch 内的旧默认路线。

显式 roslaunch 参数仍优先于从 scene JSON 自动提取的值。

### `ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch`

新增两个仅用于编排的 launch arg：

```xml
launch_scene
launch_experiment
```

- `launch_scene=true` 控制 scene/map/metrics node；
- `launch_experiment=true` 控制 bridge、target、viewpoint manager 和三套
  `run_in_sim.launch`；
- `rviz` 仍独立控制 RViz；
- 默认值均保持兼容：正常直接 roslaunch 时仍一次启动完整系统。

## 新启动时序

### RViz ON

```text
Phase A
  roslaunch/master
  -> scene/map/markers/metrics node
     dynamic motion held at elapsed=0
  -> RViz
  -> wait /rviz
  -> wait /map_generator/global_cloud publisher
  -> configurable warm-up

Phase B
  bridge
  -> target coordinator (initial target remains stationary)
  -> three planners/traj_servers
  -> three simulators/controllers/local-sensing nodes
  -> three UAV odometry ready
  -> existing 2 s target settle
  -> publish /target_tracking/target_start_time
  -> target starts moving and dynamic obstacle epoch starts
  -> metrics/CSV start
```

Phase A 强制 `sync_dynamic_motion_to_target_start:=true`。scene 在
`native_egov2_rviz_scene.py:159-165` 将 `motion_start` 保持为 `None`，所以
warm-up 时动态障碍显示的是 elapsed=0 状态；收到 target start 后，
`native_egov2_rviz_scene.py:444-452` 才将动态障碍 epoch 同步到 target epoch。

### RViz OFF

```text
single roslaunch -> original batch behavior
```

不等待 `/rviz`，不等待地图 warm-up，不改变 batch 运行耗时。

## Experiment t=0 与 metrics

正式实验 t=0 定义为 `/target_tracking/target_start_time`，不是 RViz 进程启动时刻，
也不是 Phase A 起点。

依据：

- scene 的 `tracking_start` 只由 target start callback 设置，见
  `native_egov2_rviz_scene.py:444-446`；
- visibility 在 `tracking_start` 为空、当前时刻早于 start、或目标尚未运动时不采样，
  见 `native_egov2_rviz_scene.py:498-515`；
- trajectory CSV 同样在 `tracking_start` 为空时不记录，且 CSV `time_s` 为
  `now - tracking_start`，见 `native_egov2_rviz_scene.py:675-710`；
- collision/static clearance、path 和 tracking 后处理都来自这些正式 trajectory
  rows；warm-up 没有 rows，因此不进入这些指标；
- planner latency 是规划调用内部/epoch-to-commit 的差值，planner 在 Phase A
  根本未启动，因此 warm-up 不进入 latency。

本工程未设置 `/use_sim_time`，ROS time 实际使用 wall-clock ROS time，但所有正式
CSV elapsed 仍减去 target start epoch，所以 warm-up 不进入实验时间。

## 轻度验证

### Build

命令：

```bash
cd /home/bob/ALP/egov2_fc65423_constvel/ros_ws
catkin build multi_uav_formation --no-status -j2
```

结果：PASS。`plan_env` 和 `multi_uav_formation` 成功；无 package failure。

### 静态检查

- `bash -n run_constvel_gradient_rviz.sh`: PASS
- `xmllint --noout native_egov2_rviz.launch`: PASS
- Phase A `roslaunch --nodes`: 仅 scene（RViz=false 的展开检查）
- Phase B `roslaunch --nodes`: 22 个 experiment nodes，包含 target、3 planners、
  3 traj_server、3 simulator、3 controller/local-sensing 链；不含 scene/RViz。

### RViz OFF smoke

- ROS master: isolated port 11471
- 场景：`long_cylinder_forest_visibility_stress.json`
- 运行约 15 秒后 SIGINT
- 日志首行：`RViz OFF; starting the original single-phase experiment without delay`
- trajectory/visibility CSV 正常生成
- 无残留 master/node
- 结果：PASS

### RViz ON smoke

- ROS master: isolated port 11472
- 场景：`long_cylinder_forest_visibility_stress.json`
- configured warm-up: 2.5 s
- RViz node ready: `2026-09-04 12:42:26.201 +08:00`
- map publisher ready: `2026-09-04 12:42:27.113 +08:00`
- warm-up begin: `2026-09-04 12:42:27.116 +08:00`
- Phase B experiment start: `2026-09-04 12:42:29.627 +08:00`
- target clock/first motion epoch: `2026-09-04 12:42:33.278 +08:00`
- dynamic motion synchronized: ROS time `1788496953.278465`
- first planner committed trajectory: ROS time `1788496956.336377`
- Phase B minus RViz node-ready: 3.426 s
- Phase B minus warm-up begin: 2.511 s
- first trajectory CSV row: `time_s=0.029251`, already relative to target start
- RViz 在 Phase B 前已订阅 `/native_egov2/obstacles`、target marker、三机 marker
  和 camera FOV topics
- 结束后 isolated master 不可达，无残留 node/process
- 结果：PASS

shutdown 时 scene Python callback 出现一次 `publish() to a closed topic`，发生在
SIGINT 关闭 publisher 与最后一个 target callback 的竞争窗口；没有残留进程，
也没有发生在运行期。它是短 smoke 强制退出时的 shutdown race，不影响本次
startup sequencing 结论，本轮未扩大范围修改 scene callback。

## 近期场景核对

用户列出的 `/home/bob/ALP/egov2_fc65423_constvel/scenes/...` 目录实际不存在。
当前生产场景均位于：

```text
/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/
```

以下五个近期主场景均存在：

| Scene | Exists | Moving obstacles | Target waypoints |
|---|---:|---:|---:|
| long_cylinder_forest.json | YES | 10 | 25 |
| long_cylinder_forest_dynamic_gates_v2.json | YES | 5 | 26 |
| long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json | YES | 5 | 26 |
| long_cylinder_forest_visibility_stress.json | YES | 3 | 26 |
| long_cylinder_forest_visibility_stress_v2.json | YES | 3 | 26 |

`NATIVE_EGOV2_SCENE_FILE` 会读取各 JSON 的真实 target route/speed/count，因此下面
命令不再依赖 launch 文件中与这些 JSON 不一致的默认 target waypoints。

## 所有可直接复制执行的命令

说明：每条命令均为 RViz ON、等待 RViz/map ready、warm-up 2 秒、target-facing
yaw ON、`max_jer=22`。Native/Gradient/ALP 模式定义与当前公平 benchmark 脚本
一致：

- Native = moving cost OFF, risk candidates OFF, hard corridor SCP OFF
- Gradient = moving cost ON, risk candidates OFF, hard corridor SCP OFF
- ALP = moving cost ON, risk candidates ON, hard corridor SCP ON

Stage1/Stage2/viewpoint 均显式 OFF；MINCO visibility 通过 launch arg 显式 OFF。

### 1. long_cylinder_forest.json

Native:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=false NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/forest_native.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/forest_native_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

Gradient:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/forest_gradient.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/forest_gradient_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

ALP:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=true NATIVE_EGOV2_HARD_CORRIDOR_SCP=true NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/forest_alp.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/forest_alp_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

### 2. long_cylinder_forest_dynamic_gates_v2.json

Native:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=false NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/gates_v2_native.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/gates_v2_native_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

Gradient:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/gates_v2_gradient.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/gates_v2_gradient_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

ALP:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=true NATIVE_EGOV2_HARD_CORRIDOR_SCP=true NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/gates_v2_alp.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/gates_v2_alp_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

### 3. long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json

Native:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=false NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/c1_relaxed_v2_native.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/c1_relaxed_v2_native_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

Gradient:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/c1_relaxed_v2_gradient.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/c1_relaxed_v2_gradient_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

ALP:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=true NATIVE_EGOV2_HARD_CORRIDOR_SCP=true NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/c1_relaxed_v2_alp.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/c1_relaxed_v2_alp_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

### 4. long_cylinder_forest_visibility_stress.json

Native:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=false NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_native.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_native_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

Gradient:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_gradient.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_gradient_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

ALP:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=true NATIVE_EGOV2_HARD_CORRIDOR_SCP=true NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_alp.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_alp_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

### 5. long_cylinder_forest_visibility_stress_v2.json

注意：该场景此前出现过 ALP UAV3 tracking collapse/static collision；命令可用于
人工诊断，不应把它作为当前唯一正式 visibility benchmark。

Native:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=false NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_v2_native.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_v2_native_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

Gradient:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=false NATIVE_EGOV2_HARD_CORRIDOR_SCP=false NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_v2_gradient.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_v2_gradient_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

ALP:

```bash
cd /home/bob/ALP/egov2_fc65423_constvel && mkdir -p manual_rviz_runs && env NATIVE_EGOV2_SCENE_FILE="$PWD/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress_v2.json" NATIVE_EGOV2_ENABLE_RVIZ=1 NATIVE_EGOV2_WAIT_FOR_RVIZ=1 NATIVE_EGOV2_RVIZ_WARMUP_SEC=2 NATIVE_EGOV2_MOVING_COST=true NATIVE_EGOV2_RISK_CANDIDATES=true NATIVE_EGOV2_HARD_CORRIDOR_SCP=true NATIVE_EGOV2_EARLY_AVOIDANCE=false NATIVE_EGOV2_TARGET_FACING_YAW=true NATIVE_EGOV2_VISIBILITY_CANDIDATE_RANKING=false NATIVE_EGOV2_ASTAR_VISIBILITY_COST=false NATIVE_EGOV2_COOPERATIVE_VIEWPOINT=false NATIVE_EGOV2_LOG_FILE="$PWD/manual_rviz_runs/visibility_stress_v2_alp.log" NATIVE_EGOV2_VISIBILITY_CSV="$PWD/manual_rviz_runs/visibility_stress_v2_alp_visibility.csv" ./run_constvel_gradient_rviz.sh max_jer:=22 enable_minco_visibility_cost:=false
```

## 配置调整与兼容入口

只需改变一次 warm-up：

```bash
NATIVE_EGOV2_RVIZ_WARMUP_SEC=3
```

临时恢复旧的 RViz 同时启动行为：

```bash
NATIVE_EGOV2_WAIT_FOR_RVIZ=0 ./run_constvel_gradient_rviz.sh
```

headless batch 保持无额外等待：

```bash
NATIVE_EGOV2_ENABLE_RVIZ=0 ./run_constvel_gradient_rviz.sh
```

## Git diff 摘要

本轮触及文件仅有：

1. `run_constvel_gradient_rviz.sh`
2. `ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch`
3. 本报告 `feedback/feedback_17.md`

工作区原本已有大量未提交修改；未 reset、checkout、clean、commit 或覆盖其它
修改。未修改任何 planner/controller 源码。

## 最终字段

RVIZ_STARTUP_ROOT_CAUSE_CONFIRMED: YES

RVIZ_WAS_LAUNCHED_CONCURRENTLY_WITH_EXPERIMENT: YES

TARGET_COULD_START_BEFORE_RVIZ_RENDER_WARMUP_COMPLETED: YES

UAV_SIMULATOR_INTEGRATED_BEFORE_RVIZ_READY: YES

PLANNER_HAD_NO_RVIZ_READY_BARRIER: YES

EXISTING_RVIZ_READY_TRIGGER_FOUND: NO

EXISTING_TARGET_START_TRIGGER_REUSED: YES

RVIZ_NODE_REGISTRATION_ALONE_CONSIDERED_SUFFICIENT: NO

RVIZ_ON_TWO_PHASE_STARTUP_IMPLEMENTED: YES

RVIZ_OFF_EXTRA_WAIT: NO

RVIZ_WARMUP_DEFAULT: 2.0 s

EXPERIMENT_T0: /target_tracking/target_start_time

WARMUP_INCLUDED_IN_METRICS: NO

BUILD: PASS

RVIZ_OFF_SMOKE: PASS

RVIZ_ON_SMOKE: PASS

RESIDUAL_ROS_PROCESSES_AFTER_SMOKE: NO

PLANNER_ALGORITHM_CODE_MODIFIED: NO

SCENE_GEOMETRY_MODIFIED: NO

PREVIOUS_MAX_FEEDBACK_INDEX: 16

CURRENT_FEEDBACK_FILE: feedback_17.md
