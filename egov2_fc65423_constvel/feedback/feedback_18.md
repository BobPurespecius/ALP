# Feedback 18 — Stop → Restart → Straight Dynamic Collision Audit

## Scope and verdict

This was a read-only audit. No production source, parameter, scene, or runtime state was changed, and no new experiment was run.

The repeated mechanism is confirmed in two historical full runs:

1. recovery repeatedly fails while a previously safe finite trajectory approaches its end;
2. the previous trajectory is validated early in the planning cycle, but expires during the long bilateral recovery solve;
3. selection still uses the stale `previous_safe_available` result and republishes the already expired trajectory;
4. `traj_server`/SO3 execution consequently holds the last position, producing the visible pause;
5. the next FSM cycle detects expiry and replans from odometry;
6. both SIDE recoveries fail, so the planner commits a newly solved NOMINAL trajectory even though dynamic risk explicitly classifies it `INVALID`;
7. because the target-relative reference has moved 2.81 m / 3.50 m away, that nominal trajectory is a near-straight catch-up trajectory;
8. command and odometry follow that trajectory, and it intersects a moving obstacle about one second after motion restarts.

Thus the repeated collision is **TYPE R2 — NEW_NOMINAL_OR_FALLBACK_STRAIGHT_TRAJECTORY**, with a preceding finite-trajectory lifecycle gap. It is not an old trajectory resuming, not a controller-stuck event, and not primarily a 2 s prediction-horizon miss: the planner already predicted the restart fallback to be unsafe.

## Run selection and evidence scope

The chronologically newest full manual runs were checked first:

- `/home/bob/ALP/egov2_fc65423_constvel/manual_rviz_runs/c1_relaxed_v2_alp.log` with `c1_relaxed_v2_alp_visibility_trajectory.csv`, modified September 4, 2026 15:10. It contains a UAV3 dynamic collision, but UAV speed during that collision is 0.552–1.491 m/s and no stop→restart precedes it.
- `/home/bob/ALP/egov2_fc65423_constvel/manual_rviz_runs/forest_alp.log` with `forest_alp_visibility_trajectory.csv`, modified September 4, 2026 15:07. It has no static or dynamic collision.

They therefore do not constitute two reproductions of the requested pattern. The newest two complete runs that do contain the pattern and have launcher lifecycle logs plus trajectory CSV are:

- **RECENT_RUN_A:** `/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_benchmark/alp/`
  - scene: `long_cylinder_forest_visibility_stress.json`
  - scene SHA256: `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`
  - artifacts: `alp.launcher.log`, `alp_trajectory.csv`, visibility CSVs, `run_meta.json`, `watchdog.log`
  - completed normally; target final stop detected after the collision.
- **RECENT_RUN_B:** `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/alp_3/`
  - scene: `long_cylinder_forest_dynamic_gates.json`
  - artifacts: `alp.launcher.log`, `alp_trajectory.csv`, visibility CSVs, `run_meta.json`
  - completed normally; target final stop occurred after the collision.

Run A is later than Run B by artifact modification time (September 2, 2026 21:11 versus 19:00). Both events concern CSV `uav_id=1`, which maps to planner `drone_id=0`.

## Runtime source and execution chain

The active build cache resolves `ego_planner`, `traj_opt`, `path_searching`, `plan_env`, and `traj_utils` to:

`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`

The actual chain is:

`run_constvel_gradient_rviz.sh`
→ `multi_uav_formation/launch/native_egov2_rviz.launch`
→ scene and dynamic-obstacle publisher + target coordinator
→ three `ego_planner/launch/run_in_sim.launch` instances
→ FSM / planner manager
→ `PolyTraj`
→ `traj_server`
→ `PositionCommand`
→ SO3 controller
→ SO3 quadrotor simulator
→ odometry
→ scene CSV collision/visibility metrics.

Source evidence:

- `run_constvel_gradient_rviz.sh:48,167-214` selects the launch and orchestrates it.
- `native_egov2_rviz.launch:83-109` starts the scene and target; `:140-200` starts the first planner chain, repeated for the other UAVs.
- `ego_replan_fsm.cpp:159-164` publishes one heartbeat at the beginning of each FSM callback; a synchronous long planning callback prevents another heartbeat until it returns.
- `ego_replan_fsm.cpp:243-252` retries `REPLAN_TRAJ`; `:679-695` publishes `PolyTraj` only when planning reports success.
- `planner_manager.cpp:5014-5028` selects and commits the candidate.
- `traj_server.cpp:47-89` reconstructs and activates every received polynomial. It has no trajectory-generation/start-time monotonic acceptance gate.
- `traj_server.cpp:244-284` publishes commands only while `0 <= t_cur < duration`.
- `so3_control_nodelet.cpp:96-118` stores each `PositionCommand`; `:122-147` continues controlling to the last desired state when no new position command arrives.
- `quadrotor_simulator_so3.cpp:267-300` continuously integrates the last SO3 command and publishes odometry.

## Automated stop/restart definition

The trajectory CSV was time-sorted per UAV; duplicated timestamps were removed before finite differences. The requested diagnostic was applied with three thresholds:

- stationary: odom speed below 0.03, 0.05, or 0.10 m/s;
- main requested duration: at least 0.5 s;
- restart: sustained odom speed above 0.15 m/s, corroborated by a new nonzero `PositionCommand`.

Important qualification: neither run has 0.5 continuous seconds below the strict 0.05 m/s odom threshold. Both do have an approximately 0.80 s near-hover below 0.10 m/s, and their commanded velocity reaches essentially zero. Therefore the visual stop/hold pattern is confirmed, but it must not be mislabeled as satisfying the strict 0.05 m/s definition.

| Detection | Run A | Run B |
|---|---:|---:|
| longest last span below 0.03 m/s | 0.101 s or less | 0.266 s |
| longest last span below 0.05 m/s | 0.200 s | 0.333 s |
| near-hover below 0.10 m/s | 75.266630–76.066879, 0.800249 s | 60.537157–61.337257, 0.800100 s |
| sustained restart above 0.15 m/s | 76.099872 s | 61.370234 s |
| collision start | 77.099845 s | 62.370540 s |
| restart→collision | 0.999973 s | 1.000306 s |

## Quantitative collision and restart geometry

| Evidence | Run A | Run B | Repeatable |
|---|---:|---:|---|
| obstacle | `dynamic_gate_V3`, id=2 | `dynamic_gate_C2`, id=3 | moving gate |
| collision interval | 77.099845–77.299869 s | 62.370540–62.443536 s | yes |
| collision samples | 7 | 3 | yes |
| deepest signed surface clearance | -0.229323 m | -0.066708 m | penetration |
| obstacle speed at deepest point | 0.250213 m/s | 0.461961 m/s | obstacle moving |
| relative radial speed at collision entry | -2.560 m/s | -1.647 m/s | closing |
| static clearance at collision entry | 0.927047 m | 0.504608 m | no static collision |
| restart tracking error to fixed formation reference | 2.812162 m | 3.501017 m | reference ran ahead |
| restart→collision executed path length | 1.888462 m | 1.756413 m | — |
| chord/path straightness | 0.998501 | 0.999485 | nearly straight |
| maximum lateral deviation from chord | 0.040550 m | 0.016433 m | nearly straight |
| total discrete heading change | 15.116° | 11.251° | small |
| motion direction vs fixed-reference direction | 2.855° | 3.328° | target catch-up aligned |
| odom speed at collision entry | 2.560496 m/s | 2.718885 m/s | UAV restarted strongly |

Both collisions are dynamic. The observation “straight restart then static collision” is not present in these two newest reproducible samples, so this report does not claim a repeated static-collision mechanism.

## Run A autopsy

### Timeline

| Relative/ROS time | State and evidence | Event |
|---|---|---|
| before 75.27 s | trajectory 254 is NOMINAL, classified safe when committed; ROS start `1788354684.000697136`, duration 2.217082 s, piece count 2 | LAST_VALID_TRAJECTORY |
| ROS 1788354685.815–4686.324 | repeated SIDE recovery failures: `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`, `QP_MAX_ITER_EXHAUSTED`, and retimed static collision; no usable replacement | PLANNER_FAILURES |
| 75.266630 s | odom enters the final <0.10 m/s near-hover span | STOP_BEGIN |
| ROS 1788354686.209 approximately | command from trajectory 254 has speed about 0.000109 m/s | COMMAND_HOLD |
| ROS 1788354686.217779 | finite trajectory 254 reaches its mathematical end | TRAJECTORY_EXPIRE |
| ROS 1788354686.323654–.323814 | PLUS=`QP_MAX_ITER_EXHAUSTED`, MINUS=`DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`; nominal clearance 0.320561 and `INVALID`; nevertheless prior cycle result says `KEEP_PREVIOUS_SAFE`, logs age 2.323 s and remaining 0, and republishes id 254 | STALE PREVIOUS TOCTOU |
| ROS 1788354686.334343 | FSM reports local trajectory expired and replans from odometry | RESTART_TRIGGER |
| ROS 1788354686.334431 | fresh nominal initializer: start `(27.692908,-2.467415,1.500474)`, local target `(34.197693,1.267148,1.499907)`, piece count 5 | TARGET CATCH-UP INIT |
| ROS 1788354686.435838 | SIDE_PLUS A*/SFC path reaches SCP but ends `QP_MAX_ITER_EXHAUSTED`; its predicted clearance was 1.714893 m | LAST SIDE FAILURE |
| ROS 1788354686.436300–.436821 | SIDE_MINUS has A* and `SFC_NOT_REQUIRED`, then `SIDE_INIT_RETIMED_STATIC_COLLISION` | OTHER SIDE FAILURE |
| ROS 1788354686.436821 | NOMINAL dynamic center distance 0.327638 m, threshold 1.1 m, class `INVALID`, reason `UNSAFE_FALLBACK` | UNSAFE SELECTION |
| ROS 1788354686.473050–.473588 | fresh-head check passes (0.005689 m); NOMINAL id 255, duration 4.308592 s, piece count 5 is selected, committed, and published | RESTART_TRAJECTORY_COMMIT |
| ROS 1788354686.474100 | traj_server activates id 255 | RESTART_ACTIVATION |
| ROS 1788354686.479–.690 | command speed changes from about 0.059 to 0.092 to 0.472 m/s | RESTART_COMMAND |
| 76.099872 s | odom speed stays above 0.15 m/s | RESTART_ODOM_MOTION |
| following second | ids 256–260 are successively committed; all are NOMINAL unsafe fallbacks with predicted clearance about 0.329–0.339 m | CONTINUED UNSAFE FALLBACK |
| 77.099845 s | UAV1 enters `dynamic_gate_V3`; static clearance remains positive | COLLISION |

Log anchors: `alp.launcher.log:44720,44786,44796-44804,44812-44822,44908,44925,44941,44946-44953,45151-45177,45346-45352,45754-45880`.

### Stop cause

In the five seconds before the final hold, the drone-0 log contains 18 previous-trajectory retain decisions, four `NO_STATIC_FEASIBLE_SIDE`, eleven `SIDE_INIT_RETIMED_STATIC_COLLISION`, fourteen dynamics-model/trust exhaustions, two QP max-iteration failures, one spatial-trust exhaustion, and one reported true infeasibility. During the immediate stop/restart interval, recovery still produces five model/trust failures and two QP max-iteration failures.

The stop is therefore not controller failure. The usable old trajectory is consumed while recovery cannot deliver a replacement. Its terminal velocity approaches zero, then no newer safe command exists.

### Restart trajectory

- kind: NOMINAL emergency fallback;
- class: INVALID;
- predicted dynamic center distance: 0.327638 m versus 1.1 m absolute threshold;
- A*: not used;
- simplified guide count `N_g`: 0;
- MINCO piece count: 5, hence internal-junction count `N_P=4`;
- Local-SFC: not active;
- SIDE hard-corridor SCP: not applicable to this nominal trajectory;
- final static/dynamics post-check: passed sufficiently for the solved nominal to remain committable;
- P does not preserve A* guide geometry because this candidate has no A* guide.

The straightness is not evidence that A* junctions were lost: no A* candidate was selected. It is the ordinary nominal reconstruction from current odometry toward a reference already 2.812 m ahead. Its executed direction differs from the fixed-reference line by only 2.855°.

## Run B autopsy

### Timeline

| Relative/ROS time | State and evidence | Event |
|---|---|---|
| before 60.54 s | trajectory 210 is NOMINAL; ROS start `1788346778.482151031`, duration 1.851400 s, piece count 2 | LAST_VALID_TRAJECTORY |
| ROS 1788346779.187–6780.527 | several long recovery calls delay the next FSM heartbeat; SIDE failures include QP max iteration, model/trust exhaustion, P-trust exhaustion, and static preinit failures | PLANNER_FAILURES |
| 60.537157 s | odom enters final <0.10 m/s near-hover | STOP_BEGIN |
| ROS 1788346780.333551 | finite trajectory 210 expires | TRAJECTORY_EXPIRE |
| ROS 1788346780.525372–.526225 | SIDE_PLUS ends `P_TRUST_TOO_SMALL`; SIDE_MINUS has no feasible static side; stale earlier previous validation still causes `KEEP_PREVIOUS_SAFE`, age 2.044 s, remaining 0, and republish of id 210 | STALE PREVIOUS TOCTOU |
| ROS 1788346780.527014 | traj_server heartbeat exceeds 0.5 s; it publishes last position with zero v/a/j and disables trajectory reception | COMMAND_HOLD |
| ROS 1788346780.536725 | FSM detects expired local trajectory and replans from odometry | RESTART_TRIGGER |
| ROS 1788346780.536811 | fresh nominal initializer: start `(17.446249,2.423709,1.499545)`, local target `(24.918936,3.357321,1.499901)`, piece count 5 | TARGET CATCH-UP INIT |
| ROS 1788346780.600–.644 | both SIDE attempts fail. PLUS A* succeeds with 49 raw/25 simplified points and semantic PASS, but historical zero-plane handling reports `SFC_BUILD_FAILED`; MINUS is static-infeasible | RECOVERY FAILURE |
| ROS 1788346780.644667 | NOMINAL predicted distance 0.418743 m, class `INVALID`, reason `UNSAFE_FALLBACK` | UNSAFE SELECTION |
| ROS 1788346780.690421–.691016 | head error 0.000828 m; NOMINAL id 211, duration 2.987132 s, piece count 5 is selected, committed, and published | RESTART_TRAJECTORY_COMMIT |
| ROS 1788346780.691525 | traj_server activates id 211 | RESTART_ACTIVATION |
| ROS 1788346780.6968–.9070 | command speed changes from about 0.019 to 0.234 to 0.723 m/s | RESTART_COMMAND |
| 61.370234 s | odom speed remains above 0.15 m/s | RESTART_ODOM_MOTION |
| following second | ids 212–219 are successively committed NOMINAL trajectories; logged dynamic clearances remain 0.356–0.621 m, below 1.1 m | CONTINUED UNSAFE FALLBACK |
| 62.370540 s | UAV1 enters `dynamic_gate_C2`; static clearance remains positive | COLLISION |

Log anchors: `alp.launcher.log:41237,41241,41252,41256,41260-41268,41290-41298,41315-41335,41383-41390,41455-41636,41701-42386,42713-44041`.

### Stop cause

Within the five seconds preceding the hold, drone 0 has three `NO_STATIC_FEASIBLE_SIDE`, two retimed-static failures, three QP max-iteration failures, five model/trust exhaustions, one `P_TRUST_TOO_SMALL`, and seven previous-retain decisions. In the stop/restart interval it has one `SIDE_INIT_FROM_PREVIOUS_FAILED`, three no-static-side failures, one P-trust failure, one stale-head rejection, one expiry, and three heartbeat-loss messages.

The historical zero-plane Local-SFC rejection is a real Run B recovery blocker, but not the repeated cross-run root by itself: Run A reaches `SFC_NOT_REQUIRED` correctly and still follows the same stop→unsafe-nominal restart chain.

### Restart trajectory

- kind: NOMINAL emergency fallback;
- class: INVALID;
- predicted dynamic center distance: 0.418743 m;
- A*: not used by the selected trajectory;
- `N_g=0`, MINCO piece count 5, therefore `N_P=4`;
- Local-SFC: not active on the selected nominal;
- SIDE hard-corridor SCP: not applicable;
- static/dynamics checking passes, but dynamic classification explicitly fails;
- no repaired guide geometry exists to preserve.

The fixed-reference tracking error is 3.501 m, and executed motion is within 3.328° of the reference catch-up direction. The path has only 0.0164 m maximum lateral deviation over 1.756 m.

## Why the UAV stops

The immediate execution mechanism is a commanded hold, not a frozen simulator:

1. `planner_manager.cpp:4721-4740` uses `previous_safe_available`, computed earlier in the cycle, after potentially long bilateral SIDE/SCP work. It does not revalidate the remaining duration at the final retain decision.
2. In both logs the previous trajectory is reported with `remaining_duration=0.000` and is nevertheless retained and republished.
3. `ego_replan_fsm.cpp:159-164` emits the heartbeat only at callback entry. In Run B, the long synchronous planning call also trips the 0.5 s heartbeat timeout.
4. `traj_server.cpp:236-242` responds to heartbeat loss with exactly one last-position, zero-velocity command and sets `receive_traj_=false`.
5. On ordinary trajectory expiry, `traj_server.cpp:250-284` simply ceases publishing; it does not publish a new terminal trajectory. `so3_control_nodelet.cpp:136-146` continues applying control using the last desired position, so the physical effect is still a hold.

This is a time-of-check/time-of-use lifecycle gap: the finite previous trajectory can be safe when checked near cycle start and expired by selection time.

## What makes it restart

`ego_replan_fsm.cpp:739-748` detects that the local trajectory is expired and calls `planFromGlobalTraj`, which resets head position/velocity from current odometry (`:700-708`). A freshly solved nominal polynomial is then accepted because it has a valid polynomial and passes static/dynamics checks. The new `PolyTraj` activates immediately in `traj_server`, and the controller responds to its nonzero desired velocity.

There is no evidence of an old trajectory or old generation resuming. IDs advance from 254→255 in Run A and 210→211 in Run B. No out-of-order old `PolyTraj` causes the restart.

## Why the restart is straight

The selected candidate is not SIDE, not A* repair, and not Local-SFC constrained. It is a new NOMINAL fallback. Therefore:

- no `g` path exists (`N_g=0`);
- no A* repair junctions can be lost or preserved;
- `N_P=4` is the normal four internal junctions for the five-piece nominal initializer;
- SIDE preservation, Local-SFC planes, and candidate SCP constraints belong only to the failed alternatives, not to the committed polynomial.

During the hold the target/reference continues moving. At restart the fixed formation reference is 2.812 m (A) / 3.501 m (B) away. The nominal objective consequently produces a strong catch-up path. Direct measurement, not appearance, shows chord/path ratios above 0.9985 and reference-direction errors below 3.33°.

## Why safety checking does not stop it

The dynamic checker does detect the danger:

- Run A restart nominal: 0.327638 m center distance;
- Run B restart nominal: 0.418743 m center distance;
- absolute-safe threshold: 1.1 m;
- both classes: `INVALID`;
- both selection reasons: `UNSAFE_FALLBACK`.

Current production logic explains the commit:

- `planner_manager.cpp:1738-1767` correctly classifies a solved candidate below 1.1 m as not `ABSOLUTE_SAFE`.
- `planner_manager.cpp:4721-4740` protects a still-valid previous ABS trajectory.
- once no previous safe trajectory remains, `planner_manager.cpp:4817-4824` rejects only an unsolved/empty candidate.
- `planner_manager.cpp:5014-5028` commits a solved NOMINAL candidate regardless of its `INVALID` dynamic class.

This is the explicitly nonblocking emergency fallback policy. The static and dynamics checks are not the failure here; both collision locations remain static-free. Dynamic risk is advisory to the fallback selection once no safe alternative exists.

The 2 s horizon is a design limitation (`planner_manager.cpp:845-901` checks at most `min(2/3 T, horizon)`), but it is not causal for these two collisions: the risk was already unsafe inside the evaluated horizon before commit.

## Command versus execution

Both runs are **TYPE 1 COMMAND_HOLD** during the pause and **TYPE R2** after restart.

- Run A command velocity falls to approximately 0.000109 m/s before id 255 arrives.
- Run B explicitly receives the heartbeat-loss zero-velocity command.
- After restart, command and odometry move together. Near Run A collision, id 260 commands approximately `(29.320,-1.423,1.470)` while collision odometry is within about 0.06 m. Near Run B collision, id 218 commands `(19.238,2.756,1.480)` and collision odometry is within about 0.061 m.
- The simulator continues 1000 Hz integration and 100 Hz odometry publication; no command-topic interruption, NaN, frozen odometry timestamp, or controller saturation explains either collision.

Controller/simulator execution is therefore not primary.

## Cross-run comparison

| Evidence | Run A | Run B | Repeatable |
|---|---|---|---|
| command stopped | yes, terminal command nearly zero | yes, heartbeat zero command | yes |
| trajectory expired during stop | id 254 | id 210 | yes |
| planner continued replanning | yes | yes | yes |
| recovery failed during stop | QP/model trust + retimed static | P trust + SFC/static feasibility | yes, exact backend status differs |
| restart caused by new commit | id 255 | id 211 | yes |
| restart kind/class | NOMINAL / INVALID fallback | NOMINAL / INVALID fallback | yes |
| restart nearly straight | 0.998501 chord/path | 0.999485 chord/path | yes |
| A* used by committed restart | no | no | yes |
| Local-SFC on committed restart | no | no | yes |
| candidate SCP on committed restart | N/A | N/A | yes |
| dynamic risk valid | yes | yes | yes |
| risk passed 1.1 m | no, 0.327638 | no, 0.418743 | yes |
| collision type | dynamic V3 | dynamic C2 | yes |
| execution followed command | yes | yes | yes |

## Related collision-design audit

| Item | Classification | Evidence / relevance |
|---|---|---|
| previous-safe validated before long recovery then used after expiry | **CONFIRMED CAUSAL** | both logs retain age greater than duration with remaining 0 |
| solved INVALID nominal fallback can commit | **CONFIRMED CAUSAL** | source `4817-4824`, `5014-5028`; both runs log `UNSAFE_FALLBACK` |
| terminal trajectory/hold is outside finite polynomial safety contract | **CONFIRMED CONTRIBUTING** | traj_server stops polynomial commands after T; planner risk checks only finite candidate |
| heartbeat coupled to synchronous FSM callback | **CONFIRMED CONTRIBUTING, Run B** | heartbeat only at callback entry; timeout generates zero command |
| 2 s dynamic horizon | **DESIGN_LIMITATION, NOT CAUSAL HERE** | both restart trajectories are already predicted unsafe inside horizon |
| dynamic separation is absent from candidate SCP hard rows | **DESIGN_LIMITATION** | SIDE recovery relies on risk/region and final classification; nominal fallback remains committable |
| final static check samples only first 2/3 when not touching goal | **LIKELY RISK, NOT CAUSAL HERE** | `checkTrajectoryStaticSafety:1110-1159`; both actual collisions are dynamic with positive static clearance |
| static shell/interior voxel issue | **NOT CAUSAL HERE** | collision points are not static penetrations |
| stale prediction epoch | **NOT CAUSAL HERE** | risk is valid and already predicts a sub-threshold path |
| target/reference catch-up | **CONFIRMED CONTRIBUTING** | 2.81/3.50 m errors and <3.33° direction alignment |
| old generation overwrites new at traj_server | **LIKELY RISK, NOT OBSERVED HERE** | callback has no monotonic gate, but actual restart IDs increase |
| controller or simulator stuck | **NOT CAUSAL HERE** | command/odom agreement and immediate response after new command |
| final target stop / auto cutoff | **NOT CAUSAL HERE** | both collisions precede final target stop |
| dynamic obstacle epoch offset from target start | **DESIGN/FAIRNESS LIMITATION, NOT PRIMARY HERE** | launch default is unsynchronized in these historical runs; both prediction and actual obstacle still identify the danger |

## Historical run versus current dirty source

The run logs predate some later fixes in the dirty worktree. Current source now:

- uses `ros::Time::now()` when initially slicing the previous trajectory (`planner_manager.cpp:752-775`);
- reruns dynamic risk/classification after post-check mutation (`:4922-4955`);
- caches only committed `ABSOLUTE_SAFE` trajectories (`:5058-5124`);
- handles zero-plane Local-SFC as `SFC_NOT_REQUIRED` on the corrected path.

However, the key TOCTOU remains structurally possible: `previous_safe_available` is computed before bilateral recovery and is not revalidated immediately before `KEEP_PREVIOUS_SAFE`. Separately, the nonblocking commit path still allows a solved `INVALID` nominal when no previous ABS trajectory is available.

## Four direct answers

1. **Why does the UAV suddenly stop?** Recovery cannot produce a new usable SIDE trajectory before the previous finite trajectory reaches its terminal state; a stale early-cycle previous-safe result republishes an already expired trajectory, and traj_server/SO3 execution holds the last commanded point. Run B additionally loses the planner heartbeat during a long synchronous solve and explicitly receives a zero-velocity hold command.
2. **What makes it move again?** The next FSM callback detects trajectory expiry, replans from current odometry, and commits a newly solved NOMINAL trajectory (id 255 / 211); traj_server activates it and nonzero commands resume.
3. **Which trajectory is executed, and why is it straight?** It is a new five-piece NOMINAL `INVALID/UNSAFE_FALLBACK`, not the failed SIDE/A*/SFC path. With no A* guide and a reference already 2.81/3.50 m ahead, its four nominal junctions form a catch-up curve whose measured straightness exceeds 0.9985.
4. **Why did safety checking not block the collision?** Static/dynamics checks passed and dynamic risk correctly reported only 0.328/0.419 m, but once previous ABS safety expired the nonblocking selection policy commits any solved nominal polynomial even when its dynamic safety class is `INVALID`; the checker diagnosed the hazard but did not have veto authority over that fallback.

## Minimal fix direction — recommendation only

1. Revalidate previous remaining duration and safety at the final selection/retain point, after all bilateral solving, using current time; never report or republish an already expired previous trajectory as retained safe.
2. Define one explicit terminal execution contract shared by planner and traj_server: either plan/check a bounded terminal hold interval or require a fresh executable replacement before finite-trajectory expiry. Do not leave “no command but controller holds last setpoint” implicit.
3. Decouple planner heartbeat publication from the long synchronous optimization callback so computational latency cannot itself generate an execution hold.
4. Preserve nonblocking behavior, but distinguish “solved polynomial” from “dynamically executable”: an `INVALID` fallback known to cross an obstacle should not silently become a normal catch-up trajectory. The exact fallback behavior requires a separate design decision; it is not implemented here.
5. Add lifecycle telemetry for final-time previous revalidation, terminal-hold age, and active safety class so future stop/restart events can be reconstructed without inference.

## Required final fields

RECENT_RUN_A: /home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_benchmark/alp/

RECENT_RUN_B: /home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/alp_3/

STOP_RESTART_COLLISION_PATTERN_CONFIRMED_RUN_A: YES — command hold + 0.10 m/s near-hover; strict 0.05 m/s for 0.5 s is not met

STOP_RESTART_COLLISION_PATTERN_CONFIRMED_RUN_B: YES — explicit heartbeat hold + 0.10 m/s near-hover; strict 0.05 m/s for 0.5 s is not met

STOP_DURATION_RUN_A: 0.800249 s below 0.10 m/s; longest final span below 0.05 m/s = 0.200 s

STOP_DURATION_RUN_B: 0.800100 s below 0.10 m/s; longest final span below 0.05 m/s = 0.333 s

STOP_PRIMARY_CAUSE_RUN_A: previous finite trajectory exhausted while SIDE recovery repeatedly failed; stale pre-solve previous-safe result retained/published it after expiry, producing terminal command hold

STOP_PRIMARY_CAUSE_RUN_B: same previous-safe expiry/TOCTOU plus a planner-heartbeat timeout during long recovery, which explicitly commanded zero-velocity hold

COMMAND_STOPPED_RUN_A: YES

COMMAND_STOPPED_RUN_B: YES

TRAJECTORY_EXPIRED_DURING_STOP_RUN_A: YES — id 254

TRAJECTORY_EXPIRED_DURING_STOP_RUN_B: YES — id 210

PLANNER_CONTINUED_REPLANNING_DURING_STOP_RUN_A: YES

PLANNER_CONTINUED_REPLANNING_DURING_STOP_RUN_B: YES

PLANNER_FAILURES_DURING_STOP_RUN_A: YES — QP_MAX_ITER_EXHAUSTED, DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED, SIDE_INIT_RETIMED_STATIC_COLLISION

PLANNER_FAILURES_DURING_STOP_RUN_B: YES — P_TRUST_TOO_SMALL, NO_STATIC_FEASIBLE_SIDE, historical zero-plane SFC_BUILD_FAILED, heartbeat loss

RESTART_TRIGGER_RUN_A: FSM trajectory-expiry detection → replan from odometry → new nominal commit id 255

RESTART_TRIGGER_RUN_B: FSM trajectory-expiry detection → replan from odometry → new nominal commit id 211

RESTART_TRAJECTORY_KIND_RUN_A: NOMINAL / UNSAFE_FALLBACK

RESTART_TRAJECTORY_KIND_RUN_B: NOMINAL / UNSAFE_FALLBACK

RESTART_TRAJECTORY_SAFETY_CLASS_RUN_A: INVALID

RESTART_TRAJECTORY_SAFETY_CLASS_RUN_B: INVALID

RESTART_TRAJECTORY_NEARLY_STRAIGHT_RUN_A: YES — chord/path 0.998501, lateral deviation 0.040550 m

RESTART_TRAJECTORY_NEARLY_STRAIGHT_RUN_B: YES — chord/path 0.999485, lateral deviation 0.016433 m

RESTART_TRAJECTORY_USED_ASTAR_RUN_A: NO

RESTART_TRAJECTORY_USED_ASTAR_RUN_B: NO

RESTART_TRAJECTORY_LOCAL_SFC_ACTIVE_RUN_A: NO

RESTART_TRAJECTORY_LOCAL_SFC_ACTIVE_RUN_B: NO

RESTART_TRAJECTORY_NATIVE_STATIC_ACTIVE_RUN_A: BASELINE NOMINAL STATIC COST/CHECK YES; SIDE/NATIVE HARD CORRIDOR NO

RESTART_TRAJECTORY_NATIVE_STATIC_ACTIVE_RUN_B: BASELINE NOMINAL STATIC COST/CHECK YES; SIDE/NATIVE HARD CORRIDOR NO

RESTART_TRAJECTORY_SCP_OK_RUN_A: NOT_APPLICABLE — committed candidate was nominal, failed SIDE candidates entered SCP

RESTART_TRAJECTORY_SCP_OK_RUN_B: NOT_APPLICABLE — committed candidate was nominal, failed SIDE candidate entered SCP

RESTART_P_COUNT_RUN_A: 4 internal MINCO junctions (5 pieces)

RESTART_P_COUNT_RUN_B: 4 internal MINCO junctions (5 pieces)

RESTART_G_COUNT_RUN_A: 0

RESTART_G_COUNT_RUN_B: 0

RESTART_P_PRESERVED_GUIDE_GEOMETRY_RUN_A: NO/NOT_APPLICABLE — no A* guide in selected nominal

RESTART_P_PRESERVED_GUIDE_GEOMETRY_RUN_B: NO/NOT_APPLICABLE — no A* guide in selected nominal

TRACKING_ERROR_AT_RESTART_RUN_A: 2.812162 m to fixed formation reference

TRACKING_ERROR_AT_RESTART_RUN_B: 3.501017 m to fixed formation reference

RESTART_DIRECTION_MATCHES_TARGET_CATCHUP_RUN_A: YES — 2.855° difference

RESTART_DIRECTION_MATCHES_TARGET_CATCHUP_RUN_B: YES — 3.328° difference

COLLISION_TYPE_RUN_A: DYNAMIC — dynamic_gate_V3 / TYPE R2

COLLISION_TYPE_RUN_B: DYNAMIC — dynamic_gate_C2 / TYPE R2

STATIC_FINAL_CHECK_PASSED_BEFORE_COLLISION_RUN_A: YES for the committed restart nominal; actual static clearance remained positive

STATIC_FINAL_CHECK_PASSED_BEFORE_COLLISION_RUN_B: YES for the committed restart nominal; actual static clearance remained positive

DYNAMIC_RISK_PASSED_BEFORE_COLLISION_RUN_A: NO — risk valid, predicted 0.327638 m and class INVALID

DYNAMIC_RISK_PASSED_BEFORE_COLLISION_RUN_B: NO — risk valid, predicted 0.418743 m and class INVALID

DYNAMIC_COLLISION_OUTSIDE_PREDICTION_HORIZON_RUN_A: NO

DYNAMIC_COLLISION_OUTSIDE_PREDICTION_HORIZON_RUN_B: NO

EXECUTION_MATCHED_COMMAND_RUN_A: YES

EXECUTION_MATCHED_COMMAND_RUN_B: YES

REPEATABLE_ROOT_CAUSE: YES — same lifecycle-expiry → odom replan → known-unsafe nominal fallback → straight catch-up → dynamic collision chain; backend SIDE failure details differ

PRIMARY_ROOT_CAUSE: planner selection/lifecycle contract permits a dynamically INVALID nominal fallback to be committed after previous ABS safety expires, while previous-safe validity is not refreshed at final selection time

SECONDARY_ROOT_CAUSE: repeated SIDE recovery failure consumes the remaining old trajectory; moving target/reference creates a large catch-up error, and Run B heartbeat coupling strengthens the hold

STOP_CAUSES_STRAIGHT_RESTART: PARTIAL — expiry forces odom-based fresh nominal planning, while straightness specifically comes from target-reference catch-up and absence of a usable SIDE candidate

STRAIGHT_RESTART_CAUSES_COLLISION: YES — the executed near-line intersects the moving gate; the planner had already predicted the insufficient clearance

PLANNER_PRIMARY: YES

TRAJ_SERVER_PRIMARY: NO — its terminal/heartbeat hold exposes and amplifies the planner lifecycle gap but does not choose the unsafe restart

CONTROLLER_PRIMARY: NO

STATIC_CONSTRAINT_LIFECYCLE_PRIMARY: NO

DYNAMIC_HORIZON_PRIMARY: NO

TARGET_CATCHUP_PRIMARY: NO — confirmed contributing geometry, not the safety-policy root

OTHER_CONFIRMED_ISSUES: previous-safe final-selection TOCTOU; heartbeat tied to blocking planner callback; implicit terminal hold contract; traj_server lacks monotonic generation gate; non-goal static/dynamic checks cover only finite prefixes

MINIMAL_FIX_DIRECTION: final-time previous revalidation + explicit checked terminal-hold contract + heartbeat decoupling + prevent known-dynamically-INVALID fallback from masquerading as normal executable recovery

PRODUCTION_CODE_CHANGED: NO

SCENE_CHANGED: NO

NEW_EXPERIMENT_RUN: NO

CURRENT_FEEDBACK_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_18.md
