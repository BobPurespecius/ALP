# Feedback 091 — ALPv2 production 架构闭环：Local 唯一轨迹权威、realized validator 与 executor 原子事务

日期：2026-09-21  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
承接：`feedback/feedback_090.md` 及其后的中途重构工作  
最终实跑：`runs/20260921_131436_177465`

## 0. 结论

本轮完成的是既定 production 架构闭环，不是重新设计，也没有新增另一套 Planner：

```text
CooperativeTaskReference（任务几何权威）
→ Local Planner（唯一 executable polynomial producer）
→ target-centered Team R / PHI / reference-phase 建议
→ 三台 Local 分别 realization
→ RealizedTeamValidator 只评价三条真实 polynomial
→ executor PREPARE
→ 3 EXECUTOR_READY
→ coordinator COMMIT
→ 三机 common activation
```

旧的 full-Cartesian `TeamVisibilityOptimizer` 已从源码、CMake 与其专属测试中删除；
coordinator 不再生成或优化三条可执行 Cartesian trajectory。Team optimizer 只能改变围绕
目标的低维参考量 `radius / bearing / reference phase`，Local 的 A*、Local-SFC、MINCO、
hard preflight 与 trajectory identity 仍是唯一轨迹生产权威。

真实 FULL 运行证明新链不是纸面合同：12 个 transaction 完成三机原子激活，未出现 partial
activation；被 abort/revoke 的 transaction 没有一条后来激活。realized validator 在运行中
既 PASS 也 REJECT，说明它不是恒真旁路。

但整机健康性仍为 FAIL：同一 FULL run 有 2 次 terminal hold、2 次
end-before-next、2 条 starvation、目标窗口 1933 个 unvalidated executed samples，以及
17 个 static-contact samples。它们属于仍未闭合的 Local rolling/liveness 与静态安全问题，
不能用 Team transaction 成功掩盖。本轮按任务边界没有继续改 Safety、clearance、A*、
MINCO 或 post-deadline recovery。

```text
FINAL_ARCHITECTURE_CLOSURE: COMPLETE
LOCAL_IS_SOLE_EXECUTABLE_TRAJECTORY_AUTHORITY: YES
TEAM_TARGET_CENTERED_REFERENCE_ONLY: YES
REALIZED_TEAM_VALIDATOR_RUNTIME_EXERCISED: YES
EXECUTOR_PREPARE_3READY_COMMIT_RUNTIME_EXERCISED: YES
PARTIAL_TEAM_ACTIVATION: 0
OLD_FULL_CARTESIAN_TEAM_OPTIMIZER_REMOVED: YES

FULL_RUN_COMPLETED: YES
WHOLE_SYSTEM_HEALTHY: NO
```

## 1. 修改文件（按模块）

以下列出本次架构闭环涉及的主要 production 文件。工作区原有未提交修改全部保留，没有
reset、checkout、clean 或覆盖用户历史工作。

### 1.1 集中式 ablation / runtime truth

```text
run_on.sh
scripts/lib/alp_params.sh
scripts/run_alp_full_on.sh

ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/
  traj_utils/include/traj_utils/ablation_config.h
  traj_utils/include/traj_utils/ablation_runtime.h
```

`run_on.sh --ablation full` 解析为一份明确 manifest；节点接收 resolved configuration，
避免由多处 launch/default 各自猜测 ON/OFF。

### 1.2 target / world snapshot identity

```text
.../plan_env/include/plan_env/obj_predictor.h
.../plan_env/src/obj_predictor.cpp

ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  team_planning_context.h
  tracking_visibility_geometry.h
```

`PlanningWorldSnapshot` 携带 target source、dynamic prediction、static map revision、camera
contract/version 等真实 identity。snapshot ID 不再用 activation time 伪装。

### 1.3 Cooperative task authority

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  cooperative_viewpoint_core.h
ros_ws/src/multi_uav_formation/src/
  cooperative_viewpoint_manager.cpp
```

Cooperative 只发布 soft task geometry/reference，不生产 executable trajectory。

### 1.4 Local trajectory producer 与 J_vis

```text
.../plan_manage/include/plan_manage/planner_manager.h
.../plan_manage/include/plan_manage/ego_replan_fsm.h
.../plan_manage/src/planner_manager.cpp
.../plan_manage/src/ego_replan_fsm.cpp

.../traj_opt/include/optimizer/poly_traj_optimizer.h
.../traj_opt/src/poly_traj_optimizer.cpp
```

Local 消费 task/reference，独立完成 topology、A*/Local-SFC、MINCO、J_vis、hard preflight
和 trajectory commit。Team reference realization 仍走同一 Local 生产链，没有 direct seed
execution 或 coordinator-owned polynomial。

### 1.5 Team target-centered optimizer 与 coordinator

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  team_target_centered_optimizer.h
  team_objective_primitives.h
  topology_coordinator_core.h
ros_ws/src/multi_uav_formation/src/
  team_target_centered_optimizer.cpp
  topology_coordinator_core.cpp
  multi_uav_topology_coordinator.cpp
```

Team 的优化变量限定为 target-centered `R / PHI / reference-phase`。源码内部已把旧
`time_shift/time_trust` 语义重命名为 `reference_phase_shift/phase_trust`；该 phase 不控制
Local MINCO duration。

### 1.6 realized validation

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  realized_team_validator.h
ros_ws/src/multi_uav_formation/src/
  realized_team_validator.cpp
```

### 1.7 executor transaction

```text
.../plan_manage/src/planner_manager.cpp
.../plan_manage/src/ego_replan_fsm.cpp
.../plan_manage/src/traj_server.cpp

ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  team_solution_commit_contract.h
```

### 1.8 ROS messages

```text
.../traj_utils/msg/CooperativeTaskReference.msg
.../traj_utils/msg/TaskReferenceAdjustment.msg
.../traj_utils/msg/TeamReferenceSchedule.msg
.../traj_utils/msg/TeamTrajectoryAck.msg
.../traj_utils/msg/TeamTrajectorySolution.msg
.../traj_utils/msg/TopologyCandidate.msg
.../traj_utils/msg/TopologyCandidateBundle.msg
.../traj_utils/msg/PolyTraj.msg
```

`TeamReferenceSchedule.time_shifts` 和 `early_joint_primary` 仍保留为 ROS wire compatibility
字段，但源码注释与生产语义已明确：前者是 reference phase，后者表示 certified Local
realization；它们不代表旧 Cartesian optimizer 仍然存在。

### 1.9 launch / build / tests

```text
ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch
ros_ws/src/multi_uav_formation/CMakeLists.txt
ros_ws/src/multi_uav_formation/package.xml
.../plan_manage/launch/advanced_param.xml
.../plan_manage/launch/run_in_sim.launch
.../plan_manage/CMakeLists.txt
.../traj_opt/CMakeLists.txt

ros_ws/src/multi_uav_formation/test/
  team_target_centered_optimizer_contract_test.cpp
  adaptive_execution_contract_test.py
```

已删除：

```text
include/multi_uav_formation/team_visibility_optimizer.h
src/team_visibility_optimizer.cpp
test/team_visibility_optimizer_contract_test.cpp
test/joint_adoption_atomic_contract_test.cpp
test/encirclement_geometry_contract_test.cpp
test/multiview_contract_test.cpp
```

这些文件只服务旧 full-Cartesian Team optimizer，不再被 production 或 CMake 引用。

## 2. 最终 production 链：函数、topic 与 authority

### 2.1 Cooperative → Local

```text
CooperativeViewpointManager::timerCallback()
  → publish /cooperative_task_reference

可选反馈：
  /cooperative_task_adjustment
```

Cooperative 给出任务参考几何；它不调用 `EGOPlannerManager`，也不创建 polynomial。

### 2.2 Local candidate 与 realization

```text
EGOPlannerManager::teamReferenceScheduleCallback()
  → 校验 reference/snapshot/activation identity
  → Local topology + A*/Local-SFC + MINCO
  → current-revision hard preflight
  → publishTeamReferenceAck(LOCAL_REALIZATION)

正常 Local rolling：
  finalizeCapturedCandidates()
  → setLocalTrajFromOpt()
  → EGOReplanFSM::publishCurrentTrajectory()
```

Local candidate bundle topic：

```text
/topology_coordination/uav0/candidate_bundle
/topology_coordination/uav1/candidate_bundle
/topology_coordination/uav2/candidate_bundle
```

### 2.3 target-centered Team reference

```text
MultiUavTopologyCoordinator::attemptTargetCenteredTeamReference()
  → TeamTargetCenteredOptimizer::optimize()
  → publish /topology_coordination/team_reference_schedule
```

Team optimizer 只输出 reference schedule；三条实际 trajectory 必须由三个 Local Planner
分别 realization。

### 2.4 realized validation → prepare

```text
/topology_coordination/team_ack
  → coordinator 收齐 3 个 LOCAL_REALIZATION
  → MultiUavTopologyCoordinator::commitPendingTeamProposal()
  → RealizedTeamValidator::validate()
  → PASS 才 publish PREPARE 到 /topology_coordination/team_solution
```

### 2.5 executor ready → commit → activation

```text
traj_server::teamSolutionCallback(PREPARE)
  → 写入 prepared_team_trajectory_
  → publish EXECUTOR_READY 到 /topology_coordination/team_ack

coordinator 收齐 3 EXECUTOR_READY
  → commitPreparedTeamExecution()
  → publish COMMIT 到 /topology_coordination/team_solution

traj_server::teamSolutionCallback(COMMIT)
  → prepared slot 原子提升到 future queue

traj_server::cmdCallback()
  → common activation
  → publish /trajectory_execution/activated
```

任一 PREPARE/READY/commit-CAS/timeout/identity 失败都会广播 abort。executor 对同一
transaction 清除 prepared、scheduled 及 future queue 中的全部成员，不允许单机残留执行。

## 3. 最终 authority 表

| 对象 | 唯一 production authority | 非 authority 的角色 |
|---|---|---|
| target state / prediction identity | target producer + `ObjPredictor` snapshot | Team/Local 只能消费 |
| cooperative task geometry | `CooperativeViewpointManager` | Team adjustment 只作建议 |
| Local topology | `EGOPlannerManager` 的 NOMINAL/LEFT/RIGHT、A*/Local-SFC | coordinator 不生产 topology |
| executable polynomial | 三台 `EGOPlannerManager` / Local MINCO | Team reference 不可直接执行 |
| Team low-dimensional reference | `TeamTargetCenteredOptimizer` + coordinator | 只改 R/PHI/reference-phase |
| realized team acceptance | `RealizedTeamValidator` | 只评价，不修改 trajectory |
| execution transaction | `MultiUavTopologyCoordinator` | 负责 PREPARE/3READY/COMMIT/ABORT |
| physical activation | 三台 `traj_server` | 只执行已 commit 的 prepared identity |

## 4. A–I 完成状态

| 阶段 | 状态 | production 证据 |
|---|---|---|
| A — 集中式 ablation truth | COMPLETE | `run_on.sh --ablation full`、manifest、resolved params 与节点核对 |
| B — CooperativeTaskReference 单一任务几何权威 | COMPLETE | Cooperative 只发 reference；Local 消费并产轨迹 |
| C — Local J_vis 与 encirclement generator 解耦 | COMPLETE | J_vis 位于 Local optimizer，Team/Cooperative 不拥有其 trajectory gradient |
| D — J_vis 真实激活可观测 | COMPLETE | FULL run 的 active/cost/gradient 计数均非零 |
| E — common activation + world snapshot | COMPLETE | 三 Local realization 共用 activation 与 snapshot identity |
| F — realized validator | COMPLETE | FULL run 同时出现 PASS 与 REJECT |
| G — executor 两阶段原子事务 | COMPLETE | 12 次三机 activation，partial=0；abort 均 fail closed |
| H — snapshot / camera / yaw 语义统一 | COMPLETE | `PlanningWorldSnapshot` + shared camera contract + `advanceTargetFacingYaw()` |
| I — 旧 full-Cartesian Team optimizer 清除 | COMPLETE | 源/头文件、CMake 与专属 tests 已删除；无 production 引用 |

G 的“COMPLETE”表示 transaction 语义已经闭合，不表示每个 proposal 都成功。最终 run 仍有
一次 3READY 后 commit CAS stale；它正确取消三机、未产生 partial activation。

## 5. RealizedTeamValidator 的实际职责

### 5.1 输入

```text
team_reference_id
common activation_time / execution_horizon
task_reference_generation
target_prediction_revision
dynamic_prediction_identity
static_map_revision
visibility_model_version / camera contract
三条 Local realized polynomial + realization certificate
同 activation / horizon 的 committed Local baseline triple
```

### 5.2 检查顺序

1. identity 非零且属于同一 planning world；
2. 三条 Local realization certificate 完整；
3. 在真实 polynomial 上按共同时间轴采样；
4. 检查 realized pairwise swarm separation；
5. 用共享 camera/FOV、target prediction 与 target-facing yaw rollout 计算 Q2、累计可见性、
   redundancy、encirclement；
6. 对同 activation/horizon 的 committed Local baseline 做 visibility-first lexicographic 比较；
7. 只有 realized triple 非劣且有可证明收益才 PASS。

validator 不执行以下任何操作：

```text
不改 waypoint
不改 knot time / MINCO duration
不调用 A*
不调用 Local-SFC
不重新优化 polynomial
不直接 publish trajectory
```

因此 reference optimizer 的预测指标不能冒充 commit 指标；commit metadata 来自三条 Local
realization 的再次测量。

## 6. Executor transaction 与 fail-closed 语义

最终状态序列：

```text
REFERENCE_PROPOSED
→ 3 LOCAL_REALIZATION ACK
→ REALIZED_TEAM_VALIDATION PASS
→ PREPARE
→ 3 EXECUTOR_READY
→ COMMIT
→ common activation
```

失败序列：

```text
任一 identity / handoff / deadline / CAS / executor-ready 失败
→ coordinator ABORT
→ 三台 traj_server CANCEL_ALL_FUTURE(transaction_id)
→ Local rolling authority 不变
```

idempotent COMMIT 仍用于抗消息丢失，但重复日志改为 throttle；语义没有削弱。PREPARE 前
不改变 active/scheduled trajectory，COMMIT 前 prepared trajectory 不可执行，activation
必须等于 proposal 中的共同绝对时刻。

## 7. 构建、定向检查与最终 FULL

### 7.1 构建

最终 production 源码执行：

```text
cd ros_ws
catkin build -j2 --no-status
```

结果：`25/25 packages succeeded`。唯一 warning 是既有 Gazebo Classic / `gazebo_msgs`
弃用提示，不是本轮编译错误。

最终清理又只改了一个 Python wiring test 的旧文件入口和一条注释，没有改变 production
语义；对应定向 test 随后 `11/11 PASS`。

```text
xmllint native_egov2_rviz.launch = PASS
adaptive_execution_contract_test.py = 11/11 PASS
git diff --check -- . = PASS
OLD CARTESIAN SOURCE REFERENCES = 0
```

没有运行无关测试矩阵。

### 7.2 真实 FULL

```text
命令：./run_on.sh --ablation full --headless --timeout 200
目录：runs/20260921_131436_177465
BOOT：BOOT-12 reached
CORE_EXIT_CODE：0
CLEANUP_STATUS：OK
```

该 run 后的源码清理只包括参数显式化、内部命名、dead branch/旧实现删除和日志节流；
active algorithm 默认值与本次 executable semantics 不变。清理后已全量重建，但没有把另一次
仿真伪装成本 run。

### 7.3 Team reference / validation / transaction 计数

并发 ROS stdout 会发生行交织，因此优先报告可解析 unique identity，不把损坏行补造：

```text
TEAM_REFERENCE_PROPOSED:          84 条可解析；serial 实际到 85
TEAM_REALIZATION_3READY:          29
REALIZED_TEAM_VALIDATION_PASS:    29
REALIZED_TEAM_VALIDATION_REJECT:  12 unique reference IDs
TEAM_EXECUTION_PREPARE:           29 unique transaction IDs

3 EXECUTOR_READY 可解析 IDs:      14
EXECUTION_COMMIT:                 13 unique transaction IDs
FULL THREE-UAV ACTIVATION:        12 unique transaction IDs
PARTIAL ACTIVATION:               0

ABORT/CANCEL:                     17 unique transaction IDs
POST_3READY_COMMIT_REJECT:         1（ID 26，CAS stale，三机全取消）
EXECUTOR COMMIT REJECT:            0
```

完成 activation 的 IDs：

```text
6, 7, 16, 23, 24, 27, 29, 32, 33, 34, 39, 40
```

收到 cancel 的 IDs：

```text
26, 28, 43, 44, 45, 53, 54, 55, 56, 58, 60, 61, 63, 68, 77, 79, 82
```

两组交集为空。ID 43 已 commit 但在共同 activation 前被 revoke，三台均取消且 activation=0，
证明 abort 可以撤销 committed-but-not-yet-active 的整组 future transaction。

## 8. Local J_vis 真实计数

最终日志没有在该计数行携带 drone ID，因此按三个 planner process 的最终 band 报告，不
强行映射 UAV：

| process final band | active | nonzero cost | nonzero gradient |
|---|---:|---:|---:|
| A | 124,257,800 | 121,596,939 | 121,134,023 |
| B | 8,756,883 | 7,438,725 | 7,080,085 |
| C | 915,119 | 305,560 | 242,211 |
| 合计 | 133,929,802 | 129,341,224 | 128,456,319 |

```text
J_VIS_ACTIVE: YES
J_VIS_NONZERO_COST: YES
J_VIS_NONZERO_GRADIENT: YES
```

这证明 J_vis 真实进入 Local nonlinear optimization，而不是只有配置开关或日志标签。

## 9. reference metric 与 realized metric 的实例

transaction/reference ID 16：

Team target-centered reference optimizer 输出：

```text
REFERENCE_METRIC:
  j_k2_raw  = 0
  j_acc_raw = 0
  j_comp_raw = 0
  j_enc_raw = 1.198225
```

三台 Local realization 后，validator 对真实 polynomial 重新计算：

```text
REALIZED:
  q2          = 1.000000
  accumulated = 0.987996
  redundancy  = 0.000000
  enc          = 0.000000
  min_pair     = 2.450 m

COMMITTED LOCAL BASELINE（同 activation/horizon）:
  q2          = 1.000000
  accumulated = 0.925861
  redundancy  = 0.000000
  enc          = 0.000000
  min_pair     = 2.645 m

REALIZED COST BEFORE → AFTER:
  K2 cost      0 → 0
  acc cost     0.074139 → 0.012004
  geometry     0 → 0
```

该 transaction 的 commit 理由来自 realized accumulated visibility 改善，而不是拿 reference
optimizer 的 `j_enc_raw` 直接当作执行收益。这正是 reference 与 execution authority 分离的
运行证据。

## 10. FULL 可见性结果

目标有效窗口 `76.462861 s`，同步样本 `2294`：

```text
VISIBLE_COUNT_0/1/2/3: 9 / 804 / 761 / 720
NONE:                  0.003923
K2:                    0.645597
ALL3:                  0.313862
MEAN_VISIBLE:          1.955536
CAMERA_TIME:           149.525891 camera·s
LONGEST_K2_LOSS:       22.967301 s
LONGEST_BLACKOUT:      0.233038 s
```

这些是当前 FULL 的运行结果，不代表论文收益已经成立。可见性仍明显受 Local hold、
unvalidated execution 和 static contact 污染，不能用本次单 run 宣称优于 OFF/baseline。

## 11. 未闭合的 production 问题

### 11.1 Local rolling liveness

```text
TERMINAL_HOLD_COUNT:                         2
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: 2
STARVATION_LOG_COUNT:                        2
```

- drone0 trajectory 77：hold 后 `13.679678 s` 恢复；
- drone2 trajectory 76：到 run 结束仍未退出 hold。

这不是 Team validator 或 executor partial commit：它们发生在 Local rolling successor
coverage 上。

### 11.2 unvalidated execution

目标有效窗口：

```text
UNVALIDATED_EXECUTED_SAMPLES: 1933
  CSV uav_id=1 / drone0: 410
  CSV uav_id=2 / drone1:   0
  CSV uav_id=3 / drone2: 1523
```

全 recorder 窗口为 6608，二者口径不同，报告不混用。

### 11.3 安全样本

```text
STATIC_CONTACT_SAMPLES:       17（全部 drone2 / CSV uav_id=3）
MIN_STATIC_CLEARANCE:          0 m
DYNAMIC_CONTACT_SAMPLES:       0
MIN_MOVING_CLEARANCE:          0.020326 m
MIN_PAIRWISE_EUCLIDEAN:        1.001869 m
PAIRWISE_SAMPLES_BELOW_0P5_M:  0
```

因此最终结论必须是：Team reference/realization/validation/execution transaction 架构闭合，
但 Local production 仍有真实 liveness 和静态接触故障。后续若继续，应直接追这两个 Local
failure 的第一分叉，不能再在 Team 层增加 recovery/fallback。

## 12. 最终字段

```text
PRODUCTION_ARCHITECTURE:
  CooperativeTaskReference
  -> Local-only executable polynomial
  -> target-centered Team R/PHI/reference-phase suggestion
  -> three Local realizations
  -> RealizedTeamValidator
  -> PREPARE / 3 EXECUTOR_READY / COMMIT
  -> common activation

COOPERATIVE_TASK_AUTHORITY_UNIQUE: YES
LOCAL_EXECUTABLE_AUTHORITY_UNIQUE: YES
TEAM_DIRECT_POLYNOMIAL_AUTHORITY: NO
REALIZED_VALIDATOR_MODIFIES_TRAJECTORY: NO

PLANNING_WORLD_SNAPSHOT_REAL_IDENTITIES: YES
CAMERA_MODEL_SHARED: YES
TARGET_FACING_YAW_ROLLOUT_SHARED: YES

OLD_CARTESIAN_TEAM_OPTIMIZER_SOURCE_PRESENT: NO
OLD_CARTESIAN_TEAM_OPTIMIZER_PRODUCTION_REFERENCES: 0
LEGACY_WIRE_FIELD_NAMES_REMAIN: YES（schema compatibility only）

BUILD: 25/25 PASS
DIRECT_WIRING_TEST: 11/11 PASS
XML: PASS
DIFF_CHECK: PASS

FULL_RUN_COUNT: 1
FULL_RUN_BOOT_12: YES
FULL_RUN_CORE_EXIT_CODE: 0
FULL_RUN_CLEANUP_OK: YES

REFERENCE_SERIAL_MAX: 85
REALIZATION_3READY: 29
VALIDATOR_PASS: 29
VALIDATOR_REJECT_UNIQUE: 12
EXECUTION_COMMIT_UNIQUE: 13
FULL_ATOMIC_ACTIVATION_UNIQUE: 12
PARTIAL_ACTIVATION: 0
ABORT_CANCEL_UNIQUE: 17
POST_3READY_COMMIT_REJECT: 1

J_VIS_ACTIVE_COUNT: 133929802
J_VIS_NONZERO_COST_COUNT: 129341224
J_VIS_NONZERO_GRADIENT_COUNT: 128456319

WHOLE_SYSTEM_HEALTHY: NO
TERMINAL_HOLD_COUNT: 2
END_BEFORE_NEXT_COUNT: 2
STARVATION_LOG_COUNT: 2
UNVALIDATED_EXECUTED_SAMPLES_TARGET_WINDOW: 1933
STATIC_CONTACT_SAMPLES: 17
DYNAMIC_CONTACT_SAMPLES: 0
SWARM_BELOW_0P5_M_SAMPLES: 0

SAFETY_THRESHOLD_RELAXED: NO
HARD_PREFLIGHT_BYPASSED: NO
RAW_REFERENCE_EXECUTED: NO
NEW_RECOVERY_OR_FALLBACK_ADDED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```

最终判断：本次已经把 production 的责任边界收敛为“任务参考—Local 轨迹—Team 低维建议—
Local realization—realized evaluation—executor 原子事务”，并删除旧的 Cartesian 双轨实现。
架构目标完成；运行健康目标未完成。下一轮不应再改 Team 架构，而应单独处理最终 FULL 中
drone2 的 Local hold、unvalidated coverage 与 static contact 第一分叉。

