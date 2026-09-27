# Recovery 三项行为问题闭环与生产候选权限审查

日期：2026-09-12

项目：`/home/bob/ALP/egov2_fc65423_constvel`

承接：`feedback_43.md`–`feedback_47.md`

范围：Recovery production candidate、worker admission、Team Reference/local continuation；未运行 Scenario A、OFF、A/B 或仿真。

## 1. 最终结论

三个目标行为已经由真实 production 入口和加强后的合同闭合。此前阻塞 recovery fixture 的问题属于测试环境/测试语义缺陷，不是 production Planner 缺陷，因此没有修改 production 源码、参数或安全门。

```text
THREE_RECOVERY_BEHAVIOR_FIXES = PASS

PROBLEM_1_PRODUCTION_CANDIDATE_CHAIN = PASS
PROBLEM_2_WORKER_ADMISSION = PASS
PROBLEM_3_TEAM_REFERENCE_CONTINUATION = PASS

PRODUCTION_BUGS_FIXED = NONE
FIXTURE_BUGS_FIXED = YES
PRODUCTION_SOURCE_CHANGED = NO
```

本轮在 `feedback_47` 已通过的基础上又发现并修复一个会制造假阳性的 fixture 合同漏洞：`CandidateSetOutput.candidates` 非空并不意味着 MINCO 求解成功，因为 capture 输出可以保留失败 polynomial 作为诊断/后续 guide。Team Reference 测试原先只检查 `ok && !candidates.empty()`，现在要求至少一个候选同时满足：

- `candidate.success == true`；
- polynomial 非空；
- 首尾位移大于 0.05 m；
- 最大速度大于 0.05 m/s。

同时，在前序合同替换各 manager 的 local trajectory 后，测试会刷新三机 `swarm_traj` 到同一当前 generation，避免旧 peer 副本形成合成的 self/peer collision。加强后的 recovery fixture 独立复跑两次均退出 0；最终复跑共 133 条 `PASS`、0 条 `FAIL`。

## 2. 三个问题的根因与修复

### 2.1 问题一：NOMINAL/SIDE/MINCO candidate generation/preflight

原始首个因果断点不是 production 算法不能生成候选，而是 synthetic fixture 缺少 launch normally supplied context：

1. `grid_map/virtual_ceil_height` 未设置，继承 `-0.1 m`；`z=1.2 m` 的候选在读取 voxel buffer 前即被 virtual ceiling 判 occupied。`setFree()` 只能修改 voxel buffer，不能覆盖这个独立 hard gate。
2. fixture 只设置 `manager/max_vel`，没有设置 optimizer 自己的 `optimization/max_vel`；其 `-1` sentinel 使 `samplePointsToCheck()` 在 NOMINAL precheck 失败。
3. optimizer weights/clearances 仍是未初始化 sentinel，导致后续求解失败和异常时间尺度。
4. dynamic negative subcase 设置了 `prediction/obj_num=1`，却未打开 `prediction/use_time_aware_moving_obj_cost`，所以没有真正构造 predictor consumer。

这些缺项已在 production fixture 中补齐；没有放宽 unknown/static/dynamic/swarm/dynamics/local-SFC/PVA handoff 条件。当前真实路径是：

```text
active trajectory future P/V/A
-> buildFreshMovingInitializer
-> reboundReplan NOMINAL MINCO
-> deferred candidate capture
-> common-epoch reanchor
-> validateRetimedLocalSfc
-> checkActiveHandoff
-> validateExecutionTrajectory
-> validateRecoveryTeam
```

最终复跑三机每条 worker evidence 的 duration 为 1.938688 s、length 为 0.500000 m，且 static/dynamic/swarm/dynamics/local-SFC/handoff 全为 1。

```text
FIRST_FAILURE_STAGE_BEFORE_FIX = STAGE_06_NOMINAL_PRECHECK
FIRST_FAILURE_FUNCTION_BEFORE_FIX = GridMap::getInflateOccupancy
FIRST_FAILURE_CONDITION_BEFORE_FIX = z=1.2 >= virtual_ceil_height=-0.1
FIRST_FAILURE_STAGE_AFTER_FIX = NONE
PROBLEM_1_CLASSIFICATION = FIXTURE
```

NOMINAL 已在 primary case 成功，所以 SIDE 和 raw A* repair 没有被该正例自然触发；没有以“未触发”冒充“已运行”。静态 barrier negative case仍验证完整路径会被 production static gate 拒绝。

### 2.2 问题二：worker admission/candidate bundle

`TeamTargetReachability` 内部按 production namespace `/drone_X_ego_planner_node` 构造三份 manager。原 fixture 只给 `/probe_test/uavX` 配置参数，导致直接 probe manager 与 worker-owned manager 的运行上下文不同。

fixture 现在在构造 worker 前，把三份完整配置复制到 worker 实际消费的 namespace。worker 的真实 admission 保持如下权限链：

- 在同一 future epoch 采样三机 source trajectory；
- 每机调用 `probeRecoveryTarget()`；
- 删除既非 success、也非已验证 initializer fallback 的 candidate；
- 枚举三机候选组合并调用 `finalCheck()`；
- 只有完整 team preflight 通过才发布 active target、三份 guide/evidence 与新的 reachability generation。

运行结果出现 `TARGET_REACHABLE`、`GUIDE_CREATED`、`TARGET_ACTIVATED`，并通过 `real_worker_initial_admission_native_team_evidence`。这里的 bundle 指 worker 发布的三机 active recovery guide/evidence bundle，不宣称本次触发了 joint topology selection。

```text
PROBLEM_2_FIRST_ADMISSION_REJECT_BEFORE_FIX = WORKER_MANAGER_CONFIG_INGRESS_MISMATCH
PROBLEM_2_FIRST_ADMISSION_REJECT_AFTER_FIX = NONE
PROBLEM_2_CLASSIFICATION = FIXTURE
```

### 2.3 问题三：连续 Team Reference 与 reference 缺失时 local continuation

production 语义本身是 optional enhancement：

- FSM 先通过 `getLocalTarget()` 建立普通 local target；
- `teamReferenceGoal()` 成功时才覆盖为 team rolling reference；
- team-follow seed/solve 失败不会阻塞原 local fallback chain；
- `buildRecoveryGuideSeed()` 无 guide/reference 时返回 false，随后进入 `buildFreshMovingInitializer()`；
- reference 过期不会延长 epoch，也不会触发 brake/stop。

原 fixture 有两个测试问题：

1. missing-reference case 把 goal 设为 current position，正确进入 “Close to goal” 终止分支，却被错误解释成 continuation failure；现改为 0.4 m moving goal。
2. continuous Team Reference case只要求候选数组非空，并保留了前序测试的旧 peer trajectory snapshot；现已刷新 peer generation，并严格要求成功的非零移动 MINCO candidate。

加强后的合同对三架 UAV 各连续运行三轮，九次 `continuous_team_follow_production_local_candidate_supply` 全通过；三次 `missing_team_reference_production_local_continues` 全通过。

```text
TEAM_REFERENCE_IS_HARD_DEPENDENCY = NO
MISSING_REFERENCE_CAUSES_STOP_OR_BRAKE = NO
REFERENCE_EPOCH_RENEWED_BY_LOCAL_REPLAN = NO
PROBLEM_3_CLASSIFICATION = FIXTURE_CONTRACT
```

## 3. 问题一的生产权限专项审查

### 3.1 raw A* path 是否可绕过 MINCO/最终检查直接执行

结论：未发现。

SIDE local A* success只产生几何 guide。其 raw/simplified path 用于生成 local SFC 和重建 `side_init_mjo`；最终 candidate 仍来自 MINCO/SCP，或来自单独经过完整验证的 feasible initializer polynomial。全项目 production 搜索只找到一个 `traj_.setLocalTraj(...)` 调用点，位于 `EGOPlannerManager::setLocalTrajFromOpt()`；raw A* path 没有 direct assignment 到 active trajectory。

### 3.2 早期 safety label 是否拥有执行权限

结论：未发现。

`finalizeCapturedCandidates()` 明确忽略早期 `ABSOLUTE_SAFE/IMPROVED_ONLY` 作为执行 authority，先要求 `candidate.success`，再对当前 revision 重做 rehead、dynamics/static repair、local-SFC、handoff 与完整 execution validation。`finalizeTopologyCandidate()` 对最终 revision 再计算 dynamics/static/swarm/dynamic risk，并仅在全部通过时重写为 `ABSOLUTE_SAFE`。`IMPROVED_ONLY` 不能通过最终 adoption。

### 3.3 stale revision metadata 是否继续授权执行

结论：未发现当前可达路径。

本地 commit 只能经 `setLocalTrajFromOpt()`；该函数在写入 `traj_.local_traj` 前重新检查 activation lead、P/V/A handoff、predecessor prefix 和完整 execution trajectory，成功后才增加 `active_traj_generation_`。候选 source/hypothesis/reference metadata 在 commit 后绑定到该新 generation。joint team commit同样通过该入口和 team-aware full validation。

### 3.4 Team Reference 是否成为硬依赖

结论：不是。

`ego_replan_fsm.cpp` 中 team-follow 失败后显式继续原 local fallback chain；测试同时覆盖 reference present、expired、missing 和 invalid。缺失/失效只移除其 attraction/reference，不取消当前执行，也不触发 Recovery brake/stop。

```text
RAW_ASTAR_EXECUTION_BYPASS_FOUND = NO
EARLY_SAFETY_LABEL_EXECUTION_AUTHORITY_FOUND = NO
STALE_REVISION_METADATA_FOUND = NO
TEAM_REFERENCE_HARD_DEPENDENCY_FOUND = NO
SIMPLE_CONTRACT_LOGIC_ERRORS_AUTOFIXED = YES
```

## 4. 本轮实际修改

本轮新增修改只有：

- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/test/team_reference_contract.h`
  - 刷新三机当前 local/peer trajectory generation；
  - 新增 `has_moving_minco_candidate()`；
  - continuous Team Reference 与 missing-reference assertion 必须看到真实成功的移动 MINCO candidate。

承接 `feedback_47`、仍属于本次三问题闭环的既有 fixture 修复：

- `plan_manage/test/recovery_probe_production_test.cpp`
  - 补齐 manager/optimizer/map/dynamic-predictor production launch context；
  - 对齐 worker namespace。
- `plan_manage/test/continuous_motion_contract.h`
  - 用“保持已验证连续运动”替代过时的 exact old trajectory ID 要求；
  - 隔离后续 prefix 子用例状态。

没有修改 Planner、optimizer、worker、FSM、Safety、RecoveryIntent、Transition Pool 或任何参数文件。

## 5. 构建与运行结果

### 5.1 构建

执行：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

结果：6 个实际参与构建的 package（含依赖）全部成功，0 failure，0 warning，约 40 s。

### 5.2 recovery production fixture

在独立 ROS master 上、重新构建后运行两次：

```text
ros_ws/devel/lib/ego_planner/recovery_probe_production_test
```

最终复跑：exit 0，133 PASS，0 FAIL。关键输出：

```text
PASS actual_NOMINAL_SIDE_MINCO_capture_generates_moving_paths
PROBE_REASON=VALIDATED_PRODUCTION_NOMINAL_SIDE_MINCO_PREFLIGHT
PASS endpoint_valid_AND_production_paths_preflight_safe
PASS real_worker_initial_admission_native_team_evidence
PASS continuous_team_follow_production_local_candidate_supply  (9/9)
PASS missing_team_reference_production_local_continues         (3/3)
```

负例同时通过：future P/V/A mismatch、dynamic predictor update、changed-map、crossing/swarm conflict、static barrier 均未被新 fixture 条件绕过。

### 5.3 unit/contract/gradient

CTest：

- `ego_planner`：3/3 passed；
- `multi_uav_formation`：9/9 passed。

额外 executable contracts：

- `time_only_feasibility_contract_test`：PASS；
- `time_only_swarm_temporal_contract_test`：PASS；
- `elastic_visibility_contract_test`：PASS，`directional_gradient_contracts=PASS`。

合计相关独立测试 15/15 passed，外加 recovery production fixture passed。数值梯度证据：

```text
PRODUCTION_LOCAL_GUIDE_POSITION_TIME_FD_ERROR = 1.3789e-11
TEAM_REFERENCE_PRODUCTION_FD_MAX = 1.28832e-10
TRACKING_GRAD_COST_P_TEAM_FD = 4.03511e-10, 9.75798e-11
```

## 6. 要求字段

```text
FRESH_INITIALIZER_PASS: YES
NOMINAL_MINCO_BUILT: YES
SIDE_DISPATCH_WORKS: NOT EXERCISED (NOMINAL primary succeeded; SIDE production path unchanged)
ASTAR_PATH_AVAILABLE_IF_REQUIRED: NOT EXERCISED (NOMINAL primary succeeded; raw-A* bypass audit=NO)
LOCAL_SFC_BUILT: NOT_REQUIRED_IN_PRIMARY; LOCAL_SFC_GATE_PASS=YES
FINAL_PREFLIGHT_PASS: YES
WORKER_ADMISSION_PASS: YES
CANDIDATE_BUNDLE_FORMED: YES
CONTINUOUS_TEAM_REFERENCE_FOLLOW_PASS: YES
MISSING_REFERENCE_LOCAL_CONTINUATION_PASS: YES

RAW_ASTAR_EXECUTION_BYPASS_FOUND: NO
EARLY_SAFETY_LABEL_EXECUTION_AUTHORITY_FOUND: NO
STALE_REVISION_METADATA_FOUND: NO
TEAM_REFERENCE_HARD_DEPENDENCY_FOUND: NO
SIMPLE_CONTRACT_LOGIC_ERRORS_AUTOFIXED: YES

BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES

PRODUCTION_SOURCE_CHANGED: NO
PRODUCTION_CONFIG_CHANGED: NO
SAFETY_OR_DYNAMICS_LIMIT_CHANGED: NO
NEW_FSM_ADDED: NO
SCENARIO_A_RUN: NO
OFF_OR_AB_RUN: NO
SIMULATION_RUN: NO
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
COMMITS: none
```

本轮创建的独立 ROS master 均已退出。系统中另有一个本轮开始前已经存在、端口为 12647 的 roscore；它不是本轮启动的进程，按隔离原则未终止。
