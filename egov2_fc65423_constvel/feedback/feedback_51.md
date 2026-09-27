# Feedback 051 — All3 本地排序重应用、三次 FULL ON 与 rolling execution 失效审计

日期：2026-09-12  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
承接：`feedback_49.md`、`feedback_50.md`

## 0. 结论

feedback_50 已确认的 All3 本地候选排序已重新应用，hard safety/executable set 和 blackout/K2 保护均未改变。确定性合同证明：K2 完全相同时，mean-visible/All3 先于 weakest-camera/max-loss，方向多样性与角度只作更后级 tie-break。三次真实运行中 `LOCAL_SELECTED_LOWER_MEAN_VISIBLE_COUNT` 均为 0，旧的“为了 diversity 选择更低 mean-visible 候选”没有再出现。

但是三次相同源码、参数和场景的 FULL ON 都发生 rolling execution 系统失效，因此没有得到可用于 All3 物理验收的 nonfailed run，也不能把任何一次的 All3 数值解释为排序修复的健康效果。三次均保留，没有挑选最好结果，也没有启动第四次仿真。

```text
ALL3_ORDERING_FIX_REAPPLIED: YES

RUN_COUNT: 3
FIRST_NONFAILED_RUN_FOUND: NO
FIRST_NONFAILED_RUN_INDEX: NONE
```

## 1. 排序根因与修改

feedback_50 的代码级根因是：local executable candidates 已经通过相同 current-revision final preflight，且 blackout/K2 相同后，旧 comparator 先比较 diversity/spread/min-pairwise-angle，mean-visible/All3 排在后面。因此 production 曾记录 92 次“存在更高 mean-visible 的可执行候选，却选择了更低 mean-visible 候选”。这不是 candidate supply 或 safety gate 问题，而是同一安全集合内的次级排序错误。

当前顺序为：

```text
less blackout / NONE
→ higher K2 / atleast2
→ optional higher atleast-k
→ higher mean-visible
→ higher All3
→ higher weakest-camera visibility
→ lower max-loss duration
→ higher diversity / spread
→ higher min-pairwise-angle
→ higher team utility
```

All3 仍是 soft preference，不是 hard gate；任何候选仍须先通过原有安全、可执行性与 current-revision preflight。

production/test 修改：

- `plan_manage/include/plan_manage/local_visibility_preference.h`：新增纯 comparator 合同。
- `plan_manage/src/planner_manager.cpp`：在既有 executable candidates 内调用该合同；没有扩展或缩小安全集合。
- `plan_manage/test/local_execution_contract_test.cpp`：覆盖 mean-visible/All3、K2、blackout、weakest-camera/max-loss 和 diversity 的优先级。

测试/分析支撑修改：

- `scenario_a_20260911_234722/run_full.py`：输出目录和 ROS port 可配置，并固定使用本项目脚本目录，便于三次隔离运行；不参与 production 规划。
- `early_visibility_topology_validation_20260912/analyze_run.py`：修正 terminal-tail starvation 的离线计数，兼容压缩 rosout；不改变仿真结果。

明确未改：J_vis/deep-risk、Q_dir、25°/170°、shared-risk-support、SIDE/A*、Stage2/3A/3B、joint optimizer/coordinator、static/dynamic/swarm safety threshold、planner brake/stop 或 trajectory executor production 语义。

## 2. Build 与测试

构建：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
6 requested/dependency packages: PASS
failed packages: 0
```

通过的相关测试包括：

```text
local_execution_contract_test
topology_coordinator_contract_test                 16/16
team_visibility_optimizer_contract_test            analytic/FD P,tau,piece,K2,blackout,K2-cont,J_acc,joint-yaw
multiview_contract_test
encirclement_geometry_contract_test
cooperative_viewpoint_contract_test
elastic_visibility_contract_test                   deep-risk non-decaying contract
visibility_topology_production_test                 8/8
```

受控 topology 测试继续证明：shared risk 在 LOS loss 前触发，双 SIDE 进入 production chain，target-side/mirror 选择相反，simple SIDE 静态失败时 A*/Local-SFC/MINCO rescue 可形成 loss 前 activation。`git diff --check` 与相关 Python compile 均 PASS。

## 3. 三次 Scenario A FULL ON

三次均使用：

```text
scene = long_cylinder_forest_visibility_stress.json
scene SHA256 = 430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
mode = FULL ON, native RViz
production source / parameters / scene = identical
parameter tuning between runs = NO
route duration ≈ 80.16–80.18 s
```

Run1/Run2 的仿真本身完整，但 recorder 继承了 `/home/bob/catkin_ws` 的旧 Python message binding，无法导入本项目 `PolyTraj`，所以 activated-polynomial/command side recorder 不完整；rosout、trajectory、visibility 与 lifecycle 证据完整，运行没有被丢弃。Run3 显式 source 本项目 `ros_ws/devel`，recorder 完整。

| Run | SYSTEM_FAILURE | hold | starvation | unvalidated samples | camera-time | mean visible | K2 | All3 | Q_dir | encircle | collision |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | YES | 2 | 2 | 2678 | 144.990828 | 1.808256 | 0.506340 | 0.319312 | 0.803547 | 0.182891 | 0 |
| 2 | YES | 2 | 2 | 2344 | 168.712885 | 2.104468 | 0.599972 | 0.507399 | 0.808900 | 0.308669 | 0 |
| 3 | YES | 1 | 1 | 2194 | 162.697196 | 2.029739 | 0.944005 | 0.091112 | 0.845275 | 0.405361 | 0 |

逐 run 要求字段：

```text
Run1:
  SYSTEM_FAILURE: YES
  TERMINAL_HOLD_COUNT: 2
  UNVALIDATED_EXECUTED_SAMPLES: 2678
  K2: 0.506340
  ALL3: 0.319312
  camera-time: 144.990828 camera-s
  Q_dir: 0.803547
  encirclement: 0.182891
  collision: 0

Run2:
  SYSTEM_FAILURE: YES
  TERMINAL_HOLD_COUNT: 2
  UNVALIDATED_EXECUTED_SAMPLES: 2344
  K2: 0.599972
  ALL3: 0.507399
  camera-time: 168.712885 camera-s
  Q_dir: 0.808900
  encirclement: 0.308669
  collision: 0

Run3:
  SYSTEM_FAILURE: YES
  TERMINAL_HOLD_COUNT: 1
  UNVALIDATED_EXECUTED_SAMPLES: 2194
  K2: 0.944005
  ALL3: 0.091112
  camera-time: 162.697196 camera-s
  Q_dir: 0.845275
  encirclement: 0.405361
  collision: 0
```

三次 `SWARM_CLEARANCE_VIOLATION_SAMPLES=0`，collision 也均为 0；但数千个 unvalidated execution samples 和 terminal hold 是明确连续性/执行合同失效，不能因为没有碰撞就判为健康。

## 4. 可见性与 All3 事件审计

完整可见性指标：

| Run | NONE | longest K2 loss s | blackout s | static LOS loss s | dynamic LOS loss s | HFOV loss s | VFOV loss s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.017396 | 39.582984 | 0.664823 | 40.591679 | 20.415244 | 86.433357 | 4.828720 |
| 2 | 0.002903 | 31.535893 | 0.232758 | 21.548937 | 22.985683 | 1.001068 | 1.167850 |
| 3 | 0.005378 | 1.063536 | 0.431054 | 73.519655 | 10.058895 | 73.044646 | 2.368063 |

`All3:1→0 且 K2=1`：

| Run | events | cumulative s | cause decomposition s | trigger before | safe SIDE before | on hold/unvalidated | lower-mean selection |
|---:|---:|---:|---|---:|---:|---:|---:|
| 1 | 7 | 14.996412 | static 5.098907; VFOV 2.427304; HFOV 5.903548; range 1.566653 | 7/7 | 5/7 | 2 | 0 |
| 2 | 16 | 6.421436 | static 1.818667; VFOV 1.135448; HFOV 0.598275; range 1.301736; dynamic 1.567310 | 15/16 | 15/16 | 1 | 0 |
| 3 | 1 | 7.399898 | static 7.399898 | 1/1 | 0/1 | 1 | 0 |

逐事件原始分类保存在：

```text
all3_ordering_validation_20260912/analysis/run1_all3_events.json
all3_ordering_validation_20260912/analysis/run2_all3_events.json
all3_ordering_validation_20260912/analysis/run3_all3_events.json
```

这批结果说明 comparator 合同确实生效：三次没有发现“更高 mean-visible/All3 的 executable candidate 被后级 diversity 压过”。但失视并未因此成为可评价的健康样本：部分事件已经处于 terminal hold，更多事件虽有 trigger/safe SIDE，后续仍受 candidate supply 与 rolling execution 断链影响。

## 5. 与 feedback_49 好运行及旧 OFF 的关系

feedback_49 的 Run1/2/5 好运行均值：

```text
camera-time = 227.483786 camera-s
mean-visible = 2.837703
K2 = 0.985196
All3 = 0.852768
Q_dir = 0.972633
encirclement = 0.729853
```

旧 OFF（feedback_24）：

```text
camera-time = 227.317129 camera-s
mean-visible = 2.905279
K2 = 0.982248
All3 = 0.923316
longest K2 loss = 0.877114 s
blackout = 0.022395 s
```

三次新运行的 All3、camera-time、Q_dir、encirclement 都明显低于健康参考，Run1/2 的 K2 也严重降低；这与 terminal hold/unvalidated execution 同时发生。由于用户规定只用 nonfailed run 判断排序优化，本轮结论必须是 `NOT_EVALUABLE_NO_NONFAILED_RUN`，不能从三个失效 run 宣称 All3 提升或回归。

```text
ALL3_INCREASED: NOT_EVALUABLE_NO_NONFAILED_RUN
K2_MAINTAINED: NOT_EVALUABLE_NO_NONFAILED_RUN
Q_DIR_MAINTAINED: NOT_EVALUABLE_NO_NONFAILED_RUN
ENCIRCLEMENT_MAINTAINED: NOT_EVALUABLE_NO_NONFAILED_RUN
CAMERA_TIME_IMPROVED: NOT_EVALUABLE_NO_NONFAILED_RUN
LOCAL_SELECTED_LOWER_MEAN_VISIBLE_COUNT: Run1=0, Run2=0, Run3=0
```

## 6. 系统失效的代码级第一断点

共同结构不是 coordinator/ACK/commit，也不是 All3 comparator：Run3 在失效前仍完成 4 条 optimizer→proposal→3 ACK→commit→3 adoption→optimized-yaw execution 链。共同问题是 finite validated moving suffix 被逐步耗尽，而在可桥接下一 activation 的时间窗口内没有 adoptable successor。

正常链与断链位置：

```text
active finite trajectory
→ candidate generation / NOMINAL and SIDE
→ A*/MINCO/SCP/final preflight
→ current trajectory仍在“当前时刻安全”，反复 retain_previous
→ remaining suffix < planning budget + 0.10 s activation margin
→ late candidate (Run1/3) 或仍无 candidate (Run2)
→ next activation cannot be covered by predecessor finite suffix
→ executor reaches endpoint
→ terminal hold / unvalidated execution
```

### 6.1 Run1

UAV0 trajectory 126：

```text
start = 1789217803.461788
duration = 1.553296 s
end = 1789217805.015084
runtime starvation remaining = 0.298112 s, later 0.092430 s
```

reserve window 内多次 NOMINAL `plan_success=0`，日志包括 terminal point in obstacle；SIDE 也 invalid。约 `1789217804.949` 才出现通过 normal safety 的 NOMINAL（dynamic clearance 9.312863 m），但计划 activation 为 `1789217805.088428`，已比 old end 晚 0.073344 s。`setLocalTrajFromOpt()` 最终由 `validateActivePrefixUntil()` 以 `PREDECESSOR_PREFIX_MOVING_SUFFIX_EXHAUSTED` 拒绝。随后 trajectory 126 到端进入 hold；UAV1 后续再发生第二次 hold。

### 6.2 Run2

```text
UAV0 trajectory150 end = 1789218032.311983
UAV1 trajectory146 end = 1789218032.354522
first starvation remaining ≈ 0.299314 / 0.299548 s
later remaining ≈ 0.089275 / 0.087239 s
```

这一轮在后缀耗尽前始终没有形成完全 executable successor：UAV1 fresh initializer 有效，但 LBFGS line search 返回 `-1005`，NOMINAL `plan_success=0`；同时 dynamic minimum clearance 0.344822 m 小于原 1.1 m gate，双 SIDE invalid。UAV0 后续同样 NOMINAL `plan_success=0`；即使某次 risk minimum 1.224832 m，`candidate.success=false` 仍由 `classify_candidate()` 判 INVALID，双 SIDE也未闭合。这里没有观察到晚到候选的 `PREDECESSOR_PREFIX...`，第一断点是 reserve window 内 candidate supply 没有产出可执行组合，随后两条 active suffix 同时到期。

### 6.3 Run3

UAV0（ROS namespace `uav1`）trajectory 22：

```text
start = 1789218227.031349
duration = 1.545824 s
end = 1789218228.577173
1789218228.519771 remaining = 0.057402 s, previous retained safe
1789218228.531922 runtime starvation safe coverage = 0.045371 s
```

此前同样连续出现 NOMINAL solve failure，SIDE 多为 `NO_STATIC_FEASIBLE_SIDE`。`1789218228.531955` 后终于得到 candidate 225：fresh initializer valid、MINCO `plan_success=1`、NOMINAL `ABSOLUTE_SAFE`、dynamic clearance 27.257913 m、swarm minimum 0.947958 m，final current-revision safety 均有效。但新 activation 为 `1789218228.676390`：planning budget 0.044438 s + activation margin 0.1 s，使 activation 比 old end 晚约 0.099217 s。

`checkActiveHandoff()` 对已越过末端的 old trajectory 使用 clamp sample，仍打印 accepted；紧接着 `setLocalTrajFromOpt()` 的 `validateActivePrefixUntil(activation)` 正确拒绝 `PREDECESSOR_PREFIX_MOVING_SUFFIX_EXHAUSTED`。此前“现在仍安全”的 previous 被保留，却无法覆盖下一 activation；executor 随后进入约 72 s terminal hold。

## 7. 为什么 planner 看似正常、starvation 原指标却为 0

Planner 的 `retain_previous()` 只证明 previous 在当前时刻仍安全，不证明它能覆盖下一候选的 activation。`ensureExecutionCoverage()` 能在 coverage 接近 budget+margin 时记录 starvation，但当候选求解持续失败时它不能凭空制造 successor；而 `checkActiveHandoff()` 的 clamped after-end sample 又会产生一次误导性的 accepted telemetry，直到更严格的 `validateActivePrefixUntil()` 才拒绝。

旧离线 starvation 统计只遍历相邻 activation `(old,new)` 并计算正 gap。terminal tail 没有 `new` activation，是 right-censored，因此最严重的“最后一条 trajectory 后再无 successor”反而得到 0。production rosout 已经记录 `MOVING_SUCCESSOR_STARVATION`，本轮分析器改为把 `(drone, trajectory_id)` runtime starvation identity 与 activation-gap identity 做去重并集；修正后 Run1/2/3 分别为 2/2/1，与 hold identity 一致。这个修改只修 telemetry/accounting，没有改变三次运行或 All3 数值。

```text
SYSTEM_FAILURE_CODE_ROOT_CAUSE: finite validated moving suffix is retained as safe-now while no adoptable successor covers the scheduled activation; Run1/3 late safe candidates are rejected after activation exceeds predecessor end, Run2 candidate supply never closes before expiry
FIRST_FAILURE_FUNCTION: Run1/3 EGOPlannerManager::setLocalTrajFromOpt -> validateActivePrefixUntil; Run2 candidate generation/classify_candidate before selection
FIRST_FAILURE_CONDITION: PREDECESSOR_PREFIX_MOVING_SUFFIX_EXHAUSTED (Run1/3); candidate.success=false / LBFGS -1005 plus unsafe-or-invalid alternatives throughout reserve window (Run2)
FIRST_FAILURE_RUNTIME_VALUES: Run1 activation-old_end=+0.073344s; Run3 old_remaining=-0.099217s at handoff, budget=0.044438s, activation_margin=0.100000s; Run2 remaining fell from about 0.299s to 0.087-0.089s with no executable successor
STARVATION_METRIC_MISSED_FAILURE_REASON: old analyzer required a later activation and omitted right-censored terminal tails; runtime starvation identities now supply the missing terminal episodes
```

## 8. Lifecycle 修复建议（本轮未实施）

本轮遵守“失效根因审计只诊断”，没有把 lifecycle 修复混入已完成的三次 All3 实验版本。建议下一轮隔离处理：

1. 把 successor-due 变成明确 invariant：remaining validated moving coverage 必须大于估计 solve time + activation margin + execution margin；不足时更早开始既有安全候选链，而不是等到后缀最后 0.1 s。
2. `checkActiveHandoff()` 在 `activation > predecessor_end` 时必须 fail closed，不得用 clamp 到 terminal sample 的 P/V/A 打印 accepted。
3. 在原 final current-revision preflight 不变的前提下，允许保留最近一条已完全验证、identity 一致且能在 suffix reserve 前 activation 的候选；不能用 brake/stop 或降低 safety threshold 替代。
4. 增加显式 `NO_EXECUTABLE_SUCCESSOR_BEFORE_SUFFIX_EXPIRY` identity，把 candidate supply、final preflight、selection、activation 和 predecessor coverage 分开记录。
5. terminal-tail starvation 继续按 right-censored episode 计数。

```text
LIFECYCLE_FIX_RECOMMENDATION: enforce activation-within-finite-suffix and start the existing safety-validated successor pipeline before coverage reserve is consumed; fail checkActiveHandoff before clamped after-end sampling; retain only fully preflighted current-revision candidates, without brake/stop or safety-threshold changes
```

## 9. 执行与清理声明

三次原始结果：

```text
all3_ordering_validation_20260912/run1
all3_ordering_validation_20260912/run2
all3_ordering_validation_20260912/run3
```

分析结果：

```text
all3_ordering_validation_20260912/analysis/run1_metrics.json
all3_ordering_validation_20260912/analysis/run2_metrics.json
all3_ordering_validation_20260912/analysis/run3_metrics.json
all3_ordering_validation_20260912/analysis/run1_all3_events.json
all3_ordering_validation_20260912/analysis/run2_all3_events.json
all3_ordering_validation_20260912/analysis/run3_all3_events.json
```

大体积 launcher/rosout 已 gzip 原地保留以释放空间，没有删除三次有效结果。最终检查无本项目 ROS master、RViz、Gazebo、planner、recorder 或 target publisher 残留。

```text
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES
```
