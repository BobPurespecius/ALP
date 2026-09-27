# Feedback 090 — Team R/PHI/T visibility-first hierarchy 与一次 FULL 验证

日期：2026-09-20  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_089.md` / `runs/20260920_160211_684140`  
正式场景：`long_cylinder_forest.json`

## 0. 结论

本轮没有改变 Feedback089 已完成的 Team reference realization 架构，只把同一个
`TeamTargetCenteredOptimizer` 从普通 weighted sum 改成了真正的多阶段词典序优化：

```text
Stage 1A: maximize normalized Q2
Stage 1B: preserve Q2 floor, maximize normalized accumulated visibility
Stage 2:  preserve Q2/V floors, maximize normalized complementarity
Stage 3:  preserve Q2/V/C floors, minimize soft encirclement + regularization
```

平滑 barrier 只帮助当前求解器回到 admissible region；每阶段结束后的 hard quality
filter、tuple 间的 lexicographic comparator，以及发布前相对本 tuple Local source baseline
的 non-worsening guard 才是 authority。`J_ENC` 的数值规模已不能越过这些 floor 换取
Q2、accumulated visibility 或已经取得的 complementarity。

算法逻辑验收全部通过。受控 visibility、complementarity、Stage floor、最终 guard 和
temporal deconfliction contracts 均通过；受控 simultaneous LOS loss 仍为 `0.4 s -> 0`，
T shift 为 `+0.304998 / -0.306714 / 0 s`。真实运行中，78 个可从并发 stdout 完整配对的
selected transaction 没有一个违反 final primary/floor contract，production 中也没有
`TEAM_QUALITY_REJECT`。

但一次真实 FULL 的整机结果不能判健康。相对 Feedback089，actual complementarity 与
pair angle 改善，但 K2、mean-visible、All3 均轻微下降；仍有 1 次 Local starvation，且
出现 16 个 dynamic surface-contact samples。另有 3 次 3ACK 后 final-preflight revoke，
其中两个 transaction 已在另外两机形成部分激活。本轮按用户边界没有修改 Local、消息或
lifecycle/commit 架构，因此只如实记录，不用 hierarchy success 掩盖这些运行失败。

```text
TEAM_HIERARCHICAL_OBJECTIVE_IMPLEMENTED: YES
TEAM_ARCHITECTURE_CHANGED: NO
TEAM_DECISION_VARIABLES: R / PHI / T
TEAM_EXECUTION_AUTHORITY: LOCAL_ONLY
ENCIRCLEMENT: ON
JOINT_YAW: OFF

ALGORITHMIC_ACCEPTANCE: PASS
PRODUCTION_PERFORMANCE_ACCEPTANCE: PARTIAL / SAFETY-LIFECYCLE FAIL
```

## 1. Production 修改

修改文件仅位于既有 Team optimizer/tuple ranking：

```text
ros_ws/src/multi_uav_formation/include/multi_uav_formation/
  team_objective_primitives.h
  team_target_centered_optimizer.h
  topology_coordinator_core.h

ros_ws/src/multi_uav_formation/src/
  team_target_centered_optimizer.cpp
  topology_coordinator_core.cpp
  multi_uav_topology_coordinator.cpp

ros_ws/src/multi_uav_formation/test/
  team_target_centered_optimizer_contract_test.cpp
```

没有修改消息、Local Planner、N/L/R、Local MINCO/J_vis、hard preflight、executor、
TeamReferenceSchedule realization、yaw、recovery、FSM 或任何 safety threshold。

### 1.1 统一归一化质量向量

新增共享：

```cpp
TeamQualityVector {
  q2_ratio;                         // [0,1], larger is better
  accumulated_visibility_ratio;     // [0,1], larger is better
  complementarity;                 // [0,1], larger is better
  encirclement_cost;                // smaller is better
  regularization_cost;              // smaller is better
};
```

三项归一化均使用同一个 `H_pred=2 s`、同一组 trapezoidal world-time samples 和同一
target/static/dynamic visibility evaluator：

```text
Qbar2 = integral(Q2) / H
Vbar  = integral(sum_i v_i) / (3H)
Cbar  = integral(sum_i<j v_i v_j q(theta_ij)) / (3H)
```

```text
Q2_NORMALIZED: YES
ACC_NORMALIZED: YES
COMP_NORMALIZED: YES
```

### 1.2 数值容差不是任务级损失额度

本轮没有新增 ROS 参数。quality epsilon 由当前离散分辨率确定：

```text
epsilon = clamp(0.01 * sample_dt / H, 1e-5, 1e-3)
sample_dt = 0.1 s, H = 2 s
eps_Q = eps_V = eps_C = 0.0005
```

它等于一个离散 sample 归一化质量的百分之一，只覆盖采样/收敛噪声；不是允许
K2 下降 1% 或 2% 的任务级 margin。

### 1.3 四阶段与 fallback

- Stage 1A 仅优化 Q2，加 `1e-8` numerical regularization；失败回到 Local baseline。
- Stage 1B 在 `Q >= max(Q_base, Q_stage1A-eps_Q)` 下优化 V；失败回到 Stage 1A。
- Stage 2 在 `Q/V` floors 下优化 C；失败或 floor 不满足时回到 Stage 1。
- Stage 3 在 `Q/V/C` floors 下优化 `J_ENC + lambda_reg J_REG`；失败或 floor 不满足时回到 Stage 2。
- 每个 stage 使用同一个 tuple wall deadline的公平剩余切片；未完成 stage 不拖延 Local，直接返回最高优先级的已完成合法 stage。

barrier 使用 C1 smooth positive-part，并按 quality epsilon 归一化。它不决定最终可接受性；
hard filter 才决定 stage 是否能覆盖前一结果。

### 1.4 两处选择统一为词典序

`betterTeamQuality(a,b)` 顺序固定为：

```text
Q2 > accumulated visibility > complementarity > soft encirclement > regularization
```

它同时用于：

1. 27 tuples 的低分辨率 cheap Top-K 排序；
2. Top-K 连续优化结果的最终 tuple 选择。

旧 `cheap_objective` 和 weighted terms只保留为兼容 telemetry，不再拥有排序 authority。

### 1.5 发布边界 guard

发布 `TeamReferenceSchedule` 前再次检查：

```text
Q_team >= Q_local_base - eps_Q
Q 等价时：V_team >= V_local_base - eps_V
```

否则记录：

```text
TEAM_QUALITY_REJECT reason=PRIMARY_VISIBILITY_WORSENED
```

并 strict no-op。该 guard 只存在于 background Team path；没有进入 Local hard preflight，
也不会禁止任何 Local hard-safe first-safe trajectory立即执行。

## 2. 构建与直接合同

```text
catkin build multi_uav_formation -j2 --no-status --workspace ros_ws
BUILD: PASS（5 packages，0 warnings）
git diff --check: PASS
```

只运行本轮直接相关 contracts 和既有 reference/atomic wiring：

```text
TEAM_QUALITY_VECTOR_NORMALIZATION_PASS
HIERARCHICAL_VISIBILITY_PRIORITY_TEST_PASS
HIERARCHICAL_COMPLEMENTARITY_PRIORITY_TEST_PASS
STAGE2_VISIBILITY_FLOOR_PASS
STAGE3_VISIBILITY_COMPLEMENTARITY_FLOOR_PASS
FINAL_TEAM_NON_WORSENING_GUARD_PASS

J_COMP_PHI_FINITE_DIFFERENCE_PASS
J_K2_J_ACC_J_COMP_TIME_FINITE_DIFFERENCE_PASS
WORLD_TIME_SYNCHRONIZATION_PASS
TEAM_REFERENCE_REVISION_TOPOLOGY_IDENTITY_PASS

topology_coordinator_contract_test: PASS（16 cases）
team_solution_commit_contract_test: PASS
joint_adoption_atomic_contract_test: PASS
```

受控场景：

```text
HIERARCHICAL_VISIBILITY_PRIORITY_TEST: PASS
  Q2 略低但 J_ENC=0 的 solution，仍输给 Q2不下降而 J_ENC=1e9 的 solution。

HIERARCHICAL_COMPLEMENTARITY_PRIORITY_TEST: PASS
  Q2/V 等价时，C=0.90、J_ENC=1e9 的 solution，仍优先于 C=0.20、J_ENC=0。

TEMPORAL_DECONFLICTION_TEST: PASS
SIMULTANEOUS_LOS_LOSS_BEFORE: 0.400 s
SIMULTANEOUS_LOS_LOSS_AFTER:  0.000 s
TEAM_TIME_SHIFT: +0.304998 / -0.306714 / 0 s

FINAL_TEAM_NON_WORSENING_GUARD: PASS
```

## 3. 唯一正式 FULL 运行

```text
RUN_ID:          20260920_190112_693595
RUN_DIR:         runs/20260920_190112_693595
COMMAND:         ./run_on.sh --ablation full --headless --timeout 200
SCENARIO:        long_cylinder_forest.json
BOOT_12:         YES
CORE_EXIT_CODE:  0
CLEANUP_STATUS:  OK
FORMAL_MODE:     FULL
JOINT_YAW:       OFF
```

没有为随机指标波动调参或重跑。

### 3.1 Runtime hierarchy

最终 aggregate counter：

```text
PRIMARY_VISIBILITY_REJECT_COUNT: 14
COMPLEMENTARITY_REJECT_COUNT:    130

STAGE1_FALLBACK_COUNT: 0
STAGE2_FALLBACK_COUNT: 2
STAGE3_FALLBACK_COUNT: 142

FINAL_STAGE_Q2_COUNT:   0
FINAL_STAGE_ACC_COUNT:  0
FINAL_STAGE_COMP_COUNT: 42
FINAL_STAGE_ENC_COUNT:  51
```

前两个 reject 是 tuple 内后级 refinement 被 floor拒绝的次数，不是 Local execution
reject。93 个最终 proposal 均来自已经回退到的合法 stage；发布边界
`TEAM_QUALITY_REJECT` 为 0。

### 3.2 Prediction quality

并发 ROS stdout 会把少量长行互相插入。以下统计只使用 78 个能够把 selected tuple、
Local baseline和最终 optimizer record完整配对的 transaction，不补造损坏记录：

| normalized / cost | BASE P50 / P95 | FINAL P50 / P95 |
|---|---:|---:|
| Q2 | 1.000000 / 1.000000 | 1.000000 / 1.000000 |
| accumulated visibility | 0.996628 / 1.000000 | 1.000000 / 1.000000 |
| complementarity | 0.958658 / 1.000000 | 0.999532 / 1.000000 |
| soft encirclement cost | 57.466870 / 73.553069 | 50.012708 / 76.717676 |

```text
TEAM_Q2_BASE_P50/P95:    1.000000 / 1.000000
TEAM_Q2_FINAL_P50/P95:   1.000000 / 1.000000

TEAM_ACC_BASE_P50/P95:   0.996628 / 1.000000
TEAM_ACC_FINAL_P50/P95:  1.000000 / 1.000000

TEAM_COMP_BASE_P50/P95:  0.958658 / 1.000000
TEAM_COMP_FINAL_P50/P95: 0.999532 / 1.000000

TEAM_ENC_BASE_P50/P95:   57.466870 / 73.553069
TEAM_ENC_FINAL_P50/P95:  50.012708 / 76.717676
```

配对复算结果：

```text
FINAL_PRIMARY_NON_WORSENING_VIOLATION: 0 / 78
FINAL_Q/V_FLOOR_VIOLATION:             0 / 78
FINAL_COMPLEMENTARITY_FLOOR_VIOLATION: 0 / 78
```

ENC median下降，但 P95上升，说明 Stage3只在上层 floor 内改善各自 tuple，并不保证单次
运行的 encirclement tail全面变好。

### 3.3 Temporal 与 latency

```text
TEAM_TEMPORAL_DECONFLICTION_COUNT: 1 cleanly matched selected proposal
SIMULTANEOUS_LOS_LOSS_BEFORE:      0.500 s
SIMULTANEOUS_LOS_LOSS_AFTER:       0.000 s

TEAM_TIME_SHIFT_P50/P95/MAX:
  0.005706 / 0.181375 / 0.450000 s（absolute）
```

所有 Top-K tuple 中另有 5 个 parseable temporal improvement；最终 selected 的明确非零
T 去冲突仍存在，因此 hierarchy 没有冻结 T 或破坏 world-time sampling phase。

186 条完整 stage timing records：

```text
STAGE_Q2_LATENCY_P50/P95:   0.161 / 0.424 ms
STAGE_ACC_LATENCY_P50/P95:  0.211 / 2.682 ms
STAGE_COMP_LATENCY_P50/P95: 1.470 / 5.630 ms
STAGE_ENC_LATENCY_P50/P95:  4.172 / 9.331 ms

TEAM_TOTAL_LATENCY_P50/P95/MAX:
  7.155 / 16.014 / 30.266 ms per tuple
```

多阶段成本明显高于 Feedback089，但 max仍低于每 tuple 45 ms background budget；Local
first-safe没有等待 Team。Stage timeout/realization timeout均继续 strict no-op。

## 4. Actual-state production 指标

目标运动窗口 `76.445690 s`、2294 samples：

```text
MEAN_VISIBLE:     2.919355
K2:               99.564080%
ALL3:             92.371404%
BLACKOUT:         0 s / 0%
LONGEST_K2_LOSS:  0.299011 s

TEAM_COMPLEMENTARITY_SCORE:       2.900169
VISIBILITY_WEIGHTED_PAIR_QUALITY: 0.966723
PAIR_ANGLE_P50/P90:               137.887° / 158.459°

BEARING_ERROR_P50/P90: 20.661° / 35.330°
RADIUS_ERROR_P50/P90:  0.291 / 0.439 m
HEIGHT_ERROR_P90/MAX:  0.173 / 1.970 m
CIRCULAR_GAP_P90/MAX:  171.730° / 309.067°
```

`TEAM_COMPLEMENTARITY_SCORE` 与 Feedback089 保持同一 actual-state 口径：三机 target
relative 3D pair angle经25°到60° shared smooth pair-quality后求和；除以3得到 pair quality。

### 4.1 Team reference conversion

```text
TEAM_REFERENCE_PROPOSED: 93（serial 1..93；1条长日志被并发stdout插入）
TEAM_3ACK:               48
TEAM_COMMIT:             48 common commit broadcasts
TEAM_ACTIVATED:          45 complete three-UAV transaction IDs
PROPOSAL_TO_ACTIVATION_RATE: 48.39%（45/93）
```

另有：

```text
PARTIAL_TEAM_ACTIVATION: 2 transaction IDs（34、80）
POST_3ACK_COMMIT_REJECT: 3 transaction IDs（34、79、80）
```

三者都是3ACK/commit broadcast后某一 Local final preflight 返回
`POINTS_TO_CHECK_FAILED`。ID79在三机 adoption前全部no-op；ID34/80已有两机排队并激活，
第三机拒绝，形成实际 partial activation。这不是 hierarchy quality floor 能解决的问题，
而是既有 commit/final-preflight时序问题；本轮禁止修改 lifecycle/消息架构，因此没有越界
修补，但它使真实运行不能写成 atomic健康通过。

### 4.2 Liveness 与 safety

```text
TERMINAL_HOLD_EPISODES: 0
STARVATION_EPISODES:    1
END_BEFORE_NEXT:        0
UNVALIDATED_SAMPLES:    0

COLLISION_COUNT:        16 dynamic surface-contact samples
SWARM_VIOLATION_COUNT:  0
MIN_STATIC_CLEARANCE:   0.205624 m
MIN_DYNAMIC_CLEARANCE:  0.000000 m
MIN_SWARM_DISTANCE:     0.749415 m elliptical / 0.759415 m Euclidean
```

16个接触样本形成两个episode，均执行 `team_solution_id=0` 的 Local NOMINAL：

- UAV0 trajectory 224：10 samples，约 `t=64.479..64.780 s`；
- UAV1 trajectory 248：6 samples，约 `t=72.280..72.446 s`。

唯一 starvation同样发生于 UAV1 trajectory 248。样本本身不是 Team executable
polynomial，但不能因此把整次 closed-loop run判为安全；本轮也没有借 Team objective
修改 dynamic threshold、Local recovery或planner cadence。

## 5. 与 Feedback089 直接比较

| metric | Feedback089 | Feedback090 hierarchy | delta / 判断 |
|---|---:|---:|---:|
| K2 | 99.694989% | 99.564080% | -0.130909 pp，退化 |
| mean-visible | 2.926362 | 2.919355 | -0.007007，退化 |
| All3 | 92.941176% | 92.371404% | -0.569772 pp，退化 |
| blackout | 0 s | 0 s | 持平 |
| complementarity score | 2.892043 | 2.900169 | +0.008126，改善 |
| pair quality | 0.964014 | 0.966723 | +0.002709，改善 |
| pair angle P50/P90 | 135.367° / 157.722° | 137.887° / 158.459° | +2.520° / +0.737° |
| bearing error P50/P90 | 20.214° / 36.066° | 20.661° / 35.330° | median略差，P90改善 |
| radius error P50/P90 | 0.303 / 0.425 m | 0.291 / 0.439 m | median改善，P90略差 |
| height error P90/MAX | 0.258 / 0.777 m | 0.173 / 1.970 m | P90改善，max退化 |
| circular gap P90/MAX | 171.705° / 311.312° | 171.730° / 309.067° | P90近似持平，max改善 |
| terminal hold | 0 | 0 | 持平 |
| starvation | 1 | 1 | 持平但仍失败 |
| collision samples | 0 | 16 | 退化 |
| min swarm elliptical | 0.968284 m | 0.749415 m | 下降但未违反0.5 m |
| Team full activation | 72 | 45 | 下降 |
| proposal→full activation | 59.02% | 48.39% | 下降 |
| tuple latency P50/P95 | 0.894 / 1.661 ms | 7.155 / 16.014 ms | 多阶段成本上升 |

结论必须分层：

- **算法级 guard 正常。** 每个 Team refinement相对自己的 Local source baseline均受
  Q/V/C hierarchy约束，J_ENC不能再直接交换上层质量；预测 complementarity中位数明显
  上升。
- **单次 closed-loop结果不全面改善。** actual complementarity改善，但 K2/mean/All3
  轻微下降；这是不同 transaction/local realization/closed-loop轨迹的整体差异，不是发现
  某个 Team proposal越过了自己的 quality floor。
- **运行不健康。** dynamic contact和partial Team activation不允许用目标函数逻辑通过来
  掩盖。

## 6. 最终问题回答

1. **是。** Team 已从单次普通 weighted sum改为 Q2→ACC→COMP→ENC/REG 的真正多阶段
   hierarchy；每阶段 objective不同，后级由 hard floor过滤。
2. **是，在 Team refinement authority 内。** J_ENC 无法通过数值规模换取超过数值容差的
   Q2/V下降；78个完整配对记录为0违反，最终发布 guard也没有发现违规proposal。
3. **是。** Q2/ACC 饱和时 Stage2仍优化 C；prediction C P50 从0.958658升至0.999532，
   最终42个proposal停在COMP stage、51个继续到ENC stage。
4. **是。** Stage3只能在 Q/V/C floors内恢复 soft encirclement；130次尝试因C floor被拒并
   回退Stage2。
5. **是。** 受控 T 去冲突仍为0.4→0；production至少1个完整配对selected proposal为
   0.5→0，且time shift非零。
6. **是。** Local first-safe、Local polynomial、MINCO/J_vis/hard preflight和executor
   authority均未修改，Local不等待Team。
7. **是。** 没有引入 recovery、shadow planner、新FSM、yaw或K2 safety gate，也没有改变
   safety threshold。
8. **真实结果为混合且运行失败。** complementarity、pair angle、bearing P90和radius median
   改善；blackout/hold持平；K2、mean-visible、All3、Team conversion、collision与partial
   activation退化，starvation仍为1。

```text
PRIMARY_PRIORITY: Q2
SECOND_PRIMARY: ACCUMULATED_VISIBILITY
SECONDARY: COMPLEMENTARITY
TERTIARY: SOFT_ENCIRCLEMENT
LAST: REGULARIZATION

GLOBAL_PRIOR_MAP: PRESERVED
SOFT_ENCIRCLEMENT: ON
LOCAL_J_VIS: ON
N_L_R_TOPOLOGY: PRESERVED
TARGET_FACING_ASTAR: PRESERVED
H_PRED: 2 s
TEAM_EXECUTABLE_TRAJECTORY_AUTHORITY: LOCAL_ONLY
LOCAL_FIRST_SAFE_IMMEDIATE_COMMIT: PRESERVED
TEAM_FAILURE: STRICT_NOOP

NEW_SHADOW_PLANNER: NO
NEW_RECOVERY: NO
NEW_FSM: NO
SAFETY_THRESHOLD_CHANGED: NO
MESSAGE_ARCHITECTURE_CHANGED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES

REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_090.md
```
