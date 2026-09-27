# Feedback 089 — 目标中心 R/PHI/T Team 优化与 Local-only reference realization

日期：2026-09-20  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
生产基线：`feedback/feedback_087.md` 的旧 Team P/T 运行  
正式场景：`long_cylinder_forest.json`

## 0. 架构结论

本轮已把生产 Team P/T 从“直接修改 Local Cartesian MINCO polynomial”改为：

```text
三机 Local N/L/R candidates（最多 27 tuples）
→ 共享五项目标的 cheap score
→ Top-K（默认 3）
→ 每个 tuple 优化少量 target-centered R/PHI/T knots
→ TeamReferenceSchedule（只有 reference，没有 executable polynomial）
→ 三台真实 Local Planner 在绑定 topology 内各自 MINCO/J_vis/hard preflight
→ 三条 Local realization ACK
→ coordinator 原样汇集三条 Local polynomial
→ 3ACK / common COMMIT / common activation
→ 任一失败、stale、超时或 CAS 变化：strict no-op
```

Local first-safe 仍立即提交，不等待 Team。Team 的 2 s future prediction 不取得
execution authority；真正的 polynomial、hard validation、trajectory ID 和 executor handoff
全部仍由 Local Planner 产生。

```text
TEAM_ARCHITECTURE_REWORK_COMPLETE: YES
TEAM_EXECUTABLE_TRAJECTORY_AUTHORITY: LOCAL_ONLY
TEAM_DECISION_VARIABLES: R / PHI / T
JOINT_YAW: OFF
GLOBAL_PRIOR_MAP: YES
ENCIRCLEMENT: ON（soft）

OVERALL_RUNTIME_ASSESSMENT: PARTIAL
```

`PARTIAL` 不是结构没有接通：最终运行有 72 次完整三机 Team activation。它表示性能验收
没有全部优于旧基线：K2 轻微下降，实际 visibility-weighted complementarity 轻微下降，
且运行中出现 1 次 Local starvation。不能用 Team conversion 成功掩盖这些结果。

## 1. Production 修改

### 1.1 共享 objective primitives 与 Top-K

新增 `team_objective_primitives.h`，tuple cheap score 和连续优化共用以下五项：

```text
J_team = w_K J_K2 + w_A J_ACC + w_C J_COMP + w_E J_ENC + w_R J_REG
```

- `J_K2` 使用 `Q2=v0v1+v0v2+v1v2-2v0v1v2`；
- `J_ACC` 保留第三相机恢复梯度；
- `J_COMP` 使用实际可见性乘以 25°→60° target-centered pair smoothstep；
- `J_ENC` 只包含 soft radius/bearing reference；
- `J_REG` 只约束 `Δr/Δphi/Δtau`，避免小收益大重排。

All3、blackout、weakest camera、Q_dir、gap、diversity、min pair angle均只作 telemetry，
不再参与生产 Team continuous decision 或 final quality gate。旧 Cartesian Team optimizer
实现仍留作已有 regression contract 的源码依赖，但生产 `evaluateSnapshot()` 在
target-centered reference 分支结束，不再调用旧 optimizer。

```text
ACTIVE_TEAM_OBJECTIVES:
  J_K2 / J_ACC / J_COMP / J_ENC / J_REG
OLD_DUPLICATE_OBJECTIVES_REMOVED_FROM_DECISION: YES
METRIC_ONLY:
  All3 / blackout / weakest-camera / Q_dir / circular-gap /
  angular-diversity / min-pairwise-angle
```

最终运行中可解析 selected proposal 的五项目标中位数（raw / weighted）：

| objective | raw P50 | weighted P50 |
|---|---:|---:|
| J_K2 | 0.000000 | 0.000000 |
| J_ACC | 0.000000 | 0.000000 |
| J_COMP | 0.167893 | 0.251840 |
| J_ENC | 43.843299 | 15.345155 |
| J_REG | 7.406333 | 1.481267 |

`J_K2/J_ACC` 中位数为 0 是因为多数 2 s prediction window 已有三机可见，不表示其梯度
未接入；受控遮挡用例与 runtime 的遮挡 tuple 均产生了非零 T 梯度和时间错峰。

### 1.2 Target-centered optimizer

新增 `TeamTargetCenteredOptimizer`：

- 每机 4 个 reference knots；
- 自由变量为 knot `Δr`、`Δphi` 和每机一个 `Δtau`；
- height 固定为输入 cooperative/reference height；
- 所有 visibility 在同一 `evaluation_start_world + k*dt` 计算；
- `Δtau` 改变该世界时刻查询到的 reference phase，因此真实进入
  `J_K2/J_ACC/J_COMP` 导数；
- 每个 tuple 使用相同短预算，默认最多优化 Top-3。

最终运行共观测 123 个 Top-K cycle；可完整解析的 121 个 cycle 中，cheap stage枚举
179 个可行 tuples，保留并连续求解 171 个，其中 12 个 cycle 实际保留 Top-3、26 个
保留 Top-2、83 个只有一个 coarse-feasible tuple。172 条 optimizer 日志中 171 条可完整
解析，170 成功、1 失败。

```text
TEAM_TOPK: 3
NLR_TOPOLOGY_IDENTITY_PRESERVED: YES
TARGET_PREDICTION_REVISION_SHARED: YES
```

### 1.3 TeamReferenceSchedule 与 Local-only realization

新增 `TeamReferenceSchedule.msg`，携带：

- reference/revision/context identity；
- target prediction、dynamic prediction、static map、visibility model revision；
- common evaluation/activation time；
- N/L/R topology 和 Local source candidate identity；
- 每机 `world_time/radius/bearing/fixed_height` knots 与 time shift；
- 五项目标和 metric-only telemetry。

三台 Planner 必须找到 schedule 绑定的原 Local topology seed，复用其 A*/Local-SFC
几何，在 schedule 的小幅 target-centered reference 下重新运行 Local MINCO/J_vis，并重新
执行 dynamics/static/dynamic/swarm/current-revision/handoff hard preflight。任一 Planner
失败时只回 negative ACK，当前 Local commit 不变。

第一次集成运行暴露 commit payload 经 float ROS message round-trip 后不能用 `1e-9`
重建采样误差判 identity；第二次运行进一步证明高阶系数会偶发放大该误差。最终实现保存
Local ACK 的原始 `MINCOTraj` payload，并要求 coordinator COMMIT 原样回传，再由 Planner
重建；最终运行 `TEAM_REFERENCE_COMMIT_REJECT=0`。这没有放宽 hard safety。

## 2. Time 与受控 simultaneous LOS 验证

确定性受控场景中，两条默认参考在相近世界时刻进入遮挡，第三机保持可见。优化结果：

```text
SIMULTANEOUS_LOS_LOSS_BEFORE: 0.400 s
SIMULTANEOUS_LOS_LOSS_AFTER:  0.000 s
TEAM_TIME_SHIFT: +0.347457 / -0.350227 / 0.000000 s
```

finite difference 同时验证 `J_K2/J_ACC/J_COMP` 对 T 的导数；该结果不是只改变总 duration。

最终 production run 中：

```text
WORLD_TIME_SYNCHRONIZED: YES
TEAM_PREDICTION_HORIZON: 2.000 s
TEAM_EXECUTION_AUTHORITY_HORIZON:
  Local hard-validated prefix；proposal observed min/P50/P95/max
  = 0.426 / 3.015 / 3.348 / 3.640 s

TEAM_TEMPORAL_DECONFLICTION_COUNT: 2 selected production references
SIMULTANEOUS_LOS_LOSS_BEFORE: 0.500 s（上述2个selected合计）
SIMULTANEOUS_LOS_LOSS_AFTER:  0.000 s
TEAM_TIME_SHIFT_P50/P95/MAX: 0.0337 / 0.3212 / 0.4500 s（absolute）
```

H_pred 与 H_exec 明确分离：Team 可看 2 s future visibility；真正执行段长度来自每架
Local 当时的 hard-validated committed prefix，不由 prediction horizon 自动授权。

## 3. 构建与必要测试

```text
catkin build traj_utils ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws

BUILD: PASS（6 packages；最终增量 ego_planner rebuild亦 PASS）
git diff --check: PASS
```

只运行本次直接相关 contracts：

```text
TARGET_CENTERED_MAPPING_PASS
J_COMP_PHI_FINITE_DIFFERENCE_PASS
J_K2_J_ACC_J_COMP_TIME_FINITE_DIFFERENCE_PASS
WORLD_TIME_SYNCHRONIZATION_PASS
TEAM_REFERENCE_REVISION_TOPOLOGY_IDENTITY_PASS
topology_coordinator_contract_test: PASS（16 cases）
team_solution_commit_contract_test: PASS
joint_adoption_atomic_contract_test: PASS
```

没有建立新的大型测试矩阵。

## 4. 最终 FULL 真实运行

```text
RUN_ID:          20260920_160211_684140
RUN_DIR:         runs/20260920_160211_684140
COMMAND:         ./run_on.sh --ablation full --headless --timeout 200
SCENARIO:        long_cylinder_forest.json
BOOT_12:         YES
CORE_EXIT_CODE:  0
CLEANUP_STATUS:  OK
FORMAL_MODE:     FULL
JOINT_YAW:       OFF
```

最终运行之前有两次主动终止的集成诊断运行：第一次发现所有 COMMIT 的过严 double
比较，第二次发现偶发高阶 float 重建误差。两次均在确认断点后立即清理，均不计最终性能
样本。最终运行没有 commit identity reject，且 72 个 commit 全部形成三机 activation。

### 4.1 Team pipeline

```text
TEAM_TUPLE_ENUMERATED:                 179 parseable feasible tuples / 121 cycles
TEAM_TUPLE_TOPK_COUNT:                 171 tuple solves / 121 parseable cycles
TEAM_OPT_ATTEMPT:                      172 observed logs
TEAM_OPT_SUCCESS:                      170 / 171 fully parseable logs

TEAM_REFERENCE_PROPOSED:               122
TEAM_REFERENCE_REALIZATION_SUCCESS:    80 all-three realization events
TEAM_REFERENCE_REALIZATION_FAILED:     44 per-UAV failure log lines
TEAM_3ACK:                             73
TEAM_COMMIT:                           72
TEAM_ACTIVATED:                        72 team IDs / 216 UAV activations
TEAM_REFERENCE_COMMIT_REJECT:          0
TEAM_NOOP:                             246 log observations

PROPOSAL_TO_ACTIVATION_RATE:           59.02% (72/122)
COMMIT_TO_THREE_UAV_ACTIVATION_RATE:   100% (72/72)
```

`TEAM_NOOP` 是节流/广播观测行，不冒充 unique transaction 数。主要原因包括 Local
realization hard preflight/MINCO failure、stale bundle、post-ACK CAS stale 以及没有 common
frontier；这些都保持已有 Local trajectory，不启动 recovery。

### 4.2 Visibility 与 complementarity

目标运动窗口 `76.479385 s`、2295 samples：

```text
MEAN_VISIBLE:          2.926362
K2:                    99.694989%
ALL3:                  92.941176%
BLACKOUT:              0 s / 0%
LONGEST_K2_LOSS:       0.233414 s
LONGEST_BLACKOUT:      0 s

TEAM_COMPLEMENTARITY_SCORE:          2.892043（actual binary visibility × pair quality sum）
PAIR_ANGLE_P50/P90:                  135.367° / 157.722°
VISIBILITY_WEIGHTED_PAIR_QUALITY:    0.964014
```

### 4.3 Soft encirclement

以下 bearing error 是每帧相对最佳拟合 soft-120° common rotation 的三机误差；radius error
相对生产 cooperative per-UAV radius；height error 相对 target/reference height：

```text
BEARING_ERROR_P50/P90: 20.214° / 36.066°
RADIUS_ERROR_P50/P90:  0.303 / 0.425 m
HEIGHT_ERROR_P90/MAX:  0.258 / 0.777 m
CIRCULAR_GAP_P90/MAX:  171.705° / 311.312°
```

因此 encirclement 机制仍完整 ON 且是 soft objective，但实际几何并非全程理想：P90 gap
略高于历史 170°诊断线，max 含启动/瞬态大缺口。没有把 120°重新改成 hard gate。

### 4.4 Liveness 与 safety

```text
TERMINAL_HOLD_EPISODES:       0
TERMINAL_HOLD_TOTAL_DURATION: 0 s
TERMINAL_HOLD_MAX_DURATION:   0 s
STARVATION_EPISODES:          1
END_BEFORE_NEXT:              0
```

唯一 starvation 在 drone1 trajectory 252：previous-safe suffix 遇到动态障碍，clearance
`0.431071 m`，被既有 hard checker正确拒绝；同一 Local Planner 从 odometry 恢复并在约
`0.973 s` 后提交 trajectory 253。期间没有 executor terminal hold，也不是 Team 等待造成。

```text
COLLISION_COUNT:        0（scene surface samples <= 0）
SWARM_VIOLATION_COUNT:  0（elliptical distance < 0.5 m）
UNVALIDATED_SAMPLES:    0
MIN_STATIC_CLEARANCE:   0.332835 m
MIN_DYNAMIC_CLEARANCE:  0.004208 m
MIN_SWARM_DISTANCE:     0.968284 m elliptical / 0.968296 m Euclidean
```

最小 dynamic surface clearance 很小但仍为正；没有降低 physical threshold，也不把它
写成宽裕的 safety margin。

### 4.5 Performance

```text
TEAM_OPT_LATENCY_P50/P95/MAX:
  0.894 / 1.661 / 2.244 ms per tuple

LOCAL_REALIZATION_LATENCY_P50/P95/MAX:
  0.427 / 1.089 / 17.276 ms

END_TO_END_TEAM_REFERENCE_TO_COMMIT_LATENCY_P50/P95/MAX:
  27.078 / 31.101 / 47.507 ms（68 cleanly timestamp-paired commits）
```

这些 background latency 不占用 Local first-safe deadline；超时周期为 strict no-op。

## 5. 与旧 Team P/T 基线对照

旧基线为 `runs/20260919_205929_346132`（Feedback087）。同一 actual-state 统计口径：

| metric | OLD_TEAM_PT | NEW_TARGET_CENTERED_TEAM | delta |
|---|---:|---:|---:|
| K2 | 100.0000% | 99.6950% | -0.3050 pp |
| blackout | 0 s | 0 s | 0 |
| mean visible | 2.854466 | 2.926362 | +0.071896 |
| All3 | 85.4466% | 92.9412% | +7.4946 pp |
| complementarity score | 2.951449 | 2.892043 | -0.059405 |
| pair angle P50/P90 | 131.908° / 151.763° | 135.367° / 157.722° | +3.458° / +5.958° |
| target-window hold | 0 | 0 | 0 |
| starvation | 0 known | 1 | +1 |
| min swarm elliptical | 1.005381 m | 0.968284 m | -0.037097 m |
| Team commit / activated | 0 / 0 | 72 / 72 | +72 / +72 |
| Team success rate | 0% | 59.02% proposal→activation | improved |
| Team tuple latency P95 | not recorded | 1.661 ms | N/A |

新结构显著改善 All3、mean-visible 和实际 Team conversion，pair angles也增大；但 K2
并未提升，visibility-weighted complementarity 因少量相机丢失反而略降，且增加一次
starvation。因此对“性能优于或不低于旧 Team P/T”的整体答案是 **NO**，本轮不能宣称
论文指标全面成功。

## 6. 最终 12 项回答

1. **是。** Team 生产路径不再生成 executable polynomial，只发布 reference schedule。
2. **是。** 只有真实 Local Planner 生成 polynomial、执行 hard preflight并取得 trajectory authority。
3. **是。** Team 连续变量是少量 `R/PHI/T` knots，高度固定，不再高维修改 Local control points。
4. **是。** 最多 27 个 N/L/R tuples 经 cheap score 后保留 Top-K=3 连续优化。
5. **是。** tuple ranking 与连续优化共用 `TeamObjectivePrimitives` 的五项目标哲学。
6. **是。** visibility-weighted target-centered `J_COMP` 有 analytic gradient，并通过 phi finite difference。
7. **是。** 受控场景中 T 将 simultaneous LOS loss 从 0.4 s 降至 0；production 另有2次 selected deconfliction。
8. **是。** H_pred 固定 2 s，H_exec仍由当时 Local hard-validated prefix决定。
9. **是。** realization失败/stale/timeout/CAS变化均 strict no-op；Local rolling不等待。
10. **是（机制）/部分（效果）。** soft encirclement完整保留，但真实 gap P90 为171.705°。
11. **否。** mean-visible/All3/pair angle改善，但 K2和visibility-weighted complementarity低于旧基线。
12. **是。** 没有重新引入 shadow planner、recovery target/guide、peer planning、新 FSM、brake/stop或Joint yaw。

```text
NEW_SHADOW_PLANNER: NO
NEW_RECOVERY_MECHANISM: NO
NEW_PEER_PLANNING: NO
NEW_FSM: NO
SAFETY_THRESHOLD_CHANGED: NO
LOCAL_J_VIS_REMOVED: NO
THREE_THREAT_TOPOLOGY_REMOVED: NO
TARGET_FACING_ASTAR_REMOVED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
COMMITS: none
PREVIOUS_UNCOMMITTED_WORK_PRESERVED: YES

REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_089.md
```
