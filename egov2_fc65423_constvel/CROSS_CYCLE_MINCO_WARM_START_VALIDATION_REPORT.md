# CROSS-CYCLE MINCO WARM-START VALIDATION REPORT

日期：2026-09-01；场景：long_cylinder_forest.json

## 1. 最终障碍物结果

三次既有 warm-start 运行的离线动态障碍重算：run1 obstacle 9 UAV1 collision，min clearance -0.1174 m，9 samples；run2 -0.0888 m，16 samples；run3 -0.0922 m，7 samples。每次 obstacle 9 为一个 collision episode，共 3 episodes、32 samples。obstacle 6 为 0 collision episodes，但有 4 个 unsafe encounters（clearance < 0.5 m）：run1 min 0.0005/0.2041 m，run2 0.3850 m，run3 0.1461 m。

OBSTACLE9_COLLISION_BEFORE: 3/3
OBSTACLE9_COLLISION_AFTER: 3/3
OBSTACLE6_COLLISION_AFTER: 0/3

## 2. 严格 counterfactual A/B

最终 smoke log 为 warmstart_counterfactual_20260901_d.log。该轮使用源码原生 computeInitState(flag_polyInit=true) 生成 A，共 8 个 strict pair、16 条 fresh/warm 记录。B 使用 shifted accepted trajectory P/T warm seed；两者共享当前 start/target、prediction epoch、SIDE context、piece partition 和现有约束。每个 pair 都完成 optimizer 调用，未出现 fresh FAIL -> warm OK 的恢复事件。

典型 pair：cycle 49 fresh/warm init jerk 129.455/6.206，final jerk 4.001/4.015，clearance 0.941 m；cycle 52 为 132.684/4.579、3.520/3.520、0.364 m；UAV1 cycle 18 为 123.459/1.787、3.171/3.172、0.552 m。

STRICT_COUNTERFACTUAL_CASES: 8
WARM_START_COUNTERFACTUAL_SUCCESS: 0/8
WARM_START_CAUSAL_BENEFIT: PARTIAL

PARTIAL 仅表示初始化 jerk 显著改善；本批没有 fresh 失败而 warm 恢复，最终 jerk/clearance 基本相同，因此不能声称 warm 已降低 obstacle-9 碰撞率。

## 3. 初始化 jerk 与异常值

严格 pair smoke 统计：fresh n=8，P50=131.637，P95=133.996，max=135.695；warm n=8，P50=4.840，P95=5.781，max=6.206。

历史 run2/run3 曾有 warm max jerk 9374.55/1374.64。本轮真实 slice-head 诊断覆盖 536 次 warm slice：remaining duration 最小 3.0165 s，最大 head mismatch 为 position 0.0979 m、velocity 0.0362 m/s、acceleration 0.0862 m/s²；本轮未复现 >1000。piece-middle 多项式平移、剩余 duration 和 topology 均正常。现有证据最符合历史 accepted polynomial 的病态高阶样本，而非当前 slice/head 映射 bug；未发现生产 warm-start 实现 bug。

WARM_JERK_OUTLIER_ROOT_CAUSE: historical pathological accepted-polynomial sample; current slice/head mapping not reproducing it
WARM_START_IMPLEMENTATION_BUG_FOUND: NO
WARM_START_IMPLEMENTATION_BUG_FIXED: NOT_NEEDED

## 4. 生产链与安全性

accepted cache 只在 setLocalTrajFromOpt 成功提交后更新，拒绝 candidate 不会污染 cache。warm slice 保留 accepted piece partition 和 physical durations，再用 authoritative 当前 start P/V/A 与 target tail 重建 MINCO。Local-SFC、SIDE、native static、v/a/j、trust-region、OSQP 和 final checker 均未改动。

smoke 中无 QP/OSQP crash、NaN/Inf、dimension mismatch、double free 或 segmentation fault。通过 Ctrl-C/SIGINT clean shutdown，退出码 0；未发现本次 launch 创建的 planner/roslaunch 残留。检查时存在一组先前 /home/bob/RRCT/reopen 任务的 rosmaster/nodelet 进程，明确不属于本次运行。

## 5. Build

按要求执行 catkin build traj_opt ego_planner path_searching -j2 并指定本地 OSQP；三次增量构建均 PASS，OSQP 链接正常。

BUILD: PASS
RUNTIME: PASS

## 6. Required final status

OBSTACLE9_COLLISION_BEFORE: 3/3
OBSTACLE9_COLLISION_AFTER: 3/3
OBSTACLE6_COLLISION_AFTER: 0/3
STRICT_COUNTERFACTUAL_CASES: 8
WARM_START_COUNTERFACTUAL_SUCCESS: 0/8
WARM_START_CAUSAL_BENEFIT: PARTIAL
FRESH_INIT_JERK_P95: 133.996
WARM_INIT_JERK_P95: 5.781
WARM_JERK_OUTLIER_ROOT_CAUSE: historical pathological accepted-polynomial sample; not reproduced after real slice-head diagnostics
WARM_START_IMPLEMENTATION_BUG_FOUND: NO
WARM_START_IMPLEMENTATION_BUG_FIXED: NOT_NEEDED
SCP_CORE_CHANGED: NO
TRUST_CHANGED: NO
OSQP_CHANGED: NO
JERK_LIMIT_CHANGED: NO
STATIC_COLLISION_REGRESSION: NO
TRACKING_REGRESSION: NO_EVIDENCE
VISIBILITY_REGRESSION: NO_EVIDENCE
PLANNING_LATENCY_REGRESSION: NO_EVIDENCE
BUILD: PASS
RUNTIME: PASS

结论：cross-cycle warm-start 已在生产链真实运行，并显著降低典型初始化 jerk；但 obstacle 9 仍为 3/3 碰撞，严格 A/B 没有 fresh-fail -> warm-success。当前第一 blocker 是 obstacle-9 recovery 的下游动态/时序可行性与执行结果，而不是 warm-start 是否进入。
