# Post-2026-09-08 Delta Audit (read-only, evidence-verified)

Audit scope: `/home/bob/ALP/egov2_fc65423_constvel` + `/home/bob/ALP/encirclement_*`
Baseline boundary: **everything up to and including 2026-09-08 = BASELINE, not new.**
Mode: READ-ONLY. No file modified, no build, no simulation. Written 2026-09-18.

---

## 0. Method, and the mtime caveat (read this before trusting any date below)

Three independent dating channels were used, in decreasing order of strength:

| Channel | Strength | Why |
|---|---|---|
| **Experiment run directories + their `run_meta.json` `launch_arguments`** | **Strongest** | Each run records the exact ROS launch arguments actually passed on that date. If a parameter does not appear in a 09-08 run's argument list, the mechanism did not exist in a switchable/parameterised form on 09-08. |
| **ROS `.msg` file mtimes + generated `devel/include/*.h` header mtimes** | Strong | A `.msg` is written once at creation and almost never edited; the generated header is written when the message definition changes. The two agree in every case checked. |
| **Source file mtimes** | **Weak — upper bound only** | mtime is the LAST edit (or a bulk copy/`touch`), not creation. **Every date taken from an mtime below is explicitly flagged.** Many files share identical mtimes (e.g. 09-11 12:56, 09-16 01:27–01:28, 09-18 18:26) which is the signature of a batch copy, not of simultaneous authorship. |

Git is unusable as a separator, confirmed:
- `git log --oneline` → `8cef241 Finalize EGOv2 sinusoidal gradient avoidance`, and `BASELINE.md` says *"Base commit: `fc654236127c907db48a9794b8f5e434997d9aa2`"*.
- `git ls-tree -r --name-only 8cef241 -- .../planner/` returns **0 files** and `-- .../multi_uav_formation/` returns **0 files**. The entire planner and formation source tree is untracked, so `git diff` cannot date anything.

**Negative structural finding (important):** there is **no experiment directory dated 2026-09-10 or later anywhere under `/home/bob/ALP`.** Confirmed by
`find /home/bob/ALP -maxdepth 3 -type d -name '*2026091*'` → the only hit is `.ros_log_archive_20260915`.
The newest `encirclement_*` directory is `encirclement_team_visibility_final_5runs_20260909` (09-09 15:11).
The 09-10…09-18 run directories named inside the feedback reports (e.g. `scenario_a_full_rviz_20260910`, `true_encirclement_gap_recovery_20260910`, `visibility_effect_validation_20260910/runs_final2`) **have been deleted**. So for 09-10 onward there is no `run_meta.json` evidence at all, and the surviving evidence is the reports, source mtimes, and two surviving run logs (`sim_run_round2.log`, `egov2_constvel_run_latest.log`).

Experiment directories that DO survive and carry the baseline:
`encirclement_a_ablation_20260908`, `encirclement_a_directed_fix_20260908`, `encirclement_actual_fov_20260908`, `encirclement_trajectory_fov_20260908`, `encirclement_adaptive_outer_20260908`, `encirclement_fixed_multirate_20260908` (106 `run_meta.json` files total), plus `encirclement_*_20260909` (5 dirs).

---

## A. What existed by the end of 2026-09-08

### A.1 Hard evidence: the 09-08 launch argument set

Every one of the six 09-08 experiment directories passed **exactly this** switchable parameter set (union over all 106 `run_meta.json` files):

```
early_avoidance_enabled, enable_astar_visibility_cost, enable_candidate_acceptance,
enable_candidate_hard_corridor_scp, enable_cooperative_viewpoint_reference,
enable_elastic_tracking_region, enable_encirclement_tracking,
enable_joint_topology_coordination, enable_k_of_n_team_visibility,
enable_minco_visibility_cost, enable_relative_safety_temporal_elasticity,
enable_risk_triggered_candidates, enable_target_facing_yaw,
enable_visibility_candidate_ranking, enable_warm_start_counterfactual,
encirclement_outer_period, encirclement_prediction_horizon, max_jer,
moving_obj_num, moving_obstacle_time_aware_cost_enabled, rviz, scene_file,
sync_dynamic_motion_to_target_start, target_dynamic_obstacle_avoidance_enabled,
target_segment_speeds, target_speed, target_waypoints, team_visibility_k,
trajectory_csv, viewpoint_independent_mode, visibility_csv
```

The **only** parameter that appears in the 09-09 argument sets and **not** in the 09-08 sets is:

```
weight_visibility
```

This single-key delta is the cleanest mechanical proof in the whole audit, and it is corroborated by a plain content grep: `grep -rn "weight_visibility\|directional" encirclement_*_20260908/` returns **zero hits**, while `encirclement_directional_visibility_20260909/run_single.py` carries `weight_visibility`.

Representative 09-08 baseline flags (verbatim from `encirclement_a_ablation_20260908/off/run_1/run_meta.json`):
`enable_elastic_tracking_region:=true`, `enable_cooperative_viewpoint_reference:=true`, `viewpoint_independent_mode:=true`, `enable_visibility_candidate_ranking:=true`, `enable_k_of_n_team_visibility:=true`, `team_visibility_k:=2`, `enable_joint_topology_coordination:=true`, `enable_encirclement_tracking:=false`, `enable_candidate_hard_corridor_scp:=true`.

### A.2 Confirmed BASELINE (existed on or before 09-08)

| # | Mechanism | Evidence |
|---|---|---|
| 1 | **SIDE± LEFT/RIGHT candidates, side directionality contract** | `ALP_SIDE_DIRECTIONALITY_AUDIT_REPORT.md`, *"Date: 2026-09-02"*, audits "`SIDE_PLUS`/`SIDE_MINUS` … 代码使用的是'路线右法向'作为正方向" |
| 2 | **A\* static repair + A\* rejoin + semantic-v2 + arc-length time init** | `feedback_1.md` / `RUN1_C2_POST_THREE_CLASS_AUTOPSY.md`, *"Date: 2026-09-02"*, *"PLUS A*: SUCCESS, 13 raw points, 9 simplified points"*; `THREE_CLASS_AND_ARCLENGTH_TIME_INIT_REPORT.md`, *"Date: 2026-09-02"*, *"only dynamic safety classification/selection and A* repaired-guide time initialization"* |
| 3 | **Local SFC (SFC plane construction + empty-plane rejection)** | same 09-02 autopsy: *"rejected before MINCO because Local-SFC produced no planes"*; `ALP_SFC_MINCO_SCP_SOURCE_CHAIN_AUDIT_20260902.txt`, *"Date: 2026-09-02"* |
| 4 | **MINCO + SCP + OSQP backend** | `ALP_SFC_MINCO_SCP_SOURCE_CHAIN_AUDIT_20260902.txt` — *"ALP Native Local-SFC -> MINCO -> SCP -> OSQP Source Audit, Date: 2026-09-02"* |
| 5 | **Three-class candidate safety classification (SAFE/IMPROVED/INVALID, `CandidateSafetyClass`)** | `THREE_CLASS_AND_ARCLENGTH_TIME_INIT_REPORT.md` (09-02): *"`planner_manager.h`: adds `CandidateSafetyClass` and classification fields"* |
| 6 | **Time-only inter-UAV conflict repair / relative safety temporal elasticity** | flag `enable_relative_safety_temporal_elasticity` present in every 09-08 run; `DYNAMIC_SAFETY_CONTRACT_AND_CLOSURE_AUDIT.md`, *"审计日期：2026-09-01"* |
| 7 | **N/L/R = 3-way tuple enumeration/filtering (≤27) + team scoring/hysteresis** | `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md` §12 describes the selector "1. executable 条件；2. safety lattice … 3. best K2 … 4. mean visible count 最大；All3 最大" as **current** source on 09-10, and the 09-08 runs already switch `enable_joint_topology_coordination:=true` + `enable_k_of_n_team_visibility:=true` + `team_visibility_k:=2` |
| 8 | **K-of-N (k=2) team visibility selection; All3/mean-visible safety lattice** | same as #7. n.b. **"K2" here = `atleast2` (visible count ≥ 2), NOT the `Q2` continuity score of item B-5** |
| 9 | **Elastic tracking region: radial band + angular deadband `alpha(ρ)`, frozen risk ρ** | `enable_elastic_tracking_region:=true` in all 09-08 runs; the parameter *names* `elastic_delta_phi_min_deg`, `elastic_delta_phi_max_deg`, `elastic_radial_slack_max`, `elastic_tracking_min_weight_scale`, `elastic_static_risk_margin`, `elastic_moving_risk_margin`, `elastic_swarm_risk_margin` are all already passed by the 09-08 runs. Also `cooperative_benchmark_validation_20260905/` and `final_cooperative_benchmark_20260906/` already contain `elastic_*` keys, and `alp_feature_validation_20260905/` already logs `elastic_risk_mean`, `elastic_angular_deviation_mean_deg`, `elastic_radial_deviation_mean_m` |
| 10 | **120° slot cooperative viewpoint reference + slot-permutation init + hypotheses** | `enable_cooperative_viewpoint_reference:=true`, `viewpoint_independent_mode:=true` in 09-08 runs; `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md:31` — *"`encirclementAngles()` 明确生成 `θ_i=wrap(φ0+2π s_i/3)`"*; *"约 1043 行：枚举六种 slot permutation"* |
| 11 | **`EncirclementReferenceBundle` / `EncirclementReferenceHypothesis.msg` (max 3 hypotheses, φ0, φ0±15°)** | `.msg` mtimes **2026-09-08 21:05 / 19:41**; generated headers `devel/include/traj_utils/EncirclementReferenceHypothesis.h` and `EncirclementReferenceBundle.h` both **2026-09-08 19:55** — a message that did not exist could not have been generated. Report: *"只提出当前、−π/12、+π/12 三种共同角"* |
| 12 | **`StaticLosGeometry` authoritative static LOS cylinder/wall query** | `feedback_10.md`, **日期：2026-09-03** — *"评价使用 Feedback 9 的 authoritative solid-cylinder helper：`plan_env/static_los_geometry.h`"*; `generate_cooperative_benchmark_scenes.py` mtime 2026-09-07 also references it |
| 13 | **`TopologyCoordination.msg` + `TopologyCandidate.msg`** | mtimes 2026-09-08 21:05 and **2026-09-05 17:42** (`devel/include/traj_utils/TopologyCandidate.h` = 09-05 17:42) |
| 14 | **Legacy `J_los` / `enable_minco_visibility_cost` (λ=150) — present but OFF** | `minco-visibility-config] enabled=0 lambda=150.000000`; `enable_minco_visibility_cost:=false` in all 09-08 runs |
| 15 | **Visibility used only as a RANKING signal** | `feedback_22.md`(09-05) and `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md:18` — *"当前 directional J_vis"* is described as **current** on 09-10, i.e. added between; and the 09-08 runs have `enable_visibility_candidate_ranking:=true` but no `weight_visibility` argument |

### A.3 The midnight crossover (stated honestly, not forced)

`encirclement_visibility_elastic_20260909/` was created **2026-09-09 01:01**, and `elastic_tracking_contract.h` has mtime **2026-09-09 02:11**. This is a continuous overnight session spilling past midnight. Two facts bound it:

- The 09-08 ablation runs completed at **2026-09-09 00:09** (`encirclement_fixed_multirate_20260908/run_meta.json` auto-cutoff timestamps read `2026-09-09T00:05…00:09+08:00`).
- `elastic_tracking_contract.h` is an **untracked new file** (`git status` shows it as `??`), and its helpers `elasticAngularSlack(...)`, `angularDeadbandViolation(...)`, `directionalVisibilityCostEnabled(...)` are **not** referenced anywhere in the 09-08 experiment evidence.

Verdict: **the elastic *region* mechanism is baseline; the `elastic_tracking_contract.h` helper header is a 09-09 artefact.** Both statements are given rather than picking one, because the artefact straddles the boundary.

---

## B. What was added 2026-09-09 … 2026-09-17, in causal order

Dates below are either **in-document dates** (quoted) or **mtime** (flagged `[mtime]`). Production status is from the 09-18 production run configuration printouts.

### B-1. **09-09 — Visibility stops being only a ranking signal: directional `J_vis` becomes a continuous cost in the local MINCO optimizer**

- **(i) Problem solved:** The 09-10 structural report states the pre-change state — visibility only entered *final ranking*, so a continuously-blocked UAV could not change the *shape* of its trajectory. The 09-09 `directional_visibility` experiments were run specifically to test a gradient.
- **(ii) Date/trigger:** 09-09. `encirclement_visibility_elastic_20260909` (09-09 **01:01**), then `encirclement_directional_visibility_20260909` (09-09 **02:36**), then `encirclement_directional_visibility_final_ab_20260909` (09-09 **13:24**), `encirclement_team_visibility_alignment_20260909` (09-09 **14:33**), `encirclement_team_visibility_final_5runs_20260909` (09-09 **15:11**). The `weight_visibility` launch argument is the mechanical marker of the change.
- **(iii) File / function / identifiers:** `traj_opt/src/poly_traj_optimizer.cpp` → `PolyTrajOptimizer::directionalVisibilityGradCostP()` (line ~4992) and `PolyTrajOptimizer::addDirectionalVisibilityGradCost2CT()` (line ~5186, called at ~8102); parameter `optimization/weight_visibility` read at ~`7519` with default `20.0`; guard `directionalVisibilityCostEnabled(encirclement_tracking_configured_, encirclement_visibility_guidance_active_, weight_visibility_)`; helpers `multi_uav_formation::directionalClearanceRisk()` and `multi_uav_formation::trackingCameraFovDirectionalRisk()` in `multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h:75` and `:246`; `TrackingCameraContract` in the same header. Cost form: `J_vis = weight_visibility_·(R_static² + max_dynamic R_id² + R_fov²)`, gradient written into **P and T** (`gdC`, `gdT`, and previous-piece `gdT`).
- **(iv) Status:** **Production-ACTIVE** (flag default `true` when encirclement is on, `weight_visibility=20.0`), but note it only fires when encirclement visibility guidance is active — in the 09-18 production run the log shows `VISIBILITY_DIRECTIONAL_COST_ACTIVATIONS=0` on the sampled lines, because that run is `team-vis-opt-config … scene_status=DISABLED` (encirclement off) for the `long_cylinder_forest.json` scene.

### B-2. **09-09 → 09-10 — Clearance continuation: deep-occlusion escape gradient**

- **(i) Problem:** A smoothstep risk saturates at 1.0 inside deep occlusion, so the gradient vanishes exactly where escape is needed.
- **(ii) Date:** 09-09 → 09-10 `[mtime: tracking_visibility_geometry.h 2026-09-11 12:56 — batch mtime, treat as upper bound]`.
- **(iii) File / identifiers:** `multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h:75-105`, `directionalClearanceRisk(c, c_in, c_out, κ, β)`; launch args `directional_visibility_deep_risk_kappa` default `1.0` (`native_egov2_rviz.launch:37`, `advanced_param.xml:82`) and `directional_visibility_deep_risk_beta` default `2.0` (`:38`, `:83`). For `c ≤ c_in`: `R = 1 + κ[d − (1−e^{−βd})/β]`, `dR/dc = −κ(1−e^{−βd})/w`.
- **(iv) Status:** **Production-ACTIVE** as part of B-1's risk function.

### B-3. **09-09 — `TrackingCameraContract` / `trackingCameraFovRisk` / `trackingCameraFovDirectionalRisk`**

- **(i) Problem:** FOV was previously an execution-side/telemetry notion; it needed to become a differentiable objective input, with the camera extrinsics captured as an explicit contract (`horizontal_fov`, `vertical_fov`, `min_range`, `max_range`, `body_from_camera`).
- **(ii) Date:** 09-09 → 09-10 (`feedback_23` Stage 3A).
- **(iii) File / identifiers:** `tracking_visibility_geometry.h:214 trackingCameraFovRisk()`, `:246 trackingCameraFovDirectionalRisk()`, `struct TrackingCameraContract` (top of same header); consumers `poly_traj_optimizer.cpp:5147`, `multi_uav_topology_coordinator.cpp:913`, tests `traj_opt/test/elastic_visibility_contract_test.cpp`, `multi_uav_formation/test/multiview_contract_test.cpp`.
- **(iv) Status:** **Production-ACTIVE**; the contract-test target `elastic_visibility_contract_test` exists and is built (`devel/.private/traj_opt/lib/traj_opt/elastic_visibility_contract_test`).

### B-4. **09-09/09-10 — Stage 2/3A/3B: joint `{P, τ, Ψ}` team optimizer, K2-continuity, blackout, 3-ACK commit**

This is the single largest post-09-08 addition and is documented in the log's own words.

- **(i) Problems:** (Stage 2) K2 is high instantaneously but intermittent, and full blackout is unpenalised → add windowed continuity/blackout surrogates. (Stage 3A) yaw/FOV were evaluated once at candidate time rather than re-predicted per P/T. (Stage 3B) each UAV optimising alone ≠ team optimum → joint yaw decision variables.
- **(ii) Date/trigger:** in-document date **2026-09-09**, `feedback/feedback_23.md`:
  > *"报告日期：2026-09-09（Asia/Shanghai）"*
  > *"当前真实源码已经完成 Stage 2、Stage 3A、Stage 3B 的生产实现，并通过构建、梯度和 contract tests。"*
  The replicated copy at the repo root (`feedback_17.md`, mtime 09-10 00:38) is byte-identical in its header — do not mistake it for an 09-04 document.
- **(iii) Files / identifiers:**
  - `multi_uav_formation/src/team_visibility_optimizer.cpp` — `TeamVisibilityOptimizer::slidingWindowSquaredCost()` (`:519`), `TeamVisibilityOptimizer::blackout()` (`:504`), `blackoutGradient()` (`:511`); `metrics.k2_continuity_cost = slidingWindowSquaredCost(b_k2, width, …)` (`:989`), `metrics.blackout_cost = slidingWindowSquaredCost(b_blackout, blackout_width, …)` (`:992`); `width = max(1, round(window/dt) + 1)` (`:987`).
  - Goal terms: `Q2 = v1v2+v1v3+v2v3−2v1v2v3`, `b_K2 = 1−Q2`, `b_0 = (1−v1)(1−v2)(1−v3)`, `J_K2`, `J_acc`, `J_K2-cont`, `J_blackout` — `feedback_23` lines 27-39, and `feedback/ACTIVATION_LIFECYCLE_REPORT_20260910.md`.
  - `multi_uav_formation/include/multi_uav_formation/team_visibility_optimizer.h` `[mtime 2026-09-16 21:25]` and `src/team_visibility_optimizer.cpp` `[mtime 2026-09-17 22:08]` — **mtime is a late edit, not creation**; the file was already described as production source on 09-10.
  - Joint solver: `traj_opt/include/optimizer/scp_optimizer.h` + `src/scp_optimizer.cpp` `[mtime 2026-09-11 00:16]`, `SCPOptimizer::solve(...)`, `max_iterations = 1`, step backtracking `{1.0, 0.5, 0.25, 0.10}`.
  - Config flags: `enable_team_visibility_optimizer` (`multi_uav_topology_coordinator.cpp:287`), `enable_joint_pt_optimization` (`:289`), `enable_joint_yaw_optimization`, `enable_team_k2_continuity_cost` (`:294`), `enable_team_blackout_cost` (`:296`), `team_k2_continuity_weight` (`:311`), `team_blackout_weight` (`:325`), gated at `:377-380`: `team_optimizer_params_.k2_continuity_weight = enable_team_k2_continuity_cost_ ? team_k2_continuity_weight_ : 0.0`.
  - `feedback_23` also records the **only** pre-09-09 file that needed a fix for this: `planner_manager.cpp` — *"现在当 `enable_team_visibility_optimizer=true` 时，将 `team_solution_max_age` 作为 proposal grace 纳入 planner 的有效协调预算"*, fixing the observed `NO_PENDING_TOPOLOGY_GENERATION` rejection.
- **(iv) Status — this needs both halves stated:**
  - The code is **source-confirmed and contract-tested**: `feedback_23` final claims block reads *"STAGE2_IMPLEMENTED: YES / STAGE2_SHORT_TEST_PASS: NO / NOT VERIFIED"*, *"STAGE3B_IMPLEMENTED: YES / STAGE3B_SHORT_TEST_PASS: NO / NOT VERIFIED"*, *"OPTIMIZED_YAW_ACTUALLY_EXECUTED: NOT VERIFIED"*.
  - In the current production configuration it is **flag-OFF**. Verified directly from the 09-18 production log `sim_run_round2.log`:
    `[team-vis-opt-config] enabled=0 joint_pt=0 runtime_ready=0 … stage2_k2_continuity=0 stage2_blackout=0 … stage3a_yaw_reprediction=0 stage3b_joint_yaw=0 yaw_trust=0.000 fallback=legacy_selector`
    `[team-vis-opt-planner-config] drone=0 enabled=0` (also drones 1, 2)
    and `multi_uav_topology_coordinator.cpp:450`: `requested_team_optimizer = enable_team_visibility_optimizer_ && enable_joint_pt_optimization_;`.
  - Subtlety worth recording: the raw `team_blackout_weight` is `0.05` (`:325`) but the *effective* weight printed into `weights_k2_acc_k2cont_blackout_dev_jerk=…,0.000000,0.000000,…` is **0** because of the `:377-380` gate. The `blackout_weight=0.050000` field in the config line is the *raw* value and must not be read as "blackout is active".

### B-5. **09-10 — Scenario A OFF/ON measurement exposes that the visibility gain is a redistribution, and the structural audit fixes the boundary**

- **(i)/(ii):** `feedback_24.md` (`实验日期：2026-09-10`) reports the ON group's `all3` falling 0.923 → 0.688 with camera-time −18 camera·s; `feedback_25.md` (`审计日期：2026-09-10`) roots it in **horizontal FOV + static LOS**, not blackout, and mentions `slidingWindowSquaredCost`; `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md` (*"日期：2026-09-10"*) is the read-only pre-change structural audit that enumerates the 19 subsystems.
- **(iii):** `feedback_25.md`, `ALP_VISIBILITY_FIRST_PRE_CHANGE_REPORT_20260910.md`.
- **(iv):** diagnostic/audit only.

### B-6. **09-10 — `AdaptiveViewpointGenerator`: the 120° reference is demoted to a seed**

- **(i) Problem:** the equiangular 120° assumption is false in occluded scenes; per-UAV bearings must be searched independently, with a diversity term so independent searches do not collapse onto the same bearing.
- **(ii) Date:** 09-10 (`feedback_26.md`, `VISIBILITY_FIRST_ADAPTIVE_ENCIRCLEMENT_REPORT_20260910.md`, both mtime 2026-09-10 19:11). `feedback_26` opens: *"本轮完成新的 adaptive hypothesis 实际执行闭环。"*
- **(iii) Files / identifiers:** `multi_uav_formation/include/multi_uav_formation/adaptive_viewpoint_generator.h` and `src/adaptive_viewpoint_generator.cpp` `[mtime 2026-09-13 02:49 — upper bound only]`; `src/cooperative_viewpoint_manager.cpp` `[mtime 2026-09-13 02:50]`; three seeds (fixed / uniform-120° / previous-accepted), per-UAV bearing search, two scales near 1.5 s / far 3.5 s, radius ±0.35, height ±0.20; `J_div = Σ_{i<j} max(0, θ_min − |wrap(θ_i−θ_j)|)²`; `angularDiversityCost`. Test targets `adaptive_viewpoint_generator_contract_test`, `cooperative_viewpoint_contract_test`.
- **(iv):** production-capable, gated by `enable_cooperative_viewpoint_reference`; `feedback_26` records the first full proposal→3ACK→commit→adoption→optimized-yaw closure but also `SAFETY_REGRESSION_OBSERVED=YES`.

### B-7. **09-10 — `activation_schedule.h`: stage-latency activation lead**

- **(i) Problem:** a fixed 240 ms activation lead fails as soon as a solve takes 310 ms.
- **(ii) Date:** 09-10 (`feedbacack/ACTIVATION_LIFECYCLE_REPORT_20260910.md`).
- **(iii) File / identifiers:** `multi_uav_formation/include/multi_uav_formation/activation_schedule.h` → `class PipelineLatencyEstimate` with `budget() { … return std::max(0.020, 1.25 * peak); }`, 32-entry window, 3-observation cold start (`cold_start_ = 0.35`); `struct ActivationSchedule { valid, activation, lead, required_margin }`; `activationReserveAvailable(...)`. `[mtime 2026-09-11 12:56 — upper bound]`
- **(iv):** production-active; `feedback_23`'s joint attempt log shows `activation_lead=0.240` still in use at that stage.

### B-8. **09-10 — trajectory lifecycle fix: rolling update and cross-generation P/V/A handoff**

- **(i) Problem:** `feedback_29.md` (*"ALP trajectory lifecycle 只读审计 — 2026-09-10"*) proves the stop-and-go has a real execution cause: *"候选生成很快，但 coordinator pending 将普通重规划阻塞约 1 秒；原始 head 随后过时，被拒绝后再次等待…traj_server 真正进入零速度/零加速度 terminal hold"*.
- **(ii)/(iii):** fixed in `feedback_30.md` (09-10). `traj_utils/include/traj_utils/trajectory_lifecycle.h` `[mtime 2026-09-13 14:23]`; `local_execution_contract.h` `[mtime 2026-09-13 14:44]`; handoff gate `p/v/a_new(0) = p/v/a_old(t_handoff)`. Reported result: *"754 次接管、零轨迹耗尽等待、零 terminal hold"*, but `feedback_30` still refuses full acceptance: *"整体安全验收仍未通过"*.
- **(iv):** production-active.

### B-9. **09-10 — encirclement criterion becomes a circular-gap interval; `multiviewQuality`**

- **(i) Problem:** 120° equiangularity is neither necessary nor sufficient; an executable interval definition was needed.
- **(ii)/(iii):** `feedback_31.md` (09-10) implements the max-empty-sector constraint with `SAME_SEMICIRCLE_RATIO=33.10%` / `EXECUTED_ENCIRCLEMENT_RATIO=42.65%`; `feedback_41.md` (09-12) adds `multiviewQuality` (`bad=25°`, `good=60°`, smoothstep `u²(3−2u)`), `Q_dir`, `J_dir` and deletes the planner-level brake; `feedback_31,34,41` land the 25°/170° gap. Code: `multi_uav_formation/include/multi_uav_formation/encirclement_geometry.h` → `struct MultiviewQuality {bad=25°, good=60°}`, `multiviewQuality()`, `circularGapGeometry()` (`:83`), `encirclementGapCost()` (max-gap only, *"There is no bearing/120-degree prior anywhere in this cost"*), `encirclementGeometryCost()` (`:128`, both bounds, *"no equiangular reward"*). Test `multiview_contract_test`, `encirclement_geometry_contract_test`. `[mtime 2026-09-12 01:41 → 09-13 02:53]`
- **(iv):** the ranking-side quantities are production-active; the `J_dir` cost term rides on the joint optimizer and is therefore flag-off.

### B-10. **09-11 — Persistent Team Recovery Target + planner-backed reachability + validated recovery guide**

- **(i) Problem:** a recovery target that merely *exists* is useless; it must be reachable by the real three-UAV production planning chain, and it must not become a blocking condition for the rolling planner.
- **(ii) Date:** `feedback_34.md` (09-11, persistent team recovery target, actual encirclement 40.65% → 55.44%), `feedback_35.md` (09-11, planner-backed reachability → validated guide), `feedback_36.md` (09-11, Target → Guide → Trajectory reduction; *"任何恢复失败都不阻塞飞行"*), `feedback_38.md` (09-11, persistent common-time TEAM_REFERENCE).
- **(iii) Files / identifiers:** `multi_uav_formation/include/multi_uav_formation/team_recovery_target.h` (`struct RecoveryTargetPolicy`, `RecoveryTargetCandidate`, `RecoveryGuide`, `TeamRecoveryTarget`, `ReachableTargetReference`, `readRecoveryTarget`) `[mtime 2026-09-13 03:05]`; `plan_manage/src/team_target_reachability.cpp` `[mtime 2026-09-13 14:36]`; `plan_manage/include/plan_manage/target_guide_lifecycle.h` `[mtime 2026-09-13 14:36]`; msgs **`RecoveryGuide.msg` and `RecoveryTargetProposals.msg` mtime 2026-09-11 10:00**, **`TeamRecoveryTarget.msg` mtime 2026-09-11 12:57** — generated headers `devel/include/traj_utils/RecoveryGuide.h` **2026-09-11 10:06**, `TeamRecoveryTarget.h` **2026-09-11 07:57**; `team_trajectory_reference.h` `[mtime 2026-09-12 00:13]`; `multi_uav_formation/include/multi_uav_formation/encirclement_recovery.h` `[mtime 2026-09-11 13:02]`; `include/plan_manage/recovery_probe_contract.h`, `local_sfc_boundary_scan.h` `[mtime 2026-09-11 12:56]`. **All paths in this bullet were confirmed present on disk.**
- **(iv):** production-active; tests `persistent_recovery_target_contract_test`, `recovery_probe_production_test`.

### B-11. **09-12 — Candidate pipeline unification and trigger de-empiricisation**

- **(i) Problem:** candidates were being pruned on early `ABSOLUTE_SAFE/IMPROVED_ONLY` labels instead of a full current-revision check; and the visibility trigger was empirical rather than reusing the existing continuous risk support.
- **(ii)/(iii):** `feedback_43.md` (*"Candidate pipeline unification — 2026-09-12"*): *"`finalizeCapturedCandidates` 不再依据早期 `ABSOLUTE_SAFE/IMPROVED_ONLY` 标签剪枝"*; `feedback_44.md` (*"Visibility-trigger verification and risk-support update — 2026-09-12"*): *"将经验性的 visibility trigger 改为复用现有连续 risk support"*; `feedback_47/48.md` (09-12) close the recovery production-candidate fixture.
- **(iv):** production-active. No simulation was run for these two turns (stated in-file).

### B-12. **09-12/09-13 — validated coverage / execution budget, two-phase supply**

- **(i) Problem:** planning had no notion of "how much valid coverage is left", so quality search starved the successor.
- **(ii)/(iii):** `feedback_52.md` (09-12): `validated_end = min(start+duration, max(now, execution_safe_until_))`, `activationEarliest()=now+0.10`, `planningDeadline()=validated_end−0.10`, `ExecutionPlanningBudget` (1.10/0.20/2.50, factor 1.35, half-life 60 s). `feedback_53.md`/`feedback_55.md` (09-13): two-phase supply (`FIRST_SAFE_RESERVED`), post-deadline fresh-state recovery. `feedback_56.md` (09-13) isolates a successor regression between 53 and 54/55.
- **(iv):** production-active.

### B-13. **09-13/09-14 — Early Joint, Unified Rolling Team Planning Transaction, Committed Prefix, atomic adoption**

- **(i) Problems, in chain order:** Joint always arrived one step late → feed the local candidate's own seed directly (Early Joint). Then Joint could still rewrite an already-executed prefix → committed-prefix CAS. Then the three UAVs did not switch at the same instant → 3-ACK + atomic adoption.
- **(ii)/(iii):**
  - `feedback_57.md` (09-13) Early Joint → `multi_uav_formation/include/multi_uav_formation/team_planning_context.h` `[mtime 2026-09-13 20:46]`, test `early_joint_primary_contract_test`.
  - `feedback_58.md` (09-13) Unified Rolling Team Planning Transaction → `multi_uav_formation/include/multi_uav_formation/unified_team_transaction.h` `[mtime 2026-09-13 19:19]`, msg `TeamTrajectorySolution.msg` **mtime 2026-09-14 02:32**.
  - `feedback_59.md` (09-14) Committed Prefix + Background Joint Future Tail → `multi_uav_formation/include/multi_uav_formation/committed_prefix_joint_contract.h` `[mtime 2026-09-13 20:59]` with `discoverCommittedFutureFrontier`, `committedFrontierStillCurrent` (CAS on `owner_revision`/`traj_id`/`committed_start`), `reheadCommittedFutureTailSeed`.
  - `feedback_61.md` (09-14) atomic joint adoption / validated-prefix authority → `multi_uav_formation/include/multi_uav_formation/team_solution_commit_contract.h` `[mtime 2026-09-14 18:47]`, test `joint_adoption_atomic_contract_test` `[mtime 2026-09-14 02:44]`.
  - **The 3-ACK message itself dates to 09-09:** `traj_utils/msg/TeamTrajectoryAck.msg` **mtime 2026-09-09 18:50**, generated header `devel/include/traj_utils/TeamTrajectoryAck.h` **2026-09-09 19:04**.
  - `feedback_60.md` (09-14) is the honest negative result: `TRANSACTION_JOINT_REJECTED=287 / SUCCESS=0`.
- **(iv):** contract-complete, **production flag-off** (`team-solution-commit` count 0 in the production run; `[team-vis-opt-planner-config] enabled=0`).

### B-14. **09-14 → 09-17 — Unified conflict representation and LOS observation-side topology**

- **(i) Problem:** LOS had only blocked/not-blocked, no blocker identity, so a LOS event could not become a trackable topological intent; and a BODY conflict could silently delete a LOS observation plane.
- **(ii)/(iii):**
  - `feedback_62.md` (09-14) read-only design audit of static occlusion / unified observation topology.
  - `feedback_63.md` (09-14) Unified BODY/LOS Conflict-Driven N/L/R implementation → `plan_manage/include/plan_manage/planner_manager.h:40-91` `enum ConflictReasonMask { CONFLICT_NONE, CONFLICT_BODY_SAFETY, CONFLICT_LOS_OCCLUSION }`, `enum class ConflictObstacleMotion`, `struct ConflictDescriptor` (separate BODY and LOS evidence channels); `LOS_OBSERVATION_SIDE` as a `LocalSfcPlane.source`, alongside `STATIC_COLLISION_CORRIDOR`.
  - `plan_env/include/plan_env/static_los_geometry.h` gains `struct StaticLosWitness` (`:40`) and the witness overload `querySegmentClearance(observer, target, clearance, StaticLosWitness *witness = nullptr)` (`:188-194`) `[mtime 2026-09-14 17:09]`. **Note: the `StaticLosGeometry` class and the basic `querySegmentClearance` are baseline (item A.2 #12); the witness is the new part.**
  - `feedback_79.md` (09-16) decouples LOS from BODY so BODY+LOS and LOS-only both build an observation plane.
  - `feedback_81.md` (09-17) adds independent raw-LOS truth sampling and forbids BODY-only conflicts from entering the visibility/topology selector.
  - `feedback_65.md` (09-14) and `feedback_68.md`/`feedback_71.md`/`feedback_76.md` (09-15) are read-only audits (dynamic candidate rejection/successor starvation, braking root cause, stop/spin/sudden-acceleration, hold-cause).
- **(iv):** production-active (witness layer, N/L/R loop, plane validation); the SCP half-space row is flag-off (`enable_candidate_hard_corridor_scp=false`).

### B-15. **09-15 — dynamic BODY hard clearance separated from the 1.1 m preference margin; SCP dynamic hard row**

- **(i) Problem:** the 1.1 m preference clearance was being used as a physical execution gate (over-conservative), and the static inflation `obstacles_inflation=0.099 m` had been wrongly reused as the dynamic body radius.
- **(ii)/(iii):** `feedback_72.md` (*"Feedback 072 — Rolling dynamic authority / hard-clearance update"*, 09-15) and `feedback_70.md` (*"Feedback 070 — Local dynamic repair wiring and hard-SCP verification"*, 09-15, `fixjng` the SCP wiring).
- **(iv):** **split status — this is the most important nuance in the whole audit.**
  - The **hard-clearance number is production-active as an ex-post rejection gate**: `poly_traj_optimizer.cpp:7616-7629 getMovingObjHardClearance() = moving_obj_hard_body_radius_(0.384) + max_id(0.5·max(scale.x, scale.y))` (`nh.param("optimization/moving_obj_hard_body_radius", …, 0.384)` at `:7500-7501`), consumed by `validateExecutionTrajectory → HARD_DYNAMIC_COLLISION_FAIL` and `classify_candidate → risk.hard_collision → INVALID`.
  - The **SCP hard row itself is production-DEAD**: `enable_candidate_hard_corridor_scp` defaults `false` (`advanced_param.xml:111`, `run_in_sim.launch:47`, `native_egov2_rviz.launch:253`, and `run_constvel_gradient_rviz.sh:37 NATIVE_EGOV2_HARD_CORRIDOR_SCP:-false`); the production log prints `[candidate-optimizer-config] hard_corridor_scp=0`.

### B-16. **09-15 — NOMINAL semantics restoration, warm-start tightening, duplicate publication, traj_server queue**

- **(i) Problem:** cooperative viewpoint / SIDE / guide geometry was contaminating the NOMINAL branch; the NOMINAL warm-start cache was too permissive; trajectories were published out of order/duplicated; `traj_server` compared future handoffs against the wrong wall time.
- **(ii)/(iii):** `feedback_73.md` (09-15) N/L/R semantic restoration; `feedback_74.md` (09-15) NOMINAL target isolation + `visibility_min_target_distance` zero-progress guard (`DROP_ALTERNATIVE`); `feedback_75.md` (09-15) publication de-duplication; `feedback_77.md` (09-15) `traj_server::executionHandoffGate()` fix; `feedback_78.md` (09-15) `NOMINAL_WARM_START_OVERFIX_EXISTED: YES`.
- **(iv):** production-active. `feedback_74` reports the invariant `NOMINAL_TARGET_OVERWRITTEN_COUNT: 0`.

### B-17. **09-16 — `active_execution_touch_goal_`: a rolling trajectory must not be re-validated as mission-terminal**

- **(i) Problem:** `lifecycleSuccessorDue()` always upgraded every re-validation to mission-terminal, so suffix conflicts erased already-certified rolling prefixes and cascaded into successor starvation.
- **(ii)/(iii):** `feedback_80.md` (09-16); `planner_manager.cpp` writes `active_execution_touch_goal_` and reads it to choose rolling vs mission-terminal.
- **(iv):** production-active.

### B-18. **09-17 — Swarm mutual non-worsening unlock**

- **(i) Problem documented in `codex_round4_prompt.md`:** once two UAVs are already inside each other's safety bubble (measured minimum separation 0.0079 m vs a 0.58 m threshold), *every* candidate is hard-rejected → self-lock → 11 `TERMINAL_HOLD`.
- **(ii)/(iii):** `codex_round4_prompt.md` `[mtime 2026-09-17 23:47]` → `planner_manager.cpp:5040-5135`: keep the absolute threshold `d² = Δx² + Δy² + 0.25·Δz²` (clearance 0.5 m), but when a sample is already violating, admit the candidate iff `d²_cand ≥ d²_current − 1e-9`.
- **(iv):** production-active.
- Also 09-17: `multi_uav_topology_coordinator.cpp` / `traj_server.cpp` edits `[mtime 2026-09-17 22:08 / 23:06]` and the `feedback_81..84` LOS/authority rounds.

---

## B-tail. The 2026-09-18 extension (beyond the stated 09-09…09-17 window, but still after the baseline)

The chronological log **stops at `feedback_84.md`** (*"Feedback 084 — … No production code was modified"*, mtime 2026-09-17 20:49). The 09-18 work exists only as root reports and source mtimes:

| Time | Item | Evidence |
|---|---|---|
| 09-18 00:42 | `poly_traj_utils.hpp` empty-trajectory/UB hardening `[mtime 2026-09-18 00:42]` | `ROUND3_NEW_MECHANISMS_AUDIT.md` §L-3; content confirmed, attribution inferred |
| 09-18 12:33 | soft `J_vis` stripped of discrete topology authority (`NO_TOPOLOGY_AUTHORITY`) → SIDE solves 1499→7, `SIDE_DEV_GT_1M` 137→3 | `FIX4_STOP_AND_GO_REPORT.md` `[mtime 09-18 14:04]`, `planner_manager.cpp:6538-6566` |
| 09-18 12:33 | `kMaxReplanAttemptsPerWindow = 2` in `ego_replan_fsm.cpp:486-522` | `codex_round_prompt.md` `[mtime 09-18 21:43?→09-17 21:43]`; audit §L-2 |
| 09-18 14:04 | rolling time allocation → `total_time = max(0.10, 1.05·d/v_ref)·scale` | `FIX4_STOP_AND_GO_REPORT.md` |
| 09-18 14:04 | prefix progress soft cost, **default weight 0.0 (off)** | `poly_traj_optimizer.h:501 setPrefixProgressCost(...)`, `:403-408`; `FIX4_STOP_AND_GO_REPORT.md`; production log `[prefix-progress-setup] drone=0 span=0.250000 v_ref=1.650000 w_base=0.000000` |
| 09-18 15:13 | LOS observation plane active interval = real occlusion interval (replacing the BODY `±0.45 s` point-event window) | `LOS_OCCLUSION_ROOT_CAUSE.md` `[mtime 09-18 15:13]` |
| 09-18 16:33 | root cause located: encirclement tracking cost has **zero tangential gradient**; `H_eval` fixed 1.5 s vs ~0.73 s actual coverage | `AUDIT_BLUE_RED_YELLOW_ROOT_CAUSE.md` `[mtime 09-18 16:33]` |
| 09-18 18:26 | `H_eval = min(H_configured, common validated prefix)`, floor 0.20 s | `topology_coordinator_core.{h,cpp}` `[mtime 09-18 18:26]` |
| 09-18 18:45 | **soft bearing recovery** (`encirclement_bearing_recovery_weight = 0.08`, prefix boost ×2, `elasticAngularSlack(ρ) = 20° + 25°ρ`) + Local circular-gap cost (`local_gap_weight = 100`) | `poly_traj_optimizer.{h,cpp}` `[mtime 09-18 18:24 / 18:45]`, `planner_manager.{h,cpp}` `[mtime 09-18 18:50]`; production log `[bearing-recovery-setup] drone=0 w_bearing=0.080000 prefix_boost=2.000 prefix_span=0.250000 sample_dt=0.100000` (183 occurrences in the surviving production log) |

---

## C. Per-mechanism BEFORE / AFTER verdicts

Legend: **BEFORE** = existed by end of 09-08 · **AFTER** = added 09-09 or later · **SPLIT** = part before, part after.

| # | Mechanism | Verdict | Evidence |
|---|---|---|---|
| **1** | `ElasticRiskState` / `elasticAngularSlack` / `angularDeadbandViolation` / `elastic_radial_slack_max` / `elastic_delta_phi_*` | **SPLIT** | `elastic_radial_slack_max` and `elastic_delta_phi_min_deg`/`_max_deg` are **BEFORE** — they are launch arguments of every 09-08 run and appear in `cooperative_benchmark_validation_20260905/` (`elastic_delta_phi_max_deg`, `elastic_radial_slack_max`) and `alp_feature_validation_20260905/`. The **header `elastic_tracking_contract.h` (`ElasticRiskState` is a member struct of `PolyTrajOptimizer`, `poly_traj_optimizer.h:850`) is AFTER**: the file is untracked/`??`, mtime **2026-09-09 02:11**, and the helper names do not appear in any 09-08 evidence. `angularDeadbandViolation` appears only in `FIVE_FIXES_REPORT.md` (09-18) and `elasticAngularSlack` only from `feedback_25.md` (09-10) onward. |
| **2** | `directionalVisibilityGradCostP` / `addDirectionalVisibilityGradCost2CT` / `directionalClearanceRisk` / `weight_visibility` | **AFTER** | `weight_visibility` is the **only** launch-arg key that appears in 09-09 and not in 09-08 (`grep "weight_visibility\|directional" encirclement_*_20260908/` → 0 hits); it is a 09-09 argument in `encirclement_directional_visibility_20260909/run_single.py`. Code: `poly_traj_optimizer.cpp:4992, :5186, :7519`; `tracking_visibility_geometry.h:75`. **Honest caveat:** this is *parameterisation* evidence plus the 09-10 pre-change report treating it as "current source"; it is not a direct "created on 09-09" commit record. |
| **3** | `directional_visibility_deep_risk_kappa` / `_beta` (clearance continuation) | **AFTER** | Launch args registered at `native_egov2_rviz.launch:37-38` / `advanced_param.xml:82-83`; no such keys in any 09-08 run; `feedback_23` (09-09) is the first to describe the deep-risk continuation. |
| **4** | `trackingCameraFovDirectionalRisk` / `trackingCameraFovRisk` / `TrackingCameraContract` | **AFTER** | `tracking_visibility_geometry.h:214, :246` + `struct TrackingCameraContract`; introduced with the same 09-09 `feedback_23` Stage 3A yaw/FOV re-prediction work. No 09-08 evidence. |
| **5** | K2 / `Q2` / `J_K2-cont` / `J_blackout` / `slidingWindowSquaredCost` | **AFTER** | `feedback_23.md` (in-document date **2026-09-09**) header: *"Stage 2：K2 continuity 与 blackout"*, with `Q2 = v1*v2+v1*v3+v2*v3 - 2*v1*v2*v3`. Code `team_visibility_optimizer.cpp:504 blackout()`, `:519 slidingWindowSquaredCost()`, `:989/:992` window widths `round(window/dt)+1`. **Which "Stage 2" the reports mean:** the K2-continuity + blackout objective stage of the joint team optimizer — *not* the 09-02 "three-class" work and *not* `atleast2`/K-of-N. `feedback_23` also names Stage 3A (yaw/FOV re-prediction) and Stage 3B (joint yaw). |
| **6** | Joint `{P, τ, yaw}` team optimizer + SCP/OSQP | **SPLIT** | **SCP/OSQP as a solver is BEFORE** — `ALP_SFC_MINCO_SCP_SOURCE_CHAIN_AUDIT_20260902.txt`: *"ALP Native Local-SFC -> MINCO -> SCP -> OSQP Source Audit, Date: 2026-09-02"*. **`TeamVisibilityOptimizer` and the `{P_i, τ_i, Ψ_i}` joint decision layout are AFTER** — `feedback_23` (09-09) *"Stage 3B：joint yaw optimization"*, variable layout `X = { P_i, tau_i, Psi_i }`, with `scp_optimizer.{h,cpp}` `[mtime 2026-09-11 00:16]`. Production status: **flag-off**. |
| **7** | Common activation / 3-ACK / atomic commit (`team_solution_commit_contract.h`, `unified_team_transaction.h`) | **AFTER** | `TeamTrajectoryAck.msg` `.msg` mtime **2026-09-09 18:50** and generated `devel/include/traj_utils/TeamTrajectoryAck.h` **2026-09-09 19:04** — a message generated on 09-09 did not exist on 09-08. `team_solution_commit_contract.h` `[mtime 2026-09-14 18:47]`, `unified_team_transaction.h` `[mtime 2026-09-13 19:19]`, `joint_adoption_atomic_contract_test.cpp` `[mtime 2026-09-14 02:44]`. Production status: **flag-off**. |
| **8** | Cooperative viewpoint hypotheses / `EncirclementReferenceBundle` / `EncirclementReferenceHypothesis.msg` / adaptive viewpoint generator | **SPLIT** | The **fixed 120° slot generator, hypothesis bundle and `.msg` types are BEFORE**: `.msg` mtimes **2026-09-08 19:41 / 21:05**, generated headers **2026-09-08 19:55**; flags `enable_cooperative_viewpoint_reference:=true`, `viewpoint_independent_mode:=true` in all 09-08 runs. The **`AdaptiveViewpointGenerator`** (fixed / uniform-120° / previous-accepted seeds, per-UAV bearing search, two scales 1.5 s/3.5 s, radius ±0.35, height ±0.20) is **AFTER** (09-10 design, 09-13 code mtime). `cooperative_viewpoint_core.h` mtime 09-11 12:56 is an upper bound only — the core was already referenced by the 09-10 structural report at line 455/1043. |
| **9** | Team recovery target / persistent recovery target / `team_target_reachability.cpp` / `RecoveryGuide.msg` | **AFTER** | `RecoveryGuide.msg` + `RecoveryTargetProposals.msg` mtime **2026-09-11 10:00**, `TeamRecoveryTarget.msg` **2026-09-11 12:57**; generated headers **2026-09-11 10:06 / 07:57**. Work reported in `feedback_34/35/36/38.md` (09-11). |
| **10** | `multiviewQuality` / `Q_dir` / `J_dir` / `angularDiversityCost` / 25°–170° circular gap (`encirclement_geometry.h`) | **AFTER** | `encirclement_geometry.h` `[mtime 2026-09-12 01:41]`; `feedback_41.md` (09-12) is the first to describe `multiviewQuality` (`bad=25°/good=60°`, smoothstep); the 25°/170° gap appears in `feedback_31/34/41` (09-10 → 09-12). No such file/key in any 09-08 artefact. |
| **11** | `activation_schedule.h` (stage-latency activation lead) | **AFTER** | File is untracked/new `[mtime 2026-09-11 12:56 — upper bound]`; first described in `feedback/ACTIVATION_LIFECYCLE_REPORT_20260910.md` (09-10) and `feedback_26.md` (09-10): `B_stage = max(20 ms, 1.25·max_recent(stage latency))`. Code confirms `PipelineLatencyEstimate::budget()` → `std::max(0.020, 1.25 * peak)`. |
| **12** | `local_execution_contract.h` / `trajectory_lifecycle.h` / `committed_prefix_joint_contract.h` / `target_guide_lifecycle.h` | **AFTER** | All four are untracked/new files. mtimes: `trajectory_lifecycle.h` **2026-09-13 14:23**, `local_execution_contract.h` **2026-09-13 14:44**, `committed_prefix_joint_contract.h` **2026-09-13 20:59**, `target_guide_lifecycle.h` **2026-09-13 14:36**. First described in `feedback_29/30` (09-10), `feedback_36` (09-11), `feedback_59/61` (09-13/14). |
| **13** | `StaticLosGeometry` / `static_los_geometry.h` / LOS witness / `LOS_OBSERVATION_SIDE` | **SPLIT** | `StaticLosGeometry` + the basic `querySegmentClearance` are **BEFORE**: `feedback_10.md` (**日期：2026-09-03**) — *"评价使用 Feedback 9 的 authoritative solid-cylinder helper：`plan_env/static_los_geometry.h`"*; `feedback_62/63` (09-14) still list it. The **`StaticLosWitness` struct and the witness-returning overload** (`static_los_geometry.h:40, :188-194`) `[mtime 2026-09-14 17:09]`, and **`LOS_OBSERVATION_SIDE`** as a plane source, are **AFTER** (09-14 → 09-17, `feedback_63/79/81`). |
| **14** | Dynamic-obstacle hard clearance / `moving_obj_hard_body_radius` / SCP dynamic row | **AFTER** | `poly_traj_optimizer.cpp:7500` `nh.param("optimization/moving_obj_hard_body_radius", …, 0.384)`, `:7616 getMovingObjHardClearance()`, SCP row `:2424-2499`; introduced by `feedback_70/72.md` (09-15). **Status split:** active as an ex-post rejection gate; the SCP row is `enable_candidate_hard_corridor_scp=false` → dead in production (`[candidate-optimizer-config] hard_corridor_scp=0`). |
| **15** | Bearing recovery (`encirclement_bearing_recovery_weight`) and prefix progress (`setPrefixProgressCost`) | **AFTER** | Both land 09-18, i.e. **after the feedback log ends**. Launch defaults `native_egov2_rviz.launch:27-28`: `prefix_progress_weight default="0.0"`, `encirclement_bearing_recovery_weight default="0.08"`. Header `poly_traj_optimizer.h:313-316` (`encirclement_bearing_recovery_weight_{0.30}` — note the **header default 0.30 is overridden to 0.08 by the launch param**), `:501 setPrefixProgressCost(...)`. Production log: `[bearing-recovery-setup] … w_bearing=0.080000 prefix_boost=2.000` (**active**) and `[prefix-progress-setup] … w_base=0.000000` (**disabled**). |

---

## D. Production configuration actually in force (verified from `sim_run_round2.log`, 09-18)

```
[team-vis-opt-config]              enabled=0 joint_pt=0 runtime_ready=0
                                   stage2_k2_continuity=0 stage2_blackout=0
                                   stage3a_yaw_reprediction=0 stage3b_joint_yaw=0 yaw_trust=0.000
                                   fallback=legacy_selector
[team-vis-opt-planner-config]      drone=0/1/2 enabled=0
[topology-coordination-planner-config] drone=0/1/2 enabled=1 timeout=0.200 result_max_age=0.350
[team-visibility-config]           enabled=1 N=3 K=2 preferred_separation_deg=25.000 … diversity_weight=0.150
[candidate-optimizer-config]       hard_corridor_scp=0 radius=0.500 …
[candidate-visibility-config]      enabled=1 …
[elastic-tracking-config]          enabled=0 base_weight=100.000000 …
[minco-visibility-config]          enabled=0 lambda=150.000000 … weight_visibility=20.000000 …
[astar-visibility-cost-config]     enabled=0 …
[bearing-recovery-setup]           drone=0 w_bearing=0.080000 prefix_boost=2.000 prefix_span=0.250000
[prefix-progress-setup]            drone=0 span=0.250000 v_ref=1.650000 w_base=0.000000
run_round2_full_on.sh:             NATIVE_EGOV2_ELASTIC_TRACKING=false
```

So, in the currently-running configuration: **active** = soft bearing recovery, Local/team visibility *ranking*, K-of-N(k=2) selection, topology tuple enumeration, dynamic hard-clearance rejection, mutual non-worsening. **Off / unreachable** = Joint `{P,τ,Ψ}` + 3-ACK atomic commit, SCP hard corridor (LOS half-space + dynamic row), Elastic Tracking region, prefix progress, legacy `J_los`, team yaw optimisation. **Active but not firing in this scene** = directional `J_vis` (encirclement visibility guidance disabled for `long_cylinder_forest.json`; log shows `VISIBILITY_DIRECTIONAL_COST_ACTIVATIONS=0`).

---

## E. UNCONFIRMED / honest gaps

1. **Creation dates of 09-10…09-17 mechanisms cannot be independently cross-checked.** Every experiment directory dated 2026-09-10 or later has been deleted. For those dates the only evidence is the reports' own in-document dates and file mtimes. `UNCONFIRMED` where a mtime is the sole basis.
2. **Bulk mtimes are indistinguishable from creation.** `cooperative_viewpoint_core.h`, `activation_schedule.h`, `team_visibility_preference.h`, `tracking_visibility_geometry.h`, `static_los_wall_contract_test.cpp` all share exactly `2026-09-11 12:56`; five `multi_uav_formation` objects share `2026-09-16 01:27-01:28`; `topology_coordinator_core.{h,cpp}` and `multi_uav_topology_coordinator.cpp` share `2026-09-18 18:26`. These look like copy/batch timestamps. **All mtime-only dates in this report are upper bounds.**
3. **The 09-08 experiment harness recorded `launch_arguments` only from 09-09 onward.** The 09-08 directories' `run_meta.json` files carry `enable_*` keys but not the full argument list; the union of their keys was reconstructed by grepping scripts plus reading the 106 individual `run_meta.json` flag dictionaries. The `weight_visibility`-only delta is derived from that reconstruction and is consistent across all six 09-08 directories, but it is a reconstruction, not a verbatim 09-08 roslaunch dump.
4. **Rooms for error in `feedback_23`'s date.** Its in-document date is **2026-09-09**, its mtime is **2026-09-10 01:20**, and an identical copy sits at the repo root as `feedback_17.md` (mtime 09-10 00:38, same header *"报告日期：2026-09-09"*). The K2/joint work therefore spans the 09-09→09-10 night. Calling it "09-09 introduced, 09-10 first documented" is the defensible statement.
5. **`team_visibility_optimizer.h/.cpp` mtimes (09-16 21:25 / 09-17 22:08) are definitively NOT creation dates** — `feedback/ACTIVATION_LIFECYCLE_REPORT_20260910.md` (09-10) already prints the full objective and the production config prints `weights_k2_acc_k2cont_blackout_dev_jerk=0.200000,4.000000,…`. Creation is 09-09/09-10.
6. **No in-source `@date`/creation markers.** A repo-wide grep for date literals in `*.h/*.cpp/*.hpp` under `ros_ws/src` returns exactly one hit, unrelated: `swarm_bridge/include/reliable_bridge.hpp:17: * @date 2021-08-11`. There is no authorship-date channel in the code itself.
7. **Two prior AI-generated reports answer this same question and were treated as secondary, not primary, evidence:** `POST_0908_DELTA_AND_FORMULAS.md` (09-18 19:27) and `ROUND3_NEW_MECHANISMS_AUDIT.md` (09-18 19:11). Where they conflict with `ROUND3_NEW_MECHANISMS_AUDIT.md`'s date-provenance labels, the raw artefacts above were used instead.
8. **Two claims from `ROUND3_NEW_MECHANISMS_AUDIT.md` that I could not reproduce and therefore do not carry forward as fact:** (a) its §0.1 statement that *"在 2026-09-14 之前就已经存在大量机制"* is a **09-14** boundary, not a 09-08 boundary, and it does **not** establish any of the listed mechanisms as pre-09-08 — for `cooperative_viewpoint_core.h` (09-11), `activation_schedule.h` (09-11), `adaptive_viewpoint_generator` (09-13), `team_solution_commit_contract.h` (09-14), `encirclement_geometry.h` (09-12) its own cited evidence post-dates 09-08; (b) `NOMINAL_WARM_START_OVERFIX`, `NO_CERTIFIED_SUCCESSOR`, `EXECUTION_RESERVE` and `activation_earliest` are report labels with **no source identifier** (its own §五 states this), so they are not mechanisms.
