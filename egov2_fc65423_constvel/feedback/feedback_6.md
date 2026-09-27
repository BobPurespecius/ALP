# Feedback 6

## 范围与公平性

- 仅新增/修改场景 JSON、场景生成/检查脚本和 benchmark/analysis 脚本。
- Planner/controller 算法代码未修改。
- 本轮唯一正式报告为本文件；没有生成重复的 C1/Visibility 报告。
- 首次 C1 launcher 尝试因相对 scene path 无法被 roslaunch 子进程解析而停在 WAIT_TARGET；未生成 trajectory、未计入 Run1，并已正常清理。
- C1 三轮使用同一 SHA256 `88f4c28938dc16f20a1da18eb905a21b9f80d5dc6b78ff46c69338fe6f4cb405`。
- Visibility 三方法使用同一冻结 SHA256 `c7c28831b4306899b3305f4b4aace136a164191429749c8fc9417d5f7b27261a`。

## C1 几何方向复核

C1 route tangent `t=(0.992, 0.124)`，physical LEFT normal `n=(-0.124, 0.992)`。上次沿 `-n` 移动不会改变 along 坐标，只会令 lateral 坐标从约 -1.050 m 降到 -1.250 m；因此 blocker 对 `+n` LEFT corridor 的 approach/conflict/rejoin 距离均增加。

| Blocker/stage | Along (m) | Lateral (m) | LEFT lane clr (m) | Nominal clr (m) | Approach clr (m) | Conflict clr (m) | Rejoin clr (m) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Original 0 | -1.150 | -1.050 | 1.600 | 0.500 | 1.600 | 1.646 | 2.286 |
| Original 1 | 1.150 | -1.050 | 1.600 | 0.500 | 2.286 | 1.646 | 1.600 |
| Previous +0.20m 0 | -1.150 | -1.250 | 1.800 | 0.700 | 1.800 | 1.842 | 2.440 |
| Previous +0.20m 1 | 1.150 | -1.250 | 1.800 | 0.700 | 2.440 | 1.842 | 1.800 |
| Final +0.35m 0 | -1.150 | -1.400 | 1.950 | 0.850 | 1.950 | 1.990 | 2.560 |
| Final +0.35m 1 | 1.150 | -1.400 | 1.950 | 0.850 | 2.560 | 1.990 | 1.950 |

- Final blocker_0: `[6.032812157, 0.218502743]`
- Final blocker_1: `[8.314412157, 0.503702743]`
- LEFT open clearance: 1.600 -> 1.800 -> 1.950 m.
- Blocked/right final lane clearance: -0.250 m，仍受限。
- Target continuous min static clearance: 0.344 m；collision=0。

## C1 三轮验证

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| Task complete | YES | YES | YES |
| C1 collision episodes | 0 | 0 | 0 |
| C1 collision samples | 0 | 0 | 0 |
| C1 unsafe episodes | 2 | 2 | 1 |
| C1 unsafe samples | 57 | 52 | 5 |
| C1 min dynamic clearance (m) | 0.256 | 0.326 | 0.485 |
| Min static clearance (m) | 0.200 | 0.158 | 0.204 |
| Global dynamic collision episodes | 1 | 0 | 1 |
| Global static collision episodes | 0 | 0 | 0 |
| Target static collision samples | 0 | 0 | 0 |
| Tracking P95 (m) | 2.162 | 2.159 | 2.600 |
| Mean path length (m) | 80.572 | 79.613 | 81.530 |
| Planning latency P95 (ms) | 0.897 | 0.843 | 0.887 |
| All-3 visibility | 92.56% | 92.72% | 92.35% |

C1 本身 3/3 无碰撞；全局碰撞并未被隐去：Run1 的 1 episode 位于 A1，Run3 的 1 episode 位于 A2，Run2 全局无碰撞。

## Visibility Stress V2 几何设计

- 三机 nominal offsets 来自当前 launch 配置：UAV1=(-1.5,-0.85), UAV2=(-1.7,0), UAV3=(-1.5,+0.85) m。
- V1/V3 使用约 -60° stress leg，LEFT open；V2 使用约 +60° stress leg，RIGHT open。
- 每个 Gate 用 4 根 r=0.38 m 柱形成约 1.7 s 连续 occlusion band；target 沿斜线从旁通过，三条固定目标相对 LOS 扇形穿过遮挡带。
- 设计只使用 target route、nominal formation offsets 和 LOS 几何，不使用任何方法实际轨迹。

| Gate | UAV | Nominal theoretical visibility | Open-side theoretical visibility | Difference |
|---|---|---:|---:|---:|
| V1 | UAV1 | 43.56% | 100.00% | 56.44% |
| V1 | UAV2 | 30.69% | 100.00% | 69.31% |
| V1 | UAV3 | 40.59% | 100.00% | 59.41% |
| V2 | UAV1 | 40.59% | 100.00% | 59.41% |
| V2 | UAV2 | 30.69% | 100.00% | 69.31% |
| V2 | UAV3 | 43.56% | 100.00% | 56.44% |
| V3 | UAV1 | 43.56% | 100.00% | 56.44% |
| V3 | UAV2 | 30.69% | 100.00% | 69.31% |
| V3 | UAV3 | 41.58% | 100.00% | 58.42% |

| Gate | Open-side min static bypass clr (m) | Blocked-side clr (m) | Inflated bypass |
|---|---:|---:|---|
| V1 | 2.051 | -0.080 | PASS |
| V2 | 2.199 | -0.059 | PASS |
| V3 | 1.514 | -0.059 | PASS |

- Target continuous min static clearance: 0.344 m，collision=0。
- Minimum dynamic-to-static surface gap over full motion: 0.453 m。
- Frozen SHA256: `c7c28831b4306899b3305f4b4aace136a164191429749c8fc9417d5f7b27261a`

## Visibility Stress V2 三方法结果

| Metric | Native | Gradient | ALP |
|---|---:|---:|---:|
| Task complete | YES | YES | YES |
| UAV1 visibility | 91.55% | 94.85% | 91.03% |
| UAV2 visibility | 84.68% | 82.23% | 90.89% |
| UAV3 visibility | 92.85% | 90.68% | 53.50% |
| All-3 visibility | 79.95% | 79.65% | 47.69% |
| At-least-2 visibility | 90.96% | 90.54% | 89.73% |
| At-least-1 visibility | 98.18% | 97.57% | 98.00% |
| None-visible | 1.82% | 2.43% | 2.00% |
| Static LOS blocked | 7.76% | 7.76% | 18.22% |
| Dynamic LOS blocked | 2.27% | 2.37% | 4.70% |
| FOV loss | 0.28% | 0.53% | 4.75% |
| Out-of-range | 0.00% | 0.21% | 11.29% |
| Dynamic collision episodes | 0 | 1 | 0 |
| Dynamic collision samples | 0 | 3 | 0 |
| Unsafe samples | 195 | 180 | 95 |
| Min dynamic clearance (m) | 0.086 | -0.007 | 0.101 |
| Static collision episodes | 0 | 0 | 2 |
| Min static clearance (m) | 0.199 | 0.156 | -0.535 |
| Tracking mean (m) | 1.585 | 1.610 | 3.134 |
| Tracking P95 (m) | 1.919 | 1.998 | 7.666 |
| Mean path length (m) | 88.756 | 87.077 | 88.667 |
| Planning latency median (ms) | 0.296 | 0.342 | 0.391 |
| Planning latency P95 (ms) | 0.766 | 0.816 | 2.776 |
| Planning latency max (ms) | 20.191 | 19.954 | 24.558 |
| Executed acceleration RMS | 1.089 | 1.051 | 1.298 |
| Executed acceleration P95 | 2.483 | 2.167 | 2.880 |
| Executed jerk RMS | 29.329 | 28.416 | 35.919 |
| Executed jerk P95 | 64.436 | 55.633 | 71.911 |
| Executed jerk max | 227.575 | 284.754 | 367.642 |

Executed acceleration/jerk 由 odom velocity 在统一 30 Hz 网格上线性重采样后 finite-difference；不等于 MINCO analytic jerk。

Configured target speed 三方法均为 0.928 m/s，但 target coordinator 的动态避障分段调度产生了不同实际 speed sequence：

- Native: `[0.45472, 0.56608, 0.67744, 0.91872, 0.95584]`
- Gradient: `[0.43616, 0.6032, 0.64032, 0.8816, 0.91872]`
- ALP: `[0.45472, 0.56608, 0.67744, 0.91872, 0.95584]`

因此几何、配置、scene hash 和结束条件是公平冻结的，但“实际分段 target speed 完全相同”这一严格运行条件未满足；三方法结果需带此限制解释。

### 逐 Gate 三机可见性

| Gate | Method | UAV1 vis | UAV2 vis | UAV3 vis | Static LOS blocked | Dynamic LOS blocked |
|---|---|---:|---:|---:|---:|---:|
| V1 | Native | 46.00% | 22.00% | 60.67% | 51.78% | 0.00% |
| V1 | Gradient | 81.33% | 10.67% | 46.00% | 47.78% | 0.00% |
| V1 | ALP | 31.54% | 95.30% | 47.65% | 32.66% | 0.00% |
| V2 | Native | 82.00% | 54.67% | 39.33% | 35.11% | 6.22% |
| V2 | Gradient | 100.00% | 49.01% | 60.26% | 18.76% | 5.74% |
| V2 | ALP | 100.00% | 43.33% | 0.00% | 52.22% | 0.00% |
| V3 | Native | 49.33% | 46.00% | 100.00% | 34.89% | 0.00% |
| V3 | Gradient | 48.34% | 38.41% | 100.00% | 37.75% | 0.00% |
| V3 | ALP | 48.32% | 39.60% | 100.00% | 37.36% | 0.00% |

## Final Feedback

C1_PREVIOUS_MOVE_DIRECTION_CORRECT: YES

C1_FINAL_BLOCKER_MOVE: original positions + 0.35 m along -n (additional 0.15 m beyond previous relaxed)

C1_OPEN_CLEARANCE_FINAL: 1.950 m

C1_BLOCKED_SIDE_STILL_CONSTRAINED: YES

C1_ALP_COLLISION_FREE_3_OF_3: YES

VISIBILITY_STRESS_V2_SCENE: /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress_v2.json

VISIBILITY_SCENE_SHA256: c7c28831b4306899b3305f4b4aace136a164191429749c8fc9417d5f7b27261a

THREE_UAV_VISIBILITY_STRESS_CREATED: NO

NATIVE_UAV_VISIBILITY: UAV1=91.55%, UAV2=84.68%, UAV3=92.85%

GRADIENT_UAV_VISIBILITY: UAV1=94.85%, UAV2=82.23%, UAV3=90.68%

ALP_UAV_VISIBILITY: UAV1=91.03%, UAV2=90.89%, UAV3=53.50%

ALP_ALL3_VISIBILITY: 47.69%

ALP_AT_LEAST2_VISIBILITY: 89.73%

ALP_VISIBILITY_ADVANTAGE: NO

FAIR_SCENE_FROZEN_BEFORE_METHOD_RUNS: YES

STRICT_RUNTIME_TARGET_SPEED_MATCH: NO

TARGET_STATIC_COLLISION_FREE: YES

PLANNER_CODE_MODIFIED: NO

PREVIOUS_MAX_FEEDBACK_INDEX: 5

CURRENT_FEEDBACK_FILE: feedback_6.md

FEEDBACK_CONTAINS_C1_AND_VISIBILITY_RESULTS: YES
