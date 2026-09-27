# Dynamic Gates V2 Three-Method Benchmark

正式报告日期：2026-09-02

新场景：`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json`

运行数据：`/home/bob/ALP/egov2_fc65423_constvel/dynamic_gates_v2_benchmark_20260902/`

## 1. 场景修改与冻结状态

- JSON parsing：PASS
- static map 主体结构：未修改
- planner/controller/算法源码：未修改
- 动态障碍数量：10 → 5
- 删除对象：
  - `dynamic_gate_background_0`
  - `dynamic_gate_background_1`
  - `dynamic_gate_background_2`
  - `dynamic_gate_background_3`
  - `dynamic_gate_background_4`
- 保留对象：A1、B1、C1、C2、A2
- 所有保留动态障碍完整运动范围均与目标/UAV corridor 相交
- 动态障碍完整运动范围均未进入静态柱内部
- model name 无重复
- 场景冻结 SHA256：`5bb54475da15a6528507a0e1aeddca5de5a74968d9a512a3b0ce66781061f733`
- Native、Gradient、ALP 三轮使用相同 SHA256

目标路径采用局部 waypoint 修正，没有重新设计为空旷直线：

- 修复 `long_forest_cylinder_00` 穿越
- 修复 `long_forest_cylinder_05` 穿越
- 提高 C1 approach 对 blocked-side 静态柱的余量
- 路径长度：74.071 m → 74.430 m
- 长度增量：0.359 m，约 +0.48%
- 连续 segment/cylinder 检查目标静态碰撞数：0
- 目标最小中心到静态柱表面间隙：0.309 m
- 当前 target radius 0.25 m + margin 0.05 m 合同：PASS
- 三轮运行日志中的真实目标最小静态间隙均约 0.309 m，负间隙样本为 0
- C1 LEFT-open 侧 sampled lane 最小静态间隙：1.599 m
- C2 RIGHT-open 侧 sampled lane 最小静态间隙：1.600 m

Gate phase 只为以下目的做同步：红球先通过，动态障碍随后进入 UAV tracking corridor，避免 target coordinator 因启动相位微差为不同方法选择不同分段速度。

离线覆盖启动相位偏差 2.45–2.85 s 后，全部 target segment 均选择同一个 0.99 speed scale；三轮 CSV 实际非零 target speed 序列完全一致：`0.918720 m/s`。

| Gate | 作用 | target 预计到达 | UAV corridor event | 最大速度 | 实际影响 |
|---|---|---:|---:|---:|---|
| A1 | CROSSING | 16.454 s | 18.254 s | 0.550 m/s | YES |
| B1 | DWELL | 31.806 s | 34.406 s | 0.524 m/s | YES |
| C1 | LEFT_OPEN | 46.353 s | 48.953 s | 0.492 m/s | YES |
| C2 | RIGHT_OPEN | 59.230 s | 61.830 s | 0.492 m/s | YES |
| A2 | CROSSING/opposite timing | 73.337 s | 75.137 s | 0.550 m/s | YES |

## 2. 公平运行条件

三种方法均使用：

- 相同 frozen scene
- 相同 static map
- 相同 target waypoints
- 相同实际 target speed sequence
- 相同 initial UAV states
- target-facing yaw ON
- max_jer=22
- RViz OFF
- 相同 visibility evaluator
- 相同 target final-stop 检测
- final stop 连续约 3 s 后 SIGINT 正常结束

三轮均：

- Task complete：YES
- target final-stop detected：YES
- auto shutdown：YES
- roslaunch exit code：0
- 残留 ROS/ALP 进程：0

## 3. 主要结果

动态和静态 clearance 使用当前 evaluator 的 UAV center 到 cylinder surface 的带符号间隙；`<=0` 计 collision，`<0.5 m` 计 unsafe。

| Metric | Native | Gradient | ALP |
|---|---:|---:|---:|
| Task complete | YES | YES | YES |
| Dynamic collision episodes | 4 | 7 | 1 |
| Dynamic collision samples | 46 | 60 | 2 |
| Unsafe samples (<0.5 m) | 308 | 330 | 107 |
| Minimum dynamic surface clearance | -0.257 m | -0.258 m | -0.029 m |
| Static collision episodes | 0 | 0 | 0 |
| Static collision samples | 0 | 0 | 0 |
| Minimum static clearance | 0.199 m | 0.217 m | 0.219 m |
| Tracking mean | 1.588 m | 1.580 m | 1.667 m |
| Tracking RMS | 1.612 m | 1.586 m | 1.696 m |
| Tracking P95 | 1.854 m | 1.798 m | 2.222 m |
| Tracking max | 3.989 m | 2.380 m | 3.200 m |
| Mean path length | 79.826 m | 78.396 m | 84.907 m |
| Planning latency median | 0.327 ms | 0.403 ms | 0.420 ms |
| Planning latency P95 | 0.684 ms | 0.887 ms | 1.014 ms |
| Planning latency max | 21.891 ms | 49.828 ms | 21.174 ms |
| UAV1 visibility | 96.09% | 94.82% | 94.28% |
| UAV2 visibility | 97.16% | 96.67% | 94.90% |
| UAV3 visibility | 96.71% | 95.85% | 93.67% |
| All-3 visibility | 91.77% | 89.35% | 88.20% |
| At-least-2 visibility | 98.19% | 97.99% | 95.39% |
| At-least-1 visibility | 100.00% | 100.00% | 99.26% |
| None-visible | 0.00% | 0.00% | 0.74% |
| Executed acceleration RMS | 0.838 m/s² | 0.705 m/s² | 1.300 m/s² |
| Executed acceleration P95 | 1.507 m/s² | 1.326 m/s² | 2.965 m/s² |
| Executed jerk RMS | 24.119 m/s³ | 19.692 m/s³ | 35.972 m/s³ |
| Executed jerk P95 | 40.254 m/s³ | 35.917 m/s³ | 78.490 m/s³ |
| Executed jerk max | 386.717 m/s³ | 213.728 m/s³ | 295.854 m/s³ |
| Planned ISJ | N/A | N/A | N/A |

Executed acceleration/jerk 来自 odometry finite difference，不等于 MINCO analytic jerk，不能与 `max_jer=22` 直接比较。

保存数据中不能可靠恢复完整 active polynomial coefficients，因此 Planned ISJ 和 analytic max jerk 均记为 N/A，没有使用 proxy 冒充。

## 4. 逐 Gate 结果

| Gate | Native | Gradient | ALP |
|---|---|---|---|
| A1 | COLLISION | COLLISION | SAFE |
| B1 | UNSAFE | UNSAFE | UNSAFE |
| C1 | COLLISION | COLLISION | COLLISION |
| C2 | COLLISION | COLLISION | UNSAFE |
| A2 | COLLISION | COLLISION | UNSAFE |

| Gate / method | Min clearance | Unsafe samples | Collision samples |
|---|---:|---:|---:|
| A1 / Native | -0.257 m | 60 | 13 |
| A1 / Gradient | -0.258 m | 85 | 26 |
| A1 / ALP | 0.787 m | 0 | 0 |
| B1 / Native | 0.111 m | 49 | 0 |
| B1 / Gradient | 0.194 m | 55 | 0 |
| B1 / ALP | 0.464 m | 10 | 0 |
| C1 / Native | -0.041 m | 95 | 12 |
| C1 / Gradient | -0.007 m | 85 | 6 |
| C1 / ALP | -0.029 m | 58 | 2 |
| C2 / Native | -0.009 m | 42 | 3 |
| C2 / Gradient | -0.241 m | 39 | 13 |
| C2 / ALP | 0.244 m | 12 | 0 |
| A2 / Native | -0.246 m | 62 | 18 |
| A2 / Gradient | -0.132 m | 66 | 15 |
| A2 / ALP | 0.290 m | 27 | 0 |

ALP 的唯一 dynamic collision episode 位于 C1，共 2 个 collision samples。ALP 在 A1 完全安全，在 C2/A2 为 unsafe-only，没有发生 collision。

## 5. 可见性原因

各原因独立统计，同一样本可能同时具有多个原因。

| Cause / all UAV visibility decisions | Native | Gradient | ALP |
|---|---:|---:|---:|
| FOV | 0.00% | 0.00% | 0.00% |
| Static LOS blocked | 0.51% | 0.44% | 1.74% |
| Dynamic LOS blocked | 2.85% | 3.78% | 4.03% |
| Out of range | 0.00% | 0.00% | 0.00% |

目标路径自身连续静态检查为 collision-free。因此 static LOS loss 表示 UAV 与目标之间存在真实静态遮挡，不是红球进入柱体导致的伪可见性下降。

## 6. 三架 UAV 执行平滑度

| Method/UAV | Path | Acc RMS | Acc P95 | Jerk RMS | Jerk P95 | Jerk max |
|---|---:|---:|---:|---:|---:|---:|
| Native UAV1 | 80.003 m | 0.817 | 1.718 | 22.069 | 44.893 | 228.630 |
| Native UAV2 | 80.109 m | 0.894 | 1.554 | 26.246 | 41.934 | 467.518 |
| Native UAV3 | 79.366 m | 0.803 | 1.248 | 24.041 | 33.937 | 464.001 |
| Gradient UAV1 | 78.356 m | 0.797 | 1.302 | 23.075 | 36.880 | 297.502 |
| Gradient UAV2 | 78.016 m | 0.583 | 1.223 | 16.051 | 33.420 | 164.622 |
| Gradient UAV3 | 78.817 m | 0.735 | 1.454 | 19.951 | 37.452 | 179.059 |
| ALP UAV1 | 80.643 m | 1.174 | 2.741 | 32.575 | 74.027 | 314.971 |
| ALP UAV2 | 90.704 m | 1.447 | 3.514 | 39.516 | 92.637 | 278.671 |
| ALP UAV3 | 83.374 m | 1.279 | 2.640 | 35.825 | 68.807 | 293.920 |

## 7. 最终字段

DYNAMIC_OBSTACLES_BEFORE:
10

DYNAMIC_OBSTACLES_AFTER:
5

IRRELEVANT_DYNAMIC_OBSTACLES_REMOVED:
YES

TARGET_PATH_STATIC_COLLISION_FREE:
YES

TARGET_MIN_STATIC_CLEARANCE:
0.309 m

SCENE_CHALLENGE_PRESERVED:
YES

NATIVE_RESULT:
Task complete YES; dynamic collision episodes 4; collision samples 46; minimum dynamic clearance -0.257 m; static collision 0.

GRADIENT_RESULT:
Task complete YES; dynamic collision episodes 7; collision samples 60; minimum dynamic clearance -0.258 m; static collision 0.

ALP_RESULT:
Task complete YES; dynamic collision episodes 1; collision samples 2; minimum dynamic clearance -0.029 m; static collision 0.

BEST_DYNAMIC_SAFETY:
ALP

BEST_VISIBILITY:
Native

BEST_TRACKING:
Gradient

BEST_EXECUTED_SMOOTHNESS:
Gradient

ALP_COLLISION_FREE:
NO

NEW_SCENE:
`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2.json`

PLANNER_ALGORITHM_CODE_MODIFIED:
NO

RESIDUAL_ALP_ROS_PROCESSES:
0
