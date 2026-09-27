# ALP SIDE Directionality Audit Report

Date: 2026-09-02  
工程: `/home/bob/ALP/egov2_fc65423_constvel`  
场景: `long_cylinder_forest_dynamic_gates.json`  
数据: `dynamic_gates_repeatability_20260902/alp_1..alp_4`

## 结论摘要

`SIDE_PLUS`/`SIDE_MINUS` 没有发生运行时随机翻转或 world-fixed 符号错误；代码使用的是“路线右法向”作为正方向，因此在本报告的 canonical frame 中：

- `SIDE_PLUS` = physical **RIGHT**
- `SIDE_MINUS` = physical **LEFT**

这与日志中的字符串名称相反于常见的“PLUS=LEFT”直觉，但实现内部是一致的。C1/C2 的非对称主要来自 **SIDE selection 没有把静态 gate 的开放侧作为选择合同**：C1 选 `SIDE_MINUS` 恰好落在 LEFT open 侧，而 C2 四次均选 `SIDE_MINUS`，落在 RIGHT-open gate 的 blocked LEFT 侧。A1/A2 也都偏向 `SIDE_MINUS`；A1 的动态障碍沿 canonical LEFT 方向横穿，A2 沿相反方向横穿，因此同一侧选择产生了明显的方向性结果。

## Canonical LEFT/RIGHT 定义

对每个 gate 使用场景 `dynamicGateDesign.gates[*].t`：

```text
t = normalize(local route tangent)
left_normal  = (-t_y,  t_x)
right_normal = -left_normal = (t_y, -t_x)
```

源码在 `planner_manager.cpp` 的 side frame、preferred-side heuristic、A* side path check 以及 `poly_traj_optimizer.cpp::setCandidateSideBias()` 中均计算：

```cpp
direction = (end - start).cross(UnitZ())
```

对水平向量 `(tx, ty, 0)`，该叉乘结果为 `(ty, -tx, 0)`，即 canonical **right_normal**。随后 `side > 0` 保持该方向，`side < 0` 取其相反方向。因此：

| Internal label | Sign | Physical side |
|---|---:|---|
| SIDE_PLUS | +1 | RIGHT |
| SIDE_MINUS | -1 | LEFT |

该结论适用于 seed displacement、candidate-side cost、region constraint、A* path side check 和 Local-SFC plane normal；没有发现 x/y 写死、obstacle velocity 直接翻转 side frame、或 tangent 反向时单独翻转 PLUS/MINUS 的代码路径。

## Gate geometry and intended topology

场景生成器明确声明 route-local `n` 为 left normal，并按 blocked side 放置静态 blocker：

| Gate | `t` | Dynamic motion | Intended/open side | Static blocker side |
|---|---|---|---|---|
| A1 | (0.981,-0.196) | `+n` (UP) | either | none |
| C1 | (0.992,0.124) | dwell at route center | LEFT | RIGHT (`blocked_sign=-1` in generator) |
| C2 | (0.8,-0.6) | dwell at route center | RIGHT | LEFT (`blocked_sign=+1` in generator) |
| A2 | (0.936,0.351) | `-n` (DOWN) | either | none |

The generated blocker locations are therefore mirror-consistent: C1 blocks `-n` (right) and leaves `+n` (left) open; C2 blocks `+n` (left) and leaves `-n` (right) open. No scene asymmetry or accidental swapped blocker sign was found.

## Observed ALP selections (four repeated runs)

The existing ALP logs contain repeated `[risk-candidate]` records. Near the four gate events, the non-nominal choices were:

| Gate | Run evidence | Observed SIDE choice | Canonical physical interpretation |
|---|---|---|---|
| A1 | alp_1..alp_4 | predominantly `SIDE_MINUS` | LEFT |
| C1 | alp_1, alp_2 had `SIDE_MINUS`; alp_3, alp_4 mostly NOMINAL | LEFT when SIDE was used |
| C2 | alp_1..alp_4 | `SIDE_MINUS` | LEFT (the blocked side) |
| A2 | alp_1..alp_4 | predominantly `SIDE_MINUS` | LEFT |

Gate outcomes from the same artifacts:

| Gate | ALP outcome |
|---|---|
| A1 CROSSING | COLLISION 4/4; UAV1 in all four recorded collision episodes |
| C1 LEFT_OPEN | SAFE 4/4 |
| C2 RIGHT_OPEN | UNSAFE 4/4; min dynamic clearance approximately 0.183–0.258 m |
| A2 CROSSING | SAFE 4/4 |

The C1/C2 pair is the strongest directional evidence: the same internal choice (`SIDE_MINUS` = LEFT) is safe when LEFT is open (C1) and unsafe when LEFT is blocked (C2). This is not evidence that the sign changes between gates; it is evidence that side selection is not conditioned on the gate's physical open side.

## A1 versus A2

A1 and A2 both use the same canonical side bias in the observed ALP runs (mostly LEFT). Their obstacle velocities are opposite:

- A1 uses `+n` (UP), so the obstacle sweeps toward the LEFT side of the route at the encounter.
- A2 uses `-n` (DOWN), so the obstacle sweeps toward the RIGHT side.

Thus a fixed LEFT preference is dynamically unfavorable for A1 and favorable for A2. The result (A1 collision 4/4 versus A2 safe 4/4) is consistent with a direction-dependent interaction between the selected side and obstacle sweep, not with a PLUS/MINUS sign flip.

## Source-level audit

Relevant code paths:

- `planner_manager.cpp:1450-1489`: tangent and preferred-side heuristic. It uses `tangent.cross(Z)` (right normal), then maps escape-direction alignment to `side = +/-1`.
- `planner_manager.cpp:2028-2066`: side frame and no-crossing test. `side * lateral >= -0.05` consistently preserves the selected signed half-plane.
- `planner_manager.cpp:2304-2355`: Local-SFC planes use each A* segment tangent crossed with Z, then multiply by `side`; guide/probe orientation is sanity checked before insertion.
- `poly_traj_optimizer.cpp:4955-4976`: candidate-side bias uses the same `(end-start).cross(Z)` direction and sign.
- `create_dynamic_gates_scene.py:119-135`: C1/C2 blocker placement uses the intended canonical `n` and opposite `blocked_sign` values.

No source evidence was found for:

- world-fixed `(0, +/-1)` side bias;
- x/y sign hard-coding independent of route tangent;
- cross-product order inconsistency between planner and optimizer;
- obstacle-velocity-dependent sign inversion;
- route tangent reversal causing only one of PLUS/MINUS to flip;
- C1/C2 static blocker geometry being accidentally swapped.

## Limitations of the existing artifacts

The repeatability logs do not emit one canonical direction-debug record containing current position, runtime tangent, obstacle vector, seed displacement, and selected physical side in a single row. Therefore the physical mapping above is reconstructed exactly from source formulas and the scene's route-local frame; it is not inferred from the string label alone. A future diagnostic row should log those vectors, but no planner rerun or algorithm change was needed to establish the present root cause.

## Final classification

```text
SIDE_DIRECTION_MAPPING_CORRECT: YES
PLUS_PHYSICAL_SIDE: RIGHT
MINUS_PHYSICAL_SIDE: LEFT

A1_FAILURE_DIRECTIONAL: YES
A2_SUCCESS_DIRECTIONAL: YES

C1_EXPECTED_SIDE: LEFT
C1_SELECTED_SIDE: LEFT when SIDE candidate was used; otherwise NOMINAL
C2_EXPECTED_SIDE: RIGHT
C2_SELECTED_SIDE: LEFT (SIDE_MINUS) in all four runs

C1_C2_ASYMMETRY_ROOT_CAUSE: SIDE_SELECTION
DIRECTIONAL_CODE_BUG_FOUND: NO
SOURCE_LEVEL_ROOT_CAUSE: fixed/preferred LEFT tendency is not constrained by gate open-side topology; C2 therefore attempts the blocked physical side, while C1 happens to match its open side.
```

The remaining uncertainty is downstream severity: once C2 selects the blocked side, static rejoin/SCP may further limit clearance. Existing full-run data already show static collision episodes remain zero, so the observed C2 result is an unsafe dynamic clearance outcome rather than a static map collision. Separating the exact contributions of selection versus MINCO/SCP shrink would require the proposed per-event direction-debug artifact; it is not necessary to identify the first directional mismatch.

## Required answer

`SIDE_DIRECTION_MAPPING_CORRECT = YES`, but the topology-aware side decision is not correct for C2. The asymmetry is therefore a **SIDE_SELECTION / topology-contract gap**, not a PLUS/MINUS implementation sign bug. No code or parameter was changed in this audit.

