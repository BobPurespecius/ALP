# Candidate-Aware Event Search

Candidates are derived deterministically from the C2 trajectory CSV, not from random phase/amplitude trials. The precheck mirrors the current MINCO-before-candidate geometry: SIDE uses `+/-0.8*sin(pi*u)` and TEMPORAL uses `p_nominal(t/scale)` for scales 1.05 through 1.35.

Source trajectory: `/tmp/v2_C2_visibility_run1_trajectory.csv`.

Grid: conflict times 8.0--87.5 s (0.5 s); radial offsets -0.30/0.00/0.30 m; amplitudes 0.35/0.70/1.10 m; periods 7.0/9.5/13.0/16.0 s; tangent/normal/two diagonal axes; analytic zero-crossing phase in both travel directions.

Filters: UAV speed >= 0.25 m/s, nominal/SIDE static clearance >= 0.65 m, moving cylinder keeps >= 0.43 m static/map margin over a full period, and event type survives at least 7/9 perturbations (`t_conflict +/- 0.2 s`, `phase +/- 0.08 rad`).

Robust candidates: **492** (Mixed 17, Temporal 0, Spatial 475).

| Type | UAV | t_conflict | d_nominal | d_plus | d_minus | d_temporal | delta_spatial | delta_temporal | side | scale | robustness |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|
| Mixed | 2 | 63.50 | 0.015 | 0.213 | 0.300 | 0.277 | 0.285 | 0.263 | MINUS | 1.30 | 8/9 |
| Temporal | - | - | no robust candidate | | | | | | | | |
| Spatial | 1 | 22.00 | 0.088 | 0.017 | 0.300 | 0.062 | 0.212 | -0.026 | MINUS | 1.05 | 7/9 |

The search models only pre-MINCO geometry. Runtime validation remains required because the replan trajectory can diverge from the C2 source trajectory and MINCO can change candidate clearance.

## One-Run Runtime Validation

The generated single-obstacle scenes were run once headless with moving-object cost, risk candidates, and temporal candidates enabled. These are diagnostic runs, not benchmark statistics.

| Scene | Predicted event | Runtime observation | Result |
|---|---|---|---|
| `candidate_event_spatial.json` | UAV 1, 22.0 s, MINUS, `d_nominal=0.088`, `delta_spatial=0.212`, `delta_temporal=-0.026` | At 22.0 s the runtime UAV 1 was about 0.40 m laterally from the C2 source path; the designed near-conflict did not occur. A later unrelated risk did select `SIDE_PLUS` with +0.133 m, but it is not validation of the designed event. | Prediction not validated: `TIMING_MISMATCH`. |
| `candidate_event_mixed.json` | UAV 2, 63.5 s, MINUS, scale 1.30, `d_nominal=0.015`, `delta_spatial=0.285`, `delta_temporal=0.263` | Source C2 at 63.5 s: `(14.73, 2.51)`, velocity `(0.92, 0.21)` m/s. Runtime: `(13.41, 2.29)`, velocity `(0.003, 0.001)` m/s. Runtime temporal scan improved only from 0.94 m to 1.04 m and was below the useful threshold; no designed Mixed event was selected. | Prediction not validated: `TIMING_MISMATCH`, before a MINCO-collapse conclusion can be made. |

Runtime logs: `/tmp/candidate_spatial.log`, `/tmp/candidate_mixed.log`. Trajectory samples: `/tmp/candidate_spatial_visibility_trajectory.csv`, `/tmp/candidate_mixed_visibility_trajectory.csv`.

## Interpretation

The inverse design is candidate-aware at the **pre-MINCO geometry** level: it found 17 robust Mixed and 475 robust Spatial events while enforcing static/map clearance and perturbation tolerance. No strict Temporal-only event was found on this C2 trajectory/grid. The failed runtime checks show that a C2 trajectory cannot yet be used as an invariant open-loop reference after the obstacle scene changes: replan/target progression changes before the intended encounter. The present bottleneck is therefore **scene-to-runtime trajectory reproducibility**, not evidence that the ranked Mixed geometry or MINCO is wrong. A reliable final benchmark needs a closed-loop source trajectory/reference that remains fixed under the injected event, or an online event injector synchronized to the actual candidate risk epoch.
