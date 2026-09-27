# ALP Candidate vs MINCO Ground Truth Validation

This is a diagnostic-only validation. Planner, MINCO, candidate selection,
weights, thresholds, and the existing scenes were not changed.

## Scenarios

| Scene | Dynamic obstacles | Purpose |
|---|---:|---|
| `candidate_ground_truth_spatial.json` | 1 | One shallow transverse obstacle with an asymmetric side opportunity. |
| `candidate_ground_truth_temporal.json` | 1 | One large transverse oscillator intended to make retiming useful. |
| `candidate_ground_truth_mixed.json` | 1 | One offset transverse oscillator intended to expose both side and timing gains. |

All three retain the existing three-UAV initialization, straight target
waypoints, map bounds, and planner values. `validationMetadata` records the
design intent; it is not consumed by the planner.

## Diagnostic Log

The existing SIDE path now emits one low-frequency `[candidate-ground-truth]`
line per attempted candidate with initial/final clearance, sampled trajectory
length, jerk metric, optimization result, and solver status. The current
`ObjPredictor` exposes predicted centers but not obstacle radii at this log
boundary, so `collision_count=UNAVAILABLE` is intentional. The existing
`[side-space]`, `[side-region]`, `[minco-candidate-diag]`,
`[risk-candidate]`, and `[temporal-candidate]` logs remain unchanged.
The TEMPORAL form of this new line was added after the recorded short runs;
the temporal numbers below therefore come from the existing
`[temporal-candidate]` records and should be re-collected if per-temporal
trajectory length/jerk is required.

## Short Headless Runs

Each scene was run once with the unchanged parameters, moving-object cost
enabled, risk candidates enabled, RViz disabled, and a 25 s wall-clock cap.
Timeout teardown
is not treated as an algorithm failure; process-death and allocator-error
searches are reported separately.

| Metric | Spatial | Temporal | Mixed |
|---|---:|---:|---:|
| triggered events | 3 | 11 | 0 |
| SIDE attempts | 6 | 19 | 0 |
| SIDE optimizer successes | 6 | 19 | 0 |
| temporal scan lines | 0 | 8 | 0 |
| useful temporal scales | 0 | 2 | 0 |
| temporal optimizer attempts/success/failure | 0/0/0 | 2/2/0 | 0/0/0 |
| TEMPORAL selected | 0 | 0 | 0 |
| `plan_success=0` | 247 | 180 | 383 |
| solver `-1005` occurrences | 105 | 61 | 169 |
| process death / bad_array / double-free | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

The high failure counts in this short run occur during the existing fake
simulator/timeout teardown and, for the mixed scene, before any risk event was
formed. They are not used to claim a candidate result.

## Candidate Clearance Comparison

### Spatial scene

| Event | Candidate | Init clearance | Final clearance | Nominal | Init improvement | Final improvement |
|---:|---|---:|---:|---:|---:|---:|
| 1 | SIDE_MINUS | 2.0271 | 1.1160 | 1.1090 | +0.9181 | +0.0070 |
| 2 | SIDE_MINUS | 1.8666 | 1.1239 | 1.1170 | +0.7496 | +0.0069 |
| 3 | SIDE_MINUS | 1.7569 | 1.2139 | 1.2090 | +0.5479 | +0.0049 |

All three events satisfy the `initial improvement >= 0.2 m` and
`final improvement < 0.05 m` rule. They are therefore
`MINCO_DESTROYED_CONFIRMED`. SIDE_PLUS was also attempted three times, but
was the inferior initial side in these events.

### Temporal scene

The two useful scan events were:

| Nominal | Best scan | Scan gain | Retimed init | Optimized final | Final gain |
|---:|---:|---:|---:|---:|---:|
| 0.989 | 1.292 (scale 1.35) | +0.303 | 1.119 | 1.060 | +0.071 |
| 0.946 | 1.069 (scale 1.35) | +0.123 | 1.053 | 1.033 | +0.087 |

Both temporal MINCO attempts returned success, but neither retained the
scan-level gain required for selection. In this run there was no temporal
solver failure; the observed issue is loss of clearance between scan/init and
optimized trajectory, not an optimizer return code.

### Mixed scene

No triggered event was observed in the 25 s run. Consequently there is no
nominal/SIDE/TEMPORAL comparison for this scene and no basis for claiming a
Mixed result. This is an execution/timing validation failure, not evidence for
or against MINCO.

## Verdict

1. The SIDE candidate can be genuinely useful before MINCO: the spatial run
   produced initial gains of 0.548--0.918 m.
2. MINCO destroyed those valid side gains in all three observed spatial
   events, so `MINCO_DESTROYED_CONFIRMED` is the current diagnosis.
3. Temporal scan can also find a useful counterfactual, but the fixed-time
   temporal MINCO reduced the observed gains below the selection gate.
4. The mixed scene did not reach a risk event and must not be used to infer a
   candidate or MINCO defect until its runtime timing/execution is made
   reproducible.

## Sources

* `/tmp/candidate_ground_truth_spatial.log`
* `/tmp/candidate_ground_truth_temporal.log`
* `/tmp/candidate_ground_truth_mixed.log`
* `candidate_ground_truth_spatial.json`
* `candidate_ground_truth_temporal.json`
* `candidate_ground_truth_mixed.json`
