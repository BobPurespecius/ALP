# Projected MINCO Implementation Report

## Implementation

The generic header-only L-BFGS interface now accepts an optional
`lbfgs_post_update_t` callback. It is called only after More-Thuente returns a
successful line-search update. It is never called for trial points.

The callback used by `PolyTrajOptimizer`:

1. checks that the active candidate is SIDE_PLUS or SIDE_MINUS;
2. projects samples in the conflict window into the candidate corridor;
3. maps projected positions at piece junctions back to the `P` part of `x`;
4. leaves free-time variables unchanged;
5. calls the normal cost callback again to refresh `fx` and the full gradient;
6. requests an L-BFGS history reset.

The old complete-solve projection/restart loop was removed. The legacy
`candidate_projection_rounds` parameter remains accepted for launch-file
compatibility but is not used to control optimization.

## L-BFGS Consistency

After a projection, correction pairs are discarded logically by resetting the
history size and ring position. The next direction is `-gradient`, the step is
reinitialized, and the `past` objective history is reset. This prevents a
secant pair formed before projection from being reused after the non-gradient
state change.

## Variable Mapping

For fixed time, `x = P`, where `P` contains the internal MINCO junction
positions. For free time, `x = [P, virtual_time]`. Projection updates only `P`.
At each internal piece boundary `t_i`, the projected position `p'(t_i)` is
written to `P_i`. A conflict window fully inside a piece uses the sampled
piece displacement distributed to adjacent junctions, because the current
MINCO parameterization has no independent interior position variables.

## Validation

Scene: `scene_spatial_challenge.json`, headless, dynamic risk enabled,
preservation/region/corridor/acceptance disabled.

| Method | SIDE attempts | Success | Regression (`dev <= 0.03 m`) | Mean improvement | `>= 0.10 m` | SIDE selected | Projection callbacks | `-1005` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Baseline | 74 | 64 | 71.9% | -0.0122 m | 3.1% | 2 | 0 | 256 |
| Projected MINCO | 65 | 54 | 48.1% | +0.0991 m | 24.1% | 13 | 6,112 | 163 |

Mean optimizer time was 0.413 ms for baseline and 0.441 ms for projected
MINCO (+6.8% in this short run). No `process has died`, `bad_array`, or
`double-free` was observed. The mutex messages occurred during forced ROS
shutdown in both runs.

The projection callback emitted `candidate-projected-minco` records including
iteration, candidate type, obstacle id, projection update/count, before/after
violation, cost, gradient norm, and history reset.

## Assessment

The requested projected-update interface is implemented and is materially
different from outer projection rounds: projection occurs inside the accepted
L-BFGS iteration and the objective/gradient/history are synchronized before
the next update.

The short validation shows lower nominal regression, higher mean clearance
improvement, and more SIDE selections. It also shows solver failures for some
events and repeated projections when a continuous polynomial cannot be fully
represented by junction-position variables. This residual is a limitation of
the current MINCO parameterization/corridor mapping, not a soft-penalty
fallback. A production hard-corridor implementation would need additional
trajectory degrees of freedom or a true constrained/SQP-style solver.
