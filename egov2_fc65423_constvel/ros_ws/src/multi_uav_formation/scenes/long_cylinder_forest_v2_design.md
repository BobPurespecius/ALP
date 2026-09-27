# Long Cylinder Forest V2 Design

This scene keeps the original long-cylinder forest, target waypoints, UAV starts,
and planner parameters. Only the ten moving-cylinder definitions are changed.

| ID | Expected type | Initial center (x,y) | Axis | Amplitude (m) | Period (s) | Phase (rad) | Design intent |
|---:|---|---|---|---:|---:|---:|---|
| 0 | Spatial-favorable (PLUS) / mixed opportunity | (-24.2, 1.55) | y | 0.65 | 10.5 | 0.70 | Offset conflict near the early upper route; lower side remains open and the larger crossing creates a timing window. |
| 1 | Spatial-favorable (MINUS) | (-17.2, 3.55) | y | 0.35 | 13.0 | 3.40 | Upper-side conflict near the early/middle route; lower side remains open. |
| 2 | Spatial-favorable (late) | (16.8, 1.85) | y | 0.32 | 11.0 | 1.80 | Late asymmetric conflict near the upper route. |
| 3 | Temporal-favorable (early) | (-8.2, 0.25) | y | 1.55 | 8.0 | 2.00 | Broad lateral crossing with a short timing window. |
| 4 | Temporal-favorable (middle) | (1.8, 0.30) | y | 1.45 | 7.0 | 4.40 | Unsynchronised middle crossing; symmetric spatial alternatives. |
| 5 | Temporal-favorable (late) | (24.0, -2.15) | y | 1.50 | 8.5 | 1.20 | Late lower-route crossing with a different phase. |
| 6 | Mixed (early) | (-20.0, 2.80) | y | 1.10 | 9.5 | 0.40 | Stronger offset crossing with both a side opening and timing window. |
| 7 | Mixed (middle) | (10.8, 1.40) | y | 1.05 | 10.5 | 2.70 | Moderate asymmetric conflict where either strategy can help. |
| 8 | Background / low-risk | (-31.0, -6.0) | y | 0.50 | 12.0 | 1.00 | Outside the nominal route; checks non-conflicting background. |
| 9 | Background / low-risk | (31.0, 7.0) | y | 0.55 | 11.0 | 4.50 | Outside the nominal route; different phase and route segment. |

The classification used during validation is reproducible and event-based:

* Spatial-favorable: best spatial candidate clearance improvement >= 0.10 m and
  temporal improvement is below 0.10 m (or no useful temporal scan exists).
* Temporal-favorable: best spatial improvement < 0.10 m and temporal improvement
  >= 0.10 m.
* Mixed: both best spatial and temporal improvements are >= 0.10 m.
* Low-information: both improvements are < 0.10 m, or the event lacks one of
  the candidate measurements.

The original JSON is not overwritten. The generated Gazebo world is a derived
artifact and can be regenerated with `generate_world_from_scene.py` whenever
the scene is edited.
