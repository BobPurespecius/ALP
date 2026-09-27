# Feedback098 — Planner-side time-indexed visibility forecast.
#
# The Local Planner owns the future position of its own UAV: its rolling
# candidate is replaced every few hundred milliseconds, so no other node may
# extrapolate an old polynomial to guess where this UAV will be.  The Planner
# therefore publishes, for the trajectory it currently considers its Local
# baseline, the visibility margin sampled on an absolute world-time grid.
#
# The Coordinator only fuses the three traces into M2(t); it never re-derives
# a UAV future position from a stale execution polynomial.
#
# Margins use the SAME authority as the Team-SCP seed evaluation
# (teamMarginAt()/composeVisibilityMargin()), so both sides are guaranteed to
# speak the same quantity.

bool valid
uint64 forecast_identity
int32 trajectory_id
uint64 trajectory_generation
int32 candidate_id
string source_kind
time activation_time
time created_world_time
# Inclusive absolute-world-time validity of the trace samples below.
time start_world_time
time end_world_time
float64 sample_dt
time mathematical_end_time

float64[] world_times
float64[] margins
# Limiter per sample: 0=INVALID 1=STATIC 2=DYNAMIC_LOS 3=FOV 4=RANGE
uint8[] limiters

# K3 binary truth at a shared world-time grid.  Planner samples the same
# visibilitySampleAt authority; Coordinator never infers binary truth from a
# continuous margin.  Empty when K3 repair is disabled.
time[] k3_world_times
bool[] k3_binary_visible
