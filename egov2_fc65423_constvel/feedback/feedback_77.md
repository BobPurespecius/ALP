# Feedback 77

## Authority / scheduled-handoff fix

PRIMARY_ROOT_CAUSE:
`traj_server::executionHandoffGate()` always compared a received future
trajectory with `active_source_`.  When a predecessor was already scheduled
but had not activated yet, this used the wrong predecessor (for example,
planner 6 -> Joint 7 was checked by the executor as 5 -> 7).  A second
failure occurred when a later message overwrote the single scheduled slot
(3 was replaced by 4 before 3 activated).

FIX_APPLIED:

- `executionHandoffGate()` now uses the already-scheduled predecessor for
  receive-time validation when the activation interval requires it.
- `traj_server` now keeps an ordered future queue instead of overwriting an
  unactivated scheduled trajectory.  Queue entries are promoted only after
  their predecessor is installed.
- The existing physical P/V/A handoff gate remains enabled; no tolerance or
  safety clearance was relaxed.

REMOVED_REDUNDANT_REJECT_PATHS:
No physical reject path was removed.  The false active-vs-scheduled mismatch
path was eliminated by making the executor and planner use the same predecessor
chain; stale/mismatched trajectories are still rejected by the physical gate.

FINAL_EXECUTION_HARD_GATE:
`traj_server::executionHandoffGate()` at receive/activation, together with
planner `validateExecutionTrajectory()` before adoption.

EXECUTOR_ACK_BEFORE_COMMIT:
NO (the existing transport has activation/adoption feedback, but no new
pre-commit ACK protocol was introduced in this narrow fix).

POST_COMMIT_EXECUTOR_REJECT_COUNT:
0 (`ACTIVE_PVA_MISMATCH=0`, `SCHEDULED_ACTIVE_PVA_MISMATCH=0` in the run).

JOINT_FAILURE_BLOCKS_LOCAL:
0 observed; `JOINT_FAILURE_LOCAL_NOOP` remained a local no-op.

TERMINAL_HOLD_COUNT:
0

END_BEFORE_NEXT_COUNT:
0

USER_VISIBLE_STOP:
IMPROVED (no hold/end-before-next occurred in the captured run; existing
planner lock-error crashes were still observed and were not changed in this
authority-only round).

SCENARIO:
long_cylinder_forest.json

SAFETY_THRESHOLD_CHANGED: NO
N_L_R_CHANGED: NO
BODY_LOS_CHANGED: NO
FREE_TIME_CHANGED: NO
STOP_FALLBACK_ADDED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO

## Verification

Required package build succeeded with `catkin build ego_planner`.

The fixed FULL-ON run produced 240 scheduled activations and 19 queued /
promoted future entries.  It produced zero active-PVA mismatch rejects, zero
terminal holds, and zero end-before-next-activation events.  Existing
`boost::lock_error` messages were recorded separately and left untouched.
