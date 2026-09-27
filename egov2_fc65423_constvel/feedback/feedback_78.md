# Feedback 78

NOMINAL_WARM_START_OVERFIX_EXISTED: YES

FIX_APPLIED:
Hypothesis-0 NOMINAL now remains bound to the normal local target and
authoritative current P/V/A, but may reuse only an `ABSOLUTE_SAFE` cache whose
candidate kind is NOMINAL.  SIDE/team/recovery caches remain ineligible for N.
The existing cache age, suffix, continuity, target and safety checks still
fall back to a fresh initializer when they fail.

NOMINAL_WARM_START_COUNT:
0 in this short run (the 5 accepted warm starts were explicitly SIDE
alternatives); 97 NOMINAL/alternative warm-start eligibility evaluations were
observed.  The new NOMINAL path is enabled and remains guarded by cache kind,
current revision, continuity, age and absolute-safety checks.

NOMINAL_TARGET_OVERWRITE_COUNT: 0

FUTURE_QUEUE_OVERSERIALIZATION_EXISTED: NO

FIX_APPLIED:
The previous run never exceeded queue depth 1; queued entries were needed to
preserve a real predecessor chain.  The queue remains ordered.  A narrow
same-activation, newer-generation replacement path was added so a revision of
the same future slot can supersede the stale slot without bypassing its true
predecessor.  No FIFO entries were collapsed across different activation
boundaries.

FUTURE_QUEUE_SUPERSEDE_COUNT: 0 in this run
STALE_FUTURE_EXECUTED_COUNT: 0 observed

TERMINAL_HOLD_COUNT: 0
END_BEFORE_NEXT_COUNT: 0

USER_VISIBLE_BRAKE_STOP: IMPROVED
LARGE_OSCILLATION: UNKNOWN

SCENARIO: long_cylinder_forest.json

RRCT_ACCESSED: NO
RRCT_CHANGED: NO

Verification also showed zero `ACTIVE_PVA_MISMATCH`, zero
`SCHEDULED_ACTIVE_PVA_MISMATCH`, and zero trajectory end-before-next events.
