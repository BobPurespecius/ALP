"""Summarize the one Feedback120 FULL run from saved logs and CSVs."""
from collections import Counter
import csv
import gzip
import json
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
run = root / "runs/20260926_105704_628281"
plain_log = run / "roslaunch_stdout.log"
if plain_log.exists():
    log = plain_log.read_text(errors="replace")
else:
    with gzip.open(str(plain_log) + ".gz", "rt", errors="replace") as fh:
        log = fh.read()

def field(line, name):
    match = re.search(r"\b" + re.escape(name) + r"=([^\s]+)", line)
    return match.group(1) if match else None

def lines(marker):
    return [line for line in log.splitlines() if marker in line]

def numbers(items, key):
    return [float(value) for line in items if (value := field(line, key))
            and re.fullmatch(r"[-+0-9.eE]+", value)]

def percentiles(values):
    ordered = sorted(values)
    if not ordered:
        return None
    def at(p):
        return ordered[min(len(ordered)-1, int(p * (len(ordered)-1)))]
    return {"count": len(ordered), "p50": at(.5), "p90": at(.9),
            "max": ordered[-1]}

def final_sum(marker, key):
    records = lines(marker)[-3:]
    return sum(int(field(line, key)) for line in records if field(line, key))

timing = lines("[A_STAR_SEED_TIMING]")
contracts = lines("[corridor-scp-contract]")
def clean_reason(line):
    value = field(line, "reason")
    if value and value.startswith("QP_MAX_ITER_"):
        return "QP_MAX_ITER_EXHAUSTED"
    return value
reasons = Counter(clean_reason(line) for line in contracts)
intent = lines("[SIDE_TOPOLOGY_INTENT]")
intent_bad = sum("intent_consistent=0" in line for line in intent)
astar = lines("[A_STAR_TO_MINCO]")
initial_set = [line for line in lines("[corridor-scp-active-set]")
               if "phase=initial" in line]

with (run / "visibility.csv").open(newline="") as fh:
    vis = [row for row in csv.DictReader(fh) if float(row["time_s"]) <= 76.5]

def duration_and_longest(predicate):
    total = 0.0
    longest = 0.0
    current = 0.0
    for row, next_row in zip(vis, vis[1:]):
        dt = float(next_row["time_s"]) - float(row["time_s"])
        if predicate(row):
            total += dt
            current += dt
            longest = max(longest, current)
        else:
            current = 0.0
    return {"total": total, "longest": longest}

trajectory = []
with (run / "visibility_trajectory.csv").open(newline="") as fh:
    trajectory = list(csv.DictReader(fh))

by_timestamp = {}
for row in trajectory:
    if float(row["time_s"]) > 76.5:
        continue
    by_timestamp.setdefault(row["timestamp"], []).append(row)
min_swarm = float("inf")
swarm_violation = 0
for rows in by_timestamp.values():
    for i in range(len(rows)):
        for j in range(i+1, len(rows)):
            a, b = rows[i], rows[j]
            if a["uav_id"] == b["uav_id"]:
                continue
            distance = sum((float(a[k]) - float(b[k]))**2
                           for k in ("x", "y", "z"))**0.5
            min_swarm = min(min_swarm, distance)
            swarm_violation += distance <= 0.0

result = {
    "run_id": run.name,
    "exit_status": (run / "exit_status.txt").read_text().strip(),
    "process_status": (run / "process_status.txt").read_text().strip(),
    "astar_required": final_sum("[corridor-contract-audit]", "ASTAR_REQUIRED"),
    "astar_raw_path_safe": final_sum("[astar-to-minco-audit]", "ASTAR_RAW_PATH_SAFE"),
    "corridor_build_success": final_sum("[astar-to-minco-audit]", "CORRIDOR_BUILD_SUCCESS"),
    "guide_rejected_hard": sum("verdict=GUIDE_REJECTED" in line for line in astar),
    "side_intent_telemetry": len(intent),
    "side_intent_inconsistent": intent_bad,
    "timing_count": len(timing),
    "timing_dilation_applied": sum(float(field(line,"lambda")) > 1.0 for line in timing
                                   if field(line,"lambda")),
    "lambda": percentiles(numbers(timing, "lambda")),
    "v_before": percentiles(numbers(timing, "v_before")),
    "v_after": percentiles(numbers(timing, "v_after")),
    "a_before": percentiles(numbers(timing, "a_before")),
    "a_after": percentiles(numbers(timing, "a_after")),
    "j_before": percentiles(numbers(timing, "j_before")),
    "j_after": percentiles(numbers(timing, "j_after")),
    "duration_before": percentiles(numbers(timing, "duration_before")),
    "duration_after": percentiles(numbers(timing, "duration_after")),
    "seed_continuous_corridor_violation": percentiles(
        numbers(initial_set, "seed_continuous_violation")),
    "corridor_scp_attempt": len(contracts),
    "corridor_scp_success": sum("scp_success=1" in line for line in contracts),
    "corridor_scp_reasons": dict(reasons),
    "qp_max_iter_status_lines": sum("solver_status=max_iter" in line for line in
                                    lines("[scp-hard-corridor]")),
    "execution_deadline_side_minco": sum("status=EXECUTION_DEADLINE" in line for line in
                                          lines("[side-minco]")),
    "min_budget_applied": len(lines("event=MIN_BUDGET_APPLIED")),
    "k3_events": len(lines("[K3_LOCAL_EVENT_CREATED]")),
    "k3_recovered": sum("binary_k3_recovered=1" in line for line in
                        lines("[K3_EVENT_RESULT]")),
    "los_hard_reject_count": final_sum("[los-soft-authority-audit]",
                                       "LOS_HARD_REJECT_COUNT"),
    "executed_all3": sum(int(row["visible_count"]) == 3 for row in vis) / len(vis),
    "executed_k2": sum(int(row["visible_count"]) >= 2 for row in vis) / len(vis),
    "all3_loss": duration_and_longest(lambda row: int(row["visible_count"]) < 3),
    "k2_loss": duration_and_longest(lambda row: int(row["visible_count"]) < 2),
    "blackout_samples": sum(int(row["visible_count"]) == 0 for row in vis),
    "static_contact": sum(float(row["static_clearance_m"]) <= 0 for row in trajectory),
    "dynamic_contact": sum(float(row["moving_clearance_m"]) <= 0 for row in trajectory),
    "min_static_clearance": min(float(row["static_clearance_m"]) for row in trajectory),
    "min_dynamic_clearance": min(float(row["moving_clearance_m"]) for row in trajectory),
    "min_swarm_separation_moving": min_swarm,
    "swarm_contact_moving": swarm_violation,
    "swarm_violation_logs": len(lines("SWARM_VIOLATION")),
    "unvalidated_executed": sum(row["safety_validated"].lower() != "true"
                                for row in trajectory),
    "pva_mismatch_logs": len(lines("PVA_MISMATCH")),
    "partial_team_activation_logs": len(lines("PARTIAL_TEAM_ACTIVATION")),
    "successor_starvation": len(lines("SUCCESSOR_STARVATION")),
    "terminal_hold_enter": len(lines("TERMINAL_HOLD_ENTER")),
}
out = root / "feedback/artifacts/feedback120_metrics.json"
out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
print(json.dumps(result, indent=2, ensure_ascii=False))
