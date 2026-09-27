#!/usr/bin/env python3
"""Summarize the two Feedback137 BOOT-08 runs without treating them as FULL."""
import collections
import csv
import gzip
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNS = ("20260927_132753_1316604", "20260927_133236_1329404",
        "20260927_134239_1338169")


def fields(line):
    return dict(re.findall(r"\b([a-zA-Z_][a-zA-Z_0-9]*)=([^\s]+)", line))


def number(values, key, default=math.nan):
    try:
        return float(values[key])
    except (KeyError, ValueError):
        return default


def percentile(values, quantile):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def summarize(run_id):
    directory = ROOT / "runs" / run_id
    seed = []
    scp = []
    fallback = []
    counts = collections.Counter()
    with gzip.open(directory / "roslaunch_stdout.log.gz", "rt", errors="replace") as stream:
        for line in stream:
            if "[SIDE_TOPOLOGY_SEED_REFERENCE]" in line:
                data = fields(line.split("[SIDE_TOPOLOGY_SEED_REFERENCE]", 1)[1])
                if data.get("kind") in ("SIDE_PLUS", "SIDE_MINUS") and "seed_violation" in data:
                    seed.append(data)
            if "[LOCAL_SIDE_CONSTRAINED_SCP]" in line:
                data = fields(line.split("[LOCAL_SIDE_CONSTRAINED_SCP]", 1)[1])
                if data.get("kind") in ("SIDE_PLUS", "SIDE_MINUS") and "reason" in data:
                    scp.append(data)
            if "[side-feasible-fallback]" in line:
                data = fields(line.split("[side-feasible-fallback]", 1)[1])
                if data.get("candidate_type") in ("SIDE_PLUS", "SIDE_MINUS"):
                    fallback.append(data)
            for token in ("TERMINAL_HOLD_ENTER", "MOVING_SUCCESSOR_STARVATION",
                          "PVA_MISMATCH", "PVAJ_VIOLATION_COMMIT", "UNVALIDATED_COMMIT",
                          "PARTIAL_TEAM_ACTIVATION", "MIN_BUDGET_APPLIED"):
                if token in line:
                    counts[token] += 1
    seeds = [number(item, "seed_violation") for item in seed if item.get("valid") == "1"]
    seeds = [value for value in seeds if math.isfinite(value)]
    reasons = collections.Counter(item["reason"] for item in scp)
    successes = [item for item in scp if item["reason"] == "SCP_FINAL_OK"]
    final_values = [number(item, "side_region_max_violation_final") for item in successes]
    final_values = [value for value in final_values if math.isfinite(value)]
    by_source = {}
    for source in ("FRESH", "WARM", "ASTAR"):
        values = [number(item, "seed_violation") for item in seed
                  if item.get("source") == source and item.get("valid") == "1"]
        values = [value for value in values if math.isfinite(value)]
        by_source[source] = {"count": len(values), "max": max(values) if values else None}

    rows = list(csv.DictReader(open(directory / "visibility.csv")))
    visible = [int(row["visible_count"]) for row in rows]
    times = [float(row["time_s"]) for row in rows]
    loss_duration = sum(max(0.0, times[i + 1] - times[i])
                        for i in range(len(rows) - 1) if visible[i] < 3)
    episodes = []
    start = None
    for i, value in enumerate(visible):
        if value < 3 and start is None:
            start = i
        if value >= 3 and start is not None:
            episodes.append(times[i] - times[start])
            start = None
    if start is not None:
        episodes.append(times[-1] - times[start])
    trajectory = list(csv.DictReader(open(directory / "visibility_trajectory.csv")))
    holds = [row for row in trajectory if row["trajectory_source"] == "TERMINAL_HOLD"]
    unvalidated = [row for row in trajectory if row["safety_validated"].lower() != "true"]
    static = [float(row["static_clearance_m"]) for row in trajectory]
    moving = [float(row["moving_clearance_m"]) for row in trajectory]
    status = dict(line.strip().split("=", 1) for line in open(directory / "exit_status.txt") if "=" in line)
    return {
        "run_id": run_id, "status": status,
        "seed_references": len(seed),
        "seed_reference_valid": sum(item.get("valid") == "1" for item in seed),
        "seed_violation_p50": percentile(seeds, 0.5),
        "seed_violation_p95": percentile(seeds, 0.95),
        "seed_violation_max": max(seeds) if seeds else None,
        "seed_sources": by_source,
        "scp_attempts": len(scp),
        "scp_final_ok": len(successes),
        "scp_reason_counts": dict(reasons),
        "scp_seed_violation_max": max((number(item, "side_region_max_violation_seed")
                                        for item in scp), default=None),
        "scp_hard_rows_positive": sum(number(item, "side_region_hard_rows") > 0 for item in scp),
        "final_topology_violation_max": max(final_values) if final_values else None,
        "final_topology_collapse_gt_0p002": sum(value > 0.002 for value in final_values),
        "fallback_checked": len(fallback),
        "fallback_accepted": sum(item.get("used") == "1" for item in fallback),
        "fallback_region_rejected": sum(item.get("side_region_ok") == "0" for item in fallback),
        "log_counts": dict(counts),
        "visibility_samples": len(rows), "visibility_elapsed_s": times[-1] if times else None,
        "K3_diagnostic": sum(value == 3 for value in visible) / len(visible) if visible else None,
        "K2_diagnostic": sum(value >= 2 for value in visible) / len(visible) if visible else None,
        "K3_loss_total_diagnostic_s": loss_duration,
        "longest_K3_loss_diagnostic_s": max(episodes, default=0.0),
        "K3_loss_episodes_diagnostic": len(episodes),
        "mean_visible_diagnostic": sum(visible) / len(visible) if visible else None,
        "blackout_samples_diagnostic": sum(value == 0 for value in visible),
        "trajectory_samples": len(trajectory),
        "terminal_hold_samples": len(holds),
        "first_terminal_hold": {key: holds[0][key] for key in
                                ("timestamp", "uav_id", "trajectory_id", "trajectory_source")}
                                if holds else None,
        "unvalidated_executed_samples": len(unvalidated),
        "static_contact_samples": sum(value <= 0 for value in static),
        "moving_contact_samples": sum(value <= 0 for value in moving),
        "min_static_clearance": min(static, default=None),
        "min_moving_clearance": min(moving, default=None),
    }


if __name__ == "__main__":
    result = {run_id: summarize(run_id) for run_id in RUNS}
    target = ROOT / "feedback" / "artifacts" / "feedback137_metrics.json"
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
