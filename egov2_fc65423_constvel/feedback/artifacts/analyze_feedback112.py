#!/usr/bin/env python3
"""Compact, reproducible executed-truth and K3-event summary for Feedback 112."""

import argparse
import csv
import gzip
import json
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path


ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def fields(line):
    clean_line = ANSI_ESCAPE_RE.sub("", line)
    return dict(re.findall(r"(?:^|\s)([A-Za-z][A-Za-z0-9_]*)=([^\s]+)", clean_line))


def numeric_field(values, key, default):
    value = str(values.get(key, default))
    match = re.match(
        r"[-+]?(?:inf|nan|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)",
        value,
        re.IGNORECASE,
    )
    return float(match.group(0)) if match else float(default)


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lo = int(position)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (position - lo)


def run_metrics(run):
    with (run / "visibility.csv").open(newline="") as stream:
        visibility = list(csv.DictReader(stream))
    with (run / "visibility_trajectory.csv").open(newline="") as stream:
        trajectory = list(csv.DictReader(stream))
    times = [float(row["time_s"]) for row in visibility]
    counts = [int(row["visible_count"]) for row in visibility]
    n = len(counts)
    intervals = []
    begin = None
    for index, count in enumerate(counts):
        if count < 3 and begin is None:
            begin = index
        if count == 3 and begin is not None:
            intervals.append((times[begin], times[index - 1]))
            begin = None
    if begin is not None:
        intervals.append((times[begin], times[-1]))
    world_offsets = [float(row["timestamp"]) - float(row["time_s"])
                     for row in trajectory if row["timestamp"]]
    world_offset = statistics.median(world_offsets) if world_offsets else None
    tags = ("RELAY_K3_EVENT", "K3_ESCALATION_SOURCE", "K3_ESCALATION_EVENT",
            "K3_LOCAL_PROGRESS", "K3_LOCAL_PROGRESS_CONFIRMED",
            "K3_RECOVERY_TAIL", "K3_EVENT_RESULT", "K3_TIME_ALIGNMENT",
            "K3_SIDE_CANDIDATES", "TEAM_M2_MIN")
    events = defaultdict(lambda: {"tail": [], "progress": [], "confirmed": []})
    sources = []
    alignment_errors = []
    live_drift = []
    tag_counts = Counter()
    liveness = Counter()
    m2_windows = []
    raw_log = run / "roslaunch_stdout.log"
    compressed_log = run / "roslaunch_stdout.log.gz"
    log_stream = (raw_log.open(errors="replace") if raw_log.exists()
                  else gzip.open(compressed_log, "rt", errors="replace"))
    with log_stream as stream:
        for line in stream:
            for tag in tags:
                if "[{}]".format(tag) not in line:
                    continue
                tag_counts[tag] += 1
                data = fields(line)
                if tag == "K3_TIME_ALIGNMENT":
                    alignment_errors.append(float(data["target_error_norm"]))
                    if "live_snapshot_drift_norm" in data:
                        live_drift.append(float(data["live_snapshot_drift_norm"]))
                elif tag == "K3_ESCALATION_SOURCE":
                    sources.append(data)
                elif tag == "RELAY_K3_EVENT":
                    events["contract:" + data["contract_id"]]["contract"] = data
                elif tag == "TEAM_M2_MIN":
                    if data.get("valid") == "1" and numeric_field(data, "m2", "inf") < numeric_field(data, "trigger", "-inf"):
                        m2_windows.append(float(data["world_time"]))
                elif "event_id" in data:
                    event = events[data["event_id"]]
                    if tag == "K3_ESCALATION_EVENT":
                        event["detected"] = data
                    elif tag == "K3_EVENT_RESULT":
                        event["result"] = data
                    elif tag == "K3_RECOVERY_TAIL":
                        event["tail"].append(data)
                    elif tag == "K3_LOCAL_PROGRESS":
                        event["progress"].append(data)
                    elif tag == "K3_LOCAL_PROGRESS_CONFIRMED":
                        event["confirmed"].append(data)
            for key in ("MIN_BUDGET_APPLIED", "TERMINAL_HOLD_ENTER",
                        "MOVING_SUCCESSOR_STARVATION", "PVA_MISMATCH",
                        "PARTIAL_TEAM_ACTIVATION", "END_BEFORE_NEXT"):
                if key in line:
                    liveness[key] += 1
    event_rows = {key: value for key, value in events.items()
                  if not key.startswith("contract:")}
    for event in event_rows.values():
        detected = event.get("detected")
        if not detected or world_offset is None:
            continue
        target_uav = int(detected["target_uav"]) + 1
        begin_s = float(detected["window_begin"]) - world_offset
        end_s = float(detected["window_end"]) - world_offset
        key = "visible_uav{}".format(target_uav)
        window_rows = [row for row in visibility
                       if begin_s <= float(row["time_s"]) <= end_s]
        event["executed_window"] = {
            "begin_s": begin_s,
            "end_s": end_s,
            "sample_count": len(window_rows),
            "target_invisible_samples": sum(int(row[key]) == 0
                                            for row in window_rows),
            "target_static_blocked_samples": sum(
                row["uav{}_static_los_clear".format(target_uav)] == "0"
                for row in window_rows),
            "target_dynamic_blocked_samples": sum(
                row["uav{}_dynamic_los_clear".format(target_uav)] == "0"
                for row in window_rows),
        }
    result_rows = [event["result"] for event in event_rows.values()
                   if "result" in event]
    close_counts = Counter(row.get("close_reason", "UNKNOWN") for row in result_rows)
    tail_counts = Counter(row.get("reason", "UNKNOWN") for event in event_rows.values()
                          for row in event["tail"])
    source_counts = Counter((row.get("source", "UNKNOWN"),
                             row.get("escalation_published", "?"))
                            for row in sources)
    vis = {
        "samples": n,
        "duration_s": times[-1] - times[0],
        "all3_ratio": sum(c == 3 for c in counts) / n,
        "k2_ratio": sum(c >= 2 for c in counts) / n,
        "mean_visible": sum(counts) / n,
        "accumulated_camera_visible_time_s": sum(
            counts[i] * (times[i] - times[i - 1]) for i in range(1, n)),
        "k3_loss_total_s": sum(times[i] - times[i - 1]
                               for i in range(1, n) if counts[i] < 3),
        "longest_all3_loss_s": max((end - start for start, end in intervals), default=0),
        "longest_k2_loss_s": max((end - start for start, end in _intervals(times, counts, 2)), default=0),
        "blackout_samples": sum(c == 0 for c in counts),
        "visible_ratio": [sum(int(row["visible_uav{}".format(u)])
                              for row in visibility) / n for u in range(1, 4)],
        "loss_intervals": [{"begin_s": start, "end_s": end,
                            "duration_s": end - start}
                           for start, end in intervals],
    }
    static_clearance = [float(row["static_clearance_m"]) for row in trajectory]
    moving_clearance = [float(row["moving_clearance_m"]) for row in trajectory]
    safe = {
        "min_static_clearance_m": min(static_clearance),
        "min_dynamic_clearance_m": min(moving_clearance),
        "static_contact_samples": sum(x <= 0 for x in static_clearance),
        "dynamic_contact_samples": sum(x <= 0 for x in moving_clearance),
        "unvalidated_executed_samples": sum(
            row["safety_validated"].lower() != "true" for row in trajectory),
    }
    return {
        "run_id": run.name,
        "exit_status": (run / "exit_status.txt").read_text().strip().splitlines(),
        "process_status": (run / "process_status.txt").read_text().strip().splitlines(),
        "world_offset_s": world_offset,
        "visibility": vis,
        "safety": safe,
        "k3": {
            "contracts": tag_counts["RELAY_K3_EVENT"],
            "escalation_events": tag_counts["K3_ESCALATION_EVENT"],
            "sources": {"{}:{}".format(*key): count for key, count in source_counts.items()},
            "result_count": len(result_rows),
            "close_reasons": dict(close_counts),
            "tail_reasons": dict(tail_counts),
            "progress_candidates": sum(int(row.get("k3_better_candidate_count", 0)) for row in result_rows),
            "progress_selected": sum(int(row.get("k3_better_selected_count", 0)) for row in result_rows),
            "progress_activated": sum(int(row.get("activated_progress_count", 0)) for row in result_rows),
            "recovered_events": sum(row.get("binary_k3_recovered") == "1" for row in result_rows),
            "forecast_recovered_but_executed_window_all_invisible": sum(
                event.get("result", {}).get("binary_k3_recovered") == "1" and
                event.get("executed_window", {}).get("sample_count", 0) > 0 and
                event["executed_window"]["target_invisible_samples"] ==
                event["executed_window"]["sample_count"]
                for event in event_rows.values()),
            "side_both_failed": sum(int(row.get("side_both_failed_count", 0)) for row in result_rows),
            "alignment_samples": len(alignment_errors),
            "alignment_p95_m": percentile(alignment_errors, .95),
            "alignment_max_m": max(alignment_errors, default=None),
            "live_snapshot_drift_p95_m": percentile(live_drift, .95),
            "live_snapshot_drift_max_m": max(live_drift, default=None),
            "event_rows": event_rows,
        },
        "m2_risk_world_times": m2_windows,
        "liveness_log_mentions": dict(liveness),
    }


def _intervals(times, counts, threshold):
    begin = None
    result = []
    for index, count in enumerate(counts):
        if count < threshold and begin is None:
            begin = index
        if count >= threshold and begin is not None:
            result.append((times[begin], times[index - 1]))
            begin = None
    if begin is not None:
        result.append((times[begin], times[-1]))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    data = {run.name: run_metrics(run) for run in args.runs}
    args.output.write_text(json.dumps(data, indent=2, sort_keys=True))
    for run, value in data.items():
        print(run, "ALL3", value["visibility"]["all3_ratio"],
              "K2", value["visibility"]["k2_ratio"],
              "K3 events", value["k3"]["escalation_events"])


if __name__ == "__main__":
    main()
