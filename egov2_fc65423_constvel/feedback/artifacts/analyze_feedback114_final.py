#!/usr/bin/env python3
"""Executed-truth audit of the single Feedback114 FULL ON run."""

import csv
import json
import re
import statistics
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "runs/20260925_200523_419199"
OUTPUT = ROOT / "feedback/artifacts/feedback114_final_metrics.json"


def fields(text):
    return dict(re.findall(r"(?:^|\s)([A-Za-z][A-Za-z0-9_]*)=([^\s]+)", text))


def event_window(created):
    begin, end = created["window"].strip("[]").split(",")
    return float(begin), float(end)


def main():
    with (RUN / "visibility.csv").open(newline="") as stream:
        visibility = list(csv.DictReader(stream))
    with (RUN / "visibility_trajectory.csv").open(newline="") as stream:
        trajectory = list(csv.DictReader(stream))
    offset = statistics.median(float(x["timestamp"]) - float(x["time_s"])
                               for x in trajectory if x["timestamp"])
    visible_samples = [(float(x["time_s"]) + offset, x) for x in visibility]
    own_trajectory = {u: [] for u in range(3)}
    for row in trajectory:
        own_trajectory[int(row["uav_id"]) - 1].append(row)

    events = {}
    counts = Counter()
    for line in (RUN / "feedback114_k3_node_events.log").open(errors="replace"):
        node = re.search(r"/drone_(\d+)_ego_planner_node", line)
        if not node:
            continue
        uav = int(node.group(1))
        for tag, entry in (("K3_LOCAL_EVENT_CREATED", "created"),
                           ("K3_RECOVERY_EVIDENCE", "evidence"),
                           ("K3_EVENT_RESULT", "result")):
            marker = "[{}]".format(tag)
            if marker not in line:
                continue
            data = fields(line.split(marker, 1)[1])
            key = (uav, int(data["event_id"]))
            event = events.setdefault(key, {"uav": uav, "event_id": key[1]})
            assert entry not in event, (key, entry)
            event[entry] = data
            event[entry + "_log_time"] = float(line.split(" ", 1)[0])
            counts[tag] += 1
    assert all("created" in e and "result" in e for e in events.values())

    missed = []
    unmatched_evidence = []
    for (uav, event_id), event in sorted(events.items()):
        created, result = event["created"], event["result"]
        begin, initial_end = event_window(created)
        close_time = event["result_log_time"]
        loss_time = float(created["predicted_binary_loss"])
        window_rows = [(t, row) for t, row in visible_samples
                       if begin <= t < initial_end and
                       event["created_log_time"] <= t <= close_time]
        visible_key = "visible_uav{}".format(uav + 1)
        saw_blocked = False
        recovered_after_block = []
        for t, row in window_rows:
            if row[visible_key] == "0":
                saw_blocked = True
            elif saw_blocked and (loss_time <= 0.0 or t >= loss_time):
                recovered_after_block.append(t)
        event["executed_initial_window"] = {
            "begin": begin, "end": initial_end,
            "event_created_log_time": event["created_log_time"],
            "event_closed_log_time": close_time,
            "samples": len(window_rows),
            "blocked_samples": sum(row[visible_key] == "0"
                                   for _, row in window_rows),
            "recovered_after_block_times": recovered_after_block,
        }
        if (result["binary_k3_recovered"] == "0" and
                result["close_reason"] != "M2_PREEMPTED" and
                recovered_after_block):
            missed.append({"uav": uav, "event_id": event_id,
                           "first_executed_recovery": recovered_after_block[0],
                           "initial_window_end": initial_end,
                           "close_reason": result["close_reason"]})
        evidence = event.get("evidence")
        if evidence:
            stamp = float(evidence["sample_world_time"])
            evidence_end = float(evidence["window_end"])
            nearest_time, nearest = min(visible_samples,
                key=lambda item: abs(item[0] - stamp))
            next_visible = next(((t, row) for t, row in visible_samples
                                 if t >= stamp and row[visible_key] == "1"), None)
            trajectory_match = min(own_trajectory[uav],
                key=lambda row: abs(float(row["timestamp"]) - stamp))
            checked = {
                "world_time": stamp,
                "window_begin": float(evidence["window_begin"]),
                "window_end": evidence_end,
                "traj_id": int(evidence["traj_id"]),
                "nearest_executed_delta_s": abs(nearest_time - stamp),
                "nearest_executed_visible": nearest[visible_key] == "1",
                "next_executed_visible_delay_s":
                    next_visible[0] - stamp if next_visible else None,
                "nearest_trajectory_id": int(trajectory_match["trajectory_id"]),
                "nearest_trajectory_delta_s": abs(
                    float(trajectory_match["timestamp"]) - stamp),
            }
            checked["source_and_identity_valid"] = (
                checked["window_begin"] <= stamp < evidence_end and
                checked["nearest_executed_delta_s"] <= 0.05 and
                checked["nearest_trajectory_delta_s"] <= 0.05 and
                checked["nearest_trajectory_id"] == checked["traj_id"] and
                result["recovery_observed"] == "1" and
                result["recovery_observed_traj_id"] == evidence["traj_id"])
            checked["executed_csv_visible_within_100ms"] = (
                next_visible is not None and
                0 <= checked["next_executed_visible_delay_s"] <= 0.1 and
                next_visible[0] < evidence_end)
            event["evidence_executed_check"] = checked
            if (not checked["source_and_identity_valid"] or
                    not checked["executed_csv_visible_within_100ms"]):
                unmatched_evidence.append({"uav": uav, "event_id": event_id,
                                       **checked})

    canonical = json.loads((RUN / "native_metrics_full.json").read_text())
    vis = canonical["visibility"]
    team = canonical["trajectory"]["team"]
    close_reasons = Counter(e["result"]["close_reason"] for e in events.values())
    failure_stages = Counter(e["result"]["failure_stage"] for e in events.values())
    output = {
        "run_id": RUN.name,
        "events": {"uav{}:event{}".format(*key): value
                   for key, value in sorted(events.items())},
        "k3": {
            "local_events": counts["K3_LOCAL_EVENT_CREATED"],
            "recovery_evidence_count": counts["K3_RECOVERY_EVIDENCE"],
            "recovered_events": close_reasons["RECOVERY_OBSERVED"],
            "close_reasons": dict(close_reasons),
            "failure_stages": dict(failure_stages),
            "missed_executed_recovery_count_definite_initial_window": len(missed),
            "missed_executed_recoveries": missed,
            "unmatched_recovery_evidence_count": len(unmatched_evidence),
            "unmatched_recovery_evidence": unmatched_evidence,
        },
        "visibility": {
            "all3": vis["all_visible_ratio"],
            "k2": vis["k2_ratio"],
            "mean_visible": vis["mean_visible_uavs"],
            "longest_all3_loss_s": vis["longest_all3_loss_s"],
            "longest_k2_loss_s": vis["longest_k2_loss_s"],
            "k3_loss_total_s": sum(
                float(visibility[i]["time_s"]) -
                float(visibility[i - 1]["time_s"])
                for i in range(1, len(visibility))
                if int(visibility[i - 1]["visible_count"]) < 3),
            "samples": vis["samples"],
        },
        "safety": {
            "static_contact_samples": sum(
                float(row["static_clearance_m"]) <= 0 for row in trajectory),
            "dynamic_contact_samples": sum(
                float(row["moving_clearance_m"]) <= 0 for row in trajectory),
            "swarm_violation_samples": team["swarm_violation_samples"],
            "unvalidated_executed_samples":
                team["unvalidated_executed_samples"],
            "min_static_clearance_m": team["min_static_clearance_m"],
            "min_dynamic_clearance_m": team["min_moving_clearance_m"],
            "min_swarm_separation_m": team["min_swarm_separation_m"],
        },
        "analysis_limit": "Missed count covers the event lifetime and initial "
                          "logged window. Later rolling windows can extend "
                          "without per-cycle window-end telemetry. Visibility "
                          "CSV can lag callback evidence by up to 100 ms; "
                          "source and identity checks are reported separately.",
    }
    OUTPUT.write_text(json.dumps(output, indent=2, sort_keys=True))
    print("events", output["k3"]["local_events"],
          "evidence", output["k3"]["recovery_evidence_count"],
          "missed_definite", len(missed),
          "unmatched_evidence", len(unmatched_evidence),
          "ALL3", output["visibility"]["all3"],
          "K2", output["visibility"]["k2"])


if __name__ == "__main__":
    main()
