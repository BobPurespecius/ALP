#!/usr/bin/env python3
"""Compare planner-COMMANDED trajectory speed vs MEASURED odometry speed and
the trajectory replacement cadence, for ON vs OFF runs.

Timebase: the trajectory CSV carries both `time_s` (elapsed) and `timestamp`
(ROS epoch).  Activation log lines are epoch-only, so they are mapped onto the
elapsed timebase through the CSV.  The analysis window is derived from the
TARGET's own measured motion (target_vx/vy/vz), i.e. the moving phase only.
"""
import csv
import math
import os
import re
import sys
from collections import defaultdict

ACT_RE = re.compile(
    r"\[continuous-motion-activation\] drone=(\d+) source=(\S+) trajectory_id=(-?\d+) "
    r"activation_time=([-\d.eE]+) previous_source=(\S+) "
    r"speed_before_activation=([-\d.eE]+) speed_after_activation=([-\d.eE]+) "
    r"current_trajectory_end_speed=([-\d.eE]+) remaining_time_before_activation=([-\d.eE]+) "
    r"minimum_speed_previous_03_reference=([-\d.eE]+) minimum_speed_next_03_reference=([-\d.eE]+)")


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def load_track(path):
    """-> tracks{uid: [(t, speed, traj_id, source)]}, target [(t, speed)]"""
    tracks = defaultdict(list)
    target = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                uid = int(row["uav_id"])
                t = float(row["time_s"])
                vx, vy, vz = float(row["vx"]), float(row["vy"]), float(row["vz"])
                tv = math.sqrt(float(row["target_vx"]) ** 2 +
                               float(row["target_vy"]) ** 2 +
                               float(row["target_vz"]) ** 2)
            except (KeyError, ValueError):
                continue
            ts = row.get("timestamp", "")
            tracks[uid].append((t, math.sqrt(vx * vx + vy * vy + vz * vz),
                                int(float(row["trajectory_id"])), row["trajectory_source"],
                                ts))
            if uid == 1:
                target.append((t, tv))
    return tracks, target


def build_epoch_map(tracks):
    pairs = []
    for uid, rows in tracks.items():
        for row in rows:
            try:
                pairs.append((float(row[4]), row[0]))
            except (TypeError, ValueError):
                continue
    pairs.sort()
    return pairs


def epoch_to_elapsed(pairs, epoch):
    if not pairs:
        return None
    lo, hi = 0, len(pairs) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if pairs[mid][0] < epoch:
            lo = mid + 1
        else:
            hi = mid
    best = pairs[lo]
    for cand in (pairs[max(0, lo - 1)], pairs[lo]):
        if abs(cand[0] - epoch) < abs(best[0] - epoch):
            best = cand
    return best[1] if abs(best[0] - epoch) < 5.0 else None


def load_activations(path, pairs):
    acts = defaultdict(list)
    with open(path, errors="ignore") as handle:
        for line in handle:
            if "continuous-motion-activation" not in line:
                continue
            m = ACT_RE.search(line)
            if not m:
                continue
            rel = epoch_to_elapsed(pairs, float(m.group(4)))
            if rel is None:
                continue
            acts[int(m.group(1))].append({
                "source": m.group(2), "traj_id": int(m.group(3)), "t": rel,
                "prev_source": m.group(5), "v_before": float(m.group(6)),
                "v_after": float(m.group(7)), "end_speed": float(m.group(8)),
                "remaining": float(m.group(9)), "min_prev": float(m.group(10)),
                "min_next": float(m.group(11))})
    return acts


def moving_window(target):
    """Last elapsed time at which the target is still actually moving."""
    moving = [t for t, v in target if v > 0.30]
    if not moving:
        return None
    t_end = max(moving)
    t_start = min(t for t, v in target if v > 0.30)
    return (t_start, t_end)


def analyse(tag, log_path, csv_path):
    print("=" * 78)
    print("### {}".format(tag))
    tracks, target = load_track(csv_path)
    pairs = build_epoch_map(tracks)
    win = moving_window(target)
    if win is None:
        print("  no moving phase detected")
        return None
    lo, hi = win
    print("  moving phase = [{:.1f}, {:.1f}] s   (target speed>0.3 m/s)".format(lo, hi))

    summary = {"tag": tag, "lo": lo, "hi": hi, "uav": {}}
    for uid in sorted(tracks):
        rows = [r for r in tracks[uid] if lo <= r[0] <= hi]
        if not rows:
            continue
        speeds = [r[1] for r in rows]
        slow = [s for s in speeds if s < 0.10]
        summary["uav"][uid] = {
            "measured_p50": pct(speeds, 50), "measured_p10": pct(speeds, 10),
            "frac_slow": len(slow) / len(speeds)}
        print("  uav{} MEASURED speed P10/P50/P90 = {:.3f}/{:.3f}/{:.3f} m/s  frac<0.10={:.3f}".format(
            uid, pct(speeds, 10), pct(speeds, 50), pct(speeds, 90), len(slow) / len(speeds)))

    acts = load_activations(log_path, pairs)
    for uid in sorted(acts):
        ev = [e for e in acts[uid] if lo <= e["t"] <= hi]
        if len(ev) < 3:
            continue
        gaps = [b["t"] - a["t"] for a, b in zip(ev, ev[1:])]
        gaps = [g for g in gaps if g > 1e-9]
        v_after = [e["v_after"] for e in ev]
        v_before = [e["v_before"] for e in ev]
        end_speed = [e["end_speed"] for e in ev]
        src = defaultdict(int)
        for e in ev:
            src[e["source"]] += 1
        print("  uav{} activations={}".format(uid, len(ev)))
        print("     replace interval     P10/P50/P90 = {:.2f}/{:.2f}/{:.2f} s".format(
            pct(gaps, 10), pct(gaps, 50), pct(gaps, 90)))
        print("     COMMANDED v@activation P10/P50/P90 = {:.3f}/{:.3f}/{:.3f}  min={:.3f}".format(
            pct(v_after, 10), pct(v_after, 50), pct(v_after, 90), min(v_after)))
        print("     predecessor v@boundary P50 = {:.3f}  (dv = v_after - v_before) P50 = {:.4f}".format(
            pct(v_before, 50), pct([a - b for a, b in zip(v_after, v_before)], 50)))
        print("     new traj TERMINAL v  P10/P50/P90 = {:.3f}/{:.3f}/{:.3f}".format(
            pct(end_speed, 10), pct(end_speed, 50), pct(end_speed, 90)))
        print("     sources: " + ", ".join("{}={}".format(k, v) for k, v in
                                           sorted(src.items(), key=lambda kv: -kv[1])))
        rows = [r for r in tracks[uid] if lo <= r[0] <= hi]
        ratios = []
        for e in ev:
            near = min(rows, key=lambda r: abs(r[0] - e["t"]))
            if abs(near[0] - e["t"]) < 0.2 and e["v_after"] > 1e-6 and near[1] > 1e-6:
                ratios.append(e["v_after"] / near[1])
        if ratios:
            print("     COMMANDED/MEASURED ratio P10/P50/P90 = {:.2f}/{:.2f}/{:.2f}".format(
                pct(ratios, 10), pct(ratios, 50), pct(ratios, 90)))
        summary["uav"].setdefault(uid, {}).update({
            "acts": len(ev), "gap_p50": pct(gaps, 50),
            "cmd_p50": pct(v_after, 50), "end_p50": pct(end_speed, 50),
            "ratio_p50": pct(ratios, 50) if ratios else float("nan")})
    print()
    return summary


if __name__ == "__main__":
    targets = [
        ("encON_v3", "sim_encON_v3.log", "vis_encON_v3_trajectory.csv"),
        ("encOFF_v3", "sim_encOFF_v3.log", "vis_encOFF_v3_trajectory.csv"),
        ("v3_prev", "sim_v3.log", "vis_v3_trajectory.csv"),
        ("oldON", "sim_encON.log", "vis_encON_trajectory.csv"),
        ("oldOFF", "sim_encOFF.log", "vis_encOFF_trajectory.csv"),
    ]
    results = []
    for tag, lp, cp in targets:
        if not (os.path.exists(lp) and os.path.exists(cp)):
            print("skip {} (missing artifacts)".format(tag))
            continue
        try:
            got = analyse(tag, lp, cp)
            if got:
                results.append(got)
        except Exception as exc:  # noqa: BLE001
            print("{} failed: {!r}".format(tag, exc))
    print("=" * 78)
    print("SUMMARY (moving phase only)")
    print("{:<12} {:<5} {:>10} {:>10} {:>10} {:>10} {:>9}".format(
        "run", "uav", "gap_p50", "cmd_p50", "end_p50", "meas_p50", "cmd/meas"))
    for res in results:
        for uid in sorted(res["uav"]):
            u = res["uav"][uid]
            print("{:<12} {:<5} {:>10.2f} {:>10.3f} {:>10.3f} {:>10.3f} {:>9}".format(
                res["tag"], "uav{}".format(uid), u.get("gap_p50", float("nan")),
                u.get("cmd_p50", float("nan")), u.get("end_p50", float("nan")),
                u.get("measured_p50", float("nan")),
                "{:.2f}".format(u["ratio_p50"]) if u.get("ratio_p50") else "n/a"))
