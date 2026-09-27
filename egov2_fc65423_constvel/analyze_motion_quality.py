#!/usr/bin/env python3
"""Motion-quality comparison on the SAME map (occlusion_forest_v3).

Window = target-moving phase only (target speed > 0.3 m/s), which is identical
in every run because the target route is shared.
"""
import csv
import math
import os
import sys
from collections import defaultdict


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))]


def load(path):
    tracks = defaultdict(list)
    target = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                uid = int(r["uav_id"]); t = float(r["time_s"])
                sp = math.hypot(float(r["vx"]), float(r["vy"]))
                tv = math.hypot(float(r["target_vx"]), float(r["target_vy"]))
            except (KeyError, ValueError):
                continue
            tracks[uid].append((t, sp, r["trajectory_source"]))
            if uid == 1:
                target.append((t, tv))
    return tracks, target


def segments(rows, thr, min_len=2):
    """Contiguous runs (in samples) with speed < thr."""
    out = []
    cur = 0
    for _, sp, _ in rows:
        if sp < thr:
            cur += 1
        else:
            if cur >= min_len:
                out.append(cur)
            cur = 0
    if cur >= min_len:
        out.append(cur)
    return out


def main(tag, path, lo, hi):
    tracks, _ = load(path)
    print("=" * 74)
    print("### {}   window [{:.0f},{:.0f}] s".format(tag, lo, hi))
    hdr = ("uav", "n", "P10", "P50", "P90", "<0.3", "seg<0.3", "maxseg", "<0.6", "seg<0.6")
    print("{:<5} {:>5} {:>6} {:>6} {:>6} {:>7} {:>8} {:>7} {:>7} {:>7}".format(*hdr))
    for uid in sorted(tracks):
        rows = [r for r in tracks[uid] if lo <= r[0] <= hi]
        if len(rows) < 10:
            continue
        sp = [r[1] for r in rows]
        s3 = segments(rows, 0.30)
        s6 = segments(rows, 0.60)
        print("{:<5} {:>5} {:>6.3f} {:>6.3f} {:>6.3f} {:>7.3f} {:>8} {:>7} {:>7.3f} {:>7}".format(
            "uav{}".format(uid), len(rows), pct(sp, 10), pct(sp, 50), pct(sp, 90),
            sum(1 for x in sp if x < 0.30) / len(sp),
            len(s3), max(s3) if s3 else 0,
            sum(1 for x in sp if x < 0.60) / len(sp),
            len(s6)))
    print()


if __name__ == "__main__":
    lo, hi = 5.0, 78.0
    for tag, path in (("encON_v3", "vis_encON_v3_trajectory.csv"),
                      ("encOFF_v3", "vis_encOFF_v3_trajectory.csv"),
                      ("v3_prev(ON)", "vis_v3_trajectory.csv"),
                      ("oldON", "vis_encON_trajectory.csv"),
                      ("oldOFF", "vis_encOFF_trajectory.csv")):
        if not os.path.exists(path):
            print("skip {}".format(tag)); continue
        main(tag, path, lo, hi)
