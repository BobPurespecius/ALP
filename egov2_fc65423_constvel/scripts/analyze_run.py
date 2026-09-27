#!/usr/bin/env python3
"""ALP 指标统一计算：只统计目标运动阶段（t <= TARGET_PHASE_END）。

用法：python3 scripts/analyze_run.py runs/<RUN_ID> [target_phase_end_s]

输出分四块：
  UAV  : 速度 / 方位误差 / 半径 / 高度偏差 / 可见性
  TEAM : K2 / blackout / all3 / mean_visible / min_uav_visibility / 最小夹角 / 环形间隙
  GEOM : 目标坐标系纵向与径向几何（判定"三机是否都跑到目标前方"）
  CHAIN: SIDE_BOTH_FAILED 分级链与 team transaction 计数
"""
import csv
import math
import os
import re
import subprocess
import sys
from collections import defaultdict


def pct(values, p):
    if not values:
        return float("nan")
    v = sorted(values)
    i = min(len(v) - 1, max(0, int(len(v) * p)))
    return v[i]


def load_traj(path, t_end):
    per = defaultdict(list)
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                u = int(r["uav_id"])
                t = float(r["time_s"])
            except (KeyError, ValueError):
                continue
            if t > t_end:
                continue
            try:
                per[u].append({
                    "t": t,
                    "x": float(r["x"]), "y": float(r["y"]), "z": float(r["z"]),
                    "vx": float(r["vx"]), "vy": float(r["vy"]),
                    "tx": float(r["target_x"]), "ty": float(r["target_y"]),
                    "tvx": float(r["target_vx"]), "tvy": float(r["target_vy"]),
                    "d": float(r["target_distance_m"]),
                })
            except (KeyError, ValueError):
                continue
    for u in per:
        per[u].sort(key=lambda a: a["t"])
    return per


def load_vis(path, t_end):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r["time_s"])
            except (KeyError, ValueError):
                continue
            if t <= t_end:
                rows.append(r)
    return rows


def uav_metrics(per):
    out = {}
    for u, rows in sorted(per.items()):
        sp = [math.hypot(r["vx"], r["vy"]) for r in rows]
        n = len(rows)
        low_eps = []
        run0 = None
        last = None
        for r in rows:
            s = math.hypot(r["vx"], r["vy"])
            if s < 0.30:
                if run0 is None:
                    run0 = r["t"]
                last = r["t"]
            else:
                if run0 is not None and last is not None and last - run0 > 0.30:
                    low_eps.append(last - run0)
                run0 = None
        if run0 is not None and last is not None and last - run0 > 0.30:
            low_eps.append(last - run0)
        out[u] = {
            "n": n,
            "speed_p10": pct(sp, 0.10), "speed_p50": pct(sp, 0.50),
            "speed_p90": pct(sp, 0.90),
            "below_030": 100.0 * sum(1 for s in sp if s < 0.30) / max(1, n),
            "low_eps": len(low_eps), "low_max": max(low_eps) if low_eps else 0.0,
            "zdev_p90": pct([abs(r["z"] - 1.5) for r in rows], 0.90),
            "zdev_max": max(abs(r["z"] - 1.5) for r in rows),
            "radius_p50": pct([math.hypot(r["x"] - r["tx"], r["y"] - r["ty"])
                               for r in rows], 0.50),
            "radius_p90": pct([math.hypot(r["x"] - r["tx"], r["y"] - r["ty"])
                               for r in rows], 0.90),
        }
    return out


def bearing_metrics(vis):
    out = {}
    for u in ("uav1", "uav2", "uav3"):
        key = f"{u}_relative_bearing_rad"
        vals = []
        for r in vis:
            v = r.get(key, "")
            try:
                vals.append(abs(math.degrees(float(v))))
            except (TypeError, ValueError):
                pass
        if vals:
            out[u] = (pct(vals, 0.50), pct(vals, 0.90), max(vals))
    return out


def vis_metrics(vis):
    def nums(key):
        out = []
        for r in vis:
            try:
                out.append(float(r[key]))
            except (KeyError, TypeError, ValueError):
                pass
        return out

    res = {}
    vc = nums("visible_count")
    if vc:
        n = len(vc)
        res["K2_RATIO"] = 100.0 * sum(1 for v in vc if v >= 2) / n
        res["ALL3_RATIO"] = 100.0 * sum(1 for v in vc if v >= 3) / n
        res["BLACKOUT_RATIO"] = 100.0 * sum(1 for v in vc if v == 0) / n
        res["MEAN_VISIBLE"] = sum(vc) / n
        # 最长 K2 丢失时长（连续 visible_count<2）
        worst = cur = 0.0
        for i, v in enumerate(vc):
            dt = (float(vis[i]["time_s"]) - float(vis[i - 1]["time_s"])) if i else 0.0
            if v < 2:
                cur += dt
                worst = max(worst, cur)
            else:
                cur = 0.0
        res["K2_LONGEST_LOSS"] = worst
    return res


def geom_metrics(per):
    """目标坐标系几何：纵向 s_i = (p_i - p_T)·v̂_T，径向 r_i = |p_i - p_T|。

    判定"三机是否都跑到目标前方"以及"两架远超目标"。
    """
    times = defaultdict(dict)
    for u, rows in per.items():
        for r in rows:
            times[round(r["t"], 2)][u] = r
    ahead_all = two_ahead = one_trail = 0
    s_all = []
    r_all = []
    n = 0
    for t in sorted(times):
        d = times[t]
        if len(d) < 3:
            continue
        ref = next(iter(d.values()))
        tvx, tvy = ref["tvx"], ref["tvy"]
        norm = math.hypot(tvx, tvy)
        if norm < 1e-6:
            continue
        ux, uy = tvx / norm, tvy / norm
        s = {}
        rr = {}
        for u, r in d.items():
            dx, dy = r["x"] - r["tx"], r["y"] - r["ty"]
            s[u] = dx * ux + dy * uy
            rr[u] = math.hypot(dx, dy)
        n += 1
        na = sum(1 for v in s.values() if v > 0.5)
        if na == 3:
            ahead_all += 1
        elif na == 2:
            two_ahead += 1
        if na == 2 and any(v < -0.5 for v in s.values()):
            one_trail += 1
        s_all.extend(s.values())
        r_all.extend(rr.values())
    if n == 0:
        return {}
    return {
        "samples": n,
        "ALL_THREE_AHEAD_RATIO": 100.0 * ahead_all / n,
        "TWO_FAR_AHEAD_RATIO": 100.0 * two_ahead / n,
        "ONE_TRAILING_TWO_AHEAD_RATIO": 100.0 * one_trail / n,
        "LONGITUDINAL_P50": pct(s_all, 0.50),
        "LONGITUDINAL_P90": pct(s_all, 0.90),
        "RADIUS_P50": pct(r_all, 0.50),
        "RADIUS_P90": pct(r_all, 0.90),
    }


def count_log(path, pattern):
    try:
        out = subprocess.run(["grep", "-ac", pattern, path],
                             capture_output=True, text=True, timeout=300)
        return int(out.stdout.strip() or 0)
    except Exception:
        return -1


def last_match(path, pattern):
    try:
        out = subprocess.run(["bash", "-c",
                              f"grep -ao '{pattern}' {path} | tail -1"],
                             capture_output=True, text=True, timeout=300)
        return out.stdout.strip()
    except Exception:
        return ""


def main():
    run_dir = sys.argv[1].rstrip("/")
    t_end = float(sys.argv[2]) if len(sys.argv) > 2 else 76.5
    log = os.path.join(run_dir, "roslaunch_stdout.log")
    if not os.path.exists(log):
        log_gz = log + ".gz"
        if os.path.exists(log_gz):
            log = log_gz

    print(f"===== {run_dir}  (target-moving phase only: t <= {t_end}s) =====")
    argv = os.path.join(run_dir, "roslaunch_argv.txt")
    if os.path.exists(argv):
        with open(argv) as fh:
            for line in fh:
                if re.search(r"joint|yaw|team_visibility", line):
                    print("ARGV:" + line.rstrip())

    vis = load_vis(os.path.join(run_dir, "visibility.csv"), t_end)
    for k, v in vis_metrics(vis).items():
        print(f"TEAM {k} = {v:.4f}" if isinstance(v, float) else f"TEAM {k} = {v}")
    for u, (p50, p90, mx) in sorted(bearing_metrics(vis).items()):
        print(f"BEARING {u} P50={p50:.2f} P90={p90:.2f} MAX={mx:.2f} (deg)")

    per = load_traj(os.path.join(run_dir, "visibility_trajectory.csv"), t_end)
    for u, m in sorted(uav_metrics(per).items()):
        print(f"UAV{u} n={m['n']} SPEED P10={m['speed_p10']:.3f} "
              f"P50={m['speed_p50']:.3f} P90={m['speed_p90']:.3f} "
              f"<0.30={m['below_030']:.2f}% LOW_EPISODES={m['low_eps']} "
              f"LOW_MAX={m['low_max']:.2f}s "
              f"ZDEV_P90={m['zdev_p90']:.3f} ZDEV_MAX={m['zdev_max']:.3f} "
              f"RADIUS_P50={m['radius_p50']:.2f} RADIUS_P90={m['radius_p90']:.2f}")
    g = geom_metrics(per)
    for k, v in g.items():
        print(f"GEOM {k} = {v:.4f}" if isinstance(v, float) else f"GEOM {k} = {v}")

    for key in ("TEAM_VIS_OPT_ATTEMPT", "EARLY_JOINT_OPT_ATTEMPT",
                "TEAM_VIS_OPT_SUCCESS", "multiview-joint",
                "TRANSACTION_JOINT_SUCCESS", "TRANSACTION_COMMIT",
                "TRANSACTION_ACTIVATED", "TRANSACTION_JOINT_REJECTED",
                "NO_PRIMARY_JOINT_PROPOSAL_WITHIN_BUDGET"):
        print(f"LOG {key} = {count_log(log, key)}")
    chain = last_match(log, "SIDE_BOTH_FAILED_NOMINAL_SELECTED=[0-9]* "
                            "SIDE_BOTH_FAILED_NOMINAL_CAPTURED=[0-9]* "
                            "SIDE_BOTH_FAILED_NOMINAL_SUBMITTED=[0-9]* "
                            "SIDE_BOTH_FAILED_NOMINAL_RECEIVED=[0-9]* "
                            "SIDE_BOTH_FAILED_NOMINAL_ACTIVATED=[0-9]*")
    if chain:
        print("CHAIN " + chain)


if __name__ == "__main__":
    main()
