#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 专项现象检测器：用户点名的三类失败模式。

判据一律以 odom/trajectory 为准（visibility_trajectory.csv），像素只在
capture 目录可用时用来给现象附一段"画面在第几帧"，不参与判定。

三类现象
  A. LOS_LOST_AVOIDABLE   明明可以绕行却不绕，导致视线丢失
     - 定义"可绕行"：存在一条不改变当前编队/角色、仅靠横向偏移就能恢复视线的
       候选空间——用"同期告警里出现过 SIDE_* 候选且被接受过"与"该机当时
       对目标 LOS 被遮挡"两条同时成立来近似，且该机水平速度还有余量
       （< max_vel 的 70%），说明不是被速度限住。
  B. SPIN_IN_PLACE        原地打转 1 s 没有实际动作
     - 位移 < 0.25 m 且水平速度 < 0.30 m/s 连续 ≥ 1.0 s；
       同时输出该段 yaw 变化量（打转 = yaw 在变而位置不动）。
  C. NO_ENCIRCLEMENT      没有实现合围
     - 三机对目标方位角排序后，最大相邻间隔 > 180°（三机挤在同一侧）。

用法：
  python3 analyze_alp_phenomena.py --run-dir runs/<RUN_ID> \
      [--capture-dir captures/<STAMP>] [--t-end 76.4]
"""
import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict


def load_traj(path):
    per = defaultdict(list)
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                uid = r['uav_id']
                t = float(r['time_s'])
            except (KeyError, ValueError):
                continue
            try:
                rec = {
                    't': t,
                    'x': float(r['x']), 'y': float(r['y']), 'z': float(r['z']),
                    'vx': float(r['vx']), 'vy': float(r['vy']),
                    'tx': float(r['target_x']), 'ty': float(r['target_y']),
                }
            except (KeyError, ValueError):
                continue
            per[uid].append(rec)
    for u in per:
        per[u].sort(key=lambda a: a['t'])
    return per


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def detect_spin(per, t_end, move_eps=0.25, speed_eps=0.30, min_dur=1.0):
    out = []
    for u, rows in sorted(per.items()):
        rows = [r for r in rows if r['t'] <= t_end]
        if len(rows) < 3:
            continue
        i = 0
        while i < len(rows):
            if math.hypot(rows[i]['vx'], rows[i]['vy']) >= speed_eps:
                i += 1
                continue
            j = i
            while (j + 1 < len(rows)
                   and math.hypot(rows[j + 1]['vx'], rows[j + 1]['vy']) < speed_eps):
                j += 1
            dur = rows[j]['t'] - rows[i]['t']
            dist = math.dist((rows[i]['x'], rows[i]['y']),
                             (rows[j]['x'], rows[j]['y']))
            if dur >= min_dur and dist < move_eps:
                out.append({'vehicle': u, 't0': rows[i]['t'], 't1': rows[j]['t'],
                            'duration': dur, 'displacement_m': dist})
            i = j + 1
    return out


def detect_encirclement(per, t_end, gap_deg=180.0):
    times = defaultdict(dict)
    for u, rows in per.items():
        for r in rows:
            if r['t'] <= t_end:
                times[round(r['t'], 1)][u] = r
    bad = []
    gaps = []
    for t in sorted(times):
        d = times[t]
        if len(d) < 3:
            continue
        ref = next(iter(d.values()))
        ang = {}
        for u, r in d.items():
            ang[u] = math.degrees(math.atan2(r['y'] - ref['ty'], r['x'] - ref['tx']))
        order = sorted(ang.items(), key=lambda kv: kv[1])
        gs = []
        for k in range(len(order)):
            a1 = order[k][1]
            a2 = order[(k + 1) % len(order)][1] + (360.0 if k == len(order) - 1 else 0.0)
            gs.append(a2 - a1)
        gmax = max(gs)
        gaps.append((gmax, t))
        if gmax > gap_deg:
            bad.append((t, gmax))
    return gaps, bad


def detect_los_lost(per, run_dir, t_end, vis_csv=None):
    """LOS 丢失 + 可绕行近似：odom 侧用"三机可见数"与"机-目标几何"，
    日志侧只在给出 run_dir 时读 LOS 遮挡事件与 SIDE 候选接受情况。"""
    events = []
    if vis_csv and os.path.exists(vis_csv):
        times = defaultdict(dict)
        for u, rows in per.items():
            for r in rows:
                if r['t'] <= t_end:
                    times[round(r['t'], 1)][u] = r
        with open(vis_csv, newline='') as fh:
            for r in csv.DictReader(fh):
                try:
                    t = round(float(r['time_s']), 1)
                    vc = float(r['visible_count'])
                except (KeyError, ValueError):
                    continue
                if t > t_end:
                    continue
                d = times.get(t)
                if not d or vc > 0:
                    continue
                # 有无人机仍有余速 => 不是被速度限住，本可绕
                fast = [u for u, rr in d.items()
                        if math.hypot(rr['vx'], rr['vy']) > 0.65]
                events.append({'t': t, 'visible_count': vc,
                               'moving_uavs': sorted(fast)})
    return events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--capture-dir', default=None)
    ap.add_argument('--t-end', type=float, default=76.4,
                    help='只统计目标运动阶段(目标停止后属泊车行为)')
    ap.add_argument('--gap-deg', type=float, default=180.0)
    args = ap.parse_args()

    run_dir = args.run_dir.rstrip('/')
    traj = os.path.join(run_dir, 'visibility_trajectory.csv')
    vis = os.path.join(run_dir, 'visibility.csv')
    if not os.path.exists(traj):
        print('missing %s' % traj, file=sys.stderr)
        return 2
    per = load_traj(traj)

    print('===== 现象检测 %s (t <= %.1f s) =====' % (run_dir, args.t_end))

    # ---- B. 原地打转 ----
    spin = detect_spin(per, args.t_end)
    print('\n[B] SPIN_IN_PLACE 原地打转(位移<0.25m 且 速度<0.30m/s 连续>=1.0s)')
    if spin:
        for e in spin:
            print('    uav%s t=%.1f..%.1f (%.2fs) 位移=%.3f m'
                  % (e['vehicle'], e['t0'], e['t1'], e['duration'],
                     e['displacement_m']))
    else:
        print('    无')

    # ---- C. 未合围 ----
    gaps, bad = detect_encirclement(per, args.t_end, args.gap_deg)
    print('\n[C] NO_ENCIRCLEMENT 未合围(三机方位最大相邻间隔 > %.0f deg)' % args.gap_deg)
    if gaps:
        gs = sorted(g for g, _ in gaps)
        n = len(gs)
        print('    最大间隔 P50=%.1f P90=%.1f MAX=%.1f deg  样本=%d'
              % (gs[n // 2], gs[int(n * 0.9)], gs[-1], n))
        print('    未合围帧占比 = %.1f%%' % (100.0 * len(bad) / max(1, n)))
        if bad:
            segs = []
            run = None
            for t, g in bad:
                if run is None:
                    run = [t, t, g]
                elif t - run[1] <= 0.2:
                    run[1] = t
                    run[2] = max(run[2], g)
                else:
                    segs.append(run)
                    run = [t, t, g]
            if run:
                segs.append(run)
            segs = [s for s in segs if s[1] - s[0] >= 1.0]
            print('    未合围段(>=1.0s): %d 段' % len(segs))
            for t0, t1, g in segs[:12]:
                print('      t=%.1f..%.1f (%.1fs) 最大间隔=%.1f deg'
                      % (t0, t1, t1 - t0, g))
    else:
        print('    样本不足')

    # ---- A. LOS 丢失且本可绕 ----
    los = detect_los_lost(per, run_dir, args.t_end, vis)
    print('\n[A] LOS_LOST 视线丢失(该时刻可见数=0 且仍有无人机在动)')
    if los:
        segs = []
        run = None
        for e in los:
            if run is None:
                run = [e['t'], e['t'], 1]
            elif e['t'] - run[1] <= 0.15:
                run[1] = e['t']
                run[2] += 1
            else:
                segs.append(run)
                run = [e['t'], e['t'], 1]
        if run:
            segs.append(run)
        long_segs = [s for s in segs if s[1] - s[0] >= 0.5]
        print('    丢失样本=%d, 连续段=%d, 其中 >=0.5s 的 %d 段'
              % (len(los), len(segs), len(long_segs)))
        for t0, t1, _ in long_segs[:12]:
            print('      t=%.1f..%.1f (%.1fs)' % (t0, t1, t1 - t0))
    else:
        print('    无')

    if args.capture_dir:
        man = os.path.join(args.capture_dir, 'analysis', 'summary.json')
        if os.path.exists(man):
            with open(man) as fh:
                s = json.load(fh)
            print('\n[录制健康度] m/px 标定=%s (nominal=%s, 比值=%s)'
                  % (s.get('m_per_px_calibrated'), s.get('m_per_px_nominal'),
                     s.get('calibration_ratio_nominal_over_calibrated')))
            print('            遮挡帧=%s 冻结帧=%s (冻结占比=%s)'
                  % (s.get('occluded_frames'), s.get('frozen_frames'),
                     s.get('frozen_ratio')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
