#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 抓帧工作流 S4:像素现象 -> odom 权威数字 -> 日志机制。

输入:capture 目录(S3 的 analysis/summary.json)与 run 目录
     (visibility_trajectory.csv + roslaunch_stdout.log)。

三步,顺序不可反:
  1. 对每个像素静止段/贴近段,用 visibility_trajectory.csv 的 odom
     (timestamp 列是 wall epoch,与 frames.jsonl 的 wall_ns 同钟)
     算出权威数字:该窗口内每机首末位置距离、最小机间/机目标距离。
     安全数字一律以 odom 为准,像素只负责定位。
  2. 对每个事件的 wall 时刻 ±0.6 s,从 roslaunch_stdout.log 拉相关行。
  3. 汇总全局日志事件计数,给出阅读提示。
"""
import argparse
import csv
import json
import math
import os
import re
import sys
from collections import Counter

LOG_MARKERS = [
    'CALL_REPLAN', 'post-deadline-recovery', 'POST_DEADLINE_RECOVERY',
    'NO_EXECUTABLE_SUCCESSOR', 'MOVING_SUCCESSOR_STARVATION',
    'SIDE_BOTH_FAILED', 'conflict-descriptor', 'COMMITTED',
    'TERMINAL_HOLD', 'execution-reserve', 'previous-safe-final-revalidation',
    'ReplanResult', 'failure_code', 'TARGET_ROUTE', 'completed',
    'EMERGENCY', 'collision', 'HARD_', 'Recovery', 'recovery',
]

LOG_TS = re.compile(r'\[INFO\] \[(\d+\.\d+)\]')


def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        for r in reader:
            if len(r) < len(header):
                continue
            try:
                rows.append({
                    'time_s': float(r[idx['time_s']]),
                    'uav': int(r[idx['uav_id']]),
                    'x': float(r[idx['x']]), 'y': float(r[idx['y']]),
                    'z': float(r[idx['z']]),
                    'vx': float(r[idx['vx']]), 'vy': float(r[idx['vy']]),
                    'vz': float(r[idx['vz']]),
                    'tx': float(r[idx['target_x']]),
                    'ty': float(r[idx['target_y']]),
                    'tz': float(r[idx['target_z']]),
                    'ts': float(r[idx['timestamp']]),
                })
            except (ValueError, KeyError):
                continue
    rows.sort(key=lambda r: r['ts'])
    return rows


def rows_in(rows, t0, t1):
    return [r for r in rows if t0 <= r['ts'] <= t1]


def odom_window_facts(rows, t0, t1):
    """窗口内的权威数字:每机首末位移 + 最小两两距离(含目标)。"""
    sel = rows_in(rows, t0, t1)
    facts = {}
    for uav in (1, 2, 3):
        pts = [r for r in sel if r['uav'] == uav]
        if len(pts) >= 2:
            a, b = pts[0], pts[-1]
            facts['uav%d_travel_m' % uav] = math.sqrt(
                (b['x'] - a['x']) ** 2 + (b['y'] - a['y']) ** 2 +
                (b['z'] - a['z']) ** 2)
    tgt = [r for r in sel if r['uav'] == 1]
    if len(tgt) >= 2:
        a, b = tgt[0], tgt[-1]
        facts['target_travel_m'] = math.sqrt(
            (b['tx'] - a['tx']) ** 2 + (b['ty'] - a['ty']) ** 2)
    if sel:
        t_mid = (t0 + t1) / 2.0
        near = sorted(sel, key=lambda r: abs(r['ts'] - t_mid))[:40]
        states = []
        seen = set()
        for r in near:
            key = (round(r['ts'], 2), r['uav'])
            if key in seen:
                continue
            seen.add(key)
            states.append((r['ts'], r['uav'], r['x'], r['y'], r['z'],
                           r['tx'], r['ty'], r['tz']))
        best = (1e9, None)
        for i in range(len(states)):
            for j in range(i + 1, len(states)):
                a, b = states[i], states[j]
                if abs(a[0] - b[0]) > 0.06:
                    continue
                pa = a[2:5] if a[1] != 0 else a[2:5]
                pb = b[2:5] if b[1] != 0 else b[2:5]
                # 目标没有独立 uav 行,目标位置挂在每行的 tx/ty/tz 上;
                # 机间距离用 (uav_i, uav_j),机目标距离用 (uav_i, target_of_i)
                d = math.sqrt((a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2 +
                              (a[4] - b[4]) ** 2)
                if d < best[0]:
                    best = (d, 'uav%d & uav%d' % (a[1], b[1]))
                dt = math.sqrt((a[2] - a[5]) ** 2 + (a[3] - a[6]) ** 2 +
                               (a[4] - a[7]) ** 2)
                if dt < best[0]:
                    best = (dt, 'uav%d & target' % a[1])
        if best[1]:
            facts['min_pair_dist_m'] = round(best[0], 4)
            facts['min_pair'] = best[1]
    return facts


def log_lines_near(path, wall_t, window=0.6, markers=None):
    hits = []
    try:
        with open(path, errors='ignore') as f:
            for line in f:
                m = LOG_TS.search(line)
                if not m:
                    continue
                try:
                    t = float(m.group(1))
                except ValueError:
                    continue
                if abs(t - wall_t) <= window:
                    if markers is None or any(k in line for k in markers):
                        hits.append((t, line.rstrip()[:500]))
    except IOError as exc:
        return [('ERR', str(exc))]
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture-dir', required=True)
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--log-window', type=float, default=0.6)
    args = ap.parse_args()

    with open(os.path.join(args.capture_dir, 'analysis', 'summary.json')) as f:
        summary = json.load(f)
    with open(os.path.join(args.capture_dir, 'frames.jsonl')) as f:
        first = json.loads(f.readline())
    capture_wall0 = first['wall_ns'] / 1e9

    csv_path = os.path.join(args.run_dir, 'visibility_trajectory.csv')
    if not os.path.exists(csv_path):
        print('[correlate] missing %s' % csv_path, file=sys.stderr)
        return 2
    rows = load_csv(csv_path)
    if not rows:
        print('[correlate] empty trajectory csv', file=sys.stderr)
        return 2
    log_path = os.path.join(args.run_dir, 'roslaunch_stdout.log')

    out = []
    out.append('# S4 相关性分析(capture=%s, run=%s)'
               % (args.capture_dir, args.run_dir))
    out.append('# CSV 覆盖 wall %.1f .. %.1f (time_s %.1f .. %.1f)'
               % (rows[0]['ts'], rows[-1]['ts'], rows[0]['time_s'],
                  rows[-1]['time_s']))
    csv_wall0 = rows[0]['ts']
    csv_t0 = rows[0]['time_s']
    offset = csv_t0 - csv_wall0  # wall epoch -> CSV time_s

    out.append('')
    out.append('## 像素静止段的权威复核(odom)')
    for e in summary.get('stall_segments', []):
        w0 = capture_wall0 + e['t0'] + offset
        w1 = capture_wall0 + e['t1'] + offset
        facts = odom_window_facts(rows, w0, w1)
        out.append('- %s t+%.1f..%.1f (%.1f s) px(%.0f,%.0f): %s'
                   % (e['vehicle'], e['t0'], e['t1'], e['duration'],
                      e['px'][0], e['px'][1], facts))
        hits = log_lines_near(log_path, (w0 + w1) / 2.0, args.log_window)
        for t, line in hits[:12]:
            out.append('    [%.2f] %s' % (t - capture_wall0, line))
        if not hits:
            out.append('    (no matching log lines in window)')

    out.append('')
    out.append('## 像素贴近段的权威复核(odom)')
    for name, info in sorted(summary.get('close_pairs_min', {}).items()):
        w0 = capture_wall0 + info['t_s'] + offset - 0.5
        w1 = capture_wall0 + info['t_s'] + offset + 0.5
        facts = odom_window_facts(rows, w0, w1)
        out.append('- %s 像素最小 %.3f m @ t+%.1f -> odom 复核: %s'
                   % (name, info['min_d'], info['t_s'], facts))
        hits = log_lines_near(log_path, (w0 + w1) / 2.0, args.log_window)
        for t, line in hits[:12]:
            out.append('    [%.2f] %s' % (t - capture_wall0, line))
        if not hits:
            out.append('    (no matching log lines in window)')

    out.append('')
    out.append('## 日志全局事件计数')
    counts = Counter()
    try:
        with open(log_path, errors='ignore') as f:
            for line in f:
                for marker in LOG_MARKERS:
                    if marker in line:
                        counts[marker] += 1
    except IOError as exc:
        out.append('(log missing: %s)' % exc)
    for k, v in counts.most_common():
        out.append('%8d  %s' % (v, k))
    if not counts:
        out.append('(no markers found)')

    out_path = os.path.join(args.capture_dir, 'analysis', 'log_correlation.txt')
    with open(out_path, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print('\n'.join(out[:40]))
    print('[correlate] full report -> %s' % out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
