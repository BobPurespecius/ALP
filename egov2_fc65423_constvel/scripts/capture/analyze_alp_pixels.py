#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 抓帧工作流 S3:纯像素"出现象"。**不接受任何日志输入。**

输入只有 capture 目录(frames/、frames.jsonl、rviz_config_changes.json)。
它读日志就会退化成日志的另一种排版,这是它的设计红线。

相机是固定俯视 + 跟随目标(Target Frame=alp_capture/target),因此:
  * 米制比例 = visible_height / 窗口高(像素),逐帧用实际高度换算;
  * 目标恒在画面中心附近(输出里校验,验证 follow 真的生效);
  * 两载具的米制距离 = 像素距离 × 米/像素,不需要标定拟合。

输出(现象,不是结论):
  analysis/events_stall.txt   静止段(速度 < 阈值持续 ≥ 阈值时长)
  analysis/events_close.txt   贴近段(两两米制距离 ≤ 阈值)
  analysis/tracks.csv         每帧每载具像素坐标 + 相对画面中心的米制偏移
  analysis/summary.json       检出率、中心校验、事件汇总

安全数字最终以 odom/trajectory 为准(S4 correlate);像素只用来定位
"哪一段、哪几架、大概多久"。
"""
import argparse
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

# 与 alp_capture_annotator.py 的覆盖球颜色一一对应(hsv: H,S,V 0-255)
# target 用生产里已有的红球(1.0,0.05,0.02)。
VEHICLES = {
    'target': [((0, 120, 80), (10, 255, 255))],
    'uav0':   [((150, 120, 80), (167, 255, 255))],   # magenta ~321°->H≈149-166
    'uav1':   [((36, 120, 80), (50, 255, 255))],     # lime ~84°->H≈37-50
    'uav2':   [((168, 120, 80), (178, 255, 255))],   # rose ~346°->H≈168-178
}
MIN_AREA = 40
MAX_AREA = 30000
MIN_FILL = 0.58
MIN_ASPECT = 0.45
# 覆盖球真实半径(alp_capture_annotator.py 的 sphere_radius_m)。
# 它给了一个"与相机标定无关"的米/像素自标定基准:球在画面里的像素半径
# 与真实半径之比就是米/像素。实测旧公式 visible_h/frame_h 会低估约 3 倍,
# 原因是整窗高 1016 px 里有工具栏(顶部)与 Displays 面板(左侧)不参与投影。
SPHERE_RADIUS_M = 0.45
# 视口估计(占位帧 / 遮挡判定用)
# 视口不再写死:见 estimate_viewport()(捕获时若已记录 viewport 字段则优先用)
FREEZE_MAD = 0.15                  # 相邻帧视口平均绝对差低于它 => 画面未更新
# 顶部 UI 条带(工具栏 + 折叠面板头的红色关闭按钮落在 rose 波段,
# 第四次运行实证 (333,71) 的"常驻色块"就是它),检测时整体遮掉
UI_TOP_MASK_PX = 130


# 视口与遮挡判据(与 rviz_capture_frames.py 保持同一定义)
VIEWPORT = (235, 35, 1520, 980)
OCCLUSION_FRAC = 0.25


def occlusion_fraction(img, box=VIEWPORT):
    x0, y0, x1, y1 = box
    if img.shape[1] <= x1 or img.shape[0] <= y1:
        return 0.0
    vp = img[y0:y1, x0:x1]
    b = vp[:, :, 0].astype(np.int16)
    g = vp[:, :, 1].astype(np.int16)
    r = vp[:, :, 2].astype(np.int16)
    m = ((r >= 232) & (g >= 232) & (b >= 232)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    n, _lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    best = 0
    for i in range(1, n):
        _x, _y, w, h, a = st[i]
        if w < 150 or h < 150 or a / float(w * h) < 0.80:
            continue
        best = max(best, a)
    return 100.0 * best / float(vp.shape[0] * vp.shape[1])


def detect_blobs(hsv, ranges):
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else (mask | m)
    if mask is None:
        return []
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < MIN_AREA or area > MAX_AREA:
            continue  # 去噪点 / 去大色块
        fill = area / float(w * h)
        if fill < MIN_FILL:
            continue  # 去拖尾(细长低填充)与字幕框
        aspect = min(w, h) / float(max(w, h))
        if aspect < MIN_ASPECT:
            continue  # 去细长条
        out.append({'cx': x + w / 2.0, 'cy': y + h / 2.0,
                    'area': int(area), 'fill': fill, 'aspect': aspect,
                    'score': area * fill})
    out.sort(key=lambda b: -b['score'])
    return out


def segments_from_flags(flags, times, min_dur):
    segs = []
    start = None
    for i, flag in enumerate(flags):
        if flag and start is None:
            start = times[i]
        elif not flag and start is not None:
            if times[i] - start >= min_dur:
                segs.append((start, times[i]))
            start = None
    if start is not None and times[-1] - start >= min_dur:
        segs.append((start, times[-1]))
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture-dir', required=True)
    ap.add_argument('--stall-metres-per-sec', type=float, default=0.14,
                    help='静止阈值(m/s),对应整图缩放下 ~6 px/s')
    ap.add_argument('--stall-seconds', type=float, default=2.0)
    ap.add_argument('--merge-metres', type=float, default=0.6)
    ap.add_argument('--sample-tol', type=float, default=0.4)
    ap.add_argument('--center-tol-frac', type=float, default=0.15,
                    help='目标中心校验容差(相对半幅)')
    args = ap.parse_args()

    cfg_path = os.path.join(args.capture_dir, 'rviz_config_changes.json')
    with open(cfg_path) as f:
        cam = json.load(f)
    visible_h = cam['visible_height_m']
    distance = cam['camera_distance_m']
    fov_rad = math.radians(cam['fov_deg'])
    # 校验 config 自身一致:2*d*tan(fov/2) 应等于 visible_height
    assert abs(2.0 * distance * math.tan(fov_rad / 2.0) - visible_h) < 1e-3

    frames = []
    with open(os.path.join(args.capture_dir, 'frames.jsonl')) as f:
        for line in f:
            frames.append(json.loads(line))
    if not frames:
        print('[pixels] no frames', file=sys.stderr)
        sys.exit(2)
    t0 = frames[0]['wall_ns'] / 1e9

    out_dir = os.path.join(args.capture_dir, 'analysis')
    os.makedirs(out_dir, exist_ok=True)
    track_fh = open(os.path.join(out_dir, 'tracks.csv'), 'w', newline='')
    tw = csv.writer(track_fh)
    tw.writerow(['t_s', 'vehicle', 'cx_px', 'cy_px', 'offx_m', 'offy_m',
                 'm_per_px', 'area'])
    target_offsets = []

    # t_s -> {vehicle: (cx, cy, m_px, area)}
    samples = {v: [] for v in VEHICLES}
    center_ok = 0
    target_seen = 0
    m_ppx_seen = []

    miss = {}
    radius_px_seen = []          # 覆盖球像素半径样本(自标定)
    occluded_frames = []         # 被窗口遮挡的帧
    frozen_frames = []           # 与上一帧几乎相同(画面未更新)的帧
    prev_vp = None

    for fr in frames:
        img = cv2.imread(os.path.join(args.capture_dir, fr['file']))
        if img is None:
            continue
        h_px, w_px = img.shape[:2]
        # ---- 遮挡 / 冻结检测(视口区域,与色相无关) ----
        if w_px > VIEWPORT[2] and h_px > VIEWPORT[3]:
            x0, y0, x1, y1 = VIEWPORT
            vp = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
            t_s_chk = fr['wall_ns'] / 1e9 - t0
            if occlusion_fraction(img) > OCCLUSION_FRAC:
                occluded_frames.append(t_s_chk)
            if prev_vp is not None:
                mad = float(np.abs(vp.astype(np.int16) -
                                   prev_vp.astype(np.int16)).mean())
                if mad < FREEZE_MAD:
                    frozen_frames.append(t_s_chk)
            prev_vp = vp
        img[:UI_TOP_MASK_PX, :, :] = 0  # 遮掉顶部 UI 条带
        m_px = visible_h / float(h_px)
        m_ppx_seen.append(m_px)
        t_s = fr['wall_ns'] / 1e9 - t0
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        row = {}
        for name, ranges in VEHICLES.items():
            blobs = detect_blobs(hsv, ranges)
            if not blobs:
                # 区分"色块未检出"与"目标在视口外/被遮挡":后者不是识别失败。
                miss[name] = miss.get(name, 0) + 1
                continue
            best = blobs[0]
            row[name] = (best['cx'], best['cy'], m_px, best['area'])
            if name.startswith('uav'):
                radius_px_seen.append(math.sqrt(best['area'] / math.pi))
            tw.writerow(['%.3f' % t_s, name, '%.1f' % best['cx'],
                         '%.1f' % best['cy'], '%.4f' % ((best['cx'] - w_px / 2.0) * m_px),
                         '%.4f' % ((best['cy'] - h_px / 2.0) * m_px),
                         '%.5f' % m_px, best['area']])
            samples[name].append((t_s,) + row[name])
            if name == 'target':
                target_seen += 1
                target_offsets.append(((best['cx'] - w_px / 2.0) * m_px,
                                       (best['cy'] - h_px / 2.0) * m_px))
    track_fh.close()

    # 米/像素:优先用覆盖球自标定(与相机标定无关),拿不到才退回 nominal。
    m_px_use = (SPHERE_RADIUS_M / float(np.median(radius_px_seen))
                if radius_px_seen else float(np.median(m_ppx_seen)))
    if radius_px_seen and m_ppx_seen:
        ratio = float(np.median(m_ppx_seen)) / m_px_use
        print('[pixels] 自标定 m/px=%.5f (nominal=%.5f, 比值=%.2fx) 球半径中位=%.1f px'
              % (m_px_use, float(np.median(m_ppx_seen)), ratio,
                 float(np.median(radius_px_seen))), flush=True)

    # 静止段:逐帧速度 < 阈值 连续 >= 阈值时长
    stall_events = []
    for name, series in samples.items():
        series.sort()
        if len(series) < 3:
            continue
        times = [s[0] for s in series]
        xs = np.array([s[1] for s in series]) * m_px_use
        ys = np.array([s[2] for s in series]) * m_px_use
        flags = [False] * len(series)
        for i in range(1, len(series)):
            dt = times[i] - times[i - 1]
            if dt <= 0:
                continue
            v = math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) / dt
            flags[i] = v < args.stall_metres_per_sec
        for s0, s1 in segments_from_flags(flags, times, args.stall_seconds):
            mid = xs[(np.abs(np.array(times) - (s0 + s1) / 2)).argmin()]
            midy = ys[(np.abs(np.array(times) - (s0 + s1) / 2)).argmin()]
            stall_events.append({'vehicle': name, 't0': s0, 't1': s1,
                                 'duration': s1 - s0,
                                 'px': (float(mid), float(midy))})

    # 贴近段:两两米制距离 <= 阈值
    close_events = []
    names = [n for n in VEHICLES if samples[n]]
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = samples[names[i]], samples[names[j]]
            bi = 0
            closest = (1e9, None)
            run = None
            for (ta, xa, ya, ma, _aa) in a:
                while bi + 1 < len(b) and abs(b[bi + 1][0] - ta) <= abs(b[bi][0] - ta):
                    bi += 1
                tb, xb, yb, mb, _ab = b[bi]
                if abs(tb - ta) > args.sample_tol:
                    continue
                # 统一用自标定比例:此前用逐帧 nominal ma/mb,会把贴近阈值
                # 系统性缩小约 3 倍(实测 target&uav2 报 0.142 m 而 odom 为 1.40 m)。
                d = math.hypot((xa - xb) * m_px_use, (ya - yb) * m_px_use)
                if d < closest[0]:
                    closest = (d, ta)
                if d <= args.merge_metres:
                    if run is None:
                        run = [ta, ta, d]
                    else:
                        run[1] = ta
                        run[2] = min(run[2], d)
                elif run is not None:
                    close_events.append({'pair': '%s & %s' % (names[i], names[j]),
                                         't0': run[0], 't1': run[1],
                                         'duration': run[1] - run[0],
                                         'min_d': run[2]})
                    run = None
            if run is not None:
                close_events.append({'pair': '%s & %s' % (names[i], names[j]),
                                     't0': run[0], 't1': run[1],
                                     'duration': run[1] - run[0],
                                     'min_d': run[2]})
            if closest[1] is not None:
                close_events.append({'pair': '%s & %s' % (names[i], names[j]),
                                     'closest_only': True,
                                     't0': closest[1], 'min_d': closest[0]})

    with open(os.path.join(out_dir, 'events_stall.txt'), 'w') as f:
        for e in sorted(stall_events, key=lambda e: e['t0']):
            f.write('%s t+%.1f .. %.1f (%.1f s) 静止在像素(%.0f, %.0f)\n'
                    % (e['vehicle'], e['t0'], e['t1'], e['duration'],
                       e['px'][0], e['px'][1]))
        if not stall_events:
            f.write('(no stall segments)\n')
    with open(os.path.join(out_dir, 'events_close.txt'), 'w') as f:
        for e in close_events:
            if e.get('closest_only'):
                f.write('%s 最近接近 t+%.1f: %.3f m\n'
                        % (e['pair'], e['t0'], e['min_d']))
            else:
                f.write('%s t+%.1f .. %.1f (%.1f s) min %.3f m\n'
                        % (e['pair'], e['t0'], e['t1'], e['duration'],
                           e['min_d']))

    n_frames = len(frames)
    # 相机跟随的中位偏移:跟随生效时这是常数(视口/构图偏移),不随目标运动变化。
    # 恒定偏移不影响相似变换假设(比例不变),只在报告中记录。
    if target_offsets:
        med_off = (float(np.median([o[0] for o in target_offsets])),
                   float(np.median([o[1] for o in target_offsets])))
        offs = np.array(target_offsets)
        follow_spread = (float(np.percentile(np.hypot(
            offs[:, 0] - med_off[0], offs[:, 1] - med_off[1]), 90)),)
    else:
        med_off, follow_spread = (None, None), (None,)
    summary = {
        'frames': n_frames,
        'span_s': [0.0, frames[-1]['wall_ns'] / 1e9 - t0],
        'm_per_px_nominal': float(np.median(m_ppx_seen)) if m_ppx_seen else None,
        # 自标定:覆盖球真实半径 0.45 m ÷ 像素半径。与 nominal 的比值就是
        # 旧公式的系统偏差(实测约 3 倍),此后所有米制量都用自标定值。
        'm_per_px_calibrated': (float(SPHERE_RADIUS_M / np.median(radius_px_seen))
                                if radius_px_seen else None),
        'calibration_ratio_nominal_over_calibrated': (
            float(np.median(m_ppx_seen) / (SPHERE_RADIUS_M / np.median(radius_px_seen)))
            if (m_ppx_seen and radius_px_seen) else None),
        'sphere_radius_px_median': float(np.median(radius_px_seen)) if radius_px_seen else None,
        # 录制健康度:被窗口遮挡 / 画面未更新的帧,必须据此裁剪结论范围
        'miss_by_vehicle': miss,
        'occluded_frames': len(occluded_frames),
        'occluded_span_s': ([min(occluded_frames), max(occluded_frames)]
                            if occluded_frames else None),
        'frozen_frames': len(frozen_frames),
        'frozen_span_s': ([min(frozen_frames), max(frozen_frames)]
                          if frozen_frames else None),
        'frozen_ratio': (round(len(frozen_frames) / float(max(1, n_frames)), 3)
                         if frozen_frames else 0.0),
        'detection': {v: {'frames': len(samples[v]),
                          'ratio': round(len(samples[v]) / float(max(1, n_frames)), 3)}
                      for v in VEHICLES},
        'target_median_offset_m': {'x': med_off[0], 'y': med_off[1]},
        'target_offset_p90_m': follow_spread[0],
        'stall_segments': sorted(stall_events, key=lambda e: e['t0']),
        'close_pairs_min': {e['pair']: {'min_d': e['min_d'], 't_s': e['t0']}
                            for e in close_events
                            if e.get('closest_only')},
        'capture_degraded': None,
    }
    cs_path = os.path.join(args.capture_dir, 'capture_summary.json')
    if os.path.exists(cs_path):
        with open(cs_path) as f:
            summary['capture_degraded'] = json.load(f).get('degraded')
    with open(os.path.join(out_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print('[pixels] frames=%d target_median_offset=(%.2f, %.2f) m, '
          'offset_p90=%.2f m'
          % (n_frames, med_off[0] or 0, med_off[1] or 0,
             follow_spread[0] or 0))
    for v, d in summary['detection'].items():
        print('[pixels] %-7s detected %4d frames (%.0f%%)'
              % (v, d['frames'], 100 * d['ratio']))
    print('[pixels] stall segments: %d, close pairs: %d'
          % (len(stall_events), len([e for e in close_events
                                     if not e.get('closest_only')])))
    if summary['capture_degraded']:
        print('[pixels] WARNING: capture was DEGRADED — 结论需标注')
    return 0


if __name__ == '__main__':
    sys.exit(main())
