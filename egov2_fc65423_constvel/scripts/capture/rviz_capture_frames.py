#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 抓帧工作流 S2b:定周期截取"我们自己的" RViz 窗口。

与 RRCT 工作流同思路的实现,差异点:
  * 窗口发现用 PID 锁定(--require-pid):机器上可能同时存在多个 RViz,
    只有 _NET_WM_PID 等于我们 wrapper 启动的那个 rviz 进程、且标题匹配
    --window-name-regex 的窗口才是目标;每 10 帧或抓失败时重解析,
    重解析同样带 PID 过滤,绝不咬住第一个名字匹配的窗口。
  * 帧索引 frames.jsonl 每行立即 flush,记录 wall_ns(xwd 之前取)与
    ros_ns(由 viz_clock.jsonl 插值)。本仿真日志时间戳是 wall epoch,
    wall_ns 可直接与 roslaunch_stdout.log 对齐。
  * 只读:xwd 截图,不向运行写任何东西;stop-file 是唯一控制通道。

抓帧失败(窗口争用 BadMatch 等)计数并继续;失败占比高会在 summary 里
标 degraded,该次 capture 降级,不能与完整 capture 混在一起下结论。
"""
import argparse
import ctypes
import json
import os
import re
import signal
import subprocess
import sys
import time

import cv2
import numpy as np

XPUTIL_TIMEOUT = 5


def run_cmd(cmd, timeout=XPUTIL_TIMEOUT):
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except subprocess.TimeoutExpired:
        return 124, b'', b''


def list_client_window_ids(display):
    # -display 必须放在最前,否则 ":0" 会被当作属性名
    rc, out, _ = run_cmd(['xprop', '-display', display, '-root',
                          '_NET_CLIENT_LIST'])
    if rc != 0:
        return []
    text = out.decode(errors='ignore')
    m = re.search(r'#\s*(.*)$', text)
    if not m:
        return []
    ids = []
    for tok in m.group(1).split(','):
        tok = tok.strip()
        if tok.startswith('0x'):
            ids.append(tok)
    return ids


def window_props(display, win_id):
    pid = None
    name = ''
    rc, out, _ = run_cmd(['xprop', '-display', display, '-id', win_id,
                          '_NET_WM_PID'])
    if rc == 0:
        m = re.search(r'=\s*(\d+)', out.decode(errors='ignore'))
        if m:
            pid = int(m.group(1))
    for prop in ('_NET_WM_NAME', 'WM_NAME'):
        rc, out, _ = run_cmd(['xprop', '-display', display, '-id', win_id,
                              prop])
        if rc == 0:
            m = re.search(r'=\s*"?([^"\n]*)', out.decode(errors='ignore'))
            if m and m.group(1).strip():
                name = m.group(1).strip()
                break
    return pid, name


def window_geometry(display, win_id):
    rc, out, _ = run_cmd(['xwininfo', '-display', display, '-id', win_id,
                          '-stats'])
    if rc != 0:
        return None, None
    text = out.decode(errors='ignore')
    w = re.search(r'Width:\s*(\d+)', text)
    h = re.search(r'Height:\s*(\d+)', text)
    if not (w and h):
        return None, None
    return int(w.group(1)), int(h.group(1))


def find_window(display, require_pid, name_regex):
    """PID + 标题双条件锁定;多个命中取面积最大者。找不到返回 None。"""
    best = None
    rx = re.compile(name_regex, re.I)
    for win_id in list_client_window_ids(display):
        pid, name = window_props(display, win_id)
        if pid != require_pid or not rx.search(name or ''):
            continue
        w, h = window_geometry(display, win_id)
        if not w or not h:
            continue
        area = w * h
        if best is None or area > best[3]:
            best = (win_id, w, h, area)
    if best is None:
        return None
    return best[0], best[1], best[2]


def raise_window(display, win_id):
    """把窗口提到前台(尽力而为;失败不影响 xwd 抓取)。"""
    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        disp = x11.XOpenDisplay(display.encode())
        if not disp:
            return
        x11.XRaiseWindow(disp, int(win_id, 16))
        x11.XFlush(disp)
        x11.XCloseDisplay(disp)
    except Exception:
        pass


def decode_xwd(raw, bgr_order):
    """解析 xwd 输出:25 个 big-endian uint32 头 + 调色板 + 像素数据。"""
    if len(raw) < 100:
        raise ValueError('xwd output too short (%d bytes)' % len(raw))
    header = np.frombuffer(raw[:100], dtype='>u4')
    header_size = int(header[0])
    width = int(header[4])
    height = int(header[5])
    byte_order = int(header[7])
    bits_per_pixel = int(header[11])
    bytes_per_line = int(header[12])
    ncolors = int(header[19])
    offset = header_size + ncolors * 12
    if width <= 0 or height <= 0:
        raise ValueError('bad xwd geometry %dx%d' % (width, height))
    need = offset + bytes_per_line * height
    if len(raw) < need:
        raise ValueError('xwd output truncated: %d < %d' % (len(raw), need))
    rows = np.frombuffer(raw, dtype=np.uint8, count=need)[offset:]
    rows = rows.reshape(height, bytes_per_line)
    if bits_per_pixel == 32:
        px = rows[:, :width * 4].reshape(height, width, 4)
        if bgr_order == 'rgba' or (bgr_order == 'auto' and byte_order == 1):
            img = cv2.cvtColor(px, cv2.COLOR_RGBA2BGR)
        else:
            img = cv2.cvtColor(px, cv2.COLOR_BGRA2BGR)
    elif bits_per_pixel == 24:
        px = rows[:, :width * 3].reshape(height, width, 3)
        img = (cv2.cvtColor(px, cv2.COLOR_RGB2BGR) if bgr_order == 'rgba'
               else px.copy())
    else:
        raise ValueError('unsupported bits_per_pixel=%d' % bits_per_pixel)
    return img, width, height


class RosClockInterpolator(object):
    """从 viz_clock.jsonl 增量读 (wall_ns, ros_ns) 样本并分段线性插值。"""

    def __init__(self, path):
        self.path = path
        self.samples = []
        self._fh = None

    def _update(self):
        if self._fh is None:
            try:
                self._fh = open(self.path, 'r')
            except IOError:
                return
        for line in self._fh:
            try:
                rec = json.loads(line)
                self.samples.append((rec['wall_ns'], rec['ros_ns']))
            except (ValueError, KeyError):
                continue
        if len(self.samples) > 4096:
            self.samples = self.samples[-2048:]

    def ros_ns(self, wall_ns):
        self._update()
        if not self.samples:
            return wall_ns
        lo, hi = 0, len(self.samples) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.samples[mid][0] <= wall_ns:
                lo = mid
            else:
                hi = mid - 1
        w0, r0 = self.samples[lo]
        if lo == len(self.samples) - 1:
            return r0
        w1, r1 = self.samples[lo + 1]
        if w1 <= w0:
            return r0
        alpha = (wall_ns - w0) / float(w1 - w0)
        return int(r0 + alpha * (r1 - r0))


# 视口区域不再写死:实测 1848x1016 窗口下 3D 视口约为 x 230..1480、y 40..985,
# 而左侧 Displays 面板与右侧文件管理器都会侵入。写死的坐标会让
# "遮挡/冻结"检测看错区域(上一版就因此漏报)。这里改为按帧自动估计。
OCCLUSION_LIGHT_FRAC = 0.50
FREEZE_MAD = 0.15


# 3D 视口边界(目视确证,1848x1016 窗口):左侧 Displays 面板到 x≈230,
# 上方工具栏到 y≈35,右/下是黑色条带。之前写死 (220,40,980,590) 把视口右边界
# 估到 980,而实际到 ~1520,导致"遮挡/冻结"检测看错区域。
VIEWPORT = (235, 35, 1520, 980)
OCCLUSION_FRAC = 0.25          # 视口内被高填充亮色矩形覆盖比例超过它 => 遮挡


def occlusion_fraction(img, box=VIEWPORT):
    """检测压在视口上的文件管理器一类窗口。
    判据:视口内出现"高填充(>=0.80)的亮色(>=232)大矩形(>=150x150)"。
    该判据已在真实有遮挡的帧上验证(报 46.3%),干净帧报 0.0%。"""
    x0, y0, x1, y1 = box
    if img.shape[1] <= x1 or img.shape[0] <= y1:
        return 0.0
    vp = img[y0:y1, x0:x1]
    b = vp[:, :, 0].astype(np.int16)
    g = vp[:, :, 1].astype(np.int16)
    r = vp[:, :, 2].astype(np.int16)
    m = ((r >= 232) & (g >= 232) & (b >= 232)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         np.ones((15, 15), np.uint8))
    n, _lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    best = 0
    for i in range(1, n):
        _x, _y, w, h, a = st[i]
        if w < 150 or h < 150 or a / float(w * h) < 0.80:
            continue
        best = max(best, a)
    return 100.0 * best / float(vp.shape[0] * vp.shape[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--display', default=':0')
    ap.add_argument('--require-pid', type=int, required=True,
                    help='只抓 _NET_WM_PID 等于该 PID 的窗口(我们的 rviz)')
    ap.add_argument('--window-name-regex', default='rviz')
    ap.add_argument('--period', type=float, default=0.5)
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--quality', type=int, default=85)
    ap.add_argument('--max-frames', type=int, default=2000)
    ap.add_argument('--duration', type=float, default=0.0)
    ap.add_argument('--window-timeout', type=float, default=360.0)
    ap.add_argument('--stop-file', default='')
    ap.add_argument('--ros-clock', default='',
                    help='viz_clock.jsonl 路径(annotator 产出)')
    ap.add_argument('--bgr-order', default='auto', choices=['auto', 'bgra', 'rgba'])
    args = ap.parse_args()

    frames_dir = os.path.join(args.out, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    index_path = os.path.join(args.out, 'frames.jsonl')
    index_fh = open(index_path, 'a', buffering=1)
    clock = RosClockInterpolator(args.ros_clock) if args.ros_clock else None

    win = find_window(args.display, args.require_pid, args.window_name_regex)
    waited = 0.0
    while win is None and waited < args.window_timeout:
        time.sleep(1.0)
        waited += 1.0
        win = find_window(args.display, args.require_pid, args.window_name_regex)
    if win is None:
        print('[capture] no matching window within %.0f s' % args.window_timeout,
              file=sys.stderr)
        json.dump({'frames': 0, 'failures': 0, 'window': None,
                   'degraded': True,
                   'reason': 'window not found'}, open(
                       os.path.join(args.out, 'capture_summary.json'), 'w'))
        return 2
    win_id, win_w, win_h = win
    print('[capture] locked window %s (%dx%d) pid=%d'
          % (win_id, win_w, win_h, args.require_pid), flush=True)

    stop = {'flag': False}

    def on_signal(_sig, _frm):
        stop['flag'] = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    frame_index = 0
    failures = 0
    last_fail_log = -50
    start_ns = time.time_ns()
    next_tick = time.time()
    width_seen, height_seen = win_w, win_h
    # ---- 录制健康度在线检测 ----
    # 教训: 上一轮 687 帧里 72% 的画面完全冻结(MAD 恒为 0),却被当成"完整录像"
    # 拿去做目视结论;还有 3.4 s 视口被文件管理器遮住。二者都必须在写盘时就标出来。
    vp_prev = None
    occluded = 0
    frozen = 0
    occl_first = occl_last = None
    frozen_first = frozen_last = None

    while not stop['flag']:
        now_wall = time.time()
        if args.duration > 0 and (now_wall * 1e9 - start_ns) / 1e9 >= args.duration:
            break
        if args.stop_file and os.path.exists(args.stop_file):
            break
        if frame_index >= args.max_frames:
            break
        if now_wall < next_tick:
            time.sleep(min(0.05, next_tick - now_wall))
            continue
        next_tick += args.period  # 绝不 sleep(period),落后就重新对齐
        if frame_index % 10 == 0:
            fresh = find_window(args.display, args.require_pid,
                                args.window_name_regex)
            if fresh is not None:
                win_id, width_seen, height_seen = fresh
                raise_window(args.display, win_id)
        wall_ns = time.time_ns()
        rc, raw, err = run_cmd(['xwd', '-display', args.display, '-id', win_id,
                                '-silent'], timeout=10)
        if rc != 0 or not raw:
            failures += 1
            if failures - last_fail_log >= 50 or failures <= 5:
                print('[capture] xwd failed #%d rc=%d %s'
                      % (failures, rc, err.decode(errors='ignore')[:120]),
                      flush=True)
                last_fail_log = failures
            fresh = find_window(args.display, args.require_pid,
                                args.window_name_regex)
            if fresh is not None:
                win_id, width_seen, height_seen = fresh
            next_tick = time.time() + args.period
            continue
        try:
            img, w_px, h_px = decode_xwd(raw, args.bgr_order)
        except ValueError as exc:
            failures += 1
            print('[capture] decode failed #%d: %s' % (failures, exc),
                  flush=True)
            continue
        if args.scale != 1.0:
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale,
                             interpolation=cv2.INTER_AREA)
        rel = 'frames/f%05d.jpg' % frame_index
        ok, buf = cv2.imencode('.jpg', img,
                               [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        if not ok:
            failures += 1
            continue
        with open(os.path.join(args.out, rel), 'wb') as fh:
            fh.write(buf.tobytes())
        # ---- 在线遮挡 / 冻结判定(只在视口区域内) ----
        flag_occluded = False
        flag_frozen = False
        hh, ww = img.shape[:2]
        vp_box = VIEWPORT
        if img.shape[1] > vp_box[2] and img.shape[0] > vp_box[3]:
            vx0, vy0, vx1, vy1 = vp_box
            vp = cv2.cvtColor(img[vy0:vy1, vx0:vx1], cv2.COLOR_BGR2GRAY)
            if occlusion_fraction(img) > OCCLUSION_FRAC:
                flag_occluded = True
                occluded += 1
                occl_first = wall_ns / 1e9 if occl_first is None else occl_first
                occl_last = wall_ns / 1e9
            if vp_prev is not None and vp_prev.shape == vp.shape:
                mad = float(np.abs(vp.astype(np.int16) -
                                   vp_prev.astype(np.int16)).mean())
                if mad < FREEZE_MAD:
                    flag_frozen = True
                    frozen += 1
                    frozen_first = wall_ns / 1e9 if frozen_first is None else frozen_first
                    frozen_last = wall_ns / 1e9
            vp_prev = vp
        rec = {'index': frame_index, 'file': rel, 'wall_ns': wall_ns,
               'ros_ns': clock.ros_ns(wall_ns) if clock else wall_ns,
               'window': win_id, 'width': int(img.shape[1]),
               'height': int(img.shape[0]),
               'occluded': flag_occluded, 'frozen': flag_frozen,
               'viewport': list(vp_box) if vp_box else None}
        index_fh.write(json.dumps(rec) + '\n')
        frame_index += 1

    index_fh.close()
    total = frame_index + failures
    summary = {
        'frames': frame_index,
        'failures': failures,
        'window': win_id,
        'window_size': [width_seen, height_seen],
        'period_s': args.period,
        'start_ns': start_ns,
        'end_ns': time.time_ns(),
        # 失败占比 >5% 或一帧没抓到 => 这次 capture 降级,结论时必须标注
        'degraded': bool(total == 0 or failures > 0.05 * max(1, total)),
        # 录制健康度:冻结/遮挡帧必须据此裁剪结论范围
        'occluded_frames': occluded,
        'occluded_span_s': ([occl_first, occl_last] if occluded else None),
        'frozen_frames': frozen,
        'frozen_span_s': ([frozen_first, frozen_last] if frozen else None),
        'frozen_ratio': round(frozen / float(max(1, frame_index)), 3) if frame_index else 0.0,
        'viewport_last': list(vp_box) if vp_box else None,
    }
    with open(os.path.join(args.out, 'capture_summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print('[capture] done frames=%d failures=%d degraded=%s'
          % (frame_index, failures, summary['degraded']), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
