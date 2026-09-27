#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 captures/<stamp>/frames/*.jpg 合成一个 mp4 视频。

为什么需要它：本机没有 ffmpeg（OpenCV 自带 ffmpeg 后端可用）。
抓帧链路（rviz_capture_frames.py）按固定周期把 RViz 窗口存成 jpg，
本脚本只做"帧序列 -> 视频"的无损打包，不改变任何时间语义。

输出：<capture-dir>/alp_capture.mp4
若帧间时间戳不均匀，按实际 wall_ns 差值累加推进，保证视频时长≈真实时长。

用法：
  python3 make_capture_video.py --capture-dir captures/<stamp> [--fps 10] [--scale 1.0]
"""
import argparse
import json
import os
import sys

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture-dir', required=True)
    ap.add_argument('--fps', type=float, default=10.0,
                    help='标称帧率(用于把真实时间差换算成重复帧数)')
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--codec', default='avc1',
                    help='avc1(H.264) / mp4v / MJPG')
    ap.add_argument('--out', default=None)
    ap.add_argument('--max-seconds', type=float, default=0.0,
                    help='>0 时只编码前 N 秒(调试用)')
    args = ap.parse_args()

    cap_dir = args.capture_dir.rstrip('/')
    index_path = os.path.join(cap_dir, 'frames.jsonl')
    if not os.path.exists(index_path):
        print('missing %s' % index_path, file=sys.stderr)
        return 2
    frames = []
    with open(index_path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    if not frames:
        print('empty frames.jsonl', file=sys.stderr)
        return 2

    # 尺寸取"众数尺寸"而不是首帧：抓帧开始时窗口可能还没就绪，
    # 首帧会是 400x287 的占位图（实测踩过），那会把整段视频压成小尺寸。
    from collections import Counter
    size_votes = Counter()
    for fr in frames:
        try:
            size_votes[(int(fr['width']), int(fr['height']))] += 1
        except (KeyError, TypeError, ValueError):
            continue
    if size_votes:
        w, h = size_votes.most_common(1)[0][0]
    else:
        first = cv2.imread(os.path.join(cap_dir, frames[0]['file']))
        if first is None:
            print('cannot read first frame', file=sys.stderr)
            return 2
        h, w = first.shape[:2]
    if args.scale != 1.0:
        w, h = int(w * args.scale), int(h * args.scale)

    out_path = args.out or os.path.join(cap_dir, 'alp_capture.mp4')
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*args.codec),
                             args.fps, (w, h))
    if not writer.isOpened():
        print('VideoWriter failed to open for codec=%s' % args.codec, file=sys.stderr)
        return 3

    t0 = frames[0]['wall_ns']
    written = 0
    missing = 0
    for i, fr in enumerate(frames):
        path = os.path.join(cap_dir, fr['file'])
        img = cv2.imread(path)
        if img is None:
            missing += 1
            continue
        if img.shape[1] != w // 1 and img.shape[0] != h:
            # 窗口未就绪的占位帧：直接跳过，不做放大（放大只会得到模糊块）
            if img.shape[1] < w or img.shape[0] < h:
                missing += 1
                continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        if args.scale != 1.0:
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        # 按真实时间差决定这一帧要重复几次：抓帧周期与 fps 不一致时
        # 仍能保持视频时长≈真实时长（不改变任何内容）。
        if i + 1 < len(frames):
            dt = (frames[i + 1]['wall_ns'] - fr['wall_ns']) / 1e9
        else:
            dt = 1.0 / args.fps
        reps = max(1, int(round(dt * args.fps)))
        for _ in range(reps):
            writer.write(img)
            written += 1
        if args.max_seconds > 0 and (fr['wall_ns'] - t0) / 1e9 >= args.max_seconds:
            break
    writer.release()

    size_mb = os.path.getsize(out_path) / (1024.0 * 1024.0)
    span = (frames[min(i, len(frames) - 1)]['wall_ns'] - t0) / 1e9
    print('[video] %s' % out_path)
    print('[video] codec=%s fps=%.2f size=%dx%d frames_written=%d '
          'missing_src=%d wall_span=%.1fs file=%.1fMB'
          % (args.codec, args.fps, w, h, written, missing, span, size_mb))
    return 0


if __name__ == '__main__':
    sys.exit(main())
