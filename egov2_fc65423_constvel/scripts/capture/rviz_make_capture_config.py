#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ALP 抓帧工作流 S1:从生产 RViz 配置派生一份只读的 capture 配置。

只做纯可视化改动(全部写入 --report),生产配置一个字节不动:
  1. Orbit 视角改为俯视(Pitch=pi/2)且 Target Frame=alp_capture/target,
     由 capture 侧 annotator 节点广播该 TF —— 相机跟随目标红球,目标恒在画面中心。
     因为地图是长走廊,固定俯视+跟随 = 像素/世界之间是固定比例的相似变换,
     这是 S3 像素分析成立的前提。
  2. 相机距离按"可见世界高度"反算:distance = H_view / (2*tan(fov/2)),
     并自检反算结果,失败即退出(RViz 解析失败会静默回落默认视图,必须防)。
  3. 关闭绿色 Cylinder forest 点云显示(色相与森林圆柱重叠,干扰 HSV 识别;
     森林几何仍由绿色圆柱 marker 显示,信息不丢)。
  4. 折叠 Displays/Views 面板,避免面板遮住画面。

不改变任何仿真/规划行为:rviz 配置只影响 rviz 自己。
"""
import argparse
import json
import math
import sys

import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True, help='生产 rviz 配置(只读)')
    ap.add_argument('--out', required=True, help='派生配置输出路径(capture 目录内)')
    ap.add_argument('--report', required=True, help='改动清单输出路径')
    ap.add_argument('--target-frame', default='alp_capture/target',
                    help='相机跟随的 TF 帧(由 alp_capture_annotator.py 广播)')
    ap.add_argument('--visible-height', type=float, default=24.0,
                    help='期望的画面竖向世界覆盖(米),决定相机距离')
    ap.add_argument('--fov-deg', type=float, default=45.0,
                    help='rviz 相机竖向 FOV(度),与反算用的常数一致')
    ap.add_argument('--window', default='1848x1016', help='窗口宽x高')
    ap.add_argument('--disable-displays', default='Cylinder forest',
                    help='逗号分隔的要关闭的显示项名(纯可视化降噪)')
    args = ap.parse_args()

    with open(args.source, 'r') as f:
        cfg = yaml.safe_load(f)

    changes = []

    def record(path, before, after):
        changes.append({'path': path, 'before': before, 'after': after})

    vm = cfg.setdefault('Visualization Manager', {})
    record('/Visualization Manager/Fixed Frame', vm.get('Fixed Frame'), 'world')
    vm['Fixed Frame'] = 'world'

    views = vm.setdefault('Views', {})
    cur = views.setdefault('Current', {})
    old_class = cur.get('Class')
    if old_class != 'rviz/Orbit':
        # 不换 TopDownOrtho:实测它会被 RViz 静默回落到世界原点默认视图。
        cur = views['Current'] = {'Class': 'rviz/Orbit'}
    cur['Target Frame'] = args.target_frame
    fov_rad = math.radians(args.fov_deg)
    distance = args.visible_height / (2.0 * math.tan(fov_rad / 2.0))
    cur['Field Of View'] = round(fov_rad, 6)
    cur['Distance'] = round(distance, 3)
    cur['Pitch'] = round(math.pi / 2.0, 6)
    cur['Yaw'] = 0.0
    cur['Focal Point'] = {'X': 0, 'Y': 0, 'Z': 0}
    cur['Invert Z Axis'] = False
    cur['Near Clip Distance'] = 0.01 if not cur.get('Near Clip Distance') else cur['Near Clip Distance']
    record('/Views/Current', old_class, cur)

    # 自检:反算一次可见高度,必须与期望一致(浮点容差 1e-3)
    visible_h = 2.0 * distance * math.tan(fov_rad / 2.0)
    if abs(visible_h - args.visible_height) > 1e-3:
        print('[capture-config] FAIL visible height check: %.6f != %.6f'
              % (visible_h, args.visible_height), file=sys.stderr)
        sys.exit(2)

    disabled = []
    wanted = [s.strip().casefold() for s in
              args.disable_displays.split(',') if s.strip()]
    for name in wanted:
        for disp in vm.get('Displays', []):
            if str(disp.get('Name', '')).strip().casefold() == name \
                    and disp.get('Enabled', True):
                record('/Displays/%s/Enabled' % disp['Name'], True, False)
                disp['Enabled'] = False
                disabled.append(disp['Name'])

    # 关键:注册覆盖球话题的 Display。rviz 只渲染 Displays 里登记过的话题,
    # 生产配置里没有 /alp_capture/vehicle_overlays,不加这一项三机覆盖球
    # 永远不会出现在画面里(第四次运行实证)。
    has_overlay_display = any(
        isinstance(d.get('Marker Topic'), str)
        and d['Marker Topic'] == '/alp_capture/vehicle_overlays'
        for d in vm.get('Displays', []))
    if not has_overlay_display:
        overlay = {
            'Class': 'rviz/MarkerArray',
            'Enabled': True,
            'Marker Topic': '/alp_capture/vehicle_overlays',
            'Name': 'ALP Capture Overlays',
            'Namespaces': {},
            'Queue Size': 100,
            'Value': True,
        }
        vm.setdefault('Displays', []).append(overlay)
        record('/Displays/ALP Capture Overlays', None, 'added')

    for panel in cfg.get('Panels', []):
        if panel.get('Collapsed') is not True:
            record('/Panels/%s/Collapsed' % panel.get('Name'),
                   panel.get('Collapsed'), True)
            panel['Collapsed'] = True

    try:
        w_str, h_str = args.window.lower().split('x')
        win_w, win_h = int(w_str), int(h_str)
    except ValueError:
        print('[capture-config] BAD --window %r' % args.window, file=sys.stderr)
        sys.exit(2)
    geo = cfg.setdefault('Window Geometry', {})
    for key, val in (('Width', win_w), ('Height', win_h)):
        if geo.get(key) != val:
            record('/Window Geometry/%s' % key, geo.get(key), val)
            geo[key] = val

    with open(args.out, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)

    # 读回校验:RViz 解析失败会静默回落默认视图,这里必须把关键项验回来
    with open(args.out, 'r') as f:
        back = yaml.safe_load(f)
    cur_back = back['Visualization Manager']['Views']['Current']
    ok = (cur_back.get('Class') == 'rviz/Orbit'
          and cur_back.get('Target Frame') == args.target_frame
          and back['Visualization Manager'].get('Fixed Frame') == 'world')
    if not ok:
        print('[capture-config] FAIL read-back verification: %r' % cur_back,
              file=sys.stderr)
        sys.exit(2)

    with open(args.report, 'w') as f:
        json.dump({
            'source': args.source,
            'out': args.out,
            'target_frame': args.target_frame,
            'visible_height_m': args.visible_height,
            'fov_deg': args.fov_deg,
            'camera_distance_m': distance,
            'metres_per_pixel_at_height': args.visible_height / float(win_h),
            'window': [win_w, win_h],
            'disabled_displays': disabled,
            'changes': changes,
        }, f, indent=2, ensure_ascii=False)
    print('[capture-config] OK distance=%.2f m, visible_h=%.1f m, m/px=%.5f, '
          'disabled=%s, %d changes' % (distance, visible_h,
                                       args.visible_height / float(win_h),
                                       disabled, len(changes)))


if __name__ == '__main__':
    main()
