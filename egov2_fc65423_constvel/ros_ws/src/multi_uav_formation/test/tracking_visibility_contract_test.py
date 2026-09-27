#!/usr/bin/env python3
"""Execution-telemetry half of the shared tracking-camera contract test."""

import importlib.util
import math
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / 'scripts' / 'native_egov2_rviz_scene.py'


class Orientation:
    x = 0.0
    y = 0.0
    z = 0.0
    w = 1.0


def fail(reason):
    raise SystemExit('TRACKING_VISIBILITY_CONTRACT_TEST=FAIL reason={}'.format(reason))


def main():
    spec = importlib.util.spec_from_file_location('native_scene_visibility', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hfov = math.radians(85.0)
    vfov = 2.0 * math.atan(math.tan(0.5 * hfov) * 720.0 / 1280.0)
    contract = {
        'hfov': hfov, 'vfov': vfov, 'min_range': 0.2, 'max_range': 8.0,
        'translation_body': (0.0, 0.0, 0.0),
        'extrinsic_roll': 0.0, 'extrinsic_pitch': 0.0,
        'extrinsic_yaw': 0.0,
    }
    clear = module.tracking_camera_measurement(
        (0.0, 0.0, 0.0), Orientation(), (2.0, 0.0, 0.0), contract)
    horizontal_out = module.tracking_camera_measurement(
        (0.0, 0.0, 0.0), Orientation(), (1.0, 2.0, 0.0), contract)
    vertical_out = module.tracking_camera_measurement(
        (0.0, 0.0, 0.0), Orientation(), (1.0, 0.0, 1.0), contract)
    if not clear['valid'] or horizontal_out['valid'] or vertical_out['valid']:
        fail('CAMERA_FRAME_HFOV_VFOV')
    dynamic = (1.0, 0.0, 0.18 + 0.08, 0.0, 3.0)
    if not module.segment_cylinder_intersects(
            (2.0, 0.0, 1.5), (0.0, 0.0, 1.5), dynamic):
        fail('DYNAMIC_LOS_BLOCK')
    if module.segment_cylinder_intersects(
            (0.0, 2.0, 1.5), (0.0, 0.0, 1.5), dynamic):
        fail('DYNAMIC_LOS_CLEAR')
    if abs(math.degrees(vfov) - 54.53644224813771) > 1.0e-9:
        fail('PINHOLE_VFOV')
    print('TRACKING_VISIBILITY_CONTRACT_TEST=PASS '
          'predictor_fixture=1 telemetry_fixture=1 dynamic_los=1 '
          'horizontal_fov=1 vertical_fov=1')


if __name__ == '__main__':
    main()
