#!/usr/bin/env python3

import os
import runpy


runpy.run_path(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "native_egov2_rviz_scene.py"),
    run_name="__main__",
)
