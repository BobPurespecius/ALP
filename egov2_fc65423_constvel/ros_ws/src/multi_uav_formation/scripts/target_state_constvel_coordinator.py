#!/usr/bin/env python3

import os
import runpy


runpy.run_path(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "target_state_coordinator.py"),
    run_name="__main__",
)
