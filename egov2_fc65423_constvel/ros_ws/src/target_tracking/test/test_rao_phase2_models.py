#!/usr/bin/env python3

import json
import math
import os
import sys
import unittest

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE, "scripts"))

from rao_2025_protocol import load_protocol, validate_protocol
from rao_scene_model import Annulus, Cube, make_scene, scene_from_dict
from rao_target_model import HybridAStar, TargetState, integrate_primitive, propagate


CONFIG = os.path.join(PACKAGE, "config", "rao_2025_protocol.yaml")


class Phase2ModelTest(unittest.TestCase):
    def setUp(self):
        self.protocol = load_protocol(CONFIG)

    def test_scene_is_deterministic_and_profile_sized(self):
        for profile, expected_size, expected_count in (
                ("simulation", [65.0, 30.0, 4.0], 140),
                ("comparison", [50.0, 30.0, 5.0], 155)):
            validate_protocol(self.protocol, profile)
            first = make_scene(self.protocol, profile)
            second = make_scene(self.protocol, profile)
            self.assertEqual(first.as_json(), second.as_json())
            self.assertEqual(first.as_dict()["scene_size_m"], expected_size)
            self.assertEqual(len(first.obstacles), expected_count)
            self.assertEqual(sum(isinstance(item, Annulus) for item in first.obstacles),
                             round(expected_count * self.protocol["reproduction"]["scene_generation"]["annulus_fraction"]))
            self.assertTrue(all(isinstance(item, (Cube, Annulus)) for item in first.obstacles))

    def test_scene_json_round_trip_preserves_geometry(self):
        scene = make_scene(self.protocol, "simulation")
        restored = scene_from_dict(json.loads(scene.as_json()))
        self.assertEqual(scene.as_json(), restored.as_json())

    def test_bicycle_state_matches_paper_state_and_equations(self):
        state = TargetState.from_vector([0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0])
        next_state = propagate(state, 0.2, 0.4, 0.3, 2.5, 2.2)
        self.assertEqual(len(next_state.as_vector()), 8)
        self.assertAlmostEqual(next_state.speed, 1.62, places=6)
        self.assertAlmostEqual(next_state.curvature, math.tan(0.2) / 2.5, places=6)
        mid_speed = (1.5 + 1.62) / 2.0
        expected_lateral = mid_speed * mid_speed * next_state.curvature * 0.3
        self.assertAlmostEqual(next_state.lateral_accel, expected_lateral, places=6)

    def test_planner_and_executor_integrate_the_same_primitive(self):
        target_cfg = self.protocol["reproduction"]["target"]
        control_rate = self.protocol["reproduction"]["timing"]["control_rate_hz"]
        max_speed = self.protocol["paper"]["common_parameters"]["target_max_velocity_mps"]
        planner = HybridAStar(make_scene(self.protocol, "simulation"), target_cfg,
                              max_speed, control_rate)
        state = TargetState.from_vector(target_cfg["initial_state"]["simulation"])
        planned = planner._expand(state, 0.49, 0.45)
        executed = integrate_primitive(
            state, 0.49, 0.45, planner.dt, 1.0 / control_rate,
            planner.wheelbase, planner.max_speed, validator=planner._valid)
        self.assertIsNotNone(planned)
        self.assertEqual(planned.as_vector(), executed.as_vector())

    def test_hybrid_astar_matches_execution_for_two_cycles(self):
        target_cfg = self.protocol["reproduction"]["target"]
        max_speed = self.protocol["paper"]["common_parameters"]["target_max_velocity_mps"]
        control_rate = self.protocol["reproduction"]["timing"]["control_rate_hz"]
        for profile in ("simulation", "comparison"):
            scene = make_scene(self.protocol, profile)
            planner = HybridAStar(scene, target_cfg, max_speed, control_rate)
            state = TargetState.from_vector(target_cfg["initial_state"][profile])
            for cycle in range(2):
                for goal in target_cfg["waypoints"][profile]:
                    actions = planner.plan(state, goal)
                    self.assertGreater(len(actions), 0,
                                       "no route for {} cycle {} -> {}".format(profile, cycle, goal))
                    for steering, acceleration in actions:
                        state = integrate_primitive(
                            state, steering, acceleration, planner.dt, planner.integration_dt,
                            planner.wheelbase, planner.max_speed, validator=planner._valid)
                        self.assertIsNotNone(state)
                    self.assertLessEqual(
                        ((state.x - goal[0]) ** 2 + (state.y - goal[1]) ** 2) ** 0.5,
                        planner.goal_tolerance)


if __name__ == "__main__":
    unittest.main()
