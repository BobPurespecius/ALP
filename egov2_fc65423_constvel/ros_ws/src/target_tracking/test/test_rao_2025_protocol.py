#!/usr/bin/env python3

import copy
import importlib.util
import os
import unittest


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PACKAGE, "scripts", "rao_2025_protocol.py")
CONFIG = os.path.join(PACKAGE, "config", "rao_2025_protocol.yaml")
SPEC = importlib.util.spec_from_file_location("rao_2025_protocol", SCRIPT)
PROTOCOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROTOCOL)


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.data = PROTOCOL.load_protocol(CONFIG)

    def test_both_paper_profiles_validate(self):
        self.assertTrue(PROTOCOL.validate_protocol(self.data, "simulation"))
        self.assertTrue(PROTOCOL.validate_protocol(self.data, "comparison"))

    def test_table_one_is_comparison_only(self):
        manifest = PROTOCOL.build_manifest(self.data, "comparison", CONFIG)
        self.assertTrue(PROTOCOL.validate_manifest(manifest))
        self.assertEqual(manifest["paper_section"], "4.2")
        self.assertEqual(manifest["paper_scene"]["scene_size_m"], [50.0, 30.0, 5.0])
        self.assertEqual(manifest["paper_scene"]["obstacle_count"], 155)
        self.assertIn("reference_visibility_percent", manifest["paper_scene"])
        simulation = PROTOCOL.build_manifest(self.data, "simulation", CONFIG)
        self.assertNotIn("reference_visibility_percent", simulation["paper_scene"])

    def test_paper_value_drift_fails_validation(self):
        changed = copy.deepcopy(self.data)
        changed["paper"]["common_parameters"]["camera_fov_deg"] = [90.0, 72.0]
        with self.assertRaises(PROTOCOL.ProtocolError):
            PROTOCOL.validate_protocol(changed, "simulation")

    def test_manifest_rejects_profile_scene_mismatch(self):
        manifest = PROTOCOL.build_manifest(self.data, "simulation", CONFIG)
        manifest["paper_scene"]["obstacle_count"] = 155
        with self.assertRaises(PROTOCOL.ProtocolError):
            PROTOCOL.validate_manifest(manifest)

    def test_seed_streams_are_stable_and_profile_specific(self):
        first = PROTOCOL.derive_seed(2025, "simulation", "scene")
        self.assertEqual(first, PROTOCOL.derive_seed(2025, "simulation", "scene"))
        self.assertNotEqual(first, PROTOCOL.derive_seed(2025, "comparison", "scene"))
        self.assertNotEqual(first, PROTOCOL.derive_seed(2025, "simulation", "target_planner"))

    def test_visibility_uses_all_samples_as_denominator(self):
        accumulator = PROTOCOL.VisibilityAccumulator()
        accumulator.add([True, True, True])
        accumulator.add([True, True, False])
        accumulator.add([True, False, False])
        accumulator.add([False, False, False])
        self.assertEqual(accumulator.percentages(), {
            "visible_3": 25.0,
            "visible_2": 25.0,
            "visible_1": 25.0,
            "visible_0": 25.0,
        })

    def test_visibility_rejects_non_boolean_or_wrong_uav_count(self):
        accumulator = PROTOCOL.VisibilityAccumulator()
        with self.assertRaises(PROTOCOL.ProtocolError):
            accumulator.add([True, False])
        with self.assertRaises(PROTOCOL.ProtocolError):
            accumulator.add([1, 0, 0])


if __name__ == "__main__":
    unittest.main()
