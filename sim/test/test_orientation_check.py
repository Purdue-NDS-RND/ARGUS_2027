"""sim/orientation_check.py: the pure parts (sequence rewrite, verdict). Needs yaml."""

import sys
import unittest
from pathlib import Path

import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import orientation_check as oc  # noqa: E402


class SequenceTest(unittest.TestCase):
    def setUp(self):
        self.original = yaml.safe_load(oc.SEQUENCE.read_text())

    def test_only_target_moves(self):
        out = oc.check_sequence(self.original, -35.0, 149.0)
        missions = out["mission_sequence"]["missions"]
        delivery = next(m for m in missions if m["type"] == "package_delivery")
        self.assertEqual(delivery["config"]["target_latitude"], -35.0)
        self.assertEqual(delivery["config"]["target_longitude"], 149.0)
        # Everything else is the demo sequence, unchanged.
        for key in ("target_latitude", "target_longitude"):
            delivery["config"][key] = next(
                m for m in self.original["mission_sequence"]["missions"]
                if m["type"] == "package_delivery"
            )["config"][key]
        self.assertEqual(out, self.original)

    def test_original_is_not_modified(self):
        before = yaml.safe_dump(self.original)
        oc.check_sequence(self.original, 0.0, 0.0)
        self.assertEqual(yaml.safe_dump(self.original), before)

    def test_needs_exactly_one_target(self):
        with self.assertRaises(ValueError):
            oc.check_sequence({"mission_sequence": {"missions": [{"type": "rtl"}]}}, 0.0, 0.0)


class VerdictTest(unittest.TestCase):
    def test_correct_camera_passes(self):
        self.assertTrue(oc.verdict("east", 0.33, 0.02, 0.45, 0.0).ok)
        self.assertTrue(oc.verdict("north", -0.01, -0.58, 0.0, 0.45).ok)

    def test_mirrored_camera_fails(self):
        v = oc.verdict("east", -0.33, 0.02, -0.45, 0.0)
        self.assertFalse(v.detection_ok)
        self.assertFalse(v.setpoint_ok)
        self.assertFalse(oc.verdict("north", 0.0, 0.58, 0.0, -0.45).ok)

    def test_rotated_camera_fails(self):
        # East target seen above centre: a 90 degree rotation.
        self.assertFalse(oc.verdict("east", 0.05, -0.6, 0.05, 0.45).ok)
        self.assertFalse(oc.verdict("north", 0.6, -0.05, 0.45, 0.05).ok)

    def test_good_detection_bad_setpoint_fails(self):
        v = oc.verdict("east", 0.33, 0.0, -0.45, 0.0)
        self.assertTrue(v.detection_ok)
        self.assertFalse(v.setpoint_ok)
        self.assertFalse(v.ok)

    def test_offsets(self):
        self.assertEqual(oc.target_offset("east", 4.0), (4.0, 0.0))
        self.assertEqual(oc.target_offset("north", 4.0), (0.0, 4.0))
        with self.assertRaises(ValueError):
            oc.target_offset("west", 4.0)

    def test_yaw(self):
        # Quaternion for yaw 90 deg (facing north in ENU).
        s = 2 ** -0.5
        self.assertAlmostEqual(oc.yaw_enu_deg(0.0, 0.0, s, s), 90.0, places=6)
        self.assertAlmostEqual(oc.yaw_enu_deg(0.0, 0.0, 0.0, 1.0), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
