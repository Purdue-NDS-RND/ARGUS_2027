"""sim/touchdown_report.py: the pure part (parsing, tracking, maths). No ROS."""

import sys
import unittest
from pathlib import Path

import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import target_placement as tp  # noqa: E402
import touchdown_report as tr  # noqa: E402

MISSION = "package_delivery"
TARGET_XY = (10.0, 15.0)

ON_GROUND = tr.LANDED_STATE_ON_GROUND
TAKEOFF = tr.LANDED_STATE_TAKEOFF
IN_AIR = tr.LANDED_STATE_IN_AIR


def transition(old, new, mission=MISSION):
    return f"[{mission}] State: {old} -> {new}"


def final_column(alt, mission=MISSION):
    return (
        f"[{mission}] Entering final touchdown column at {alt:.2f} m "
        "(threshold 6.00 m, handoff tolerance 0.20 m)"
    )


def fly_normal_delivery(tracker):
    """Takeoff, transit, track, final column, touchdown, relaunch, RTL landing."""
    tracker.on_landed_state(0.0, ON_GROUND)
    tracker.on_pose(0.0, 0.0, 0.0, 0.0)
    tracker.on_landed_state(1.0, TAKEOFF)
    tracker.on_landed_state(3.0, IN_AIR)
    tracker.on_log(10.0, transition("WAITING_FOR_GPS", "TRANSIT_TO_TARGET"))
    tracker.on_log(30.0, transition("TRANSIT_TO_TARGET", "ACQUIRE_TARGET"))
    tracker.on_log(32.0, transition("ACQUIRE_TARGET", "TRACK_AND_DESCEND"))
    tracker.on_pose(70.0, 10.1, 14.9, 6.1)
    tracker.on_log(72.0, final_column(6.08))
    tracker.on_log(72.0, transition("TRACK_AND_DESCEND", "FINAL_FIXED_COLUMN_DESCENT"))
    tracker.on_pose(90.0, 10.3, 15.4, 0.05)
    tracker.on_landed_state(91.0, ON_GROUND)


class ParseTest(unittest.TestCase):
    def test_transition(self):
        self.assertEqual(
            tr.parse_transition(transition("ACQUIRE_TARGET", "TRACK_AND_DESCEND"), MISSION),
            ("ACQUIRE_TARGET", "TRACK_AND_DESCEND"),
        )

    def test_transition_inside_a_rosout_prefix(self):
        line = "[INFO] [1700000000.1] [mission_executor]: " + transition("A", "B")
        self.assertEqual(tr.parse_transition(line, MISSION), ("A", "B"))

    def test_other_mission_and_other_lines_are_ignored(self):
        other = transition("A", "B", mission="package_drop")
        self.assertIsNone(tr.parse_transition(other, MISSION))
        self.assertIsNone(tr.parse_transition("Mission sequence complete", MISSION))

    def test_final_column_altitude(self):
        self.assertAlmostEqual(tr.parse_final_column_altitude(final_column(6.08), MISSION), 6.08)
        self.assertAlmostEqual(tr.parse_final_column_altitude(final_column(-0.5), MISSION), -0.5)
        self.assertIsNone(tr.parse_final_column_altitude(final_column(6.0, "other"), MISSION))
        self.assertIsNone(tr.parse_final_column_altitude(transition("A", "B"), MISSION))


class TargetTest(unittest.TestCase):
    def test_reads_the_first_mission_with_a_gps_target(self):
        sequence = {
            "mission_sequence": {
                "missions": [
                    {"type": "takeoff", "name": "takeoff", "config": {}},
                    {
                        "type": "package_delivery",
                        "name": "deliver",
                        "config": {"target_latitude": -35.1, "target_longitude": 149.2},
                    },
                ]
            }
        }
        self.assertEqual(tr.mission_target(sequence), tr.Target("deliver", -35.1, 149.2))

    def test_no_target_is_an_error(self):
        with self.assertRaises(ValueError):
            tr.mission_target({"mission_sequence": {"missions": [{"type": "rtl"}]}})

    def test_default_sequence_and_world(self):
        # The real files: the same target spawn_target.py puts in Gazebo,
        # about 10 m east and 15 m north of home (sim/README.md).
        target = tr.mission_target(yaml.safe_load(tr.DEFAULT_SEQUENCE.read_text()))
        self.assertEqual(target.mission_name, MISSION)
        _, origin = tp.read_world(tr.DEFAULT_WORLD.read_text())
        east, north = tr.target_local_xy(origin, target)
        self.assertEqual(
            (east, north), tp.world_xy_for(origin, target.latitude_deg, target.longitude_deg)
        )
        self.assertAlmostEqual(east, 10.1, delta=0.5)
        self.assertAlmostEqual(north, 15.0, delta=0.5)

    def test_horizontal_distance(self):
        self.assertAlmostEqual(tr.horizontal_distance_m(13.0, 19.0, 10.0, 15.0), 5.0)


class TrackerTest(unittest.TestCase):
    def setUp(self):
        self.tracker = tr.TouchdownTracker(MISSION, TARGET_XY)

    def test_normal_delivery(self):
        fly_normal_delivery(self.tracker)
        self.assertTrue(self.tracker.done)
        report = self.tracker.report()
        self.assertAlmostEqual(report.final_descent_alt_m, 6.08)
        self.assertEqual(report.final_descent_alt_source, "mission log")
        self.assertEqual(report.touchdown_xyz, (10.3, 15.4, 0.05))
        self.assertAlmostEqual(report.distance_m, 0.5)
        self.assertAlmostEqual(report.track_time_s, 40.0)
        self.assertEqual(report.track_visits, 1)
        self.assertFalse(report.track_partial)
        self.assertEqual(report.notes, [])

    def test_starting_on_the_ground_is_not_a_touchdown(self):
        self.tracker.on_pose(0.0, 0.0, 0.0, 0.0)
        self.tracker.on_landed_state(0.0, ON_GROUND)
        self.tracker.on_landed_state(0.5, ON_GROUND)
        self.assertFalse(self.tracker.done)

    def test_landing_before_the_final_column_is_not_the_delivery(self):
        # With the mission log seen, a landing before FINAL_FIXED_COLUMN_DESCENT
        # (an abort and RTL, say) is not the delivery touchdown.
        self.tracker.on_landed_state(0.0, IN_AIR)
        self.tracker.on_log(1.0, transition("ACQUIRE_TARGET", "TARGET_NOT_FOUND"))
        self.tracker.on_pose(2.0, -1.0, -1.0, 0.0)
        self.tracker.on_landed_state(2.0, ON_GROUND)
        self.assertFalse(self.tracker.done)
        self.assertIn("no touchdown seen", self.tracker.report().notes)

    def test_track_time_adds_up_every_entry(self):
        self.tracker.on_landed_state(0.0, IN_AIR)
        self.tracker.on_log(10.0, transition("ACQUIRE_TARGET", "TRACK_AND_DESCEND"))
        self.tracker.on_log(25.0, transition("TRACK_AND_DESCEND", "TARGET_NOT_FOUND"))
        self.tracker.on_log(40.0, transition("ACQUIRE_TARGET", "TRACK_AND_DESCEND"))
        self.tracker.on_pose(60.0, 10.0, 15.0, 6.0)
        self.tracker.on_log(60.0, transition("TRACK_AND_DESCEND", "FINAL_FIXED_COLUMN_DESCENT"))
        report = self.tracker.report()
        self.assertAlmostEqual(report.track_time_s, 35.0)
        self.assertEqual(report.track_visits, 2)

    def test_started_late_marks_track_time_partial(self):
        self.tracker.on_log(5.0, transition("TRACK_AND_DESCEND", "FINAL_FIXED_COLUMN_DESCENT"))
        report = self.tracker.report()
        self.assertTrue(report.track_partial)
        self.assertAlmostEqual(report.track_time_s, 0.0)

    def test_still_tracking_counts_up_to_the_last_event(self):
        self.tracker.on_log(10.0, transition("ACQUIRE_TARGET", "TRACK_AND_DESCEND"))
        self.tracker.on_pose(16.5, 10.0, 15.0, 9.0)
        report = self.tracker.report()
        self.assertAlmostEqual(report.track_time_s, 6.5)
        self.assertIn("still in TRACK_AND_DESCEND when the report was made", report.notes)

    def test_final_altitude_falls_back_to_the_pose(self):
        self.tracker.on_pose(1.0, 10.0, 15.0, 6.3)
        self.tracker.on_log(1.0, transition("TRACK_AND_DESCEND", "FINAL_FIXED_COLUMN_DESCENT"))
        report = self.tracker.report()
        self.assertAlmostEqual(report.final_descent_alt_m, 6.3)
        self.assertEqual(report.final_descent_alt_source, "local pose at the state change")

    def test_relaunch_and_rtl_landing_do_not_move_the_touchdown(self):
        fly_normal_delivery(self.tracker)
        self.tracker.on_log(97.0, transition("GROUND_DWELL", "GUIDED_RELAUNCH"))
        self.tracker.on_landed_state(98.0, TAKEOFF)
        self.tracker.on_pose(150.0, 0.0, 0.0, 0.0)
        self.tracker.on_landed_state(150.0, ON_GROUND)
        self.assertEqual(self.tracker.report().touchdown_xyz, (10.3, 15.4, 0.05))

    def test_without_mission_log_first_landing_counts(self):
        self.tracker.on_landed_state(0.0, ON_GROUND)
        self.tracker.on_landed_state(1.0, IN_AIR)
        self.tracker.on_pose(50.0, 11.0, 15.0, 0.1)
        self.tracker.on_landed_state(50.0, ON_GROUND)
        report = self.tracker.report()
        self.assertTrue(self.tracker.done)
        self.assertAlmostEqual(report.distance_m, 1.0)
        self.assertIsNone(report.final_descent_alt_m)
        self.assertIsNone(report.track_time_s)
        self.assertTrue(any("no '[package_delivery]' lines" in n for n in report.notes))


class FormatTest(unittest.TestCase):
    def test_full_report(self):
        tracker = tr.TouchdownTracker(MISSION, TARGET_XY)
        fly_normal_delivery(tracker)
        text = tr.format_report(
            tracker.report(), tr.Target(MISSION, -35.3631271, 149.1653479), "demo.yaml"
        )
        self.assertIn("began at 6.08 m (mission log)", text)
        self.assertIn("x 10.30 m, y 15.40 m, z 0.05 m", text)
        self.assertIn("distance to target:          0.50 m horizontal", text)
        self.assertIn("40.0 s (1 entry)", text)
        self.assertNotIn("note:", text)

    def test_empty_report(self):
        tracker = tr.TouchdownTracker(MISSION, TARGET_XY)
        text = tr.format_report(tracker.report(), tr.Target(MISSION, 0.0, 0.0), "demo.yaml")
        self.assertIn("touchdown:                   not seen", text)
        self.assertIn("TRACK_AND_DESCEND:   not available", text)
        self.assertIn("note: no touchdown seen", text)


if __name__ == "__main__":
    unittest.main()
