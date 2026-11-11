"""sim/target_placement.py: GPS target to Gazebo world position. No ROS."""

import math
import sys
import unittest
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import target_placement as tp  # noqa: E402

# Origin of sim/worlds/argus_runway.sdf (the ardupilot_gazebo iris_runway origin).
HOME_LAT = -35.363262
HOME_LON = 149.165237
# package_delivery_demo.yaml target.
TARGET_LAT = -35.3631271
TARGET_LON = 149.1653479


class OffsetTest(unittest.TestCase):
    def test_north_matches_ardupilot_scaling(self):
        # ArduPilot: 1e-7 deg of latitude is 0.011131884502145034 m.
        north, east = tp.offset_ne_m(HOME_LAT, HOME_LON, HOME_LAT + 1.0e-4, HOME_LON)
        self.assertAlmostEqual(north, 11.131884502145034, places=6)
        self.assertAlmostEqual(east, 0.0, places=9)

    def test_east_is_scaled_by_cos_latitude(self):
        north, east = tp.offset_ne_m(HOME_LAT, HOME_LON, HOME_LAT, HOME_LON + 1.0e-4)
        expected = 11.131884502145034 * math.cos(math.radians(HOME_LAT))
        self.assertAlmostEqual(north, 0.0, places=9)
        self.assertAlmostEqual(east, expected, places=4)

    def test_signs_south_and_west_are_negative(self):
        north, east = tp.offset_ne_m(HOME_LAT, HOME_LON, HOME_LAT - 1.0e-4, HOME_LON - 1.0e-4)
        self.assertLess(north, 0.0)
        self.assertLess(east, 0.0)

    def test_demo_target_agrees_with_spherical_earth(self):
        # Independent check: small-angle offsets on a 6371 km sphere agree
        # with ArduPilot's constant to well under the target's 5 m size.
        north, east = tp.offset_ne_m(HOME_LAT, HOME_LON, TARGET_LAT, TARGET_LON)
        r = 6371008.8
        exp_north = math.radians(TARGET_LAT - HOME_LAT) * r
        exp_east = math.radians(TARGET_LON - HOME_LON) * r * math.cos(math.radians(HOME_LAT))
        self.assertAlmostEqual(north, exp_north, delta=0.05)
        self.assertAlmostEqual(east, exp_east, delta=0.05)
        # About 15 m north and 10 m east of home.
        self.assertTrue(14.0 < north < 16.0, north)
        self.assertTrue(9.5 < east < 11.0, east)

    def test_antimeridian_wraps(self):
        north, east = tp.offset_ne_m(0.0, 179.9999, 0.0, -179.9999)
        self.assertAlmostEqual(east, 2.0e-4 * 1.0e7 * tp.LOCATION_SCALING_FACTOR, places=6)


class WorldTest(unittest.TestCase):
    def test_reads_sim_world(self):
        name, origin = tp.read_world((SIM / "worlds" / "argus_runway.sdf").read_text())
        self.assertEqual(name, "argus_runway")
        self.assertEqual(origin, tp.GeoOrigin(HOME_LAT, HOME_LON, 584.0, 0.0))

    def test_world_xy_is_east_north(self):
        origin = tp.GeoOrigin(HOME_LAT, HOME_LON, 584.0, 0.0)
        x, y = tp.world_xy_for(origin, TARGET_LAT, TARGET_LON)
        north, east = tp.offset_ne_m(HOME_LAT, HOME_LON, TARGET_LAT, TARGET_LON)
        self.assertEqual((x, y), (east, north))

    def test_rotated_world_is_rejected(self):
        origin = tp.GeoOrigin(HOME_LAT, HOME_LON, 584.0, 90.0)
        with self.assertRaises(ValueError):
            tp.world_xy_for(origin, TARGET_LAT, TARGET_LON)

    def test_world_without_coordinates_is_rejected(self):
        with self.assertRaises(ValueError):
            tp.read_world("<sdf version='1.9'><world name='w'></world></sdf>")

    def test_sitl_location_is_world_origin(self):
        origin = tp.GeoOrigin(HOME_LAT, HOME_LON, 584.0, 0.0)
        self.assertEqual(tp.sitl_location_arg(origin), "-35.363262,149.165237,584.0,0.0")


class ServiceListTest(unittest.TestCase):
    def test_finds_create_services_only(self):
        listing = "\n".join(
            [
                "/gazebo/resource_paths/get",
                "/world/argus_runway/create",
                "/world/argus_runway/create_multiple",
                "/world/argus_runway/remove",
                "/world/other/create",
                "/world/argus_runway/create",
            ]
        )
        self.assertEqual(tp.worlds_from_service_list(listing), ["argus_runway", "other"])

    def test_empty_listing(self):
        self.assertEqual(tp.worlds_from_service_list(""), [])


if __name__ == "__main__":
    unittest.main()
