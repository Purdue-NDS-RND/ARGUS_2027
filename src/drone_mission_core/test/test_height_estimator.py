"""The height estimator (CEN-03): height above the ground, and which sensor said so.

The rules are tested on the ROS-free class with a known flight. The mission's
side is tested on the real executor and its real context
(`context.height_agl()`), with rclpy replaced by the stand-ins of
test_executor_feature_overrides.py: with `use_height_estimator` off nothing is
made and FND-07's height is unchanged; with it on the height is the
rangefinder's only when the reading is in the sensor's range, fresh, taken
under 15 degrees of tilt and plausible, and otherwise the height above home
plus the known ground offset, with the reason.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import executor_module, start  # noqa: E402, F401  (executor_module is a fixture)
from test_executor_target_estimator import BASE, HEIGHT, MS, POSE_TOPIC, S, VELOCITY_TOPIC, stamp, velocity_message, wiring  # noqa: E402, F401

from drone_mission_core import height_estimator as he  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402
from drone_mission_core.pose_buffer import PoseBuffer  # noqa: E402

FLAG = "use_height_estimator"
FLAGS = "use_frame_time_pose,use_height_estimator"
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
T0 = 100 * S


def tilted(deg: float):
    """A roll of `deg` degrees as a quaternion (x, y, z, w)."""
    half = math.radians(deg) / 2.0
    return (math.sin(half), 0.0, 0.0, math.cos(half))


class Flight:
    """A vehicle whose height and tilt the test sets, with a pose history, over ground `ground_up` above home."""

    def __init__(self, up_m=6.0, ground_up=0.0, tilt_deg=0.0, **settings):
        self.buffer = PoseBuffer()
        self.estimator = he.HeightEstimator(self.buffer.pose_at, **settings)
        self.up_m, self.ground_up, self.tilt_deg = up_m, ground_up, tilt_deg
        self.now_ns = T0

    def fly(self, seconds=0.4, ranges=True, range_m=None, min_m=0.0, max_m=1000.0):
        """Poses at 30 Hz and a reading with each. The reading is the true distance along the tilted beam unless `range_m` is given."""
        self.now_ns += 33 * MS
        end = self.now_ns + int(seconds * S)
        while self.now_ns <= end:
            self.buffer.add_pose(self.now_ns, (0.0, 0.0, self.up_m), tilted(self.tilt_deg))
            if ranges:
                true_range = (self.up_m - self.ground_up) / math.cos(math.radians(self.tilt_deg))
                self.estimator.add_range(self.now_ns, true_range if range_m is None else range_m, min_m, max_m)
            self.now_ns += 33 * MS
        self.now_ns -= 33 * MS
        return self

    def height(self, **kwargs):
        return self.estimator.height(self.up_m, self.now_ns, **kwargs)


# --- the rules, on the class ----------------------------------------------------


def test_in_range_and_level_the_height_is_the_rangefinders():
    height = Flight(up_m=6.0).fly().height()
    assert (height.source, height.reason) == ("rangefinder", "")
    assert height.value_m == pytest.approx(6.0, abs=1e-6)


def test_rangefinder_height_is_above_the_ground_it_measured_not_above_home():
    # The ground under the vehicle is 1.5 m above home: the barometer would say 6.0 m, the truth is 4.5 m.
    height = Flight(up_m=6.0, ground_up=1.5).fly().height()
    assert height.source == "rangefinder" and height.value_m == pytest.approx(4.5, abs=1e-6)


def test_a_tilted_reading_is_corrected_by_the_cosine_of_the_tilt():
    flight = Flight(up_m=6.0, tilt_deg=12.0).fly()
    assert flight.height().source == "rangefinder"
    assert flight.height().value_m == pytest.approx(6.0, abs=1e-6)  # the beam itself measured 6.13 m


def test_tilted_over_15_degrees_the_height_is_the_barometers_and_says_why():
    assert Flight(tilt_deg=14.9).fly().height().source == "rangefinder"
    height = Flight(up_m=6.0, tilt_deg=15.5).fly().height()
    assert height.source == "baro" and height.value_m == pytest.approx(6.0)
    assert "tilt" in height.reason and "15" in height.reason


def test_tilting_past_15_degrees_ends_the_rangefinder_height_at_once_though_level_readings_are_fresh():
    flight = Flight(up_m=6.0).fly()
    assert flight.height().source == "rangefinder"
    flight.now_ns += 33 * MS
    flight.buffer.add_pose(flight.now_ns, (0.0, 0.0, 6.0), tilted(20.0))  # a gust: tilted now, the last reading 33 ms ago was level
    height = flight.height()
    assert height.source == "baro" and "tilted 20 degrees now" in height.reason and height.value_m == pytest.approx(6.0)
    # The caller's own orientation, when it gives one, is the tilt now.
    assert flight.estimator.height(6.0, flight.now_ns, orientation=tilted(0.0)).source == "rangefinder"
    assert flight.estimator.height(6.0, flight.now_ns - 33 * MS, orientation=tilted(16.0)).source == "baro"
    flight.tilt_deg = 0.0
    flight.fly(seconds=0.0)  # level again, with a level reading
    assert flight.height().source == "rangefinder"


def test_a_reading_that_stays_the_same_while_the_vehicle_descends_is_a_stuck_sensor():
    hover = Flight(up_m=6.0).fly(seconds=3.0)  # a hover reads the same for seconds: that is not stuck
    assert hover.height().source == "rangefinder"

    flight = Flight(up_m=6.0).fly()
    for _ in range(4):  # the sensor goes on repeating 6.00 m, freshly stamped, while the vehicle comes down
        flight.up_m -= 0.04
        flight.fly(seconds=0.0, range_m=6.0)
    assert flight.height().source == "rangefinder"  # 0.16 m of travel: not yet told apart from a hover
    for _ in range(2):
        flight.up_m -= 0.04
        flight.fly(seconds=0.0, range_m=6.0)
    stuck = flight.height()
    assert stuck.source == "baro" and "has not changed" in stuck.reason and stuck.value_m == pytest.approx(flight.up_m)
    assert he.STUCK_TRAVEL_M == 0.2
    flight.fly(seconds=0.3)  # the sensor is back: true readings again
    assert flight.height().source == "rangefinder" and flight.height().value_m == pytest.approx(flight.up_m, abs=1e-6)


def test_a_stuck_sensor_is_caught_when_each_reading_arrives_before_the_pose_of_its_moment():
    # On the aircraft a reading can be stamped ahead of the newest pose when it arrives: the pose of its moment comes later.
    flight = Flight(up_m=6.0).fly()
    for _ in range(8):
        flight.up_m -= 0.04
        flight.now_ns += 50 * MS
        flight.estimator.add_range(flight.now_ns, 6.0, 0.0, 1000.0)  # the newest pose is 50 ms older: no pose for this stamp yet
        assert flight.buffer.pose_at(flight.now_ns) is None
        flight.buffer.add_pose(flight.now_ns + 10 * MS, (0.0, 0.0, flight.up_m), tilted(0.0))
    flight.now_ns += 10 * MS
    stuck = flight.height()
    assert stuck.source == "baro" and "has not changed" in stuck.reason


def test_out_of_the_sensors_range_the_height_is_the_barometers():
    # MAVROS's rangefinder topic gives limits of 0 and 1000 m, so the sensor's own (0.1 to 12 m) are the estimator's.
    assert (he.SENSOR_MIN_M, he.SENSOR_MAX_M) == (0.1, 12.0)
    high = Flight(up_m=13.0).fly().height()
    assert high.source == "baro" and high.value_m == pytest.approx(13.0) and "range" in high.reason
    low = Flight(up_m=0.08).fly().height()
    assert low.source == "baro" and "range" in low.reason
    assert Flight(up_m=11.5).fly().height().source == "rangefinder"
    assert Flight(up_m=0.3).fly().height().source == "rangefinder"


def test_three_readings_in_a_row_out_of_range_end_the_rangefinder_height_at_once_and_one_dropout_does_not():
    flight = Flight(up_m=11.5).fly()
    assert flight.height().source == "rangefinder"
    flight.up_m = 12.3  # climbing out of the sensor's range: the readings from 11.5 m are still fresh
    flight.fly(seconds=0.04)  # two readings out of range
    assert flight.height().source == "rangefinder"
    flight.fly(seconds=0.0)  # the third
    height = flight.height()
    assert height.source == "baro" and "range" in height.reason and height.value_m == pytest.approx(12.3)

    dropout = Flight(up_m=3.0).fly()
    dropout.fly(seconds=0.0, range_m=float("nan"))  # one reading lost
    assert dropout.height().source == "rangefinder" and dropout.height().value_m == pytest.approx(3.0, abs=1e-6)
    dropout.fly(seconds=0.1)
    assert dropout.height().source == "rangefinder"


def test_a_message_with_tighter_limits_than_the_sensors_is_held_to_them():
    assert Flight(up_m=6.0).fly(min_m=0.2, max_m=5.0).height().source == "baro"
    assert Flight(up_m=6.0).fly(min_m=0.2, max_m=8.0).height().source == "rangefinder"


def test_a_reading_that_is_not_a_number_is_not_used():
    for bad in (float("nan"), float("inf"), -1.0):
        assert Flight().fly(range_m=bad).height().source == "baro"


def test_no_readings_or_stale_readings_give_the_barometer_with_the_reason():
    none = Flight().fly(ranges=False).height()
    assert none.source == "baro" and "no rangefinder reading" in none.reason
    flight = Flight().fly()
    assert flight.height().source == "rangefinder"
    flight.fly(seconds=0.6, ranges=False)  # the rangefinder has gone quiet; poses go on
    stale = flight.height()
    assert stale.source == "baro" and "old" in stale.reason


def test_readings_that_disagree_or_put_the_ground_far_from_where_it_is_known_to_be_are_not_believed():
    flight = Flight(up_m=6.0).fly(seconds=0.2)
    flight.fly(seconds=0.04, range_m=3.0)  # something passes under the beam: two readings of five
    assert flight.height().source == "baro" and "disagree" in flight.height().reason
    flight.fly(seconds=0.3)  # and it has passed
    assert flight.height().source == "rangefinder" and flight.height().value_m == pytest.approx(6.0, abs=1e-6)
    blocked = Flight(up_m=11.0).fly(range_m=4.0).height()  # a steady return 7 m above where the ground is
    assert blocked.source == "baro" and "ground" in blocked.reason


def test_the_barometer_height_has_the_known_ground_offset_added():
    flight = Flight(up_m=13.0).fly()
    assert flight.height(ground_below_home_m=2.0).value_m == pytest.approx(15.0)
    assert flight.height(ground_below_home_m=-1.0).value_m == pytest.approx(12.0)
    # And the rangefinder is believed about ground that is where the offset says, however far from home's height.
    low_ground = Flight(up_m=3.0, ground_up=-8.0).fly()
    assert low_ground.height().source == "baro"  # 8 m below home with no offset given: not plausible
    known = low_ground.height(ground_below_home_m=8.0)
    assert known.source == "rangefinder" and known.value_m == pytest.approx(11.0, abs=1e-6)


def test_a_descent_between_readings_does_not_lag_the_height():
    # The reading is of the ground, which does not move: the height now is the vehicle's height now above it.
    flight = Flight(up_m=6.0, ground_up=0.4).fly()
    flight.up_m = 5.5  # 0.5 m lower since the last reading
    flight.buffer.add_pose(flight.now_ns + 33 * MS, (0.0, 0.0, 5.5), tilted(0.0))
    flight.now_ns += 33 * MS
    assert flight.height().value_m == pytest.approx(5.1, abs=1e-6)


def test_a_height_that_is_not_a_number_gives_none():
    assert Flight().fly().estimator.height(float("nan"), T0) is None


def test_the_source_is_always_one_of_the_two_names_missions_branch_on():
    from drone_mission_core import target_access

    assert (he.RANGEFINDER, he.BARO) == (target_access.RANGEFINDER, target_access.BARO) == ("rangefinder", "baro")
    for flight in (Flight().fly(), Flight(up_m=20.0).fly(), Flight().fly(ranges=False)):
        assert flight.height().source in ("rangefinder", "baro")


# --- the flag and the executor ---------------------------------------------------


def test_the_flag_needs_the_pose_history():
    assert he.HEIGHT_ESTIMATOR_FLAG == FLAG
    assert FLAG_REQUIRES[FLAG] == ("use_frame_time_pose",)


def test_every_shipped_sequence_with_a_vision_mission_declares_the_flag_off():
    import yaml

    sequences = Path(__file__).resolve().parents[2] / "drone_mission_demo" / "config" / "sequences"
    declaring = []
    for path in sorted(sequences.glob("*.yaml")):
        root = yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] in ("package_drop", "package_delivery") for entry in root["missions"]):
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
    assert len(declaring) == 8


def pose_message(ns, up=HEIGHT, tilt_deg=0.0):
    x, y, z, w = tilted(tilt_deg)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(ns)),
        pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=up), orientation=SimpleNamespace(x=x, y=y, z=z, w=w)),
    )


def range_message(ns, range_m):
    # As MAVROS's rangefinder plugin publishes it: limits of 0 and 1000 m whatever the sensor.
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(ns)), range=range_m, min_range=0.0, max_range=1000.0)


def fly(wiring, node, seconds, up=HEIGHT, tilt_deg=0.0, range_m=None):
    wiring.now_ns += 33 * MS
    end = wiring.now_ns + int(seconds * S)
    while wiring.now_ns <= end:
        message = pose_message(wiring.now_ns, up, tilt_deg)
        node._local_pose = message  # the executor's own 2026 pose callback is not under test here
        for callback in wiring.callbacks(POSE_TOPIC):
            callback(message)
        wiring.callbacks(VELOCITY_TOPIC)[0](velocity_message(wiring.now_ns))
        if range_m is not None:
            for callback in wiring.callbacks(RANGEFINDER_TOPIC):
                callback(range_message(wiring.now_ns, range_m))
        wiring.now_ns += 33 * MS
    wiring.now_ns -= 33 * MS


def test_flag_off_the_executor_makes_no_estimator_and_fnd_07s_height_is_unchanged(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides="use_frame_time_pose")
    assert node._height_estimator is None
    assert len(wiring.callbacks(RANGEFINDER_TOPIC)) == 1  # projection v2's own, as before
    # Tilted 20 degrees: FND-07's height still takes the rangefinder (its limit is projection v2's 25 degrees).
    fly(wiring, node, 0.4, up=6.0, tilt_deg=20.0, range_m=6.0 / math.cos(math.radians(20.0)))
    height = node._mission_context.height_agl()
    assert height.source == "rangefinder" and height.value_m == pytest.approx(6.0, abs=1e-6)

    plain = start(executor_module, monkeypatch)  # every flag off: the 2026 executor
    assert plain._height_estimator is None and plain._pose_buffer is None


def test_flag_on_without_the_pose_history_the_executor_does_not_start(executor_module, monkeypatch, wiring):
    with pytest.raises(ValueError, match="use_height_estimator requires use_frame_time_pose"):  # a FeatureFlagError, of the executor's own import
        start(executor_module, monkeypatch, feature_overrides=FLAG)


def test_flag_on_a_mission_reads_the_height_and_its_source_through_the_context(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch, feature_overrides=FLAGS)
    context = node._mission_context
    assert node._height_estimator is not None
    assert len(wiring.callbacks(RANGEFINDER_TOPIC)) == 2  # projection v2's and the estimator's own

    fly(wiring, node, 0.4, up=6.0, range_m=4.8)  # level at 6 m above home over ground 1.2 m above home
    height = context.height_agl()
    assert height.source == "rangefinder" and height.value_m == pytest.approx(4.8, abs=1e-6) and height.reason == ""

    # Tilted 20 degrees: the estimator falls back where FND-07's height would not, and on the first tilted pose.
    fly(wiring, node, 0.0, up=6.0, tilt_deg=20.0)
    assert context.height_agl().source == "baro" and "tilted 20 degrees now" in context.height_agl().reason
    fly(wiring, node, 0.6, up=6.0, tilt_deg=20.0, range_m=4.8 / math.cos(math.radians(20.0)))
    height = context.height_agl()
    assert height.source == "baro" and height.value_m == pytest.approx(6.0) and "tilt" in height.reason
    assert context.height_agl(ground_below_home_m=-1.2).value_m == pytest.approx(4.8)

    # Above the sensor's 12 m: the barometer, though MAVROS's limits (0 to 1000 m) would let the reading through.
    fly(wiring, node, 0.6, up=15.0, range_m=13.8)
    height = context.height_agl()
    assert height.source == "baro" and height.value_m == pytest.approx(15.0) and "range" in height.reason

    # A stalled pose stream gives no height at all, as before.
    wiring.now_ns += 600 * MS
    assert context.height_agl() is None


def test_flag_off_the_known_ground_offset_still_reaches_the_barometer_height(executor_module, monkeypatch, wiring):
    node = start(executor_module, monkeypatch)
    node._local_pose = pose_message(wiring.now_ns, up=10.0)
    assert node._mission_context.height_agl().value_m == pytest.approx(10.0)
    height = node._mission_context.height_agl(ground_below_home_m=1.5)
    assert (height.source, height.value_m) == ("baro", pytest.approx(11.5))
    assert node._mission_context.height_agl(ground_below_home_m=float("nan")) is None  # never a height that is not a number
