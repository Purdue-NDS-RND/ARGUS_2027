"""Centring on the estimator (CEN-01), in the drop and delivery missions through their real update().

With `centre_on_estimator` on, TRACK_AND_DESCEND steers on the estimator's
bullseye track (`context.primary_target`), not on the latest raw detection;
the point steered over the target is the payload's release point, not the
vehicle's origin; the 2026 low-pass filter and deadband are not applied; and
the descent goes on only while the context's descend rule says yes. With the
flag off it is the 2026 path and nothing of the estimator is asked.

The arithmetic (yaw, the payload's offset) is tested on the ROS-free module
first. rclpy and the message packages are the stand-ins from
test_package_drop_mission.py, so this runs without ROS.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml  # noqa: F401  (the real one, before the stand-ins put an empty module in its place)

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(SRC / "argus_camera"))
import test_package_drop_mission as stand_ins  # noqa: E402  (installs the ROS stand-ins, then imports the missions)

from drone_mission_core import centring, frame_projection, target_access  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions.package_delivery_mission import PackageDeliveryMission, PackageDeliveryState  # noqa: E402
from drone_mission_demo.missions.package_drop_mission import PackageDropMission, PackageDropState  # noqa: E402

FLAG = "centre_on_estimator"
FLAGS_ON = {"use_frame_time_pose": True, "use_target_estimator": True, FLAG: True}
SIM = {"camera_mode": "sim_delivery", "airframe": "sim_iris_downcam"}
BASE = {
    "target_latitude": 47.397742,
    "target_longitude": 8.545594,
    "target_loss_grace_s": 4.0,
    "centering_deadband_m": 0.4,  # the live files' deadband and filter: both must be bypassed with the flag on
    "tracking_low_pass_alpha": 0.12,
    "centering_gain_mps_per_m": 0.25,
    "centering_tolerance_m": 0.2,
    "max_centering_speed_mps": 2.0,
    "descent_rate_mps": 0.5,
    "fake_drop": True,
    **SIM,
}
KINDS = {
    "drop": (PackageDropMission, PackageDropState.TRACK_AND_DESCEND, {"drop_altitude_m": 4.0, "drop_column_handoff_altitude_m": 0.0, "drop_hover_dwell_s": 0.0}),
    "delivery": (PackageDeliveryMission, PackageDeliveryState.TRACK_AND_DESCEND, {"landing_check_threshold_m": 6.0}),
}
GAIN = 0.25


# --- the arithmetic ---------------------------------------------------------------


def quaternion(yaw_deg=0.0, roll_deg=0.0):
    roll, yaw = math.radians(roll_deg) / 2.0, math.radians(yaw_deg) / 2.0
    return (math.sin(roll) * math.cos(yaw), math.sin(roll) * math.sin(yaw), math.cos(roll) * math.sin(yaw), math.cos(roll) * math.cos(yaw))


def test_yaw_is_the_heading_of_the_nose_from_east_counter_clockwise():
    assert centring.yaw_of(quaternion(0.0)) == pytest.approx(0.0)
    assert centring.yaw_of(quaternion(90.0)) == pytest.approx(math.pi / 2.0)
    assert centring.yaw_of(quaternion(-135.0)) == pytest.approx(-0.75 * math.pi)
    assert centring.yaw_of(quaternion(90.0, roll_deg=8.0)) == pytest.approx(math.pi / 2.0, abs=1e-6)


def test_with_no_payload_offset_the_error_is_the_target_minus_the_vehicle():
    assert centring.aim_error((3.0, 1.0), (2.0, 5.0), math.radians(37.0), (0.0, 0.0, 0.0)) == pytest.approx((1.0, -4.0))


def test_the_payload_not_the_vehicle_is_put_over_the_target():
    # The release point is 0.30 m ahead of the flight controller and 0.10 m to its left.
    offset = (0.30, 0.10, -0.25)
    # Nose east: ahead is east, left is north. The vehicle must sit 0.30 m west and 0.10 m south of the target.
    assert centring.aim_error((10.0, 20.0), (10.0, 20.0), 0.0, offset) == pytest.approx((-0.30, -0.10))
    # Nose north (the missions hold a yaw of 90 degrees): ahead is north, left is west.
    assert centring.aim_error((10.0, 20.0), (10.0, 20.0), math.pi / 2.0, offset) == pytest.approx((0.10, -0.30))
    # And once the vehicle is there, the error is zero: the payload is over the target.
    vehicle = (10.0 + 0.10, 20.0 - 0.30)
    assert centring.aim_error((10.0, 20.0), vehicle, math.pi / 2.0, offset) == pytest.approx((0.0, 0.0), abs=1e-12)
    payload = (vehicle[0] + 0.30 * math.cos(math.pi / 2) - 0.10 * math.sin(math.pi / 2), vehicle[1] + 0.30 * math.sin(math.pi / 2) + 0.10 * math.cos(math.pi / 2))
    assert payload == pytest.approx((10.0, 20.0))


def test_the_flag_needs_the_estimator_and_every_drop_and_delivery_sequence_declares_it_off():
    assert centring.CENTRE_FLAG == FLAG and FLAG_REQUIRES[FLAG] == ("use_target_estimator",)
    import yaml as real_yaml

    declaring = []
    for path in sorted((SRC / "drone_mission_demo" / "config" / "sequences").glob("*.yaml")):
        root = real_yaml.safe_load(path.read_text())["mission_sequence"]
        if any(entry["type"] in ("package_drop", "package_delivery") for entry in root["missions"]):
            assert root["features"].get(FLAG) is False, path.name
            declaring.append(path.name)
    assert len(declaring) == 12


def test_the_payload_offset_comes_from_the_robot_description_of_the_missions_airframe(tmp_path):
    assert frame_projection.payload_offset_from_config(SIM, "package_drop") == (0.0, 0.0, 0.0)
    with pytest.raises(frame_projection.FrameProjectionError, match="package_drop.*airframe"):
        frame_projection.payload_offset_from_config({}, "package_drop")
    with pytest.raises(frame_projection.FrameProjectionError, match="argus_2027.*not been measured|argus_2027.*verified"):
        frame_projection.payload_offset_from_config({"airframe": "argus_2027"}, "package_drop")  # nobody has measured it yet
    # The sim's stand-in for a vehicle whose release point is not under the flight controller (scenario drop/hold_6m_payload).
    assert frame_projection.payload_offset_from_config({"airframe": "sim_iris_payload_ahead"}, "package_drop") == (0.30, 0.10, -0.25)
    assert frame_projection.load_camera("sim_delivery", "sim_iris_payload_ahead")[1] == frame_projection.load_camera("sim_delivery", "sim_iris_downcam")[1]
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    (config_dir / "robot.yaml").write_text(
        "airframes:\n  sim_bench:\n    verified: true\n    cameras:\n      down:\n        optical_frame: down_camera_optical\n"
        "        position_m: [0.0, 0.0, 0.0]\n        image_yaw_deg: 0.0\n    payload_offset_m: [0.30, 0.10, -0.25]\n"
    )
    assert frame_projection.payload_offset_from_config({"airframe": "sim_bench"}, "package_drop", config_dir) == (0.30, 0.10, -0.25)


# --- the missions -----------------------------------------------------------------


class Context(stand_ins.FakeContext):
    """The drop test's context with an estimator: the test sets the track, the context answers as MissionContext does."""

    def __init__(self, vehicle=(0.0, 0.0, 12.0), yaw_deg=90.0) -> None:
        super().__init__()
        self._now_ns = 100 * 1_000_000_000
        self.warnings = []
        self.logger = SimpleNamespace(info=lambda text: None, warn=self.warnings.append, error=self.warnings.append)
        self.track = None  # the primary track
        self.others = {}  # other tracks of the class, by id
        self.asked = []
        self.height = None
        self.velocity = None
        self.place(vehicle, yaw_deg)

    def place(self, vehicle, yaw_deg=90.0):
        x, y, z, w = quaternion(yaw_deg)
        self.local_pose = SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=vehicle[0], y=vehicle[1], z=vehicle[2]), orientation=SimpleNamespace(x=x, y=y, z=z, w=w))
        )

    def a_track(self, east, north, state="CONFIRMED", age_s=0.05, track_id=1):
        return target_access.PrimaryTarget("bullseye", track_id, state, (east, north, 0.0), ((0.01, 0.0), (0.0, 0.01)), 0.1, age_s, self._now_ns + 80_000_000)

    def see(self, east, north, state="CONFIRMED", age_s=0.05, track_id=1):
        """Make this the class's primary track; whatever was primary before is still there, by its id."""
        if self.track is not None and self.track.track_id != track_id:
            self.others[self.track.track_id] = self.track
        self.others.pop(track_id, None)
        self.track = self.a_track(east, north, state, age_s, track_id)

    def also(self, east, north, state="CONFIRMED", age_s=0.05, track_id=2):
        """Another track of the class that is not the primary one."""
        self.others[track_id] = self.a_track(east, north, state, age_s, track_id)

    def _by_id(self, track_id):
        if self.track is not None and self.track.track_id == track_id:
            return self.track
        return self.others.get(track_id)

    def raw_detection(self, x_norm=0.9, y_norm=0.9):
        """A fresh raw detection far from where the track is: the 2026 path would steer on this."""
        self.image_size = (640, 360)
        self.target_detection = SimpleNamespace(header=SimpleNamespace(stamp=stand_ins.FakeTime(self._now_ns)), point=SimpleNamespace(x=x_norm, y=y_norm, z=900.0))

    # What MissionContext gives a mission (FND-07).
    def primary_target(self, class_id, *, near=None):
        self.asked.append(("primary_target", class_id, near))
        return self.track if class_id == "bullseye" else None

    def tracked_target(self, class_id, track_id):
        return self._by_id(track_id) if class_id == "bullseye" else None

    def may_descend(self, class_id, *, near=None, track_id=None):
        self.asked.append(("may_descend", class_id, track_id))
        return target_access.descent_permission(self.track if track_id is None else self._by_id(track_id))

    def latlon_to_local_enu(self, latitude, longitude):
        return (40.0, -7.0)  # where the mission's GPS target is in the local frame

    def height_agl(self, ground_below_home_m=0.0):
        return self.height  # what MissionContext.height_agl() gives; the test sets it

    def vehicle_velocity(self):
        return self.velocity  # (east, north, up), or None; the test sets it

    def tick(self, mission):
        self.advance(0.05)
        return mission.update(self)


def make(kind, context, features=FLAGS_ON, **config):
    mission_class, state, extra = KINDS[kind]
    spec = MissionSpec(type_name=f"package_{kind}", name=f"package_{kind}", config={**BASE, **extra, **config}, base_dir=Path("."), features=dict(features))
    mission = mission_class(spec)
    mission.on_enter(context)
    mission._state = state
    return mission


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_mission_steers_on_the_estimators_track_not_the_raw_detection(kind):
    context = Context(vehicle=(1.0, 2.0, 12.0))
    mission = make(kind, context)
    context.see(1.3, 2.1)  # 0.3 m east and 0.1 m north: inside the 0.4 m deadband the 2026 path would call zero
    context.raw_detection()  # and a raw detection somewhere else entirely
    assert context.tick(mission) == MissionStatus.RUNNING
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((GAIN * 0.3, GAIN * 0.1))  # no deadband, no low-pass filter: the first command is the full one
    # The track asked for is the one nearest the mission's own target, and the descend rule is asked about that track.
    assert ("primary_target", "bullseye", (40.0, -7.0)) in context.asked
    assert ("may_descend", "bullseye", 1) in context.asked


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_there_is_no_filter_memory_the_command_follows_the_track_tick_by_tick(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    mission = make(kind, context)
    context.see(1.0, 0.0)
    context.tick(mission)
    context.see(-1.0, 0.5)
    context.tick(mission)
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((GAIN * -1.0, GAIN * 0.5))


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_speed_cap_still_holds(kind):
    context = Context()
    mission = make(kind, context, max_centering_speed_mps=0.45)
    context.see(12.0, 16.0)
    context.tick(mission)
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    assert math.hypot(east, north) == pytest.approx(0.45) and east / north == pytest.approx(0.75)


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_2026_limits_on_a_usable_target_still_apply(kind):
    far = Context()
    mission = make(kind, far, max_projection_distance_m=25.0)
    far.see(18.0, 18.0)  # 25.5 m away: 2026 treats a target this far from the vehicle as not there
    far.tick(mission)
    assert far.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._target_loss_start is not None
    low = Context(vehicle=(0.0, 0.0, 0.5))
    mission = make(kind, low, min_projection_altitude_m=0.75)
    low.see(0.1, 0.0)
    low.tick(mission)
    assert low.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._target_loss_start is not None


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_payload_is_what_is_centred(kind, monkeypatch):
    monkeypatch.setattr(frame_projection, "payload_offset_from_config", lambda config, name, config_dir=None: (0.30, 0.10, -0.25))
    context = Context(vehicle=(10.0, 20.0, 12.0), yaw_deg=90.0)
    mission = make(kind, context)
    context.see(10.0, 20.0)  # the vehicle's origin is exactly over the target: the payload is not
    context.tick(mission)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((GAIN * 0.10, GAIN * -0.30))
    assert up == 0.0  # 0.32 m of payload error is outside the 0.2 m tolerance: no descent yet
    context.place((10.10, 19.70, 12.0))  # the vehicle where the payload is over the target
    context.tick(mission)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((0.0, 0.0), abs=1e-9) and up == pytest.approx(-0.5)


@pytest.mark.parametrize("kind", list(KINDS))
@pytest.mark.parametrize("state, age_s", [("COASTING", 0.35), ("CONFIRMED", 0.6)])
def test_flag_on_the_descent_stops_when_the_track_may_not_be_descended_on_but_the_steering_goes_on(kind, state, age_s):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    mission = make(kind, context)
    context.see(0.05, 0.0)
    context.tick(mission)
    assert context.local_velocity_commands[-1][2] == pytest.approx(-0.5)  # centred on a fresh CONFIRMED track: descending
    context.see(0.05, 0.0, state=state, age_s=age_s)
    context.tick(mission)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert up == 0.0 and (east, north) == pytest.approx((GAIN * 0.05, 0.0))
    assert any("not descending" in text for text in context.warnings)


@pytest.mark.parametrize("kind", list(KINDS))
@pytest.mark.parametrize("state", [None, "LOST", "TENTATIVE"])
def test_flag_on_no_usable_track_is_a_lost_target_whatever_the_raw_detector_says(kind, state):
    context = Context()
    mission = make(kind, context)
    if state is not None:
        context.see(0.5, 0.5, state=state)
    context.raw_detection(0.0, 0.0)  # the 2026 path would be perfectly centred on this
    context.tick(mission)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)  # the 2026 loss handling: hold through the grace time
    assert mission._target_loss_start is not None
    for _ in range(81):  # 4.05 s of it
        context.tick(mission)
    assert mission._state.name == "TARGET_NOT_FOUND"


def test_flag_on_acquisition_waits_for_the_estimators_track_too():
    context = Context()
    mission = make("drop", context)
    mission._state = PackageDropState.ACQUIRE_TARGET
    context.raw_detection(0.0, 0.0)
    for _ in range(40):
        context.tick(mission)
    assert mission._state == PackageDropState.ACQUIRE_TARGET  # a raw detection alone does not start the descent
    context.see(0.2, 0.0)
    for _ in range(40):
        context.tick(mission)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND


def test_flag_on_a_track_that_may_not_be_descended_on_is_never_centred_for_the_column_or_the_release():
    """Nothing commits on a remembered point: not the fixed column, not the dwell that ends in the release."""
    # Drop, just above the drop height and under the column's handoff height, dead centre, but the track is COASTING.
    context = Context(vehicle=(0.0, 0.0, 7.0))
    mission = make("drop", context, drop_altitude_m=6.0, drop_altitude_tolerance_m=3.0, drop_column_handoff_altitude_m=8.0, centering_dwell_s=0.5)
    context.see(0.02, 0.0, state="COASTING", age_s=0.4)
    for _ in range(30):  # 1.5 s: three times the dwell
        context.tick(mission)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND and mission._centering_dwell_start is None
    context.see(0.02, 0.0)  # seen again
    context.tick(mission)
    assert mission._state == PackageDropState.FINAL_FIXED_DROP_COLUMN

    # Drop with no column, at the drop height: the centred dwell does not run on a COASTING track, and starts again after one.
    context = Context(vehicle=(0.0, 0.0, 4.2))
    mission = make("drop", context, drop_altitude_tolerance_m=0.5, centering_dwell_s=1.0)
    context.see(0.02, 0.0)
    for _ in range(10):
        context.tick(mission)
    assert mission._centering_dwell_start is not None
    context.see(0.02, 0.0, state="COASTING", age_s=0.4)
    context.tick(mission)
    assert mission._centering_dwell_start is None
    for _ in range(40):
        context.tick(mission)
    assert mission._state == PackageDropState.TRACK_AND_DESCEND

    # Delivery, at the height where it hands over to the touchdown column.
    context = Context(vehicle=(0.0, 0.0, 6.1))
    context.landing_state_available = lambda: True
    mission = make("delivery", context, touchdown_handoff_tolerance_m=0.2)
    context.see(0.02, 0.0, state="CONFIRMED", age_s=0.6)  # CONFIRMED but too old to descend on
    for _ in range(10):
        context.tick(mission)
    assert mission._state == PackageDeliveryState.TRACK_AND_DESCEND
    context.see(0.02, 0.0)
    context.tick(mission)
    assert mission._state == PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_the_mission_keeps_to_its_track_and_does_not_take_another_over(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    mission = make(kind, context)
    context.see(0.5, 0.0, track_id=1)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:2] == pytest.approx((GAIN * 0.5, 0.0))
    # Its track coasts, and another bullseye track (a second target, or a false alarm) becomes the class's primary one.
    context.see(4.0, 3.0, track_id=2)
    context.also(0.5, 0.0, state="COASTING", age_s=0.4, track_id=1)
    context.tick(mission)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((GAIN * 0.5, 0.0)) and up == 0.0  # still its own track, and no descent on it
    # Its track is lost: that is a lost target, not a reason to fly to the other one.
    context.also(0.5, 0.0, state="LOST", age_s=2.5, track_id=1)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0) and mission._target_loss_start is not None
    assert any("track 2 is not taken over" in text for text in context.warnings)
    # Only after the 2026 loss handling has run (grace, then the recovery climb and a new acquisition) is another track taken.
    for _ in range(81):
        context.tick(mission)
    assert mission._state.name == "TARGET_NOT_FOUND"
    context.tick(mission)  # the recovery starts: the old track is given up
    mission._state = KINDS[kind][1]
    context.tick(mission)
    assert context.local_velocity_commands[-1][:2] == pytest.approx((GAIN * 4.0, GAIN * 3.0))


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_on_an_unhealthy_time_base_is_still_no_target(kind):
    context = Context()
    assert not hasattr(context, "time_health")  # no time monitor: the guard reads that as UNHEALTHY
    mission = make(kind, context, features={**FLAGS_ON, "time_health_guard": True})
    context.see(0.1, 0.0)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("kind", list(KINDS))
def test_flag_off_the_estimator_is_never_asked_and_the_2026_deadband_and_filter_apply(kind):
    class Before(Context):
        def primary_target(self, class_id, *, near=None):
            raise AssertionError("the estimator was asked with the flag off")

        def may_descend(self, class_id, *, near=None, track_id=None):
            raise AssertionError("the descend rule was asked with the flag off")

    context = Before(vehicle=(0.0, 0.0, 12.0))
    mission = make(kind, context, features={})
    assert mission._estimator_centring is None
    context.see(5.0, 5.0)  # a track the 2026 path knows nothing of
    context.raw_detection(0.0, 0.0)  # the raw detection is dead centre: zero error
    context.tick(mission)
    east, north, up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == (0.0, 0.0) and up == pytest.approx(-0.5)


def test_flag_on_without_an_airframe_the_mission_is_refused_when_the_sequence_is_built():
    spec = MissionSpec(type_name="package_drop", name="package_drop", config={**BASE, "airframe": ""}, base_dir=Path("."), features={FLAG: True})
    with pytest.raises(frame_projection.FrameProjectionError, match="airframe"):
        PackageDropMission(spec)


# --- the retuned controller (CEN-02) ----------------------------------------------

RETUNE = "centring_retune"
RETUNE_ON = {**FLAGS_ON, RETUNE: True}


def settle(context, mission, ticks=200):
    for _ in range(ticks):
        context.tick(mission)
    return context.local_velocity_commands[-1]


def test_the_retune_flag_needs_centring_on_the_estimator_and_is_declared_off():
    import yaml as real_yaml

    from drone_mission_core import centring_controller

    assert centring_controller.RETUNE_FLAG == RETUNE and FLAG_REQUIRES[RETUNE] == (FLAG,)
    declaring = [
        path.name
        for path in sorted((SRC / "drone_mission_demo" / "config" / "sequences").glob("*.yaml"))
        if real_yaml.safe_load(path.read_text())["mission_sequence"]["features"].get(RETUNE, "missing") is False
    ]
    assert len(declaring) == 12


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_on_the_command_is_the_new_controllers_not_the_2026_gain(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height = target_access.HeightAgl(12.0, "baro")
    mission = make(kind, context, features=RETUNE_ON)
    context.see(0.33, 0.0)
    context.tick(mission)
    east, north, _up, _yaw = context.local_velocity_commands[-1]
    assert (east, north) == pytest.approx((0.025, 0.0))  # the slew limit: 0.5 m/s2 over one 50 ms tick
    east, north, _up, _yaw = settle(context, mission)
    assert (east, north) == pytest.approx((0.6 * 0.30, 0.0))  # Kp 0.6 on the error less the 0.03 m soft deadband; the sequence's 0.25 is not read


@pytest.mark.parametrize("kind", list(KINDS))
@pytest.mark.parametrize(
    "height, source, cap",
    [
        (12.0, "rangefinder", 0.96),
        (6.0, "rangefinder", 0.48),
        (2.4, "rangefinder", 0.25),
        (None, None, 0.25),
        # A barometer height is above home, not above the ground: it is taken as 2 m lower than it says.
        (15.0, "baro", 1.0),
        (12.0, "baro", 0.80),
        (6.0, "baro", 0.32),
        (4.0, "baro", 0.25),
    ],
)
def test_retune_on_the_speed_cap_follows_the_height_above_the_ground(kind, height, source, cap):
    context = Context(vehicle=(0.0, 0.0, 12.0))  # 12 m above home, whatever the ground is doing
    context.height = None if height is None else target_access.HeightAgl(height, source)
    mission = make(kind, context, features=RETUNE_ON)
    context.see(6.0, 8.0)
    east, north, _up, _yaw = settle(context, mission)
    assert math.hypot(east, north) == pytest.approx(cap) and east / north == pytest.approx(0.75)


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_on_the_vehicles_velocity_gives_the_controller_its_lead(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height = target_access.HeightAgl(12.0, "baro")
    context.velocity = (0.0, 0.3, 0.0)  # over the target, drifting north at 0.3 m/s
    mission = make(kind, context, features=RETUNE_ON)
    context.see(0.0, 0.0)
    east, north, _up, _yaw = settle(context, mission)
    assert (east, north) == pytest.approx((0.0, 0.6 * -(0.3 * 0.5 - 0.03)))
    still = Context(vehicle=(0.0, 0.0, 12.0))
    still.height = target_access.HeightAgl(12.0, "baro")
    mission = make(kind, still, features=RETUNE_ON, centring_lead_s=0.0)
    still.velocity = (0.0, 0.3, 0.0)
    still.see(0.0, 0.0)
    assert settle(still, mission)[:2] == (0.0, 0.0)


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_on_a_lead_that_cannot_be_applied_is_said_in_the_log(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height = target_access.HeightAgl(12.0, "rangefinder")
    mission = make(kind, context, features=RETUNE_ON)
    context.see(0.33, 0.0)
    context.velocity = (0.0, 0.0, 0.0)
    settle(context, mission, 20)
    assert not any("lead" in text for text in context.warnings)
    context.velocity = None  # MAVROS's velocity has stopped coming
    settle(context, mission, 100)  # 5 s
    said = [text for text in context.warnings if "lead" in text]
    assert 2 <= len(said) <= 4 and "no velocity" in said[0]  # every 2 s, not every tick


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_on_a_lost_target_stops_at_once_and_the_next_command_starts_from_rest(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height = target_access.HeightAgl(12.0, "baro")
    mission = make(kind, context, features=RETUNE_ON)
    context.see(3.0, 0.0)
    assert settle(context, mission, 100)[0] == pytest.approx(0.80)
    context.also(3.0, 0.0, state="LOST", age_s=2.5, track_id=1)
    context.track = None
    context.tick(mission)
    assert context.local_velocity_commands[-1][:3] == (0.0, 0.0, 0.0)  # the 2026 hold on a lost target, not a slow ramp down
    context.see(3.0, 0.0, track_id=1)
    context.tick(mission)
    assert context.local_velocity_commands[-1][0] == pytest.approx(0.025)


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_on_the_sequences_own_gains_are_used_and_bad_ones_stop_the_sequence_being_built(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height = target_access.HeightAgl(12.0, "baro")
    mission = make(kind, context, features=RETUNE_ON, centring_kp=0.4, centring_soft_deadband_m=0.0)
    context.see(0.5, 0.0)
    assert settle(context, mission)[0] == pytest.approx(0.2)
    mission_class = KINDS[kind][0]
    spec = MissionSpec(type_name=f"package_{kind}", name=f"package_{kind}", config={**BASE, "centring_kp": 3.0}, base_dir=Path("."), features=dict(RETUNE_ON))
    with pytest.raises(ValueError, match="centring_kp"):
        mission_class(spec)


@pytest.mark.parametrize("kind", list(KINDS))
def test_retune_off_centring_on_the_estimator_keeps_the_2026_gain(kind):
    context = Context(vehicle=(0.0, 0.0, 12.0))
    context.height_agl = lambda ground_below_home_m=0.0: (_ for _ in ()).throw(AssertionError("the height was asked with the retune off"))
    mission = make(kind, context, features=FLAGS_ON, centring_kp=3.0)  # not read, so not refused either
    assert mission._centring_controller is None
    context.see(0.33, 0.0)
    context.tick(mission)
    assert context.local_velocity_commands[-1][:2] == pytest.approx((GAIN * 0.33, 0.0))
