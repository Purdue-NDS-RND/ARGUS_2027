"""The recovery mission (REC-2), through its real update(), against the recovery flight's test world.

rclpy and the message packages are the stand-ins from test_package_drop_mission.py; the world (a vehicle that
goes where it is told, a bucket, a gripper, a throttle) is drone_mission_core's test_recovery_flight.World.
"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_centring_missions as centring_tests  # noqa: E402  (installs the ROS stand-ins)

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "drone_mission_core" / "test"))
import test_recovery_flight as world_tests  # noqa: E402

from drone_mission_core import recovery_flight, set_down, target_access  # noqa: E402
from drone_mission_core.features import FLAG_REQUIRES, get_feature_flag  # noqa: E402
from drone_mission_core.mission_api import MissionSpec, MissionStatus  # noqa: E402
from drone_mission_demo.missions import package_recovery_mission as mission_module  # noqa: E402
from drone_mission_demo.missions.package_recovery_mission import PackageRecoveryMission  # noqa: E402

SEQUENCE = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "sim_package_recovery.yaml"
LOADED = yaml.safe_load(SEQUENCE.read_text())["mission_sequence"]
CONFIG = next(item["config"] for item in LOADED["missions"] if item["type"] == "package_recovery")
ON = {"use_frame_time_pose": True, "use_target_estimator": True, "use_height_estimator": True, "package_recovery": True}
S = 1_000_000_000


PICKUP = {"latlon": list(world_tests.World().anchor.to_global(40.0, 30.0))}  # the world's bucket, as a latitude and longitude
# The test world's own: its bucket's place, and the masses its throttle is made for (0.40 empty, 0.45 loaded: the
# rise 1 kg gives a 4 kg vehicle; the sim vehicle's 1.755 kg would ask for more rise than that).
WORLD = {"pickup": PICKUP, "vehicle_kg": 4.0, "load_kg": 1.0, "hook_below_m": world_tests.HOOK, "handle_up_m": world_tests.HANDLE}


def mission(features=ON, **config):
    spec = MissionSpec(type_name="package_recovery", name="package_recovery", config={**CONFIG, **WORLD, **config}, base_dir=SEQUENCE.parent, features=dict(features))
    return PackageRecoveryMission(spec)


class Context:
    """What the mission sees of the executor, backed by the recovery flight's test world."""

    def __init__(self, world):
        self.world = world
        self.infos, self.errors = [], []
        self.logger = SimpleNamespace(info=self.infos.append, warn=self.infos.append, error=self.errors.append)
        self.local_sets, self.global_sets, self.gripper_calls, self.modes, self.cleared = [], [], [], [], 0
        self.pending = []
        self.near_asked = []

    def _stamp(self, age_s=0.0):
        t = self.world.t - age_s
        return SimpleNamespace(sec=int(t), nanosec=int(round((t - int(t)) * 1.0e9)))

    def now(self):
        return centring_tests.stand_ins.FakeTime(int(round(self.world.t * S)))

    connected, pose_age_s = True, 0.02

    @property
    def mavros_state(self):
        return SimpleNamespace(connected=self.connected, mode="LAND" if self.world.landing else "GUIDED")

    @property
    def local_pose(self):
        e, n, up = self.world.pos
        half = math.pi / 4.0  # facing north: a yaw of 90 degrees
        return SimpleNamespace(
            header=SimpleNamespace(stamp=self._stamp(self.pose_age_s)),
            pose=SimpleNamespace(position=SimpleNamespace(x=e, y=n, z=up), orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))),
        )

    def height_agl(self, ground_below_home_m=0.0):
        return target_access.HeightAgl(self.world.agl(), "rangefinder")

    def rangefinder_range(self):
        return (self.world.agl(), 0.05)

    def throttle(self):
        reading = self.world.readings()
        self.world.seen.pop()
        return (reading.throttle, 0.05)

    @property
    def extended_state(self):
        return SimpleNamespace(header=SimpleNamespace(stamp=self._stamp(0.1)))

    def landing_state_available(self):
        return True

    def vehicle_is_landed(self):
        reading = self.world.readings()
        self.world.seen.pop()
        return bool(reading.landed)

    @property
    def global_gps(self):
        lat, lon = self.world.anchor.to_global(self.world.pos[0], self.world.pos[1])
        return SimpleNamespace(latitude=lat, longitude=lon, status=SimpleNamespace(status=0), header=SimpleNamespace(stamp=self._stamp(0.1)))

    def home_up_m(self):
        return self.world.home_up

    def latlon_to_local_enu(self, latitude, longitude):
        return self.world.anchor.to_local(latitude, longitude)

    def primary_target(self, class_id, near=None):
        self.near_asked.append((class_id, near))
        if class_id != "bucket":
            return None
        target = self.world.readings().target
        if target is None:
            return None
        return target_access.PrimaryTarget("bucket", 7, "CONFIRMED", (target.e, target.n, 0.2), ((0.01, 0.0), (0.0, 0.01)), target.sigma_m, target.age_s, 0)

    def set_local_position_setpoint(self, east_m, north_m, up_m, yaw_deg=90.0):
        self.local_sets.append((east_m, north_m, up_m, yaw_deg))
        self.command = recovery_flight.Command(frame=recovery_flight.LOCAL, e=east_m, n=north_m, up=up_m)

    def set_global_position_setpoint(self, latitude, longitude, altitude_m, yaw_deg=90.0, lock_yaw=True):
        self.global_sets.append((latitude, longitude, altitude_m, yaw_deg))
        self.command = recovery_flight.Command(frame=recovery_flight.GLOBAL, lat=latitude, lon=longitude, alt=altitude_m)

    def clear_local_position_setpoint(self):
        pass

    def clear_global_position_setpoint(self):
        pass

    def clear_all_setpoints(self):
        self.cleared += 1
        self.command = recovery_flight.Command()

    def request_mode_change(self, mode_name, done_callback=None):
        self.modes.append(mode_name)
        self.world.landing = True

    def command_gripper(self, *, release, done_callback=None):
        self.gripper_calls.append((round(self.world.t, 2), release))
        # The world's own gripper, through the flight's command: closed on the handle or not, opened or refused.
        attempt = 10_000 + len(self.gripper_calls)
        self.world.apply(recovery_flight.Command(gripper=recovery_flight.OPEN if release else recovery_flight.CLOSE, attempt=attempt), dt=0.0)
        success = not release or self.world.open_result == 0
        self.pending.append((self.world.t + 0.1, done_callback, success))

    def tick(self, mission_):
        self.command = recovery_flight.Command()
        status = mission_.update(self)
        for item in list(self.pending):
            if self.world.t >= item[0]:
                self.pending.remove(item)
                item[1](SimpleNamespace(result=lambda ok=item[2]: SimpleNamespace(success=ok, result=0 if ok else 4)))
        self.world.apply(self.command)
        return status


def run(mission_, context, seconds=900.0):
    mission_.on_enter(context)
    status = MissionStatus.RUNNING
    for _ in range(int(seconds * 20)):
        status = context.tick(mission_)
        if status != MissionStatus.RUNNING:
            break
    mission_.on_exit(context)
    return status


def test_with_the_flag_off_the_mission_is_skipped_and_commands_nothing():
    skipped = mission(features={})
    context = Context(world_tests.World())
    skipped.on_enter(context)
    assert skipped.update(context) == MissionStatus.SUCCESS and skipped.flight is None
    skipped.on_exit(context)
    assert (context.local_sets, context.global_sets, context.gripper_calls, context.modes, context.cleared) == ([], [], [], [], 0)
    assert "skipped" in context.infos[0]
    # Skipped, a configuration that could not be flown is not read either: the shipped sequence, flags false, loads.
    assert PackageRecoveryMission(MissionSpec(type_name="package_recovery", name="r", config={}, base_dir=SEQUENCE.parent, features={})).flight is None


def test_with_the_flag_on_the_mission_flies_the_recovery_through_the_context():
    world = world_tests.World()
    flown, context = mission(), Context(world)
    assert run(flown, context) == MissionStatus.SUCCESS
    assert flown.flight.transitions == world_tests.WALK and math.hypot(*world.bucket) < 0.35 and not world.attached
    # The gripper: closed once on the handle, opened once with the bucket standing, each answer taken with its attempt.
    assert [release for _t, release in context.gripper_calls] == [False, True] and world.opens[0][2] is True
    # GPS trusted: global setpoints on the way back, each with the yaw the mission began with; a landing at the end.
    assert len(context.global_sets) > 20 and {round(yaw) for *_rest, yaw in context.global_sets + [s[1:] for s in context.local_sets]} == {90}
    assert context.modes == ["LAND"] and context.errors == [] and context.cleared >= 2  # asked once: its mode was seen
    # The bucket's own track is asked for first, near where it was last seen.
    assert context.near_asked[0] == ("bucket", None) and any(near is not None for _class, near in context.near_asked)
    states = [line for line in context.infos if "State:" in line]
    assert "State: START -> " in states[0] and any("SET_DOWN" in line for line in states)


def test_the_untrusted_flag_keeps_every_setpoint_local_and_wants_the_pickup_given_locally():
    untrusted = {**ON, "recovery_gps_untrusted": True}
    with pytest.raises(ValueError, match="pickup"):
        mission(features=untrusted)
    world = world_tests.World()
    flown, context = mission(features=untrusted, pickup={"local": [40.0, 30.0]}), Context(world)
    assert run(flown, context) == MissionStatus.SUCCESS and context.global_sets == [] and math.hypot(*world.bucket) < 0.35


def test_a_refused_release_ends_in_a_landing_with_the_bucket_and_a_failure():
    world = world_tests.World(open_result=4)
    flown, context = mission(), Context(world)
    assert run(flown, context) == MissionStatus.FAILURE and world.attached and context.modes == ["LAND"]
    assert "landed with the bucket" in context.errors[-1] and [release for _t, release in context.gripper_calls].count(True) == 3


def test_the_shipped_sequence_declares_every_flag_false_and_its_numbers_make_a_plan():
    flags = LOADED["features"]
    assert all(value is False for value in flags.values())
    for flag in ("package_recovery", "swing_shaping", "recovery_gps_untrusted", *recovery_flight.UNBUILT_FLAGS):
        assert flag in flags, flag
    for flag in ("package_recovery", "swing_shaping", "recovery_gps_untrusted"):
        assert all(needed in flags for needed in FLAG_REQUIRES[flag])
    assert FLAG_REQUIRES["package_recovery"] == ("use_target_estimator", "use_height_estimator")
    assert mission_module.FLAG == recovery_flight.FLAG == "package_recovery" and recovery_flight.FLAG in set_down.THROTTLE_FLAGS
    plan = mission().plan
    assert (plan.transit_height_m, plan.search_height_m, plan.search_speed_mps, plan.approach_height_m, plan.centre_above_handle_m) == (15.0, 20.0, 4.0, 6.0, 3.0)
    assert plan.max_attempts == 3 and plan.land_after and not plan.swing_shaping and len(plan.search_waypoints) >= 4 and 0.5 < plan.footprint_per_metre < 2.0
    # A flag with nothing behind it, or a number that cannot be flown, stops the mission being made.
    for flag in recovery_flight.UNBUILT_FLAGS:
        with pytest.raises(ValueError, match=flag):
            mission(features={**ON, flag: True})
    for bad in ({"vehicle_kg": 0.0}, {"pickup": {"latlon": [91.0, 0.0]}}, {"pickup": [1.0, 2.0]}, {"carry_speed_mps": 5.0}, {"camera_mode": "no_such_mode"}):
        with pytest.raises(ValueError):
            mission(**bad)
    assert get_feature_flag(flags, "package_recovery") is False


def test_a_stale_pose_or_a_lost_connection_is_no_position_and_a_boundary_is_placed_before_the_pickup():
    for fault in ({"pose_age_s": 2.0}, {"connected": False}):
        context = Context(world_tests.World())
        for name, value in fault.items():
            setattr(context, name, value)
        flown = mission()
        flown.on_enter(context)
        status = MissionStatus.RUNNING
        for _ in range(int(12.0 * 20)):
            status = context.tick(flown)
            if status != MissionStatus.RUNNING:
                break
        assert status == MissionStatus.FAILURE and "no position" in context.errors[-1] and context.local_sets == [] and context.global_sets == []
    # A flight boundary is given as latitudes and longitudes; the pickup outside it is refused before anything moves.
    world = world_tests.World()
    corners = [list(world.anchor.to_global(east, north)) for east, north in ((-20.0, -20.0), (30.0, -20.0), (30.0, 50.0), (-20.0, 50.0))]
    flown, context = mission(flight_boundary=corners), Context(world)
    assert run(flown, context, seconds=5.0) == MissionStatus.FAILURE and "flight boundary" in context.errors[-1]
    with pytest.raises(ValueError, match="flight_boundary"):
        mission(flight_boundary=[[0.0, 0.0]])
    with pytest.raises(ValueError, match="plunge only"):
        mission(engage={"mode": "sweep"})
    with pytest.raises(ValueError, match="no setting"):
        mission(handle_height_m=0.3)


def test_the_home_height_feed_pairs_the_height_above_home_with_the_local_height():
    feed = recovery_flight.HomeHeight()
    assert feed.newest(5 * S) is None
    feed.add(5 * S, 2.81, 3.0)
    assert feed.newest(5 * S + S // 2) == pytest.approx(0.19) and feed.newest(8 * S) is None  # two seconds old at most
    feed.add(9 * S, float("nan"), 3.0)
    assert feed.newest(9 * S) is None
