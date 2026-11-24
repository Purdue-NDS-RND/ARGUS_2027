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
        self.abort_modes = []

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

    named = None  # what an earlier mission of the sequence left named for the abort

    def named_abort_mode(self):
        return self.named

    def set_abort_mode(self, mode):
        assert mode in (None, "LAND")  # what MissionContext.set_abort_mode takes
        self.abort_modes.append((self.world.t, mode))
        self.named = mode

    def request_mode_change(self, mode_name, done_callback=None):
        self.modes.append(mode_name)
        self.world.landing = True

    def permit_actuation(self, actuator, *, state, gates_hold, why=""):
        self.actuation_permits = getattr(self, "actuation_permits", []) + [(actuator, state, gates_hold)]
        return True

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
    # The actuation lock (PLT-09) is asked before each: the close from ENGAGE, the open from SET_DOWN.
    assert context.actuation_permits == [("gripper", "ENGAGE", True), ("gripper", "SET_DOWN", True)]
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


# --- the abort's mode (the lead's rule of 23 November 2026, as built: recovery_return.py, "Abort") ----------------


def test_the_mission_tells_the_executor_rtl_until_its_gripper_closes_and_a_landing_from_then_on():
    world = world_tests.World()
    flown, context = mission(), Context(world)
    assert run(flown, context) == MissionStatus.SUCCESS
    # Told on the first tick (the executor's own, RTL), told LAND at the close, and never told anything again:
    # not at the release, and not when the mission leaves with the bucket standing where an RTL would come down.
    assert [mode for _t, mode in context.abort_modes] == [None, "LAND"]
    assert abs(context.abort_modes[1][0] - context.gripper_calls[0][0]) < 0.01  # on the tick the close is commanded
    assert sum("would now ask for LAND" in line for line in context.infos) == 1 and sum("would now ask for RTL" in line for line in context.infos) == 1


def test_leaving_in_mid_carry_leaves_the_abort_a_landing():
    """What the executor does when update() raises: on_exit, then the abort. The landing must still be named."""
    world = world_tests.World()
    flown, context = mission(), Context(world)
    flown.on_enter(context)
    while flown.flight.state != recovery_flight.RETURN:
        assert context.tick(flown) == MissionStatus.RUNNING
    flown.on_exit(context)
    assert context.abort_modes[-1][1] == "LAND" and world.attached


def test_a_mission_that_fails_with_the_bucket_on_leaves_the_abort_a_landing():
    world = world_tests.World(open_result=4)
    flown, context = mission(), Context(world)
    assert run(flown, context) == MissionStatus.FAILURE and world.attached
    assert [mode for _t, mode in context.abort_modes] == [None, "LAND"]


def test_a_mission_that_fails_before_its_gripper_ever_closed_leaves_the_abort_the_executors_own():
    world = world_tests.World(bucket=(500.0, 500.0), pickup=(40.0, 30.0))
    flown, context = mission(time_budget_s=20.0), Context(world)
    assert run(flown, context) == MissionStatus.FAILURE
    assert [mode for _t, mode in context.abort_modes] == [None]


def test_a_mission_that_fails_for_want_of_a_position_leaves_the_abort_a_landing():
    """The flight-safety review's blocker: this failure used to hand the executor an RTL it could never fly."""
    world = world_tests.World()
    flown, context = mission(), Context(world)
    flown.on_enter(context)
    while flown.flight.state != recovery_flight.APPROACH:
        assert context.tick(flown) == MissionStatus.RUNNING
    context.connected = False  # no position counts while the flight controller is not connected
    status = MissionStatus.RUNNING
    for _ in range(100):
        status = context.tick(flown)
        if status != MissionStatus.RUNNING:
            break
    flown.on_exit(context)
    assert status == MissionStatus.FAILURE and "no position" in context.errors[-1]
    assert [mode for _t, mode in context.abort_modes] == [None, "LAND"] and context.gripper_calls == []


def test_with_gps_not_trusted_the_abort_is_a_landing_from_before_the_first_tick():
    world = world_tests.World()
    flown, context = mission(features={**ON, "recovery_gps_untrusted": True}, pickup={"local": [40.0, 30.0]}), Context(world)
    flown.on_enter(context)
    assert [mode for _t, mode in context.abort_modes] == ["LAND"]  # named on entry: a failure on the first tick lands too
    context.abort_modes.clear()
    context.named = None
    flown, context = mission(features={**ON, "recovery_gps_untrusted": True}, pickup={"local": [40.0, 30.0]}), Context(world_tests.World())
    world = context.world
    assert run(flown, context) == MissionStatus.SUCCESS
    assert [mode for _t, mode in context.abort_modes] == ["LAND"]


def test_a_landing_an_earlier_mission_left_named_is_not_undone():
    world = world_tests.World()
    flown, context = mission(), Context(world)
    context.named = "LAND"  # an earlier recovery of the sequence failed with something on its hook
    flown.on_enter(context)
    for _ in range(200):
        context.tick(flown)
    assert [mode for _t, mode in context.abort_modes] == ["LAND"]  # told once, and never told None


def test_with_the_flag_off_the_abort_is_never_touched():
    world = world_tests.World()
    flown, context = mission(features={}), Context(world)
    assert run(flown, context, seconds=1.0) == MissionStatus.SUCCESS
    assert context.abort_modes == []


def test_the_actuation_lock_is_told_no_when_the_gripper_command_is_not_its_states_own():
    """PLT-09: an open from ENGAGE, or a close from SET_DOWN, is no state's rule."""
    world = world_tests.World()
    flown, context = mission(), Context(world)
    flown.on_enter(context)
    rf = recovery_flight
    for state, gripper, said in ((rf.ENGAGE, rf.CLOSE, True), (rf.SET_DOWN, rf.OPEN, True), (rf.ENGAGE, rf.OPEN, False), (rf.SET_DOWN, rf.CLOSE, False), (rf.RETURN, rf.OPEN, False)):
        context.actuation_permits = []
        flown.flight.state = state
        flown._apply(context, rf.Command(rf.RUNNING, gripper=gripper, attempt=1))
        assert context.actuation_permits == [("gripper", state, said)], (state, gripper)


# --- REC-6: the GPS health monitor's word and the flight controller's source set ----------------------------

SOURCE_ON = {**ON, "ekf_source_switching": True}


class WatchedContext(Context):
    """The context with a GPS health monitor whose rating the test sets, and a flight controller that answers source-set commands."""

    def __init__(self, world, untrusted_from_s=None, back_at_s=None, answers=True, fed=True):
        super().__init__(world)
        self.untrusted_from_s, self.back_at_s, self.answers, self.fed = untrusted_from_s, back_at_s, answers, fed
        self.source_commands = []
        self.static_targets = []

    def gps_health(self):
        from drone_mission_core import gps_health

        t = self.world.t
        if self.untrusted_from_s is not None and t >= self.untrusted_from_s and (self.back_at_s is None or t < self.back_at_s):
            return gps_health.Rating(gps_health.UNTRUSTED, ("the GPS position moved 5.0 m beyond what was flown in 1 s (by vision)",), self.untrusted_from_s, 0.0, self.untrusted_from_s)
        return gps_health.Rating(gps_health.TRUSTED, (), 0.0, 12.0)

    def no_gps_source_fed(self):
        return self.fed

    def note_static_target(self, east_m, north_m):
        self.static_targets.append((round(self.world.t, 2), east_m, north_m))

    def command_ekf_source_set(self, source_set, done_callback=None):
        self.source_commands.append((round(self.world.t, 2), source_set))
        if self.answers:
            self.pending.append((self.world.t + 0.1, done_callback, True))


def test_the_source_flag_is_the_recovery_missions_and_ships_off():
    from drone_mission_core import gps_health
    from drone_mission_core.features import FLAG_REQUIRES
    from drone_mission_demo.missions import package_recovery_mission

    assert package_recovery_mission.SOURCE_FLAG == gps_health.FLAG and FLAG_REQUIRES[gps_health.FLAG] == ("package_recovery",)
    assert LOADED["features"]["ekf_source_switching"] is False and gps_health.FLAG not in recovery_flight.UNBUILT_FLAGS


def test_with_gps_trusted_throughout_the_flag_changes_nothing_that_is_flown():
    plain_world, watched_world = world_tests.World(), world_tests.World()
    plain, plain_context = mission(), Context(plain_world)
    watched, watched_context = mission(features=SOURCE_ON), WatchedContext(watched_world)
    assert run(plain, plain_context) == run(watched, watched_context) == MissionStatus.SUCCESS
    # One command, at the start: the flight controller is made sure to be on set 1, whatever an earlier run left it on.
    assert [source_set for _t, source_set in watched_context.source_commands] == [1] and "source set 1 acknowledged by the flight controller" in " | ".join(watched_context.infos)
    # The same setpoints, two ticks later: the watched mission holds until the flight controller has answered for set 1.
    assert "holding until the flight controller answers for source set 1" in " | ".join(watched_context.infos)
    assert len(watched_context.global_sets) == len(plain_context.global_sets) > 0
    assert all(one == pytest.approx(other, abs=1e-6) for one, other in zip(watched_context.global_sets, plain_context.global_sets))
    # (the local setpoints are a tick's worth more or fewer: the world's clock stands two ticks later under the same flight)
    assert abs(len(watched_context.local_sets) - len(plain_context.local_sets)) <= 2
    assert watched_context.local_sets[0] == pytest.approx(plain_context.local_sets[0], abs=1e-6) and watched_context.local_sets[-1] == pytest.approx(plain_context.local_sets[-1], abs=0.05)
    assert any("GPS TRUSTED" in line for line in watched_context.infos)
    # The monitor is told where the bucket stands while it stands (the approach and the descent), and to forget it once it is on the hook.
    told = [item for item in watched_context.static_targets if item[1] is not None]
    assert len(told) > 50 and all(math.hypot(east - 40.0, north - 30.0) < 1.0 for _t, east, north in told) and watched_context.static_targets[-1][1] is None
    # With the flag off the monitor is never asked: a context without one flies the mission (the plain context has no gps_health at all).
    assert not hasattr(plain_context, "gps_health")


def test_untrusted_on_the_way_home_asks_for_source_set_two_and_no_global_setpoint_is_commanded_from_then_on():
    world = world_tests.World()
    flown = mission(features=SOURCE_ON)
    context = WatchedContext(world)
    flown.on_enter(context)
    status, switched_at, globals_after = MissionStatus.RUNNING, None, 0
    for _ in range(900 * 20):
        if switched_at is None and flown.flight.state == "RETURN" and len(context.global_sets) >= 5:
            context.untrusted_from_s = switched_at = world.t  # the monitor finds the fault in mid-return, global setpoints already flowing
            before = len(context.global_sets)
        status = context.tick(flown)
        if status != MissionStatus.RUNNING:
            break
    flown.on_exit(context)
    assert switched_at is not None
    said = " | ".join(context.infos)
    assert "GPS UNTRUSTED (the GPS position moved 5.0 m beyond what was flown in 1 s (by vision))" in said
    assert "GPS UNTRUSTED: asking the flight controller for source set 2" in said and "source set 2 acknowledged by the flight controller" in said
    assert context.source_commands[1:] == [(round(switched_at, 2), 2)]  # asked once, on the tick the monitor said so: it was acknowledged
    assert "it is left there, and nothing after this should be flown on GPS" in said  # the mission's end, still UNTRUSTED: said, and no command
    assert len(context.global_sets) <= before + 1  # no global setpoint after the tick it was found on
    assert context.abort_modes[-1][-1] == "LAND"  # a failure from here is a landing: there is no GPS to fly home on
    assert status in (MissionStatus.SUCCESS, MissionStatus.FAILURE)  # the way home without GPS is the flight's own rule (REC-7): set down where it may, or a landing with the bucket


def test_the_first_source_set_is_asked_for_again_only_on_the_way_and_only_once_gps_is_trusted_again():
    world = world_tests.World()
    flown = mission(features=SOURCE_ON)
    context = WatchedContext(world)
    flown.on_enter(context)
    states_at_commands = []
    for _ in range(900 * 20):
        if context.untrusted_from_s is None and flown.flight.state == "TRANSIT" and flown._source_switch.confirmed:
            context.untrusted_from_s, context.back_at_s = world.t, world.t + 3.0  # a fault of three seconds on the way out
        sent = len(context.source_commands)
        status = context.tick(flown)
        if len(context.source_commands) > sent:
            states_at_commands.append((context.source_commands[-1][1], flown.flight.state))
        if status != MissionStatus.RUNNING:
            break
    assert [source_set for source_set, _state in states_at_commands] == [1, 2, 1]  # (the first: the start's "make sure")
    from drone_mission_demo.missions.package_recovery_mission import SOURCE_SET_BACK_STATES

    assert states_at_commands[2][1] in SOURCE_SET_BACK_STATES and context.source_commands[2][0] >= context.source_commands[1][0] + 3.0
    assert "source set 1 acknowledged by the flight controller" in " | ".join(context.infos)


def test_a_flight_controller_that_stops_answering_is_asked_on_and_the_flight_stays_off_gps():
    world = world_tests.World()
    flown = mission(features=SOURCE_ON)
    context = WatchedContext(world)
    flown.on_enter(context)
    for _ in range(60 * 20):
        if context.untrusted_from_s is None and flown.flight.state == "TRANSIT" and flown._source_switch.confirmed:
            context.untrusted_from_s, context.answers = world.t, False  # a fault, and no answer from then on
            before = len(context.global_sets)
        if context.tick(flown) != MissionStatus.RUNNING:
            break
    asked = [(at, source_set) for at, source_set in context.source_commands[1:]]
    assert [source_set for _at, source_set in asked[:5]] == [2] * 5 and all(abs((later[0] - earlier[0]) - 1.0) < 0.06 for earlier, later in zip(asked[:5], asked[1:5]))
    assert "source set 2 was not acknowledged after 5 tries: which set the flight controller is on is not known" in " | ".join(context.infos)
    # It goes on being asked, every five seconds, and nothing is flown on GPS.
    assert len(asked) > 6 and all(source_set == 2 for _at, source_set in asked) and all(later[0] - earlier[0] > 4.9 for earlier, later in zip(asked[5:], asked[6:]))
    assert flown._source_switch.acknowledged is None and len(context.global_sets) <= before + 1 and context.abort_modes[-1][-1] == "LAND"


def test_untrusted_with_nothing_feeding_source_set_two_commands_nothing_and_flies_without_the_gps():
    world = world_tests.World()
    flown = mission(features=SOURCE_ON)
    context = WatchedContext(world, fed=False)
    flown.on_enter(context)
    for _ in range(900 * 20):
        if context.untrusted_from_s is None and flown.flight.state == "RETURN" and len(context.global_sets) >= 5:
            context.untrusted_from_s = world.t
            before = len(context.global_sets)
        if context.tick(flown) != MissionStatus.RUNNING:
            break
    said = " | ".join(context.infos)
    assert [source_set for _t, source_set in context.source_commands] == [1]  # the start's only
    assert "source set 2 is not asked for, nothing is feeding it" in said and "asking the flight controller for source set 2" not in said
    assert len(context.global_sets) <= before + 1 and context.abort_modes[-1][-1] == "LAND"


def test_a_mission_that_ends_on_source_set_two_with_gps_trusted_again_asks_for_the_first_set():
    world = world_tests.World()
    flown = mission(features=SOURCE_ON)
    context = WatchedContext(world)
    flown.on_enter(context)
    status = MissionStatus.RUNNING
    for _ in range(900 * 20):
        if context.untrusted_from_s is None and flown.flight.state == "CLEAR":
            context.untrusted_from_s, context.back_at_s = world.t, world.t + 1.0  # a fault after the bucket is down: no state from here lets the set go back
        status = context.tick(flown)
        if status != MissionStatus.RUNNING:
            break
    assert context.untrusted_from_s is not None and [source_set for _t, source_set in context.source_commands] == [1, 2]
    flown.on_exit(context)
    assert context.source_commands[-1][1] == 1 and "the mission ends with the flight controller on source set 2: asking for source set 1" in " | ".join(context.infos)

