"""PLT-09: the takeover latch and the actuation lock in the executor, through the real node class and the real context.

rclpy is replaced by the stand-ins of test_executor_feature_overrides.py. The
sequence is the shipped delivery, with the flags turned on the way a launch
turns them on (`feature_overrides`).
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_executor_feature_overrides import FakeNode, executor_module, start  # noqa: E402, F401  (executor_module is a fixture)

ROOT = Path(__file__).resolve().parents[3]
SEQUENCES = ROOT / "src" / "drone_mission_demo" / "config" / "sequences"
# A sequence with the lock declared and no mission that actuates: the lock can be turned on without the estimator
# the gated release paths need (its ROS messages are not among the stand-ins).
LOCKABLE = """
mission_sequence:
  on_failure: abort_and_rtl
  features:
    takeover_latch: false
    actuation_lock: false
  missions:
    - type: takeoff
      name: takeoff
      config:
        target_altitude_m: 10.0
"""


class Clock:
    def __init__(self):
        self.ns = 1_000 * 1_000_000_000

    def now(self):
        return SimpleNamespace(nanoseconds=self.ns, to_msg=lambda: None)

    def tick(self, seconds=0.05):
        self.ns += int(seconds * 1e9)


class Mission:
    """A mission that is ticked, and counts."""

    name = "counted"

    def __init__(self):
        self.updates = self.entered = self.exited = 0
        self.spec = SimpleNamespace(failure_policy=None, features={})

    def on_enter(self, _context):
        self.entered += 1

    def update(self, _context):
        self.updates += 1
        from drone_mission_core.mission_api import MissionStatus

        return MissionStatus.RUNNING

    def on_exit(self, _context):
        self.exited += 1


@pytest.fixture
def clock(monkeypatch):
    made = Clock()
    monkeypatch.setattr(FakeNode, "get_clock", lambda self: made, raising=False)
    return made


def flying(node, clock, mode="GUIDED"):
    """The node as it is in mid-mission: took off through the context, armed, a mission active, a setpoint being sent."""
    if node._takeover is not None and not hasattr(node, "_cancel_calls"):
        node._cancel_calls = []
        node._takeoff_cancel_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: node._cancel_calls.append("cancel") or SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True)))
    node._mavros_state = SimpleNamespace(mode="STABILIZE", armed=False, connected=True)
    node._control_loop()
    node._mission_context.request_takeoff(10.0)
    node._mavros_state = SimpleNamespace(mode=mode, armed=True, connected=True)
    node._active_mission = Mission()
    node._mission_context.publish_managed_setpoints = mock.MagicMock()
    node._mission_context.clear_all_setpoints = mock.MagicMock()
    for _ in range(5):
        clock.tick()
        node._control_loop()
    return node._active_mission


# --- the takeover latch -------------------------------------------------------------------------------------


def test_with_the_flag_off_there_is_no_latch_and_a_mode_change_stops_nothing(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch)
    assert node._takeover is None and not hasattr(node, "_takeover_rearm")
    mission = flying(node, clock)
    node._mavros_state = SimpleNamespace(mode="LOITER", armed=True, connected=True)
    before = mission.updates
    node._control_loop()
    assert mission.updates == before + 1  # 2026: the executor goes on ticking


def test_a_mode_nobody_asked_for_latches_the_executor_out(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    assert node._takeover is not None and not node._takeover.latched
    mission = flying(node, clock)
    assert mission.updates == 5 and node._mission_context.publish_managed_setpoints.call_count == 5

    cancelled = []
    answers = iter([SimpleNamespace(done=lambda: True, result=lambda: None), SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))])
    node._takeoff_cancel_client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda request: cancelled.append("cancel") or next(answers))
    node._mavros_state = SimpleNamespace(mode="LOITER", armed=True, connected=True)  # the safety pilot's switch
    asked = []
    node._mission_context.request_mode_change = lambda mode, callback=None: asked.append(mode)
    for _ in range(100):
        clock.tick()
        node._control_loop()
    assert node._takeover.latched
    assert mission.updates == 5  # no mission is ticked
    assert node._mission_context.publish_managed_setpoints.call_count == 5  # no setpoint is sent
    assert node._mission_context.clear_all_setpoints.call_count == 1  # cleared once, at the takeover
    # The take-off service is told to drop what it has pending: asked again a second later when the first call
    # came back with nothing, and no more once it has said yes.
    assert cancelled == ["cancel", "cancel"]
    assert asked == []  # and no mode is asked for
    said = [str(call.args[0]) for call in node.logger.error.call_args_list]
    assert sum("TAKEOVER" in line for line in said) == 1 and "rearm_autonomy" in "".join(said)

    # Not even the abort acts while somebody else has the vehicle.
    node._abort_requested = True
    node._mavros_state = SimpleNamespace(mode="GUIDED", armed=True, connected=True)  # GUIDED again does not hand it back
    for _ in range(20):
        clock.tick()
        node._control_loop()
    assert asked == [] and mission.updates == 5


def test_a_mode_the_executor_asked_for_is_not_a_takeover(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    mission = flying(node, clock)
    node._mission_context.request_mode_change("LAND")  # a mission's own landing
    clock.tick(0.4)
    node._mavros_state = SimpleNamespace(mode="LAND", armed=True, connected=True)
    node._control_loop()
    assert not node._takeover.latched and mission.updates == 6


def test_rearming_by_hand_in_guided_gives_the_executor_back(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    mission = flying(node, clock)
    node._mavros_state = SimpleNamespace(mode="ALT_HOLD", armed=True, connected=True)
    node._control_loop()
    assert node._takeover.latched

    response = node._takeover_rearm(None, SimpleNamespace(success=None, message=""))
    assert response.success is False and "GUIDED" in response.message and node._takeover.latched
    node._mavros_state = SimpleNamespace(mode="GUIDED", armed=True, connected=True)
    node._control_loop()
    assert mission.updates == 5  # still out: nobody has re-armed
    response = node._takeover_rearm(None, SimpleNamespace(success=None, message=""))
    assert response.success is True and not node._takeover.latched
    assert (mission.exited, mission.entered) == (0, 0)
    asked = []
    node._mission_context.request_mode_change = lambda mode, callback=None: asked.append(mode) or mock.MagicMock()
    node._mode_client = SimpleNamespace(service_is_ready=lambda: True)
    clock.tick()
    node._control_loop()
    # The interrupted mission is neither resumed nor entered again: it ends by its failure policy, and the abort brings the vehicle home.
    assert (mission.exited, mission.entered, mission.updates) == (1, 0, 5)
    assert node._active_mission is None and node._abort_requested
    clock.tick()
    node._control_loop()
    assert asked == ["RTL"]


def test_a_mission_whose_policy_is_to_carry_on_goes_to_the_next_mission_after_the_rearm(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    mission = flying(node, clock)
    mission.spec.failure_policy = executor_module.MissionFailurePolicy.CONTINUE_TO_NEXT
    index = node._active_index
    node._mavros_state = SimpleNamespace(mode="LOITER", armed=True, connected=True)
    node._control_loop()
    node._mavros_state = SimpleNamespace(mode="GUIDED", armed=True, connected=True)
    node._takeover_rearm(None, SimpleNamespace(success=None, message=""))
    clock.tick()
    node._control_loop()
    assert mission.exited == 1 and node._active_index == index + 1 and not node._abort_requested


def test_an_abort_that_a_takeover_interrupted_asks_for_its_mode_again_after_the_rearm(executor_module, monkeypatch, clock):  # noqa: F811
    """The flight-safety review: the abort's RTL had been accepted, so 2026's code would never ask again."""
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    flying(node, clock)
    node._active_mission = None
    node._abort_requested, node._abort_mode_requested = True, True  # RTL asked for and accepted
    node._mission_context.request_mode_change("RTL")
    node._mavros_state = SimpleNamespace(mode="RTL", armed=True, connected=True)
    clock.tick()
    node._control_loop()
    node._mavros_state = SimpleNamespace(mode="LOITER", armed=True, connected=True)  # the pilot takes it out of the RTL
    clock.tick()
    node._control_loop()
    assert node._takeover.latched
    node._mavros_state = SimpleNamespace(mode="GUIDED", armed=True, connected=True)
    assert node._takeover_rearm(None, SimpleNamespace(success=None, message="")).success
    asked = []
    node._mission_context.request_mode_change = lambda mode, callback=None: asked.append(mode) or mock.MagicMock()
    node._mode_client = SimpleNamespace(service_is_ready=lambda: True)
    clock.tick()
    node._control_loop()
    assert asked == ["RTL"]  # the vehicle is not left hovering in GUIDED with nobody flying it


def test_every_shipped_sequence_declares_the_latch_off():
    files = sorted(SEQUENCES.glob("*.yaml"))
    assert len(files) == 20  # with the fence demonstration's (OPS-02)
    for file in files:
        features = yaml.safe_load(file.read_text())["mission_sequence"]["features"]
        assert features["takeover_latch"] is False, file.name
        if "actuation_lock" in features:
            assert features["actuation_lock"] is False, file.name
    actuating = {file.name for file in files if "actuation_lock" in yaml.safe_load(file.read_text())["mission_sequence"]["features"]}
    # Every sequence with a drop, a delivery or a recovery in it, and no other.
    for file in files:
        types = {item["type"] for item in yaml.safe_load(file.read_text())["mission_sequence"]["missions"]}
        assert (file.name in actuating) == bool(types & {"package_drop", "package_delivery", "package_recovery"}), file.name


# --- the actuation lock -------------------------------------------------------------------------------------


class DoneFuture:
    """rclpy's Future as far as the context uses it for a refusal."""

    def __init__(self):
        self._result, self._done = None, False

    def set_result(self, result):
        self._result, self._done = result, True

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


@pytest.fixture
def locked(executor_module, monkeypatch, clock, tmp_path):  # noqa: F811
    sequence = tmp_path / "lockable.yaml"
    sequence.write_text(LOCKABLE)
    node = start(executor_module, monkeypatch, sequence_file=str(sequence), feature_overrides="actuation_lock")
    assert node._actuation_lock is not None
    from drone_mission_core import mission_context

    monkeypatch.setattr(mission_context, "Future", DoneFuture)
    sent = []
    node._command_client = SimpleNamespace(call_async=lambda request: sent.append(request) or mock.MagicMock(), service_is_ready=lambda: True)
    return node, sent


def test_with_the_flag_off_there_is_no_lock_and_commands_go_out_as_in_2026(executor_module, monkeypatch, clock):  # noqa: F811
    node = start(executor_module, monkeypatch)
    assert node._actuation_lock is None
    sent = []
    node._command_client = SimpleNamespace(call_async=lambda request: sent.append(request) or mock.MagicMock())
    node._mission_context.command_gripper(release=True)
    node._mission_context.command_sprayer(enable=True)
    assert len(sent) == 2
    assert node._mission_context.permit_actuation("gripper", state="TRANSIT_TO_TARGET", gates_hold=False) is True  # nothing to ask


def test_a_command_with_no_permit_is_not_sent_and_the_mission_hears_not_accepted(locked):
    node, sent = locked
    heard = []
    future = node._mission_context.command_gripper(release=True, done_callback=lambda f: heard.append(f.result()))
    assert sent == [] and heard == [None] and future.result() is None
    node._mission_context.command_sprayer(enable=True, done_callback=lambda f: heard.append(f.result()))
    assert sent == [] and heard == [None, None]
    said = [str(call.args[0]) for call in node.logger.error.call_args_list]
    assert sum("Actuation lock" in line and "not sent" in line for line in said) == 2
    assert [r[0] for r in node._actuation_lock.refused] == ["gripper", "sprayer"]


def test_a_release_state_whose_gates_hold_gets_its_one_command_out(locked, clock):
    node, sent = locked
    context = node._mission_context
    assert context.permit_actuation("gripper", state="GROUND_DWELL", gates_hold=True) is True
    context.command_gripper(release=True)
    assert len(sent) == 1 and sent[0].command == 211
    context.command_gripper(release=True)  # the same tick, a second command: its permit is spent
    assert len(sent) == 1
    # From a state that is not a release state, or with gates that do not hold, nothing goes.
    assert context.permit_actuation("gripper", state="TRACK_AND_DESCEND", gates_hold=True) is False
    context.command_gripper(release=True)
    assert context.permit_actuation("sprayer", state="DROP_PAYLOAD", gates_hold=False, why="the track is stale") is False
    context.command_sprayer(enable=True)
    assert len(sent) == 1
    # A permit from an earlier tick is stale.
    context.permit_actuation("sprayer", state="DROP_PAYLOAD", gates_hold=True)
    clock.tick(0.3)
    context.command_sprayer(enable=True)
    assert len(sent) == 1
    # Turning the drop servo off is never locked.
    context.command_sprayer(enable=False)
    assert len(sent) == 2 and sent[1].param1 == 0.0


def test_a_locked_sequence_whose_release_would_not_ask_does_not_start(executor_module, monkeypatch, clock):  # noqa: F811
    with pytest.raises(ValueError, match="actuation_lock is on.*delivery_soft_set_down"):
        start(executor_module, monkeypatch, feature_overrides="actuation_lock")


def test_with_the_latch_on_nothing_is_flown_until_the_take_off_services_cancel_is_there(executor_module, monkeypatch, clock):  # noqa: F811
    """The flight-safety review: the latch and the cancel are two flags in two files, and a latch without the cancel
    was only found out at the takeover."""
    node = start(executor_module, monkeypatch, feature_overrides="takeover_latch")
    ready = [False]
    node._takeoff_cancel_client = SimpleNamespace(service_is_ready=lambda: ready[0], call_async=lambda request: pytest.fail("called a service that is not there"))
    node._mission_context.publish_managed_setpoints = mock.MagicMock()
    node._mavros_state = SimpleNamespace(mode="STABILIZE", armed=False, connected=True)
    for _ in range(300):  # fifteen seconds on the ground
        clock.tick()
        node._control_loop()
    assert node._active_mission is None and node._active_index == 0  # the take-off mission was never started
    assert node._mission_context.publish_managed_setpoints.call_count == 0
    said = [str(call.args[0]) for call in node.logger.error.call_args_list if "waiting for the take-off service's cancel" in str(call.args[0])]
    assert 3 <= len(said) <= 4 and "features.takeoff_cancel" in said[0]  # every five seconds, not every tick
    ready[0] = True
    clock.tick()
    node._control_loop()
    assert node._active_mission is not None  # and then the sequence begins
    ready[0] = False  # the service going away later does not ground a flight that is under way
    clock.tick()
    node._control_loop()
    assert node._mission_context.publish_managed_setpoints.call_count >= 1
