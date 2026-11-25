"""PLT-06: the ARGUS MAVROS config (time sync, only the plugins we use) and the stream keeper's logic. No ROS."""

import ast
import re
import subprocess
from pathlib import Path

import pytest
import yaml

from drone_utils.stream_keeper import (
    ANSWER_TIMEOUT_S,
    DROP_FRACTION,
    FAILURE_PAUSE_S,
    HOLD_STALE_S,
    MAX_FRUITLESS,
    MESSAGE_RATES_HZ,
    RATE_WINDOW_S,
    SET_MESSAGE_INTERVAL,
    ActuationWatch,
    StreamKeeper,
    interval_us,
    is_actuation_state,
)

REPO = Path(__file__).resolve().parents[3]
CONFIG = REPO / "config" / "mavros" / "argus_mavros.yaml"
LAUNCH = REPO / "config" / "mavros" / "argus_mavros.launch.py"
NODE = REPO / "src" / "drone_utils" / "drone_utils" / "stream_keeper_node.py"

# Which MAVROS plugin serves each first path segment under /mavros/.
PLUGIN_OF = {
    "state": "sys_status", "extended_state": "sys_status", "statustext": "sys_status", "sys_status": "sys_status",
    "battery": "sys_status", "set_stream_rate": "sys_status", "set_message_interval": "sys_status", "set_mode": "sys_status",
    "timesync_status": "sys_time", "time_reference": "sys_time", "time": "sys_time",
    "cmd": "command", "local_position": "local_position", "global_position": "global_position", "imu": "imu",
    "setpoint_raw": "setpoint_raw", "setpoint_position": "setpoint_position", "home_position": "home_position",
    "rangefinder": "rangefinder", "gpsstatus": "gps_status", "gimbal_control": "gimbal_control",
    "vfr_hud": "vfr_hud",  # the throttle, for the delivery's contact rule (CEN-05)
}  # fmt: skip
# Named in the tree without being used through MAVROS: file names, and the
# node the sim's time check knows not to ask for parameters.
NOT_TOPICS = {"argus_mavros", "install_geographiclib_datasets", "param"}
# Used only behind a flag, by code that flies on without it, and whose plugin the aircraft's MAVROS does not load:
# a plugin in the allow-list runs with every flag off, so adding one is the lead's to decide. /mavros/mission is
# the AUTO mission's upload (PLT-13, flag time_trial_auto); with no `waypoint` plugin the time trial flies in GUIDED.
# /mavros/optical_flow is where the flow odometry sends its samples (REC-5, node flag flow_odometry); with no
# `optical_flow` plugin the node still publishes its velocity on its own topics and the flight controller hears nothing.
# /mavros/landing_target/raw is where the precision-loiter bridge sends the estimator's target (CEN-09, mission flag
# precision_loiter_bridge: a benchmark); with no `landing_target` plugin (and its `listen_lt` true) nobody listens.
FLAG_ONLY_NOT_ALLOWED = {"mission": "waypoint", "optical_flow": "optical_flow", "landing_target": "landing_target"}


# --- the config file -----------------------------------------------------------------------------


def config():
    return yaml.safe_load(CONFIG.read_text())


def test_time_sync_stays_at_10_hz():
    assert config()["/**/time"]["ros__parameters"]["timesync_rate"] == 10.0


def test_only_listed_plugins_load():
    common = config()["/**"]["ros__parameters"]
    assert common["plugin_denylist"] == ["*"]  # MAVROS loads a denied plugin only if the allow-list names it
    allowed = common["plugin_allowlist"]
    assert len(allowed) == len(set(allowed)) and all(re.fullmatch(r"[a-z_]+", name) for name in allowed)
    assert set(allowed) == set(PLUGIN_OF.values())
    assert len(allowed) == 13


def test_the_allowed_plugins_are_exactly_the_ones_the_repo_uses():
    allowed = set(config()["/**"]["ros__parameters"]["plugin_allowlist"])
    found = subprocess.run(
        ["git", "grep", "-ohE", r"/mavros/[a-z_]+", "--", "src", "sim", "tools", "*.sh", "*.py", ":!*/test/*", ":!sim/gps/*"],
        cwd=REPO, capture_output=True, text=True, check=True,
    ).stdout.split()  # fmt: skip
    used = {name.split("/")[2] for name in found} - NOT_TOPICS - set(FLAG_ONLY_NOT_ALLOWED)
    assert not set(FLAG_ONLY_NOT_ALLOWED.values()) & allowed  # (once the lead allows one, it moves to PLUGIN_OF)
    assert len(used) >= 15
    unknown = sorted(used - set(PLUGIN_OF))
    assert unknown == [], f"/mavros/{unknown[0]}/... is used but no plugin is mapped to it here; add its plugin to argus_mavros.yaml"
    # Both ways: nothing used is missing, and nothing is loaded that nothing uses.
    assert {PLUGIN_OF[name] for name in used} == allowed


def test_the_config_is_a_plain_params_file():
    data = config()
    assert "features" not in data
    assert all(isinstance(block, dict) and set(block) == {"ros__parameters"} for block in data.values())


def test_the_launch_starts_the_stream_keeper_only_when_asked():
    tree = ast.parse(LAUNCH.read_text())
    defaults = {
        call.args[0].value: next((getattr(kw.value, "value", None) for kw in call.keywords if kw.arg == "default_value"), None)
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and getattr(call.func, "id", "") == "DeclareLaunchArgument"
    }
    assert defaults["stream_keeper"] == "false" and defaults["use_sim_time"] == "false"
    keeper = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node" and any(kw.arg == "executable" and getattr(kw.value, "value", "") == "stream_keeper" for kw in node.keywords)
    )
    keywords = {kw.arg: ast.unparse(kw.value) for kw in keeper.keywords}
    assert keywords["package"] == "'drone_utils'"
    assert keywords["condition"] == "IfCondition(LaunchConfiguration('stream_keeper'))"
    assert "use_sim_time" in keywords["parameters"]
    setup = (REPO / "src" / "drone_utils" / "setup.py").read_text()
    assert "'stream_keeper = drone_utils.stream_keeper_node:main'" in setup


# --- the rates -------------------------------------------------------------------------------------


def test_rates_are_the_ones_in_the_spec():
    by_name = {name: rate for name, rate in MESSAGE_RATES_HZ.values()}
    spec = {
        "LOCAL_POSITION_NED": 30.0,  # pose
        "ATTITUDE": 50.0,
        "RANGEFINDER": 20.0,
        "GPS_RAW_INT": 5.0,
        "BATTERY_STATUS": 2.0,
    }
    assert {name: by_name[name] for name in spec} == spec
    # Not in the spec's list: held at what the 2026 start-up gives them (simple_takeoff_service:
    # every stream at 20 Hz, EXTENDED_SYS_STATE at 2 Hz), so the keeper takes nothing away.
    assert {name: rate for name, rate in by_name.items() if name not in spec} == {
        "GLOBAL_POSITION_INT": 20.0,
        "EXTENDED_SYS_STATE": 2.0,
        "SYS_STATUS": 2.0,
        "VFR_HUD": 20.0,  # the throttle: three tenths of a second of it decide a delivery's contact (CEN-05)
    }
    assert {message_id: name for message_id, (name, _) in MESSAGE_RATES_HZ.items()} == {
        32: "LOCAL_POSITION_NED", 30: "ATTITUDE", 173: "RANGEFINDER", 24: "GPS_RAW_INT",
        147: "BATTERY_STATUS", 33: "GLOBAL_POSITION_INT", 245: "EXTENDED_SYS_STATE", 1: "SYS_STATUS",
        74: "VFR_HUD",
    }  # fmt: skip


def test_the_request_is_the_set_message_interval_command():
    assert SET_MESSAGE_INTERVAL == 511
    assert interval_us(30.0) == pytest.approx(33333.33, abs=0.01) and interval_us(2.0) == 500000.0


# --- the keeper ---------------------------------------------------------------------------------------

ALL = sorted((message_id, rate) for message_id, (_, rate) in MESSAGE_RATES_HZ.items())
AT_RATE = {message_id: rate for message_id, (_, rate) in MESSAGE_RATES_HZ.items()}


def connected(now=0.0):
    keeper = StreamKeeper()
    keeper.on_state(True, now)
    return keeper


def run(keeper, start, seconds=2.0, answer=True, step=0.05, arriving=AT_RATE, obeyed=False):
    """Tick the keeper, answering each request on the next tick, with messages arriving at the given rates.

    Returns the requests sent as (time, message id, rate). `answer` may be a function of the message id.
    With `obeyed`, an accepted request brings that message back to its asked rate, as a flight controller does.
    """
    arriving = dict(arriving)
    sent, pending, now, owed = [], None, start, {message_id: 0.0 for message_id in arriving}
    while now < start + seconds - 1e-9:
        for message_id, hz in arriving.items():  # the messages that arrived during this step
            owed[message_id] += hz * step
            while owed[message_id] >= 1.0:
                keeper.on_message(message_id)
                owed[message_id] -= 1.0
        if pending is not None:
            accepted = answer(pending) if callable(answer) else answer
            keeper.on_result(pending, accepted, now)
            if accepted and obeyed:
                arriving[pending] = AT_RATE[pending]
            pending = None
        for message_id, rate in keeper.due(now):
            sent.append((round(now, 3), message_id, rate))
            pending = message_id
        now = round(now + step, 6)
    if pending is not None:
        keeper.on_result(pending, answer(pending) if callable(answer) else answer, now)
    return sent


def requests(sent):
    return sorted((message_id, rate) for _when, message_id, rate in sent)


def settled(now=0.0):
    """A keeper whose every rate has been asked for and accepted, with messages arriving at their rates."""
    keeper = connected(now)
    assert requests(run(keeper, now, seconds=2.0)) == ALL
    assert keeper.holding()
    return keeper


def test_nothing_is_requested_before_the_flight_controller_is_connected():
    keeper = StreamKeeper()
    assert keeper.due(0.0) == []
    keeper.on_state(False, 1.0)
    assert keeper.due(1.0) == []


def test_every_rate_is_requested_once_on_connection_one_at_a_time():
    keeper = connected()
    first = keeper.due(0.0)
    assert len(first) == 1
    assert keeper.due(0.1) == [] and keeper.due(1.0) == []  # nothing more until that one is answered
    keeper.on_result(first[0][0], True, 1.1)
    sent = [(1.1, *first[0])] + run(keeper, 1.1)
    assert requests(sent) == ALL
    assert keeper.holding()


def test_with_the_rates_holding_nothing_is_ever_asked_again():
    # No periodic repeat: every request sits in MAVROS's command queue, where a release would wait behind it.
    keeper = settled()
    assert run(keeper, 2.0, seconds=300.0) == []
    assert keeper.rate_drops == 0 and keeper.reboots == 0 and keeper.holding()


def test_never_more_than_one_request_is_out():
    keeper = connected()
    out = 0
    for tick in range(400):
        now = tick * 0.05
        if out and tick % 3 == 0:  # answers arrive a few ticks late
            keeper.on_result(pending, True, now)
            out -= 1
        for message_id, _rate in keeper.due(now):
            pending = message_id
            out += 1
        assert out <= 1


def test_rates_are_requested_again_after_a_reboot_seen_as_a_reconnect():
    keeper = settled()
    keeper.on_state(False, 10.0)
    assert keeper.due(10.5) == [] and not keeper.holding()
    keeper.on_state(True, 14.0)
    assert requests(run(keeper, 14.0)) == ALL
    assert keeper.reboots == 1 and keeper.holding()


def test_rates_are_requested_again_after_a_reboot_seen_as_the_flight_controller_clock_going_back():
    # A reboot faster than MAVROS's 10 s heartbeat timeout never shows as a disconnect.
    keeper = connected()
    keeper.on_fcu_time(500_000_000_000, 0.0)
    run(keeper, 0.0)
    keeper.on_fcu_time(500_100_000_000, 2.1)
    assert keeper.due(2.2) == []
    keeper.on_fcu_time(3_000_000_000, 3.0)  # 3 s since boot: it restarted
    assert not keeper.holding() and keeper.reboots == 1
    assert requests(run(keeper, 3.0)) == ALL
    keeper.on_fcu_time(3_100_000_000, 5.1)
    assert keeper.due(5.2) == [] and keeper.reboots == 1


def test_a_small_backward_step_of_the_flight_controller_clock_is_not_a_reboot():
    keeper = connected()
    keeper.on_fcu_time(500_000_000_000, 0.0)
    run(keeper, 0.0)
    keeper.on_fcu_time(499_999_000_000, 2.1)  # 1 ms back: message order
    assert keeper.due(2.2) == [] and keeper.reboots == 0


def test_a_rate_that_drops_is_requested_again_and_only_that_one():
    # The takeoff service asks for every stream at 20 Hz: pose falls from 30 and attitude from 50; the rest only rise.
    keeper = settled()
    after_takeoff = {**AT_RATE, 32: 20.0, 30: 20.0, 24: 20.0, 147: 20.0, 245: 20.0, 1: 20.0}
    sent = run(keeper, 2.0, seconds=2.0 * RATE_WINDOW_S + 1.0, arriving=after_takeoff, obeyed=True)
    assert requests(sent) == [(30, 50.0), (32, 30.0)]
    assert all(when <= 2.0 + 2.0 * RATE_WINDOW_S + 0.5 for when, _id, _rate in sent)  # noticed within two windows at most
    assert keeper.rate_drops == 2
    assert run(keeper, 13.0, seconds=60.0) == [] and keeper.holding()  # restored: quiet again


def test_a_rate_a_little_under_what_was_asked_is_not_a_drop():
    keeper = settled()
    assert DROP_FRACTION == 0.8
    assert run(keeper, 2.0, seconds=30.0, arriving={**AT_RATE, 32: 25.5, 30: 42.0}) == []  # 85% and 84%
    assert requests(run(keeper, 32.0, seconds=2.0 * RATE_WINDOW_S + 1.0, arriving={**AT_RATE, 32: 22.0}, obeyed=True)) == [(32, 30.0)]  # 73%


def test_a_reboot_nobody_announced_shows_as_every_rate_dropping():
    # In the sim neither reboot signal shows; the streams just stop.
    keeper = settled()
    silent = {message_id: 0.0 for message_id in AT_RATE}
    sent = run(keeper, 2.0, seconds=RATE_WINDOW_S + 3.0, arriving=silent)
    assert requests(sent) == ALL and keeper.rate_drops == len(ALL)


def test_a_message_that_never_comes_is_asked_for_a_few_times_and_then_left_alone():
    # A vehicle with no rangefinder: the flight controller accepts the request and sends nothing.
    keeper = connected()
    no_rangefinder = {**AT_RATE, 173: 0.0}
    sent = run(keeper, 0.0, seconds=120.0, arriving=no_rangefinder)
    asked = [message_id for _when, message_id, _rate in sent]
    assert asked.count(173) == 1 + MAX_FRUITLESS and MAX_FRUITLESS == 3
    assert all(asked.count(message_id) == 1 for message_id in AT_RATE if message_id != 173)
    assert keeper.given_up == {173} and keeper.holding()
    assert run(keeper, 120.0, seconds=120.0, arriving=no_rangefinder) == []
    keeper.on_state(False, 240.0)
    keeper.on_state(True, 250.0)  # after a reboot it is tried again
    assert 173 in [message_id for _when, message_id, _rate in run(keeper, 250.0, arriving=no_rangefinder)]


def test_a_message_given_up_is_watched_again_once_it_does_arrive():
    keeper = connected()
    run(keeper, 0.0, seconds=60.0, arriving={**AT_RATE, 173: 0.0})
    assert keeper.given_up == {173}
    assert run(keeper, 60.0, seconds=RATE_WINDOW_S + 1.0) == []  # the rangefinder starts sending at its rate
    assert keeper.given_up == set() and keeper.holding()
    sent = run(keeper, 66.0, seconds=2.0 * RATE_WINDOW_S + 1.0, arriving={**AT_RATE, 173: 4.0}, obeyed=True)
    assert requests(sent) == [(173, 20.0)]  # and a later drop is acted on


def test_a_step_of_the_clock_is_not_read_as_every_rate_dropping():
    keeper = settled()
    run(keeper, 2.0, seconds=3.0)
    assert run(keeper, 3600.0, seconds=RATE_WINDOW_S + 2.0) == []  # the clock jumped forward an hour
    assert run(keeper, 100.0, seconds=RATE_WINDOW_S + 2.0) == []  # and back
    assert keeper.rate_drops == 0
    still = run(keeper, 100.0 + RATE_WINDOW_S + 2.0, seconds=2.0 * RATE_WINDOW_S + 1.0, arriving={**AT_RATE, 32: 4.0}, obeyed=True)
    assert requests(still) == [(32, 30.0)]  # still watching


def test_nothing_is_sent_while_a_mission_is_in_a_release_or_actuation_state():
    keeper = settled()
    keeper.hold = True
    dropped = {**AT_RATE, 32: 4.0}
    assert run(keeper, 2.0, seconds=30.0, arriving=dropped) == []  # the drop is seen, and waits
    assert keeper.rate_drops >= 1
    keeper.hold = False
    sent = run(keeper, 32.0, seconds=1.0, arriving=dropped)
    assert requests(sent) == [(32, 30.0)]


def test_a_hold_also_stops_the_first_requests_and_a_reboots():
    keeper = StreamKeeper()
    keeper.hold = True
    keeper.on_state(True, 0.0)
    assert run(keeper, 0.0, seconds=20.0) == []
    keeper.on_state(False, 20.0)
    keeper.on_state(True, 25.0)
    assert run(keeper, 25.0, seconds=20.0) == [] and keeper.reboots == 1
    keeper.hold = False
    assert requests(run(keeper, 45.0)) == ALL


def test_a_refused_request_stops_everything_for_a_pause_and_the_others_go_first():
    keeper = connected()
    first = keeper.due(0.0)[0]
    keeper.on_result(first[0], False, 0.1)
    assert keeper.failures == 1 and keeper.last_failed == first[0] and not keeper.holding()
    assert run(keeper, 0.1, seconds=FAILURE_PAUSE_S - 0.2) == []  # nothing at all, not even the other messages
    sent = run(keeper, FAILURE_PAUSE_S, seconds=2.0)
    assert requests(sent) == [item for item in ALL if item[0] != first[0]]
    again = run(keeper, FAILURE_PAUSE_S + 2.0, seconds=2.0 * FAILURE_PAUSE_S)
    assert [message_id for _when, message_id, _rate in again] == [first[0]]
    assert keeper.failures == 0 and keeper.holding()


def test_one_message_the_flight_controller_refuses_does_not_starve_the_others():
    keeper = connected()
    sent = run(keeper, 0.0, seconds=60.0, answer=lambda message_id: message_id != 173, arriving={**AT_RATE, 173: 0.0})
    counts = {message_id: sum(1 for _when, sent_id, _rate in sent if sent_id == message_id) for message_id in MESSAGE_RATES_HZ}
    assert all(counts[message_id] == 1 for message_id in counts if message_id != 173), counts
    assert 4 <= counts[173] <= 7  # about once per two pauses, not a storm
    assert set(message_id for when, message_id, _rate in sent if when < 12.0) == set(MESSAGE_RATES_HZ)
    assert not keeper.holding() and keeper.last_failed == 173


def test_a_flight_controller_that_refuses_everything_gets_one_request_per_pause():
    keeper = connected()
    sent = run(keeper, 0.0, seconds=60.0, answer=False)
    assert 10 <= len(sent) <= 13
    assert not keeper.holding() and keeper.failures == len(sent)


def test_a_request_that_is_never_answered_is_given_up_and_counts_as_a_failure():
    keeper = connected()
    assert len(keeper.due(0.0)) == 1
    assert keeper.due(ANSWER_TIMEOUT_S - 0.1) == []
    assert keeper.due(ANSWER_TIMEOUT_S + 0.1) == [] and keeper.failures == 1  # given up; now the pause
    assert len(keeper.due(ANSWER_TIMEOUT_S + 0.1 + FAILURE_PAUSE_S + 0.01)) == 1
    assert ANSWER_TIMEOUT_S > 5.0  # longer than MAVROS waits for the flight controller's acknowledgement


def test_a_late_or_foreign_answer_is_ignored():
    keeper = settled()
    keeper.on_result(999, False, 2.5)  # not a message the keeper holds
    keeper.on_result(32, False, 2.6)  # an answer when nothing is out
    assert keeper.holding() and keeper.failures == 0


def test_rates_can_be_given():
    keeper = StreamKeeper({32: ("LOCAL_POSITION_NED", 10.0)})
    keeper.on_state(True, 0.0)
    assert keeper.due(0.0) == [(32, 10.0)]
    with pytest.raises(ValueError):
        StreamKeeper({32: ("LOCAL_POSITION_NED", 0.0)})


# --- release and actuation states -----------------------------------------------------------------------


def test_the_watch_follows_the_executors_log_into_and_out_of_a_release_state():
    watch = ActuationWatch()
    assert not watch.active
    watch.on_log("[package_drop] State: ACQUIRE_TARGET -> TRACK_AND_DESCEND")
    assert not watch.active
    watch.on_log("[package_drop] State: FINAL_FIXED_DROP_COLUMN -> DROP_PAYLOAD")
    assert watch.active and watch.state == "package_drop: DROP_PAYLOAD"
    watch.on_log("[package_drop] Hovering at drop altitude for 8.0 s")
    watch.on_log("[package_drop] Closing sprayer for payload drop")
    assert watch.active  # other lines do not end it
    watch.on_log("[package_drop] State: DROP_PAYLOAD -> COMPLETE")
    assert not watch.active and watch.state is None


def test_the_watch_holds_from_the_final_column_through_the_release():
    watch = ActuationWatch()
    watch.on_log("[package_delivery] State: TRACK_AND_DESCEND -> FINAL_FIXED_COLUMN_DESCENT", 100.0)
    assert watch.holding(100.0)  # seconds before the release, not one tick
    watch.on_log("[package_delivery] State: FINAL_FIXED_COLUMN_DESCENT -> GROUND_DWELL", 125.0)
    assert watch.holding(125.1) and watch.state == "package_delivery: GROUND_DWELL"
    watch.on_log("[package_delivery] State: GROUND_DWELL -> GUIDED_RELAUNCH", 131.0)
    assert not watch.holding(131.0)


def test_a_hold_lapses_when_the_executor_has_gone_silent():
    watch = ActuationWatch()
    watch.on_log("[package_drop] State: FINAL_FIXED_DROP_COLUMN -> DROP_PAYLOAD", 100.0)
    watch.on_log("[package_drop] Hovering at drop altitude for 8.0 s", 100.1)
    assert watch.holding(100.1 + HOLD_STALE_S - 1.0)  # an 8 s dwell with nothing logged is still a hold
    assert not watch.holding(100.1 + HOLD_STALE_S + 1.0) and watch.state is None  # the executor died in that state
    assert HOLD_STALE_S == 60.0


def test_the_watch_ends_a_hold_when_the_mission_ends_without_another_state_line():
    for ending in ("Mission 'package_delivery' completed", "Mission 'package_delivery' ended with status FAILURE", "Starting mission 'rtl' (rtl)"):
        watch = ActuationWatch()
        watch.on_log("[package_delivery] State: FINAL_FIXED_COLUMN_DESCENT -> GROUND_DWELL")
        assert watch.active
        watch.on_log(ending)
        assert not watch.active, ending


@pytest.mark.parametrize("state", ["DROP_PAYLOAD", "GROUND_DWELL", "TAKING_OFF", "WAITING_FOR_ALTITUDE", "FINAL_FIXED_DROP_COLUMN", "FINAL_FIXED_COLUMN_DESCENT", "RELEASE_BUCKET", "ENGAGE_HOOK", "OPEN_GRIPPER", "ACTUATE"])
def test_release_and_actuation_states_by_name(state):
    assert is_actuation_state(state)


@pytest.mark.parametrize("state", ["INIT", "TRANSIT_TO_TARGET", "TRACK_AND_DESCEND", "ACQUIRE_TARGET", "GUIDED_RELAUNCH", "COMPLETE", "WAITING_FOR_GPS"])
def test_other_states_are_not(state):
    assert not is_actuation_state(state)


def test_the_logic_has_no_ros_in_it_and_the_node_is_thin():
    import drone_utils.stream_keeper as module

    tree = ast.parse(Path(module.__file__).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "re", "typing"}
    node_source = NODE.read_text()
    assert "/mavros/cmd/command" in node_source and "/mavros/timesync_status" in node_source and "/mavros/state" in node_source
    # Not MAVROS's set_message_interval service: it blocks the group that also serves set_mode for a second per call.
    assert "create_client(CommandLong" in node_source and "MessageInterval" not in node_source
    # It hears the executor's log, and only the executor's, for release and actuation states.
    assert 'self.create_subscription(Log, "/rosout", self._on_log' in node_source and 'if message.name == "mission_executor":' in node_source
    assert "self._watch.on_log(message.msg, self._now())" in node_source and "keeper.hold = self._watch.holding(self._now())" in node_source
    package = (REPO / "src" / "drone_utils" / "package.xml").read_text()
    assert "<exec_depend>rcl_interfaces</exec_depend>" in package and "<exec_depend>rosidl_runtime_py</exec_depend>" in package
    assert len(node_source.splitlines()) < 140


def test_the_node_measures_every_rate_it_holds_on_the_topic_that_message_becomes():
    tree = ast.parse(NODE.read_text())
    topics = next(ast.literal_eval(node.value) for node in ast.walk(tree) if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "TOPICS")
    assert set(topics) == set(MESSAGE_RATES_HZ)
    assert topics[32][0] == "/mavros/local_position/pose" and topics[30][0] == "/mavros/imu/data"
    import sys

    sys.path.insert(0, str(REPO / "sim"))
    import rate_check  # the sim's check measures the same topics

    by_name = {name: message_id for message_id, (name, _rate) in MESSAGE_RATES_HZ.items()}
    for topic, (type_name, mavlink_name, _optional) in rate_check.TOPICS.items():
        assert topics[by_name[mavlink_name]] == (topic, type_name)
