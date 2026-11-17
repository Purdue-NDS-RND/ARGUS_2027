"""PLT-06: the ARGUS MAVROS config (time sync, only the plugins we use) and the stream keeper's logic. No ROS."""

import ast
import re
import subprocess
from pathlib import Path

import pytest
import yaml

from drone_utils.stream_keeper import ANSWER_TIMEOUT_S, FAILURE_PAUSE_S, MESSAGE_RATES_HZ, REASSERT_S, SET_MESSAGE_INTERVAL, StreamKeeper, interval_us

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
}  # fmt: skip
# Named in the tree without being used through MAVROS: file names, and the
# node the sim's time check knows not to ask for parameters.
NOT_TOPICS = {"argus_mavros", "install_geographiclib_datasets", "param"}


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
    assert len(allowed) == 12


def test_the_allowed_plugins_are_exactly_the_ones_the_repo_uses():
    allowed = set(config()["/**"]["ros__parameters"]["plugin_allowlist"])
    found = subprocess.run(
        ["git", "grep", "-ohE", r"/mavros/[a-z_]+", "--", "src", "sim", "tools", "*.sh", "*.py", ":!*/test/*", ":!sim/gps/*"],
        cwd=REPO, capture_output=True, text=True, check=True,
    ).stdout.split()  # fmt: skip
    used = {name.split("/")[2] for name in found} - NOT_TOPICS
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
    }
    assert {message_id: name for message_id, (name, _) in MESSAGE_RATES_HZ.items()} == {
        32: "LOCAL_POSITION_NED", 30: "ATTITUDE", 173: "RANGEFINDER", 24: "GPS_RAW_INT",
        147: "BATTERY_STATUS", 33: "GLOBAL_POSITION_INT", 245: "EXTENDED_SYS_STATE", 1: "SYS_STATUS",
    }  # fmt: skip


def test_the_request_is_the_set_message_interval_command():
    assert SET_MESSAGE_INTERVAL == 511
    assert interval_us(30.0) == pytest.approx(33333.33, abs=0.01) and interval_us(2.0) == 500000.0


# --- the keeper ---------------------------------------------------------------------------------------

ALL = sorted((message_id, rate) for message_id, (_, rate) in MESSAGE_RATES_HZ.items())


def connected(now=0.0):
    keeper = StreamKeeper()
    keeper.on_state(True, now)
    return keeper


def run(keeper, start, seconds=2.0, answer=True, step=0.05):
    """Tick the keeper, answering each request on the next tick. Returns (time sent, message id, rate)."""
    sent, pending, now = [], None, start
    while now < start + seconds:
        if pending is not None:
            keeper.on_result(pending, answer, now)
            pending = None
        for message_id, rate in keeper.due(now):
            sent.append((round(now, 3), message_id, rate))
            pending = message_id
        now = round(now + step, 6)
    if pending is not None:
        keeper.on_result(pending, answer, now)
    return sent


def requests(sent):
    return sorted((message_id, rate) for _when, message_id, rate in sent)


def test_nothing_is_requested_before_the_flight_controller_is_connected():
    keeper = StreamKeeper()
    assert keeper.due(0.0) == []
    keeper.on_state(False, 1.0)
    assert keeper.due(1.0) == []


def test_every_rate_is_requested_on_connection_one_at_a_time():
    keeper = connected()
    first = keeper.due(0.0)
    assert len(first) == 1
    assert keeper.due(0.1) == [] and keeper.due(1.0) == []  # nothing more until that one is answered
    keeper.on_result(first[0][0], True, 1.1)
    sent = [(1.1, *first[0])] + run(keeper, 1.1)
    assert requests(sent) == ALL
    assert keeper.holding() and keeper.due(3.5) == []


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
    keeper = connected()
    run(keeper, 0.0)
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


def test_rates_are_asserted_again_every_ten_seconds():
    # Anything else that sets rates (the takeoff service's request for all streams) is undone within REASSERT_S.
    keeper = connected()
    first = run(keeper, 0.0)
    assert run(keeper, 2.0, seconds=REASSERT_S - 2.5) == []
    again = run(keeper, REASSERT_S - 0.5, seconds=3.0)
    assert requests(again) == ALL
    assert all(later - earlier == pytest.approx(REASSERT_S, abs=0.11) for (earlier, _, _), (later, _, _) in zip(first, again))
    assert REASSERT_S == 10.0


def test_a_refused_request_stops_everything_for_a_pause_then_is_tried_again():
    keeper = connected()
    first = keeper.due(0.0)[0]
    keeper.on_result(first[0], False, 0.1)
    assert keeper.failures == 1 and not keeper.holding()
    assert keeper.last_failed == first[0]
    assert run(keeper, 0.1, seconds=FAILURE_PAUSE_S - 0.2) == []  # nothing at all, not even the other messages
    sent = run(keeper, FAILURE_PAUSE_S, seconds=2.0)
    assert requests(sent) == [item for item in ALL if item[0] != first[0]]  # after the pause the others go first
    assert not keeper.holding()
    again = run(keeper, FAILURE_PAUSE_S + 2.0, seconds=REASSERT_S)
    assert first[0] in [message_id for _when, message_id, _rate in again]  # and the refused one a full repeat later
    assert keeper.failures == 0 and keeper.holding()


def test_one_message_the_flight_controller_never_gives_does_not_starve_the_others():
    # For example firmware with no rangefinder driver refusing RANGEFINDER, third in the table.
    keeper = connected()
    sent, pending, now = [], None, 0.0
    while now < 60.0:
        if pending is not None:
            keeper.on_result(pending, pending != 173, now)
            pending = None
        for message_id, _rate in keeper.due(now):
            sent.append((now, message_id))
            pending = message_id
        now = round(now + 0.05, 6)
    counts = {message_id: sum(1 for _when, sent_id in sent if sent_id == message_id) for message_id in MESSAGE_RATES_HZ}
    assert all(count >= 3 for count in counts.values()), counts  # every other message keeps being asserted
    assert 4 <= counts[173] <= 7  # the refused one about once per repeat, not once per pause
    first_pass = [message_id for when, message_id in sent if when < 12.0]
    assert set(first_pass) == set(MESSAGE_RATES_HZ)  # all eight asked within one repeat of connecting
    assert not keeper.holding() and keeper.last_failed == 173


def test_a_flight_controller_that_refuses_everything_gets_one_request_per_pause():
    keeper = connected()
    sent = run(keeper, 0.0, seconds=60.0, answer=False)
    assert 10 <= len(sent) <= 13  # one every FAILURE_PAUSE_S or so, not a storm
    assert not keeper.holding() and keeper.failures == len(sent)


def test_a_request_that_is_never_answered_is_given_up_and_counts_as_a_failure():
    keeper = connected()
    assert len(keeper.due(0.0)) == 1
    assert keeper.due(ANSWER_TIMEOUT_S - 0.1) == []
    assert keeper.due(ANSWER_TIMEOUT_S + 0.1) == [] and keeper.failures == 1  # given up; now the pause
    assert len(keeper.due(ANSWER_TIMEOUT_S + 0.1 + FAILURE_PAUSE_S + 0.01)) == 1
    assert ANSWER_TIMEOUT_S > 5.0  # longer than MAVROS waits for the flight controller's acknowledgement


def test_a_late_or_foreign_answer_is_ignored():
    keeper = connected()
    run(keeper, 0.0)
    keeper.on_result(999, False, 2.5)  # not a message the keeper holds
    keeper.on_result(32, False, 2.6)  # an answer when nothing is out
    assert keeper.holding() and keeper.failures == 0


def test_rates_can_be_given():
    keeper = StreamKeeper({32: ("LOCAL_POSITION_NED", 10.0)}, reassert_s=2.0)
    keeper.on_state(True, 0.0)
    assert keeper.due(0.0) == [(32, 10.0)]
    with pytest.raises(ValueError):
        StreamKeeper({32: ("LOCAL_POSITION_NED", 0.0)})


def test_the_logic_has_no_ros_in_it_and_the_node_is_thin():
    import drone_utils.stream_keeper as module

    tree = ast.parse(Path(module.__file__).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "typing"}
    node_source = NODE.read_text()
    assert "/mavros/cmd/command" in node_source and "/mavros/timesync_status" in node_source and "/mavros/state" in node_source
    # Not MAVROS's set_message_interval service: it blocks the group that also serves set_mode for a second per call.
    assert "create_client(CommandLong" in node_source and "MessageInterval" not in node_source
    assert len(node_source.splitlines()) < 110
