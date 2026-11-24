"""OPS-03: the post-flight report and autonomy audit (tools/post_flight.py), on bundles and logs built here. No ROS."""

import json
import struct
import sys
from pathlib import Path

import pytest
import yaml

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import bundle_attempt  # noqa: E402
import post_flight as pf  # noqa: E402
import test_dataflash_params as logs  # noqa: E402
import test_readiness_kit as kit_test  # noqa: E402  (the MODE, ERR, POS and FNCE builders)

HEAD = b"\xa3\x95"
MAVC_FMT = logs.fmt(20, "MAVC", "QBBBBBHffffiifBB", "TimeUS,TS,TC,SS,SC,Fr,Cmd,P1,P2,P3,P4,X,Y,Z,Res,WL")
RCIN_FMT = logs.fmt(21, "RCIN", "QHHHHHHHHHHHHHH", "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14")
RCI2_FMT = logs.fmt(22, "RCI2", "QHHHB", "TimeUS,C15,C16,OMask,Flags")
ARM_FMT = logs.fmt(23, "ARM", "QBIBB", "TimeUS,ArmState,ArmChecks,Forced,Method")
BAT_FMT = logs.fmt(24, "BAT", "QBfffffcfBBB", "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH")
RFND_FMT = logs.fmt(25, "RFND", "QBCBBb", "TimeUS,Instance,Dist,Stat,Orient,Quality")
FORMATS = [logs.FMT_FMT, logs.PARM_FMT, logs.MSG_FMT, kit_test.MODE_FMT, kit_test.ERR_FMT, kit_test.FNCE_FMT, kit_test.POS_FMT, MAVC_FMT, RCIN_FMT, RCI2_FMT, ARM_FMT, BAT_FMT, RFND_FMT]
HOME = (350500000, -1181500000)  # 35.05, -118.15 as the log holds them


def text(seconds, message):
    return HEAD + bytes([91]) + struct.pack("<Q64s", int(seconds * 1e6), message.encode())


def mavc(seconds, command, system=1, component=191, p1=0.0, p2=0.0):
    return HEAD + bytes([20]) + struct.pack("<QBBBBBHffffiifBB", int(seconds * 1e6), 1, 1, system, component, 3, command, p1, p2, 0.0, 0.0, 0, 0, 0.0, 0, 1)


def rfnd(seconds, metres, status=4):
    return HEAD + bytes([25]) + struct.pack("<QBHBBb", int(seconds * 1e6), 0, int(round(metres * 100)), status, 25, -1)


def parm_at(seconds, name, value):
    return HEAD + bytes([64]) + struct.pack("<Q16sff", int(seconds * 1e6), name.encode(), value, 0.0)


def rcin(seconds, c1=1500, c2=1500, c3=1000, c4=1500, c5=1800):
    return HEAD + bytes([21]) + struct.pack("<Q14H", int(seconds * 1e6), c1, c2, c3, c4, c5, 0, 1000, 0, 0, 0, 0, 0, 0, 0)


def rci2(seconds, mask):
    return HEAD + bytes([22]) + struct.pack("<QHHHB", int(seconds * 1e6), 0, 0, mask, 0)


def arm(seconds, state, method):
    return HEAD + bytes([23]) + struct.pack("<QBIBB", int(seconds * 1e6), state, 0, 0, method)


def bat(seconds, volts):
    return HEAD + bytes([24]) + struct.pack("<QBfffffhfBBB", int(seconds * 1e6), 0, volts, volts, 10.0, 0.0, 0.0, 0, 0.0, 90, 1, 1)


def pos(seconds, east_m=0.0, north_m=0.0, height_m=0.0):
    lat = HOME[0] + int(round(north_m / 0.0111319))
    lng = HOME[1] + int(round(east_m / (0.0111319 * 0.81865)))  # cos(35.05 degrees)
    return HEAD + bytes([8]) + struct.pack("<Qiifff", int(seconds * 1e6), lat, lng, 700.0 + height_m, height_m, height_m)


def clean_flight(extra=(), track=None):
    """An autonomous flight's log: armed at 60 s in GUIDED by the companion, a climb, RTL asked by MAVLink at 150 s, landed and disarmed at 200 s."""
    events = [text(60.0, "Arming motors"), kit_test.mode(60.0, 4, 2), mavc(60.0, 400, p1=1.0), mavc(65.0, 22), kit_test.mode(150.0, 6, 2), arm(200.0, 0, 13), text(200.0, "Disarming motors")]
    events += [rcin(60.0 + step) for step in range(0, 141)] + [bat(60.0 + step * 10, 24.5 - step * 0.05) for step in range(0, 15)]
    track = track if track is not None else [(60.0, 0, 0, 0), (70.0, 0, 0, 15), (150.0, 40, 0, 15), (190.0, 0, 0, 15), (200.0, 0, 0, 0)]
    points = []
    for (t_a, e_a, n_a, u_a), (t_b, e_b, n_b, u_b) in zip(track, track[1:]):
        steps = max(1, int((t_b - t_a) * 2))
        points += [pos(t_a + (t_b - t_a) * i / steps, e_a + (e_b - e_a) * i / steps, n_a + (n_b - n_a) * i / steps, u_a + (u_b - u_a) * i / steps) for i in range(steps)]
    points.append(pos(*track[-1]))
    return events + points + list(extra)


def write_log(path, events):
    def stamp(part):
        return struct.unpack_from("<Q", part, 3)[0]

    head = FORMATS + [logs.msg("ArduCopter V4.6.3 (92b0cd78)"), logs.parm("FLTMODE_CH", 5.0), logs.parm("FENCE_ALT_MAX", 120.0)]
    path.write_bytes(b"".join(head + sorted(events, key=stamp)))  # a log is written in time order
    return path


def make_bundle(tmp_path, events, mission="delivery", sequence_text=None, route=None):
    src = tmp_path / "flight"
    (src / "bag").mkdir(parents=True)
    (src / "bag" / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    listed = [{"topic_metadata": {"name": "/mavros/state", "type": "mavros_msgs/msg/State", "serialization_format": "cdr"}, "message_count": 10}]
    (src / "bag" / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"storage_identifier": "mcap", "topics_with_message_count": listed}}))
    write_log(src / "00000012.BIN", events)
    (src / "sequences").mkdir()
    sequence = src / "sequences" / "sequence.yaml"
    sequence.write_text(sequence_text or "mission_sequence: {features: {}, missions: []}\n")
    if route is not None:
        (src / "waypoints").mkdir()
        (src / "waypoints" / "competition_waypoints.yaml").write_text(route)
    (src / "params.yaml").write_text("{}\n")
    (src / "comp_2026.yaml").write_text("description: x\nenable: []\n")
    (src / "baseline.param").write_text("FENCE_ENABLE,1\n")
    (src / "preflight.jsonl").write_text(json.dumps({"verdict": "GO", "failed": []}) + "\n")
    name = bundle_attempt.attempt_name("2027-05-21", "2", mission)
    bundle_attempt.bundle(name, tmp_path / "attempts", mission, bag=src / "bag", dataflash=src / "00000012.BIN", sequence=sequence, params=src / "params.yaml", profile=src / "comp_2026.yaml",
                          baseline=src / "baseline.param", preflight_log=src / "preflight.jsonl", git={"hash": "f7ef276", "describe": "comp-2026-108-gf7ef276", "dirty": False})  # fmt: skip
    return tmp_path / "attempts" / name


def audited(events, **kwargs):
    import tempfile

    with tempfile.TemporaryDirectory() as folder:
        return pf.audit(pf.read_flight(write_log(Path(folder) / "x.BIN", events)), **kwargs)


def test_a_flight_nobody_touched_is_clean_and_its_own_mode_changes_are_listed_as_not_attributed(tmp_path, capsys):
    folder = make_bundle(tmp_path, clean_flight())
    assert pf.main([str(folder)]) == 0
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "NO HUMAN INPUT FOUND"
    report = (folder / "report.md").read_text()
    for line in (
        "# Post-flight report: 2027-05-21_2_delivery",
        "- Mission: delivery, slot 2, 2027-05-21.",
        "- Code: comp-2026-108-gf7ef276.",
        "- Flag profile: comp_2026 (no flags: the 2026 code).",
        "- Preflight: GO.",
        "Armed for 140 s; greatest height 15.0 m; 80 m flown; lowest battery voltage 23.8 V.",
        "-     0.0 s  armed",
        "-     0.0 s  mode GUIDED (by a MAVLink command)",
        "-    90.0 s  mode RTL (by a MAVLink command)",
        "-   140.0 s  disarmed (method 13)",
        "**NO HUMAN INPUT FOUND**, with 1 mode change(s) over MAVLink to confirm (below).",
        "(2 commands, all its own)",
        "Not visible in this log: who sent a mode change or a position target that came as a plain MAVLink message.",
        "-    90.0 s  RTL by a MAVLink command",  # not attributed: the log has no sender for a mode change
    ):
        assert line in report, line
    data = json.loads((folder / "report.json").read_text())
    assert data["audit"]["clean"] is True and data["audit"]["window"] == [60.0, 200.0] and [item["what"] for item in data["audit"]["unattributed"]] == ["RTL by a MAVLink command"]
    assert bundle_attempt.verify(folder) == []  # the report written into the bundle leaves it as its manifest says


@pytest.mark.parametrize(
    "extra, kind, word",
    [
        ([kit_test.mode(120.0, 5, 1)], "mode from the transmitter", "LOITER was switched to from the transmitter"),
        ([mavc(110.0, 176, system=255, component=190)], "command not from the companion", "DO_SET_MODE from system 255, component 190"),
        ([mavc(110.0, 20, system=255, component=190)], "command not from the companion", "RETURN_TO_LAUNCH from system 255"),
        ([rci2(100.0 + i * 0.1, 0x0F) for i in range(30)], "RC override", "an RC override was in force (mask 0xf) for 2.9 s"),
        ([rcin(100.0 + i * 0.1, c1=1650) for i in range(10)], "stick moved", "the roll stick (channel 1) moved from 1500 to 1650 us"),
        ([rcin(100.0 + i * 0.1, c3=1400) for i in range(10)], "stick moved", "the throttle stick (channel 3) moved from 1000 to 1400 us"),
        ([rcin(100.0 + i * 0.1, c5=1300) for i in range(10)], "stick moved", "the mode switch (channel 5) moved from 1800 to 1300 us"),
        ([arm(130.0, 0, 0)], "arming from the transmitter", "disarmed by the rudder stick"),
        ([arm(60.0, 1, 2)], "arming from the transmitter", "armed by a switch"),
        ([kit_test.mode(120.0, 5, 2)], "mode the stack never asks for", "LOITER by a MAVLink command; the stack asks only for GUIDED, RTL"),
        ([mavc(110.0, 192, system=255, component=190)], "command not from the companion", "DO_REPOSITION from system 255"),  # a ground station's "fly to here"
    ],
)
def test_each_kind_of_human_input_inside_the_window_is_found(extra, kind, word):
    checked = audited(clean_flight(extra), expected_modes={"GUIDED", "RTL"})
    assert checked["clean"] is False and len(checked["findings"]) == 1
    assert checked["findings"][0]["kind"] == kind and word in checked["findings"][0]["what"]
    assert 60.0 <= checked["findings"][0]["t"] <= 200.0


def test_what_is_not_a_persons_input_is_not_found():
    assert audited(clean_flight([mavc(110.0, 511, system=255, component=190)]))["clean"]  # a ground station asking for a message: housekeeping
    assert audited(clean_flight([rcin(100.0, c1=1650)]))["clean"]  # one odd sample is not a hand
    assert audited(clean_flight([rcin(100.0 + i * 0.1, c1=1530) for i in range(10)]))["clean"]  # inside the deadband
    assert audited(clean_flight([mavc(110.0, 183)]))["clean"]  # the companion's own servo command
    assert audited(clean_flight([kit_test.mode(150.0, 6, 10)]))["clean"]  # the fence's own mode change: not a person's
    # Before arming and after disarming is outside the window.
    assert audited(clean_flight([mavc(30.0, 176, system=255, component=190), kit_test.mode(230.0, 0, 1), rci2(20.0, 1)]))["clean"]
    # Another companion's ids, when said.
    other = audited(clean_flight(), companion={(1, 240)})
    assert not other["clean"] and len(other["findings"]) == 2 and other["commands_from_companion"] == 0


def test_the_window_is_the_armed_period_the_vehicle_flew_in_and_the_others_are_counted():
    """The spec audit: an arm check on the ground and then the flight are one log; the flight is what is audited."""
    check = [arm(20.0, 1, 1), kit_test.mode(25.0, 0, 1), arm(30.0, 0, 1)]  # armed and disarmed on the ground, a mode switched meanwhile
    checked = audited(clean_flight(check))
    assert checked["armed_periods"] == 2 and checked["window"] == [60.0, 200.0] and checked["clean"]  # the switch at 25 s is outside the flight
    import tempfile

    with tempfile.TemporaryDirectory() as folder:
        log = pf.read_flight(write_log(Path(folder) / "x.BIN", clean_flight(check)))
    assert pf.armed_periods(log) == [(20.0, 30.0), (60.0, 200.0)]


def test_the_modes_a_sequence_asks_for_are_read_from_the_bundle_and_parameter_writes_are_listed(tmp_path):
    sequence = "mission_sequence:\n  on_failure: abort_and_rtl\n  features: {}\n  missions:\n    - type: rtl\n      config: {rtl_mode: \"SMART_RTL\"}\n    - type: package_recovery\n      config: {abort_rtl_mode: \"LAND\"}\n"
    folder = make_bundle(tmp_path, clean_flight([kit_test.mode(170.0, 9, 2), parm_at(100.0, "WPNAV_SPEED", 300.0), parm_at(110.0, "STAT_FLTTIME", 77.0)]) + [logs.parm("WPNAV_SPEED", 500.0), logs.parm("STAT_FLTTIME", 5.0)],
                         sequence_text=sequence)  # fmt: skip
    assert pf.expected_modes_of(folder) == {"GUIDED", "RTL", "LAND", "AUTO", "SMART_RTL"}  # the stack's own, and the one this config names
    assert pf.main([str(folder)]) == 0  # LAND is the recovery's own abort: not attributed, not a finding
    data = json.loads((folder / "report.json").read_text())
    assert [item["what"] for item in data["audit"]["unattributed"]] == ["RTL by a MAVLink command", "LAND by a MAVLink command"]
    assert [item["what"] for item in data["audit"]["parameters_changed"]] == ["WPNAV_SPEED set to 300"]  # the flight controller's own counters are not listed
    assert "Parameters written during the window (the log does not say by whom):" in (folder / "report.md").read_text()


def test_the_stacks_own_modes_are_the_ones_its_code_asks_for():
    """The spec audit: the recovery's LAND and the time trial's AUTO are set in code, not in any YAML; an untouched flight must not read as human input."""
    import re

    asked = set()
    for folder in ("src/drone_mission_core/drone_mission_core", "src/drone_mission_demo/drone_mission_demo", "src/drone_utils/drone_utils"):
        for path in (REPO / folder).rglob("*.py"):
            text = path.read_text()
            asked |= set(re.findall(r'request_mode_change\(\s*"([A-Z_]+)"', text))
            asked |= set(re.findall(r'ABORT_RTL, ABORT_LAND = "([A-Z_]+)", "([A-Z_]+)"', text)[0]) if "ABORT_RTL, ABORT_LAND =" in text else set()
    assert {"GUIDED", "AUTO", "RTL", "LAND"} <= asked | {"RTL"} and asked <= set(pf.STACK_MODES)
    # Every committed sequence and params file adds nothing the audit would then miss: their mode names are known flight modes.
    for path in list((REPO / "src/drone_mission_demo/config/sequences").glob("*.yaml")) + list((REPO / "src/drone_mission_demo/config/params").glob("*.yaml")):
        for name in re.findall(r'_mode:\s*"?([A-Z_]+)"?', path.read_text()):
            assert name in pf.MODE_NAMES.values(), (path.name, name)


def test_an_untouched_recovery_or_time_trial_is_clean_and_a_ground_stations_mode_is_found_through_the_command(tmp_path, capsys):
    """Through main: LAND (the recovery's abort) and AUTO (the time trial) over MAVLink are the stack's own; LOITER is nobody's but a person's."""
    landed = make_bundle(tmp_path / "land", clean_flight([kit_test.mode(140.0, 9, 2)]), mission="recovery")
    assert pf.main([str(landed)]) == 0
    auto = make_bundle(tmp_path / "auto", clean_flight([kit_test.mode(80.0, 3, 2), kit_test.mode(140.0, 4, 2)]), mission="time_trial")
    assert pf.main([str(auto)]) == 0
    loiter = make_bundle(tmp_path / "loiter", clean_flight([kit_test.mode(120.0, 5, 2)]))
    assert pf.main([str(loiter)]) == 1
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "HUMAN INPUT: 1 finding(s)" and "mode the stack never asks for: LOITER by a MAVLink command" in out


def test_a_flight_with_human_input_says_so_and_exits_1(tmp_path, capsys):
    folder = make_bundle(tmp_path, clean_flight([kit_test.mode(120.0, 5, 1), mavc(125.0, 400, system=255, component=190)]))
    assert pf.main([str(folder)]) == 1
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "HUMAN INPUT: 2 finding(s)"
    report = (folder / "report.md").read_text()
    assert "**HUMAN INPUT: 2 finding(s).** The autonomy bonus cannot be claimed" in report
    assert "-    60.0 s  mode from the transmitter: LOITER was switched to from the transmitter" in report and "-    65.0 s  command not from the companion: ARM_DISARM from system 255, component 190" in report


ROUTE = "waypoints:\n  - {name: wp1, latitude: 35.0500000, longitude: -118.1495609, altitude_agl_ft: 49.2}\n  - {name: wp2, latitude: 35.0503593, longitude: -118.1495609, altitude_agl_ft: 49.2}\n"
ROUTE_SEQUENCE = "mission_sequence:\n  features: {}\n  missions:\n    - type: gps_waypoint\n      config:\n        waypoint_file: ../waypoints/competition_waypoints.yaml\n"


def test_a_waypoint_flight_is_scored_by_the_sims_judge_on_the_logs_own_track(tmp_path, capsys):
    track = [(60.0, 0, 0, 0), (70.0, 0, 0, 15), (90.0, 40, 0, 15), (110.0, 40, 40, 15), (150.0, 0, 0, 15), (200.0, 0, 0, 0)]  # over wp1 (40 m east) and wp2 (40 m further north)
    folder = make_bundle(tmp_path / "hit", clean_flight(track=track), mission="waypoint_navigation", sequence_text=ROUTE_SEQUENCE, route=ROUTE)
    assert pf.main([str(folder)]) == 0
    data = json.loads((folder / "report.json").read_text())
    assert data["estimate"]["estimated"] is True and data["estimate"]["details"]["hit"] == 2 and data["estimate"]["details"]["of"] == 2 and data["estimate"]["points"] > 0
    assert "the sim's waypoint_navigation judge on the log's own track" in (folder / "report.md").read_text()
    missed = make_bundle(tmp_path / "missed", clean_flight(track=[(60.0, 0, 0, 0), (70.0, 0, 0, 15), (90.0, 40, 0, 15), (150.0, 0, 0, 15), (200.0, 0, 0, 0)]), mission="waypoint_navigation",
                         sequence_text=ROUTE_SEQUENCE, route=ROUTE)  # fmt: skip
    pf.main([str(missed)])
    assert json.loads((missed / "report.json").read_text())["estimate"]["details"]["hit"] == 1
    # The time trial through its own judge: both passed in order, a time, and with no reference times the slowest team's 8 points.
    trial = make_bundle(tmp_path / "trial", clean_flight(track=track), mission="time_trial", sequence_text=ROUTE_SEQUENCE, route=ROUTE)
    assert pf.main([str(trial)]) == 0
    estimate = json.loads((trial / "report.json").read_text())["estimate"]
    assert estimate["details"]["hit"] == 2 and estimate["details"]["elapsed_s"] == pytest.approx(20.0, abs=1.5) and estimate["points"] == 8.0
    capsys.readouterr()
    no_route = make_bundle(tmp_path / "no_route", clean_flight(track=track), mission="waypoint_navigation")
    pf.main([str(no_route)])
    assert "Not estimated: the bundle has no route" in (no_route / "report.md").read_text()


TARGET_SEQUENCE = "mission_sequence:\n  features: {}\n  missions:\n    - type: package_delivery\n      config:\n        target_latitude: 35.0500000\n        target_longitude: -118.1495609\n"
DELIVERY_TRACK = [(60.0, 0, 0, 0), (70.0, 0, 0, 15), (100.0, 39, 0, 15), (120.0, 39, 0, 1), (150.0, 39, 0, 15), (190.0, 0, 0, 15), (200.0, 0, 0, 0)]  # down to 1 m, 1 m short of the target 40 m east


def test_a_delivery_is_scored_by_the_delivery_judge_from_where_the_vehicle_let_go_and_says_so(tmp_path):
    from judge.score import default_settings

    release = [mavc(120.0, 211, p1=1.0, p2=0.0), rfnd(120.0, 0.97)]
    folder = make_bundle(tmp_path / "a", clean_flight(release, track=DELIVERY_TRACK), mission="delivery", sequence_text=TARGET_SEQUENCE)
    assert pf.main([str(folder)]) == 0
    estimate = json.loads((folder / "report.json").read_text())["estimate"]
    settings = default_settings("delivery")
    base = settings["max_points"] - estimate["details"]["distance_m"] * settings["points_per_metre"]
    assert estimate["details"]["scored_from"] == "vehicle at release" and estimate["details"]["distance_m"] == pytest.approx(1.0, abs=0.2) and estimate["details"]["release_height_m"] == pytest.approx(1.0, abs=0.3)
    assert estimate["base_points"] == pytest.approx(base, abs=0.02) and estimate["autonomy_bonus"] == pytest.approx(0.25 * estimate["base_points"]) and estimate["points"] == pytest.approx(1.25 * estimate["base_points"])  # the details' distance is rounded
    assert estimate["release"] == {"t": 120.0, "command": "DO_GRIPPER", "rangefinder_m": pytest.approx(0.97)}
    report = (folder / "report.md").read_text()
    assert "the log does not say where the payload came down" in report and "- released by DO_GRIPPER at 60.0 s, 1.00 m above home by the flight controller's position; 0.97 m by the rangefinder (6 ft is 1.83 m)." in report
    # A person's input in the window takes the autonomy bonus away, as the judge does in the sim.
    touched = make_bundle(tmp_path / "touched", clean_flight(release + [kit_test.mode(130.0, 5, 1)], track=DELIVERY_TRACK), mission="delivery", sequence_text=TARGET_SEQUENCE)
    assert pf.main([str(touched)]) == 1
    estimate = json.loads((touched / "report.json").read_text())["estimate"]
    assert estimate["autonomy_bonus"] == 0.0 and estimate["points"] == pytest.approx(estimate["base_points"]) and estimate["base_points"] > 0
    # Released from 15 m: the delivery judge's own rule (above 6 ft scores nothing).
    high = make_bundle(tmp_path / "high", clean_flight([mavc(100.0, 211, p1=1.0, p2=0.0)], track=DELIVERY_TRACK), mission="delivery", sequence_text=TARGET_SEQUENCE)
    pf.main([str(high)])
    estimate = json.loads((high / "report.json").read_text())["estimate"]
    assert estimate["points"] == 0.0 and estimate["details"]["released_high"] is True and any("above" in note for note in estimate["notes"])


def test_the_release_is_the_first_command_that_lets_go_and_not_a_grab_or_the_servos_return(tmp_path):
    """The spec audit: the stack also sends DO_GRIPPER to grab and DO_SPRAYER off after a drop; neither is a release."""
    commands = [mavc(80.0, 211, p1=1.0, p2=1.0), mavc(110.0, 216, p1=1.0), mavc(112.0, 216, p1=0.0), mavc(140.0, 211, p1=1.0, p2=1.0)]  # a grab, the drop, the servo's return, a grab
    track = [(60.0, 0, 0, 0), (70.0, 0, 0, 15), (100.0, 39, 0, 15), (110.0, 39, 0, 3), (150.0, 39, 0, 15), (190.0, 0, 0, 15), (200.0, 0, 0, 0)]
    folder = make_bundle(tmp_path / "drop", clean_flight(commands + [rfnd(110.0, 2.4)], track=track), mission="drop", sequence_text=TARGET_SEQUENCE)
    assert pf.main([str(folder)]) == 0
    estimate = json.loads((folder / "report.json").read_text())["estimate"]
    assert estimate["release"]["t"] == 110.0 and estimate["release"]["command"] == "DO_SPRAYER" and estimate["release"]["rangefinder_m"] == pytest.approx(2.4)
    assert estimate["details"]["released_low"] is False and estimate["details"]["release_height_m"] == pytest.approx(3.0, abs=0.3) and estimate["points"] > 0
    # Only grabs and the servo's return: nothing was let go.
    none = make_bundle(tmp_path / "none", clean_flight([mavc(80.0, 211, p1=1.0, p2=1.0), mavc(112.0, 216, p1=0.0)], track=track), mission="drop", sequence_text=TARGET_SEQUENCE)
    pf.main([str(none)])
    assert "Not estimated: the log has no release command" in (none / "report.md").read_text()
    # A drop released under 6 ft loses the judge's penalty; a rangefinder reading that is not good is not quoted.
    low_track = [(60.0, 0, 0, 0), (70.0, 0, 0, 15), (100.0, 39, 0, 15), (110.0, 39, 0, 1.2), (150.0, 39, 0, 15), (190.0, 0, 0, 15), (200.0, 0, 0, 0)]
    low = make_bundle(tmp_path / "low", clean_flight([mavc(110.0, 216, p1=1.0), rfnd(110.0, 1.2, status=1)], track=low_track), mission="drop", sequence_text=TARGET_SEQUENCE)
    pf.main([str(low)])
    estimate = json.loads((low / "report.json").read_text())["estimate"]
    assert estimate["details"]["released_low"] is True and estimate["release"]["rangefinder_m"] is None and "no good rangefinder reading at that moment" in (low / "report.md").read_text()
    survey = make_bundle(tmp_path / "c", clean_flight(), mission="localisation")
    pf.main([str(survey)])
    assert "Not estimated: the localisation needs what the log does not hold" in (survey / "report.md").read_text()


def test_the_fence_demonstration_is_scored_by_the_geofence_judge_from_the_log_alone(tmp_path):
    """The polygon is the one the log holds: the placeholder site's, around a home at its origin."""
    events = [text(60.0, "Arming motors"), kit_test.mode(60.0, 4, 2), kit_test.fnce(), kit_test.mode(150.0, 6, 2), arm(200.0, 0, 13)]
    home, east_edge = (341750000, -1184825000), -1184808713

    def at(seconds, lng, height):
        return HEAD + bytes([8]) + struct.pack("<Qiifff", int(seconds * 1e6), home[0], lng, 215.0 + height, height, height)

    hold_lng = east_edge - 217  # 2 m inside the east edge
    events += [at(60.0, home[1], 0.0), at(70.0, home[1], 10.0)] + [at(100.0 + i, hold_lng, 10.0) for i in range(40)] + [at(190.0, home[1], 10.0), at(200.0, home[1], 0.0)]
    folder = make_bundle(tmp_path, events, mission="geofence")
    assert pf.main([str(folder)]) == 0
    details = json.loads((folder / "report.json").read_text())["estimate"]["details"]
    assert details["passed"] is True and details["held"]["closest_m"] == pytest.approx(2.0, abs=0.1) and details["held"]["held_s"] >= 39.0


def test_not_a_bundle_or_no_log_is_exit_2(tmp_path, capsys):
    assert pf.main([str(tmp_path / "nowhere")]) == 2 and "not an attempt's bundle" in capsys.readouterr().err
    folder = make_bundle(tmp_path, clean_flight())
    (folder / "dataflash" / "00000012.BIN").unlink()
    assert pf.main([str(folder)]) == 2 and "no dataflash log" in capsys.readouterr().err
    assert pf.main([str(folder), "--companion", "1"]) == 2
