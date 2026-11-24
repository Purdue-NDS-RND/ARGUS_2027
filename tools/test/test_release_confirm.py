"""CEN-14: release confirmation (tools/release_confirm.py, and its place in tools/post_flight.py), on logs built here. No ROS."""

import json
import struct
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import post_flight as pf  # noqa: E402
import release_confirm as rc  # noqa: E402
import test_dataflash_params as logs  # noqa: E402
import test_post_flight as flights  # noqa: E402  (the log builders)

HEAD = b"\xa3\x95"
RCOU_FMT = logs.fmt(26, "RCOU", "QHHHHHHHHHHHHHH", "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14")
CTUN_FMT = logs.fmt(27, "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt")  # ArduCopter 4.6's own
SITL = {"SERVO7_FUNCTION": 28.0, "SERVO8_FUNCTION": 22.0}  # the sim's outputs, and the 2027 baseline's gripper


def rcou(seconds, c7=1500, c8=0):
    return HEAD + bytes([26]) + struct.pack("<Q14H", int(seconds * 1e6), 1500, 1500, 1500, 1500, 0, 0, c7, c8, 0, 0, 0, 0, 0, 0)


def ctun(seconds, throttle, asked_mps=0.0, climb_mps=0.0):
    return HEAD + bytes([27]) + struct.pack("<Qffffffifffhh", int(seconds * 1e6), 0.0, 0.0, throttle, 0.39, 0.0, 0.0, 0, 0.0, 0.0, 0.0, int(round(asked_mps * 100)), int(round(climb_mps * 100)))


def series(t_release, gripper=False, output=True, payload_kg=0.5, on_ground=False, ack=0):
    """A flight's CTUN and RCOU around a release at t_release, as the sim's real logs show one (docs/results, CEN-14).

    In the air (the drop): a hover at 0.375; the output moves 0.13 s after the command; a 0.5 kg payload gone makes the
    vehicle rise and the throttle dip to 0.333 within half a second; a second after the command the next mission's
    climb is asked for and the throttle is the climb's. On the ground (the delivery): a descent at 0.369, the motors
    idle from 3 s before the command to 4 s after, and a climb away at 0.344 without the payload.
    """
    lighter = (1.5 / (1.5 + payload_kg)) ** 0.5  # throttle goes with the square root of the weight
    rows = []
    for step in range(-200, 200):
        t = t_release + step * 0.1
        moved = output and step >= 2
        rows.append(rcou(t, c7=1100 if (gripper and moved) else 1500, c8=0 if (gripper or not moved) else 1900))
        if on_ground:
            if step < -30:
                rows.append(ctun(t, 0.369 + 0.001 * (step % 2), -0.25, -0.25))
            elif step < 45:
                rows.append(ctun(t, 0.0))
            else:
                rows.append(ctun(t, 0.369 * (lighter + 0.065) + 0.001 * (step % 2), 0.6, 0.6))  # a climb takes a little more than a hover
        elif step <= 4:
            rows.append(ctun(t, 0.375))
        elif step <= 8:
            rows.append(ctun(t, 0.375 * lighter if payload_kg else 0.375, 0.0, 0.12 if payload_kg else 0.0))
        else:
            asked = min(2.5, (step - 8) * 0.1)
            rows.append(ctun(t, 0.378 if payload_kg else 0.43, asked, asked))
    return rows


def write_log(path, events, params=SITL):
    """A log in time order: the formats (the post-flight tests' and the two here), the firmware line, the parameters, the events."""
    head = flights.FORMATS + [RCOU_FMT, CTUN_FMT, logs.msg("ArduCopter V4.6.3 (92b0cd78)")] + [logs.parm(name, value) for name, value in params.items()]
    path.write_bytes(b"".join(head + sorted(events, key=lambda part: struct.unpack_from("<Q", part, 3)[0])))
    return path


def log_with(tmp_path, extra, params=SITL, release=None):
    release = release if release is not None else flights.mavc(120.0, 216, p1=1.0)
    return pf.read_flight(write_log(tmp_path / "flight.BIN", flights.clean_flight(extra=[release] + list(extra)), params))


def test_a_drop_in_the_air_is_confirmed_by_the_step_the_real_sim_log_shows(tmp_path):
    found = pf.release_confirmation(log_with(tmp_path, series(120.0)))
    assert found["verdict"] == "CONFIRMED" and found["acknowledged"] and found["output"]["moved"] and found["output"]["channel"] == 8 and "step" in found["left_by"]
    step = found["evidence"]["step"]
    assert step["throttle_before"] == pytest.approx(0.375, abs=1e-3) and step["throttle_drop"] == pytest.approx(1 - (1.5 / 2.0) ** 0.5, abs=0.005) and step["rise_mps"] == pytest.approx(0.12)
    assert found["lines"][0] == "command: DO_SPRAYER (the drop servo), accepted by the flight controller" and "output 8 went from 0 to 1900 us" in found["lines"][1]
    assert found["missing"] == []


def test_a_release_that_lets_nothing_go_is_not_confirmed(tmp_path):
    """The latch that did not open: acknowledged, the output moved, and the vehicle is as heavy as before."""
    found = pf.release_confirmation(log_with(tmp_path, series(120.0, payload_kg=0.0)))
    assert found["verdict"] == "NOT CONFIRMED" and found["acknowledged"] and found["output"]["moved"] is True and found["left_by"] == []
    assert found["missing"] == ["nothing shows the payload leaving"]
    assert found["evidence"]["step"]["seen"] is False and found["evidence"]["flight"]["seen"] is not True


def test_a_delivery_released_on_the_ground_is_confirmed_by_the_lighter_climb_away(tmp_path):
    release = flights.mavc(120.0, 211, p1=1.0, p2=0.0)
    found = pf.release_confirmation(log_with(tmp_path, series(120.0, gripper=True, on_ground=True), release=release))
    assert found["verdict"] == "CONFIRMED" and found["left_by"] == ["flight"] and found["output"]["channel"] == 7 and found["output"]["after_us"] == 1100
    assert found["evidence"]["step"]["seen"] is None and "not flying at the command" in found["evidence"]["step"]["why"]
    assert 0.04 <= found["evidence"]["flight"]["throttle_drop"] <= 0.09
    # The same flight with nothing let go: the climb away is no lighter.
    empty = pf.release_confirmation(log_with(tmp_path, series(120.0, gripper=True, on_ground=True, payload_kg=0.0), release=release))
    assert empty["verdict"] == "NOT CONFIRMED" and empty["evidence"]["flight"]["seen"] is False and "a rise of" in empty["evidence"]["flight"]["why"]


def test_an_output_that_does_not_move_or_is_not_assigned_is_not_confirmed_whatever_else_is_seen(tmp_path):
    stuck = pf.release_confirmation(log_with(tmp_path, series(120.0, output=False)))
    assert stuck["verdict"] == "NOT CONFIRMED" and stuck["output"]["moved"] is False and "output 8 stayed at 0 us" in stuck["output"]["why"] and "the output is not seen to move" in stuck["missing"]
    # The June 2026 aircraft: no output had the gripper's or the sprayer's function.
    unassigned = pf.release_confirmation(log_with(tmp_path, series(120.0), params={"SERVO9_FUNCTION": 0.0}))
    assert unassigned["verdict"] == "NOT CONFIRMED" and unassigned["output"]["channel"] is None and "no servo output has the drop servo (sprayer)'s function" in unassigned["output"]["why"]
    # A log with no servo outputs in it cannot say.
    silent = pf.release_confirmation(log_with(tmp_path, [row for row in series(120.0) if row[2] != 26]))
    assert silent["verdict"] == "NOT CONFIRMED" and silent["output"]["moved"] is None


def test_a_command_the_flight_controller_did_not_accept_is_not_confirmed(tmp_path):
    denied = HEAD + bytes([20]) + struct.pack("<QBBBBBHffffiifBB", int(120.0 * 1e6), 1, 1, 1, 191, 3, 216, 1.0, 0.0, 0.0, 0.0, 0, 0, 0.0, 4, 1)
    found = pf.release_confirmation(log_with(tmp_path, series(120.0), release=denied))
    assert found["verdict"] == "NOT CONFIRMED" and not found["acknowledged"] and found["missing"][0] == "the command was failed" and "failed by the flight controller" in found["lines"][0]


def test_a_log_whose_commands_carry_no_result_acknowledges_nothing(tmp_path):
    """The spec audit: a MAVC row without its result column was read as accepted."""
    old_format = logs.fmt(20, "MAVC", "QBBBBBHffffiif", "TimeUS,TS,TC,SS,SC,Fr,Cmd,P1,P2,P3,P4,X,Y,Z")
    release = HEAD + bytes([20]) + struct.pack("<QBBBBBHffffiif", int(120.0 * 1e6), 1, 1, 1, 191, 3, 216, 1.0, 0.0, 0.0, 0.0, 0, 0, 0.0)
    formats = [old_format if part == flights.MAVC_FMT else part for part in flights.FORMATS] + [RCOU_FMT, CTUN_FMT, logs.msg("ArduCopter V4.6.3 (92b0cd78)")] + [logs.parm(name, value) for name, value in SITL.items()]
    events = [part for part in flights.clean_flight(extra=series(120.0)) if part[2] != 20] + [release]
    path = tmp_path / "old.BIN"
    path.write_bytes(b"".join(formats + sorted(events, key=lambda part: struct.unpack_from("<Q", part, 3)[0])))
    found = pf.release_confirmation(pf.read_flight(path))
    assert found["verdict"] == "NOT CONFIRMED" and found["acknowledged"] is False and found["result"] == -1 and found["missing"][0] == "the command was answered with result -1"
    assert "step" in found["left_by"]  # the payload is seen to leave all the same: what is missing is the acknowledgement


def test_the_rangefinder_seeing_the_payload_fall_away_is_evidence_and_a_ground_that_only_gets_farther_is_not():
    ground = [(t * 0.1, 5.0, 4) for t in range(1150, 1202)]
    passing = [(120.2 + 0.1 * i, metres, 4) for i, metres in enumerate([0.4, 0.9, 1.7, 2.8, 4.2])] + [(120.7 + 0.1 * i, 5.0, 4) for i in range(1, 15)]
    seen = rc.fall(120.13, ground + passing)
    assert seen["seen"] is True and seen["short_readings"] == 5 and "something fell through the beam" in seen["why"]
    climbing = [(120.2 + 0.1 * i, 5.0 + 0.2 * i, 4) for i in range(20)]
    assert rc.fall(120.13, ground + climbing)["seen"] is False  # the real drop log: the ground gets farther as the vehicle climbs
    assert rc.fall(120.13, ground + [(120.3, 2.0, 4)] + [(120.4 + 0.1 * i, 5.0, 4) for i in range(10)])["seen"] is False  # one short reading is a glitch
    coming = [(120.2 + 0.1 * i, metres, 4) for i, metres in enumerate([4.0, 3.0, 2.0, 1.0])] + [(120.7, 5.0, 4)]
    assert rc.fall(120.13, ground + coming)["seen"] is False  # getting nearer is not falling away
    assert rc.fall(120.13, [(t, m, 1) for t, m, _s in ground + passing])["seen"] is None  # no good readings: cannot say
    # With the output and the acknowledgement, the fall alone confirms.
    command = {"t": 120.0, "command": 211, "p1": 1.0, "p2": 0.0, "result": 0}
    outputs = [(119.9, (0,) * 6 + (1500,) + (0,) * 7), (120.13, (0,) * 6 + (1100,) + (0,) * 7)]
    found = rc.confirm(command, outputs, [], ground + passing, SITL)
    assert found["verdict"] == "CONFIRMED" and found["left_by"] == ["fall"]


def test_an_output_logged_with_the_commands_own_time_stamp_is_seen_to_move():
    """The sim's delivery flown as an attempt: the flight controller logged the gripper's output at 1100 with the very time stamp of the command, and the first rule took that sample for "before"."""
    command = {"t": 135.48, "command": 211, "p1": 1.0, "p2": 0.0, "result": 0}
    held = (0,) * 6 + (1500,) + (0,) * 7
    open_ = (0,) * 6 + (1100,) + (0,) * 7
    outputs = [(135.28, held), (135.38, held), (135.48, open_), (135.58, open_), (135.68, open_)]
    found = rc.output_moved(command, outputs, SITL)
    assert found["moved"] is True and found["before_us"] == 1500 and found["after_us"] == 1100 and found["t"] == 135.48 and "0.00 s after the command" in found["why"]
    # An output that was already there well before the command did not move for it.
    assert rc.output_moved(command, [(135.0, open_), (135.3, open_), (135.48, open_), (135.9, open_)], SITL)["moved"] is False


def test_the_servo_a_command_moves_is_the_one_the_parameters_give_its_function():
    assert rc.release_channel({"command": 211}, {"SERVO7_FUNCTION": 28.0}) == 7 and rc.release_channel({"command": 216}, {"SERVO8_FUNCTION": 22.0, "SERVO7_FUNCTION": 28.0}) == 8
    assert rc.release_channel({"command": 183, "p1": 9.0}, {}) == 9 and rc.release_channel({"command": 211}, {"SERVO8_FUNCTION": 22.0}) is None
    assert rc.confirm(None, [], [], [], {}) == {"verdict": "NO RELEASE", "lines": ["no release command in the flight"]}


def test_the_result_goes_into_the_attempts_bundle_and_the_report(tmp_path, capsys):
    bundle = tmp_path / "2027-06-01_1_drop"
    (bundle / "dataflash").mkdir(parents=True)
    write_log(bundle / "dataflash" / "00000007.BIN", flights.clean_flight(extra=[flights.mavc(120.0, 216, p1=1.0)] + series(120.0)))
    (bundle / "manifest.json").write_text(json.dumps({"attempt": bundle.name, "mission": "drop", "slot": "1", "date": "2027-06-01", "complete": True}))
    assert pf.main([str(bundle)]) == 0
    out = capsys.readouterr().out
    assert "## Release confirmation" in out and "**RELEASE CONFIRMED**. At 60.0 s, from the flight controller's log:" in out and "- step: with no climb asked for, the throttle fell" in out
    assert out.strip().splitlines()[-2:] == ["RELEASE CONFIRMED", "NO HUMAN INPUT FOUND"]
    saved = json.loads((bundle / "release_confirmation.json").read_text())
    assert saved["verdict"] == "CONFIRMED" and saved == json.loads((bundle / "report.json").read_text())["audit"]["release_confirmation"]
    assert "RELEASE CONFIRMED" in (bundle / "report.md").read_text()
    # A flight that released nothing says so, and one whose payload stayed says NOT CONFIRMED with why.
    write_log(bundle / "dataflash" / "00000007.BIN", flights.clean_flight())
    pf.main([str(bundle)])
    out = capsys.readouterr().out
    assert "No release command in the flight: nothing to confirm." in out and "no release in the flight" in out
    write_log(bundle / "dataflash" / "00000007.BIN", flights.clean_flight(extra=[flights.mavc(120.0, 216, p1=1.0)] + series(120.0, payload_kg=0.0)))
    pf.main([str(bundle)])
    assert "**RELEASE NOT CONFIRMED**: nothing shows the payload leaving." in capsys.readouterr().out
