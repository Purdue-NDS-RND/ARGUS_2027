"""OPS-02: the readiness kit (tools/readiness_kit.py), on bundles and dataflash logs built here. No ROS."""

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
import dataflash_params as dfp  # noqa: E402
import readiness_kit as kit  # noqa: E402
import test_dataflash_params as logs  # noqa: E402  (the byte-by-byte log builders)

MODE_FMT = logs.fmt(3, "MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn")
ERR_FMT = logs.fmt(4, "ERR", "QBB", "TimeUS,Subsys,ECode")
SA_FMT = logs.fmt(5, "SA", "QBffffffB", "TimeUS,State,DVelX,DVelY,DVelZ,MVelX,MVelY,MVelZ,Back")
GUIP_FMT = logs.fmt(6, "GUIP", "QBfffbffffff", "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ")
FNCE_FMT = logs.fmt(7, "FNCE", "QBBBLLBf", "TimeUS,Tot,Seq,Type,Lat,Lng,Count,Radius")
POS_FMT = logs.fmt(8, "POS", "QLLfff", "TimeUS,Lat,Lng,Alt,RelHomeAlt,RelOriginAlt")
HEAD = b"\xa3\x95"
# The placeholder site's fence, as the flight controller logs it, and a point 2 m inside its east edge.
FENCE = [(341744610, -1184841287), (341744610, -1184808713), (341766170, -1184808713), (341766170, -1184841287)]
AT_THE_FENCE = (341750000, -1184808930)


def sa(seconds, allowed_mps):
    return HEAD + bytes([5]) + struct.pack("<QBffffffB", int(seconds * 1e6), 1, 0.0, allowed_mps, 0.0, 0.0, allowed_mps, 0.0, 0)


def guip(seconds, east_mps):
    return HEAD + bytes([6]) + struct.pack("<QBfffbffffff", int(seconds * 1e6), 4, 0.0, 0.0, 0.0, 0, 0.0, east_mps * 100.0, 0.0, 0.0, 0.0, 0.0)


def fnce():
    return b"".join(HEAD + bytes([7]) + struct.pack("<QBBBiiBf", 60_000_000, 4, seq, 98, lat, lng, 4, 0.0) for seq, (lat, lng) in enumerate(FENCE))


def pos(seconds, lat_lng):
    return HEAD + bytes([8]) + struct.pack("<Qiifff", int(seconds * 1e6), lat_lng[0], lat_lng[1], 225.0, 10.0, 10.0)


def held_events(hold_s=30.0, commanded=5.0, where=AT_THE_FENCE, home_mode=6, guided=4):
    """A hold's log: guided from 60 s, the avoidance allowing nothing from 140 s for `hold_s` with a velocity commanded, then RTL asked for."""
    events = [mode(60.0, guided), fnce()]
    steps = int(hold_s * 10)
    for index in range(steps + 1):
        t = 140.0 + index * 0.1
        events += [sa(t, 0.02), guip(t, commanded), pos(t, where)]
    return events + ([mode(140.0 + hold_s + 0.2, home_mode, 2)] if home_mode is not None else [])


def mode(seconds, number, reason=2):
    return HEAD + bytes([3]) + struct.pack("<QBBB", int(seconds * 1e6), number, number, reason)


def err(seconds, subsystem, code):
    return HEAD + bytes([4]) + struct.pack("<QBB", int(seconds * 1e6), subsystem, code)


FENCE_ON = {"FENCE_ENABLE": 1.0, "FENCE_ACTION": 1.0, "FENCE_TYPE": 5.0, "FENCE_TOTAL": 6.0}


def flight_log(path, params=None, events=None):
    """A demonstration's log: guided from 60 s, the polygon breached at 140 s and the fence's RTL in the same instant, back inside at 146 s.

    In the order a real log has them (ArduCopter 4.6.3): the mode change first, then the error, with one time stamp.
    """
    params = FENCE_ON if params is None else params
    events = [mode(60.0, 4), mode(140.0, 6, 10), err(140.0, 9, 4), err(146.0, 9, 0)] if events is None else events
    parts = [logs.FMT_FMT, logs.PARM_FMT, logs.MSG_FMT, MODE_FMT, ERR_FMT, SA_FMT, GUIP_FMT, FNCE_FMT, POS_FMT, logs.msg("ArduCopter V4.6.3 (92b0cd78)")]
    parts += [logs.parm(name, value) for name, value in params.items()]
    path.write_bytes(b"".join(parts + events))
    return path


def make_bundle(tmp_path, mission="geofence", profile_flags=("fence_demo",), preflight=None, **log):
    """A fence demonstration's attempt bundle, made with the real bundler."""
    src = tmp_path / "flight"
    (src / "bag").mkdir(parents=True)
    (src / "bag" / "bag_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    listed = [{"topic_metadata": {"name": "/mavros/state", "type": "mavros_msgs/msg/State", "serialization_format": "cdr"}, "message_count": 10}]
    (src / "bag" / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"storage_identifier": "mcap", "topics_with_message_count": listed}}))
    flight_log(src / "00000007.BIN", **log)
    (src / "sequence.yaml").write_text("mission_sequence: {features: {fence_demo: false}, missions: []}\n")
    (src / "params.yaml").write_text("{}\n")
    (src / "fence_demo.yaml").write_text("description: x\nenable: [" + ", ".join(profile_flags) + "]\n")
    (src / "baseline.param").write_text("FENCE_ENABLE,1\n")
    (src / "preflight.jsonl").write_text(json.dumps(preflight or {"verdict": "GO", "failed": []}) + "\n")
    name = bundle_attempt.attempt_name("2027-04-20", "readiness", mission)
    bundle_attempt.bundle(name, tmp_path / "attempts", mission, bag=src / "bag", dataflash=src / "00000007.BIN", sequence=src / "sequence.yaml", params=src / "params.yaml",
                          profile=src / "fence_demo.yaml", baseline=src / "baseline.param", preflight_log=src / "preflight.jsonl", git={"hash": "0bdf615", "describe": "comp-2026-106-g0bdf615", "dirty": False})  # fmt: skip
    return tmp_path / "attempts" / name


def test_a_demonstration_whose_log_shows_the_fence_acting_is_ready(tmp_path, capsys):
    folder = make_bundle(tmp_path)
    assert kit.main([str(folder)]) == 0
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "READY"
    text = (folder / "readiness.md").read_text()
    for line in (
        "Verdict: **READY**",
        "ArduCopter V4.6.3",
        "- FENCE_ENABLE is 1: the fence was on",
        "- FENCE_ACTION is 1: a breach ends the flight",
        "- FENCE_TYPE 5 has the polygon, and 6 fence points were stored",
        "- at 140.0 s the flight controller reported a breach of the polygon",
        "- the vehicle was in GUIDED, under the companion computer's command, when it was breached",
        "- 0.00 s later the flight controller was in RTL by its own fence action (mode change reason 10)",
        "- 6.0 s after the breach the vehicle was back inside the fence",
        "- the bundle is complete",
        "- the preflight said GO, with no override",
        "- the flag profile flown was fence_demo, and nothing else was on",
        "- the code was at comp-2026-106-g0bdf615",
        "## The video",
        "- [ ] (log) The vehicle was in GUIDED, with a velocity commanded, when the flight controller stopped it.",
    ):
        assert line in text, line
    assert "## What is missing" not in text
    assert bundle_attempt.verify(folder) == []  # writing readiness.md into the bundle leaves it as its manifest says


@pytest.mark.parametrize(
    "log, word",
    [
        ({"params": {**FENCE_ON, "FENCE_ENABLE": 0.0}}, "FENCE_ENABLE is 0.0: the fence was not on"),
        ({"params": {**FENCE_ON, "FENCE_ACTION": 0.0}}, "FENCE_ACTION is 0.0: not an action that ends the flight"),
        ({"params": {**FENCE_ON, "FENCE_TYPE": 1.0}}, "no polygon fence in the parameters (FENCE_TYPE 1.0"),
        ({"params": {**FENCE_ON, "FENCE_TOTAL": 0.0}}, "no polygon fence in the parameters (FENCE_TYPE 5.0, FENCE_TOTAL 0.0)"),
        ({"events": [mode(60.0, 4), mode(200.0, 6, 2)]}, "the log has no fence breach (no ERR message of the fence subsystem) and no hold at the fence"),
        ({"events": [mode(60.0, 5), mode(140.0, 6, 10), err(140.0, 9, 4)]}, "the vehicle was in LOITER at the breach, not in GUIDED"),
        ({"events": [mode(60.0, 4), err(140.0, 9, 4), mode(143.5, 6, 10)]}, "3.50 s later the flight controller was in RTL by its own fence action (mode change reason 10): later than 2 s"),
        ({"events": [mode(60.0, 4), err(140.0, 9, 4), mode(141.0, 5, 1)]}, "changed to no mode that brings the vehicle back or stops it"),
        ({"events": [mode(60.0, 4), err(140.0, 9, 4), mode(140.8, 6, 1)]}, "0.80 s after the breach the vehicle was in RTL, but by mode change reason 1, not the fence's own (10): somebody else brought it back"),  # the pilot's switch
        ({"events": [mode(60.0, 4), err(140.0, 9, 4), mode(140.8, 6, 2)]}, "by mode change reason 2, not the fence's own"),  # the companion's or the ground station's command
        ({"events": [mode(60.0, 4), err(140.0, 3, 1), mode(141.0, 6, 3)]}, "the log has no fence breach"),  # another subsystem's error is not the fence's
    ],
)
def test_each_thing_the_log_does_not_show_makes_it_not_ready_and_is_said(log, word, tmp_path, capsys):
    folder = make_bundle(tmp_path, **log)
    assert kit.main([str(folder)]) == 1
    out = capsys.readouterr().out
    assert out.strip().splitlines()[-1] == "NOT READY" and word in out
    text = (folder / "readiness.md").read_text()
    assert "Verdict: **NOT READY**" in text and "## What is missing" in text and word in text


def test_the_bundle_itself_is_held_to_the_demonstration(tmp_path, capsys):
    cases = (
        (dict(mission="delivery"), "the bundle is of a delivery attempt, not of the geofence demonstration"),
        (dict(profile_flags=("fence_demo", "takeover_latch")), "turns on ['fence_demo', 'takeover_latch'], not fence_demo alone"),
        (dict(profile_flags=()), "turns on [], not fence_demo alone"),
        (dict(preflight={"verdict": "NO-GO OVERRIDDEN", "failed": ["gps"], "override": {"reason": "roof", "by": "x"}}), "the preflight's last verdict was NO-GO OVERRIDDEN (overridden), not a clean GO"),
    )
    for index, (changes, word) in enumerate(cases):
        folder = make_bundle(tmp_path / str(index), **changes)
        assert kit.main([str(folder)]) == 1
        assert word in capsys.readouterr().out, word
    folder = make_bundle(tmp_path / "gone")
    (folder / "dataflash" / "00000007.BIN").unlink()
    assert kit.main([str(folder)]) == 1
    out = capsys.readouterr().out
    assert "no dataflash log" in out and "dataflash/00000007.BIN is gone" in out
    assert kit.main([str(tmp_path / "nowhere")]) == 2 and "not an attempt's bundle" in capsys.readouterr().err


def test_a_breach_code_names_what_was_breached_and_the_mode_is_the_one_at_that_time():
    assert kit.breach_names(4) == "the polygon" and kit.breach_names(5) == "the ceiling and the polygon" and kit.breach_names(64) == "code 64"
    story = kit.LogStory(modes=[(10.0, 0, 1), (60.0, 4, 2), (140.3, 6, 10)])
    assert kit.mode_at(story, 5.0) is None and kit.mode_at(story, 60.0) == 4 and kit.mode_at(story, 140.0) == 4 and kit.mode_at(story, 141.0) == 6


def test_the_checklist_is_the_documents_and_names_what_the_log_answers(capsys):
    assert kit.main(["--checklist"]) == 0
    out = capsys.readouterr().out
    document = (REPO / "docs" / "ops" / "readiness_video.md").read_text()
    assert out.startswith("## The video") and out.strip() in document
    assert out.count("- [ ] (log)") == 4 and out.count("- [ ]") >= 10
    # The procedure names the real commands and the real files.
    for name in ("tools/field_setup.py", "tools/go.py --profile fence_demo", "tools/dataflash_pull.py", "tools/bundle_attempt.py --slot readiness --mission geofence", "tools/readiness_kit.py"):
        assert name in document, name
    assert (REPO / "profiles" / "fence_demo.yaml").is_file() and (REPO / "src/drone_mission_demo/config/sequences/fence_demo.yaml").is_file()


def test_the_log_reader_gives_every_message_asked_for_in_order(tmp_path):
    path = flight_log(tmp_path / "x.BIN")
    names = [name for name, _row in dfp.messages(path, ("MODE", "ERR"))]
    assert names == ["MODE", "MODE", "ERR", "ERR"]
    rows = [row for name, row in dfp.messages(path, ("ERR",))]
    assert [(row["Subsys"], row["ECode"]) for row in rows] == [(9, 4), (9, 0)] and rows[0]["TimeUS"] == 140_000_000
    assert sum(1 for _ in dfp.messages(path)) == 9 + 1 + 4 + 4  # the nine FMTs, the firmware line, four parameters, four events


def test_a_vehicle_the_flight_controller_held_at_the_fence_is_ready_too(tmp_path, capsys):
    """What ArduCopter's avoidance does to a velocity commanded at the fence: no breach, the vehicle kept inside. Seen in the sim three times."""
    folder = make_bundle(tmp_path, events=held_events())
    assert kit.main([str(folder)]) == 0
    text = (folder / "readiness.md").read_text()
    for line in (
        "Verdict: **READY**",
        "- from 140.0 s, for 30.0 s, the flight controller's avoidance held the vehicle still with 5.0 m/s commanded, in GUIDED: no breach",
        "- it was held 2.0 m inside the fence line, by the polygon of 4 corners the log itself holds",
        "- 0.2 s after the hold the flight controller was in RTL (mode change reason 2), on the way home",
    ):
        assert line in text, line


@pytest.mark.parametrize(
    "events, word",
    [
        (held_events(hold_s=3.0), "no hold at the fence"),  # too short to be a hold
        (held_events(commanded=0.0), "no hold at the fence"),  # nothing was being commanded: a hover, not a hold
        (held_events(guided=5), "no hold at the fence"),  # in LOITER: not the autonomous stack
        (held_events(where=(341750000, -1184815000)), " m inside the fence line: not at the fence"),  # 58 m from the east edge
        (held_events(where=(341750000, -1184808000)), "during the hold the vehicle was 6.6 m outside the fence polygon in the log"),
        (held_events(home_mode=None), "after the hold the log shows no mode that brings the vehicle home"),
        ([item for item in held_events() if item != fnce()], "the log does not say where the vehicle was held"),
    ],
)
def test_a_hold_that_is_not_one_or_is_not_at_the_fence_is_not_ready(events, word, tmp_path, capsys):
    folder = make_bundle(tmp_path, events=events)
    assert kit.main([str(folder)]) == 1
    assert word in capsys.readouterr().out


def test_a_breach_after_a_hold_says_both(tmp_path):
    """The sim's own sequence: held at the margin, then over the line, and the fence's RTL."""
    events = held_events(hold_s=14.0, home_mode=None) + [mode(154.5, 6, 10), err(154.5, 9, 4), err(158.0, 9, 0)]
    folder = make_bundle(tmp_path, events=events)
    assert kit.main([str(folder)]) == 0
    text = (folder / "readiness.md").read_text()
    assert "- before the breach the flight controller's avoidance held the vehicle still for 14.0 s with 5.0 m/s commanded" in text
    assert "- at 154.5 s the flight controller reported a breach of the polygon" in text and "- 0.00 s later the flight controller was in RTL by its own fence action (mode change reason 10)" in text and "the vehicle was in GUIDED" in text


def test_how_far_inside_a_polygon_a_point_is():
    polygon = [(lat * 1e-7, lng * 1e-7) for lat, lng in FENCE]
    assert kit.inside_by_m(polygon, 34.1750000, -118.4808930) == pytest.approx(2.0, abs=0.05)
    assert kit.inside_by_m(polygon, 34.1750000, -118.4825000) == pytest.approx(60.0, abs=0.2)  # the origin: 60 m from the south edge
    assert kit.inside_by_m(polygon, 34.1750000, -118.4808000) == pytest.approx(-6.6, abs=0.1)
    assert kit.inside_by_m(polygon[:2], 34.175, -118.4825) is None
