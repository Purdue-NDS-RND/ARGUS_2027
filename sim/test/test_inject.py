"""GPS faults in the sim (SIM-04): the schedule, the parameter values and names, the glitch's pass line, and the runner's verdict. No MAVLink, no Gazebo."""

import json
import math
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import inject  # noqa: E402
from inject import InjectError, Schedule  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import BuildError, build_run  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

GLITCH = SIM / "scenarios" / "hover" / "gps_glitch.yaml"
APOLLO_LAT = 34.175
V46 = {"SIM_GPS_GLITCH_X", "SIM_GPS_GLITCH_Y", "SIM_GPS_GLITCH_Z", "SIM_GPS_NOISE", "SIM_GPS_NUMSATS", "SIM_GPS_JAM", "SIM_GPS_DISABLE"}
LATER = {"SIM_GPS1_GLTCH_X", "SIM_GPS1_GLTCH_Y", "SIM_GPS1_GLTCH_Z", "SIM_GPS1_NOISE", "SIM_GPS1_NUMSATS", "SIM_GPS1_JAM", "SIM_GPS1_DISABLE"}
DEFAULTS = {"glitch_lat": 0.0, "glitch_lon": 0.0, "glitch_alt": 0.0, "noise": 0.0, "satellites": 10.0, "jam": 0.0, "disable": 0.0}
WITH_SONAR = {**DEFAULTS, "sonar_scale": 12.12}


def test_a_step_in_metres_is_the_degrees_sitl_adds_to_the_fix():
    d_lat, d_lon = inject.glitch_degrees(5.0, 0.0, APOLLO_LAT)
    assert d_lat == pytest.approx(5.0 / 111319.5) and d_lon == 0.0
    d_lat, d_lon = inject.glitch_degrees(0.0, 5.0, APOLLO_LAT)
    assert d_lat == 0.0 and d_lon == pytest.approx(5.0 / (111319.5 * math.cos(math.radians(APOLLO_LAT))))
    # And back: the fix that moved by those degrees moved by those metres.
    north, east = inject.fix_step_m((APOLLO_LAT, -118.4825), (APOLLO_LAT + inject.glitch_degrees(5.0, -3.0, APOLLO_LAT)[0], -118.4825 + inject.glitch_degrees(5.0, -3.0, APOLLO_LAT)[1]))
    assert (north, east) == pytest.approx((5.0, -3.0), abs=1e-6)


def test_the_parameter_names_are_read_from_the_running_sitl_whichever_family_it_has():
    assert inject.resolve_names(V46)["glitch_lat"] == "SIM_GPS_GLITCH_X" and inject.resolve_names(V46)["satellites"] == "SIM_GPS_NUMSATS"
    assert inject.resolve_names(LATER)["glitch_lat"] == "SIM_GPS1_GLTCH_X" and inject.resolve_names(LATER)["disable"] == "SIM_GPS1_DISABLE"
    assert set(inject.resolve_names(V46 | {"SIM_TERRAIN"})) == set(inject.PARAMETER_NAMES) - set(inject.OPTIONAL_ROLES)
    with pytest.raises(InjectError, match="SIM_GPS_GLITCH_X or SIM_GPS1_GLTCH_X"):  # neither: stop, do not fly it calm
        inject.resolve_names(V46 - {"SIM_GPS_GLITCH_X"})
    with pytest.raises(InjectError, match="none of the parameters"):
        inject.resolve_names(set())


def test_the_name_lookup_is_over_as_soon_as_every_fault_has_a_name():
    # Half the candidate names never answer; waiting for them made the injector see the arming seconds late.
    assert inject.names_complete(V46) and inject.names_complete(LATER) and inject.names_complete(V46 | LATER)
    assert not inject.names_complete(V46 - {"SIM_GPS_JAM"}) and not inject.names_complete(set())
    source = (SIM / "inject.py").read_text()
    assert "not names_complete(set(found), needed)" in source and 'type=["PARAM_VALUE", "HEARTBEAT"]' in source  # and heartbeats are heard meanwhile


def test_each_kind_sets_its_parameters_and_knows_how_to_undo_them():
    on, off = inject.changes_for({"kind": "glitch", "north_m": 5.0, "up_m": 2.0}, APOLLO_LAT, DEFAULTS)
    assert on == {"glitch_lat": pytest.approx(5.0 / 111319.5), "glitch_lon": 0.0, "glitch_alt": 2.0} and off == {"glitch_lat": 0.0, "glitch_lon": 0.0, "glitch_alt": 0.0}
    assert inject.changes_for({"kind": "noise", "amplitude_m": 3.0}, APOLLO_LAT, DEFAULTS) == ({"noise": 3.0}, {"noise": 0.0})
    assert inject.changes_for({"kind": "satellites", "count": 5}, APOLLO_LAT, DEFAULTS) == ({"satellites": 5.0}, {"satellites": 10.0})
    assert inject.changes_for({"kind": "jam"}, APOLLO_LAT, DEFAULTS) == ({"jam": 1.0}, {"jam": 0.0})
    assert inject.changes_for({"kind": "dropout"}, APOLLO_LAT, DEFAULTS) == ({"disable": 1.0}, {"disable": 0.0})
    assert inject.changes_for({"kind": "clear"}, APOLLO_LAT, DEFAULTS) == (DEFAULTS, {})
    # The rangefinder reads `scale` times the true range: SITL's metres a volt go up by 1 / scale (LOC-1's dips).
    on, off = inject.changes_for({"kind": "rangefinder", "scale": 0.75}, APOLLO_LAT, WITH_SONAR)
    assert on == {"sonar_scale": pytest.approx(16.16)} and off == {"sonar_scale": 12.12}
    # Its parameter is asked of SITL only by a scenario that has such an event, and only then is it an error not to have it.
    events = [{"at_s": 40, "kind": "rangefinder", "scale": 0.75, "for_s": 8}]
    assert inject.roles_needed(events) == {"sonar_scale"} and inject.roles_needed([{"at_s": 1, "kind": "jam"}]) == set()
    assert "sonar_scale" not in inject.resolve_names(V46) and inject.names_complete(V46)
    assert inject.resolve_names(V46 | {"SIM_SONAR_SCALE"}, frozenset({"sonar_scale"}))["sonar_scale"] == "SIM_SONAR_SCALE"
    assert not inject.names_complete(V46, frozenset({"sonar_scale"})) and inject.names_complete(V46 | {"SIM_SONAR_SCALE"}, frozenset({"sonar_scale"}))
    with pytest.raises(InjectError, match="SIM_SONAR_SCALE"):
        inject.resolve_names(V46, frozenset({"sonar_scale"}))


def test_the_schedule_gives_each_change_once_at_its_time_since_arming():
    schedule = Schedule(
        [{"at_s": 30.0, "kind": "dropout", "for_s": 5.0}, {"at_s": 20.0, "kind": "glitch", "north_m": 5.0, "for_s": 20.0}, {"at_s": 50.0, "kind": "clear"}],
        APOLLO_LAT, DEFAULTS,
    )  # fmt: skip
    assert len(schedule) == 5 and schedule.due(19.99) == [] and not schedule.done
    ((values, about),) = schedule.due(20.0)
    assert about["event"] == "glitch" and about["what"] == "on" and about["north_m"] == 5.0 and values["glitch_lat"] > 0.0
    assert schedule.due(20.5) == []  # once
    kinds = [(about["event"], about["what"]) for _values, about in schedule.due(41.0)]  # a late caller gets all that is due, in order
    assert kinds == [("dropout", "on"), ("dropout", "off"), ("glitch", "off")]
    ((values, about),) = schedule.due(60.0)
    assert about["event"] == "clear" and values == DEFAULTS and schedule.done


@pytest.mark.parametrize(
    "bad, message",
    [
        ([{"kind": "glitch"}], "needs at_s and a kind"),
        ([{"at_s": 5, "kind": "earthquake"}], "one of: glitch"),
        ([{"at_s": 5, "kind": "glitch", "south_m": 5}], "unknown setting"),
        ([{"at_s": -1, "kind": "jam"}], "cannot be negative"),
        ([{"at_s": 5, "kind": "jam", "for_s": 0}], "above zero"),
        ([{"at_s": "soon", "kind": "jam"}], "numbers of seconds"),
        ([{"at_s": 5, "kind": "rangefinder", "for_s": 5}], "scale"),
        ([{"at_s": 5, "kind": "rangefinder", "scale": 0.0}], "scale"),
        ([{"at_s": 5, "kind": "rangefinder", "scale": 2.5}], "scale"),
        ([{"at_s": 5, "kind": "rangefinder", "scale": "low"}], "scale"),
    ],
)
def test_bad_events_are_refused(bad, message):
    with pytest.raises(InjectError, match=message):
        inject.check_events(bad)


def test_a_scenario_with_a_bad_event_is_refused_when_it_is_built(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(GLITCH.read_text().replace("kind: glitch", "kind: earthquake"))
    with pytest.raises(BuildError, match="gps_events"):
        build_run(bad, tmp_path / "run")
    run = build_run(GLITCH, tmp_path / "good")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["gps_events"] == [{"at_s": 25.0, "kind": "glitch", "north_m": 5.0, "for_s": 10.0}] and inject.wanted(manifest)
    assert events.tools_needed(manifest) >= {"gps_events"}


# --- the pass line: a 5 m north glitch moves the raw fix 5 +/- 0.5 m within 1 s ---

BEFORE = (APOLLO_LAT, -118.4825)


def fixes_after(step_north_m, at_s, t_set=100.0, drift_m=0.0):
    """Raw fixes at 5 Hz after a command at t_set: unmoved until at_s after it, then stepped."""
    rows = []
    for index in range(1, 11):
        t = t_set + index * 0.2
        north = (step_north_m if t - t_set >= at_s else 0.0) + drift_m * (t - t_set)
        rows.append((t, BEFORE[0] + north / inject.METRES_PER_DEGREE, BEFORE[1]))
    return rows


def test_a_glitch_that_shows_within_a_second_passes():
    seen = inject.glitch_result((5.0, 0.0), fixes_after(5.0, at_s=0.3), 100.0, BEFORE)
    assert seen == {"passed": True, "after_s": pytest.approx(0.4), "moved_north_m": pytest.approx(5.0, abs=1e-3), "moved_east_m": pytest.approx(0.0, abs=1e-3)}
    assert inject.glitch_result((5.0, 0.0), fixes_after(4.6, at_s=0.3), 100.0, BEFORE)["passed"]  # inside 5 +/- 0.5
    assert inject.glitch_result((5.0, 0.0), fixes_after(5.0, at_s=0.3, drift_m=0.3), 100.0, BEFORE)["passed"]  # a slow drift on top


def test_a_glitch_that_is_late_too_small_or_absent_fails():
    late = inject.glitch_result((5.0, 0.0), fixes_after(5.0, at_s=1.3), 100.0, BEFORE)
    assert late["passed"] is False and late["after_s"] is None
    small = inject.glitch_result((5.0, 0.0), fixes_after(4.0, at_s=0.3), 100.0, BEFORE)
    assert small["passed"] is False and small["moved_north_m"] == pytest.approx(4.0, abs=1e-3)
    wrong_way = inject.glitch_result((5.0, 0.0), fixes_after(-5.0, at_s=0.3), 100.0, BEFORE)
    assert wrong_way["passed"] is False
    assert inject.glitch_result((5.0, 0.0), [], 100.0, BEFORE) == {"passed": False, "after_s": None, "moved_north_m": None, "moved_east_m": None}
    assert (inject.GLITCH_PASS_M, inject.GLITCH_PASS_S) == (0.5, 1.0)


# --- the report and the runner ---

NAMES = inject.resolve_names(V46)
SEEN = {"passed": True, "after_s": 0.4, "moved_north_m": 5.02, "moved_east_m": 0.01}


def change(event, what, since_arm_s, seen=None, confirmed=True):
    return {"t": 100.0 + since_arm_s, "since_arm_s": since_arm_s, "event": event, "what": what, "set": {}, "confirmed": confirmed, "seen": seen}


def test_the_report_passes_only_if_everything_scheduled_happened_and_every_glitch_showed():
    good = inject.build_report(2, [change("glitch", "on", 25.0, SEEN), change("glitch", "off", 35.0)], NAMES, armed=True)
    assert good["passed"] and good["problems"] == [] and good["applied"] == 2 and good["glitches"] == [SEEN]
    assert inject.summary_line(good) == "2 of 2 changes made; 1 of 1 glitch(es) seen in the raw fix; the first moved the raw fix +5.02 m north, +0.01 m east 0.40 s after the command"
    cut_short = inject.build_report(2, [change("glitch", "on", 25.0, SEEN)], NAMES, armed=True)
    assert not cut_short["passed"] and "only 1 of 2 scheduled changes" in cut_short["problems"][0]
    unseen = inject.build_report(1, [change("glitch", "on", 25.0, {**SEEN, "passed": False, "after_s": None})], NAMES, armed=True)
    assert not unseen["passed"] and "did not show in the raw GPS fix within 1 s to 0.5 m" in unseen["problems"][0]
    unconfirmed = inject.build_report(1, [change("jam", "on", 10.0, confirmed=False)], NAMES, armed=True)
    assert not unconfirmed["passed"] and "did not confirm jam (on) at 10.0 s" in unconfirmed["problems"][0]
    never = inject.build_report(1, [], NAMES, armed=False)
    assert never["problems"] == ["the vehicle never armed, so no event's time came"]


def test_the_report_file_round_trips_and_the_tool_wants_a_run_folder(tmp_path):
    assert inject.read_gps_events(tmp_path) is None
    (tmp_path / "gps_events.json").write_text(json.dumps(inject.build_report(1, [change("jam", "on", 10.0)], NAMES, armed=True)))
    assert inject.read_gps_events(tmp_path)["passed"] is True
    assert inject.main([]) == 2
    assert inject.wanted({"gps_events": [{"at_s": 1, "kind": "jam"}]}) and not inject.wanted({"gps_events": []}) and not inject.wanted({})


def faulted_run(tmp_path, report):
    run = make_run(tmp_path, score=SCORE, manifest={"variant": "off", "gps_events": [{"at_s": 25.0, "kind": "glitch", "north_m": 5.0, "for_s": 10.0}]})
    if report is not None:
        (run / "gps_events.json").write_text(json.dumps(report))
    return run


def test_a_run_with_gps_events_counts_only_if_they_were_injected(tmp_path):
    good = inject.build_report(2, [change("glitch", "on", 25.0, SEEN), change("glitch", "off", 35.0)], NAMES, armed=True)
    passed = result.run_result(faulted_run(tmp_path / "a", good))
    assert passed["status"] == "OK"
    assert "| GPS faults | 2 of 2 changes made; 1 of 1 glitch(es) seen in the raw fix; the first moved the raw fix +5.02 m north, +0.01 m east 0.40 s after the command |" in result.render_report(passed)
    absent = result.run_result(faulted_run(tmp_path / "b", None))
    assert absent["status"] == "INVALID" and "gps_events.json" in absent["reason"]  # it may have been flown without its faults
    missing_parameters = {"passed": False, "problems": ["this SITL has none of the parameters for: SIM_GPS_GLITCH_X or SIM_GPS1_GLTCH_X"], "scheduled": 0, "applied": 0, "glitches": [], "parameters": {}, "changes": []}
    invalid = result.run_result(faulted_run(tmp_path / "c", missing_parameters))
    assert invalid["status"] == "INVALID" and "none of the parameters" in invalid["reason"]
    plain = result.run_result(make_run(tmp_path / "d", score=SCORE, manifest={"variant": "off"}))
    assert plain["status"] == "OK" and "| GPS faults |" not in result.render_report(plain)


def test_the_injector_has_its_own_mavlink_port_and_the_runner_starts_it():
    start = (SIM / "container" / "start.sh").read_text()
    assert "FAULTS_PORT=14553" in start and '--out "udp:127.0.0.1:$FAULTS_PORT"' in start
    assert "udpin:127.0.0.1:14553" in (SIM / "inject.py").read_text()  # not the tools port a driver or the payload bridge listens on
    fly = (SIM / "runner" / "fly.py").read_text()
    assert '"gps_events": SIM_DIR / "inject.py"' in fly
    assert events.unsupported({"gps_events": [{"at_s": 1, "kind": "jam"}]}, {"gps_events"}) == ""
    assert "sim/inject.py" in events.unsupported({"gps_events": [{"at_s": 1, "kind": "jam"}]}, set())
