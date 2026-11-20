"""The check of the localisation pipeline beside a sim flight (LOC-5): its verdict, and its place in the runner."""

import json
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(SIM), str(Path(__file__).resolve().parent)]

import loc_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import build_run  # noqa: E402
from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)

SURVEY = SIM / "scenarios" / "waypoint_navigation" / "survey_pipeline.yaml"


def status(t_s, enabled=True, frames=0, rate_hz=9.8, age_ms=45.0, **more):
    return (t_s, {"enabled": enabled, "frames": frames, "dropped": 0, "rate_hz": rate_hz, "frame_age_ms": age_ms, "detections": frames * 2, "placed": frames * 2, "refused": 0, "faults": 0, "detector": "oracle", **more})


def a_survey(rate_hz=9.8, age_ms=45.0, stop=True, **more):
    """Statuses once a second: off for 20 s, on for 60 s, then off."""
    statuses = [status(float(t), enabled=False) for t in range(600, 620)]
    statuses += [status(float(t), frames=int((t - 620) * rate_hz), rate_hz=rate_hz if t > 621 else 0.0, age_ms=age_ms, **more) for t in range(620, 680)]
    if stop:
        statuses += [status(float(t), enabled=False, frames=int(60 * rate_hz), rate_hz=0.0, **more) for t in range(680, 690)]
    return statuses


def boxes(classes=("marker", "boundary_marker"), count=300):
    return [(620.0 + index * 0.1, classes) for index in range(count)]


def test_the_lines():
    assert loc_check.MIN_RATE_HZ == 8.0 and loc_check.MAX_FRAME_AGE_MS == 250.0 and loc_check.MIN_PLACED_SHARE == 0.9
    assert loc_check.STATUS_TOPIC == "/loc/status" and loc_check.DETECTIONS_TOPIC == "/loc/detections"


def test_a_pipeline_that_ran_through_the_survey_at_its_rate_passes():
    report = loc_check.build_report(a_survey(), boxes(), {"marker", "boundary_marker"})
    assert report["passed"] and report["problems"] == []
    assert report["rate_hz"]["median"] == pytest.approx(9.8) and report["started"] and report["stopped"]
    assert report["frames"] == 588 and report["classes"] == {"boundary_marker": 300, "marker": 300}
    lines = loc_check.summary_lines(report)
    assert any("9.8 frames a second" in line and "PASS" in line for line in lines)


@pytest.mark.parametrize(
    "statuses, found, expected, said",
    [
        ([status(float(t), enabled=False) for t in range(600, 690)], [], {"marker"}, "never started"),
        (a_survey(stop=False), boxes(), {"marker"}, "still running"),
        (a_survey(rate_hz=6.0), boxes(), {"marker"}, "6.0 frames a second"),
        (a_survey()[:50] + [status(float(t), frames=290, rate_hz=2.0) for t in range(650, 656)] + a_survey()[56:], boxes(), {"marker"}, "slowest"),
        (a_survey(age_ms=900.0), boxes(), {"marker"}, "old"),
        (a_survey(), boxes(classes=("marker",)), {"marker", "boundary_marker"}, "boundary_marker"),
        (a_survey(placed=10), boxes(), {"marker"}, "put on the ground"),
        (a_survey(faults=3, last_fault="RuntimeError: x"), boxes(), {"marker"}, "3 frame(s) faulted"),
        ([], [], {"marker"}, "no status"),
    ],
)
def test_what_fails_it(statuses, found, expected, said):
    report = loc_check.build_report(statuses, found, expected)
    assert not report["passed"] and any(said in problem for problem in report["problems"]), report["problems"]


def test_the_check_runs_beside_flights_with_the_node_on_and_only_those(tmp_path):
    on = json.loads((build_run(SURVEY, tmp_path / "on", variant="on") / "scenario.json").read_text())
    off = json.loads((build_run(SURVEY, tmp_path / "off", variant="off") / "scenario.json").read_text())
    assert loc_check.wanted(on) and not loc_check.wanted(off) and not loc_check.wanted({})
    assert "loc_check" in events.tools_needed(on) and "truth_poses" in events.tools_needed(on) and "loc_check" not in events.tools_needed(off)
    assert "sim/loc_check.py" in events.unsupported(on, events.tools_needed(on) - {"loc_check"})
    assert '"loc_check": SIM_DIR / "loc_check.py"' in (SIM / "runner" / "fly.py").read_text()
    assert loc_check.expected_classes(tmp_path / "on") == {"marker", "boundary_marker"}


def test_a_run_with_the_node_on_counts_only_with_a_passing_check(tmp_path):
    manifest = {"variant": "on", "node_flags": {"loc_node": {"loc_node": True, "sim_oracle_detector": True}}}
    run = make_run(tmp_path / "a", score=SCORE, manifest=manifest)
    (run / "loc_check.json").write_text(json.dumps(loc_check.build_report(a_survey(), boxes(), {"marker"})))
    passed = result.run_result(run)
    assert passed["status"] == "OK" and "| Localisation pipeline |" in result.render_report(passed)
    run = make_run(tmp_path / "b", score=SCORE, manifest=manifest)
    (run / "loc_check.json").write_text(json.dumps(loc_check.build_report(a_survey(rate_hz=6.0), boxes(), {"marker"})))
    slow = result.run_result(run)
    assert slow["status"] == "FAIL" and slow["reason"].startswith("the localisation pipeline missed a pass line: ")
    absent = result.run_result(make_run(tmp_path / "c", score=SCORE, manifest=manifest))
    assert absent["status"] == "INVALID" and "loc_check.json" in absent["reason"]
    plain = result.run_result(make_run(tmp_path / "d", score=SCORE))
    assert plain["status"] == "OK" and plain["loc_check"] is None


def test_the_scenario_hands_the_oracle_the_true_objects_and_starts_the_node(tmp_path):
    import yaml
    from scenario.spec import BuildError

    run = build_run(SURVEY, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert manifest["launch"]["file"] == "search_pattern_demo.launch.py" and manifest["launch"]["arguments"]["loc_node"] == "true"
    assert manifest["truth_poses"] is True
    settings = yaml.safe_load((run / "params.yaml").read_text())["loc_node"]["ros__parameters"]
    assert settings["features"] == {"loc_node": True, "sim_oracle_detector": True, "loc_tiling": False, "marker_tracker": True, "loc_median_position": True, "survey_frame_log": False, "digit_reader": False, "template_digit_classifier": False, "digit_vote": False, "judges_list": False}
    assert settings["oracle_objects_file"] == str(run / "oracle_objects.json") and settings["sim_only"] is True and settings["oracle_seed"] == 1
    objects = json.loads((run / "oracle_objects.json").read_text())
    assert objects["vehicle"] == manifest["vehicle"]["model"]
    by_name = {item["name"]: item for item in objects["objects"]}
    assert by_name["marker_3"] == {"name": "marker_3", "class_id": "marker", "size_m": 0.6, "top_m": 0.02}
    assert by_name["corner_sw"]["class_id"] == "boundary_marker" and by_name["corner_sw"]["size_m"] == 1.2
    assert len(by_name) == 8  # three numbered markers inside the corners, one outside them, and the four corners
    # Flags off: the node is not started, and its flags stay false.
    off = build_run(SURVEY, tmp_path / "off", variant="off")
    assert "loc_node" not in json.loads((off / "scenario.json").read_text())["launch"]["arguments"]
    assert yaml.safe_load((off / "params.yaml").read_text())["loc_node"]["ros__parameters"]["features"]["loc_node"] is False
    # The oracle without the true poses on a topic has nothing to report: refused when built.
    broken = tmp_path / "broken.yaml"
    broken.write_text(SURVEY.read_text().replace("truth_poses: true\n", ""))
    with pytest.raises(BuildError, match="truth_poses"):
        build_run(broken, tmp_path / "broken", variant="on")


# --- the tracker's tracks against the truth (LOC-7) -------------------------------------------

MARKERS = [{"name": "marker_3", "e": 35.0, "n": 20.0}, {"name": "marker_7", "e": 62.0, "n": 31.0}, {"name": "marker_9", "e": 83.5, "n": 30.0}]
CORNERS = [(20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0)]


def a_track(track_id, east, north, state=2, outside=False, sightings=25):
    return {"id": track_id, "state": state, "e": east, "n": north, "outside": outside, "sightings": sightings}


GOOD = [a_track(1, 35.1, 20.05), a_track(2, 61.9, 31.1), a_track(3, 83.4, 30.0, outside=True)]


def test_the_trackers_list_is_held_to_the_true_markers():
    assert loc_check.TRACK_TOLERANCE_M == 1.0 and loc_check.TRACKS_TOPIC == "/loc/tracks"
    verdict = loc_check.judge_tracks(GOOD, MARKERS, CORNERS)
    assert verdict["problems"] == [] and verdict["listed"] == 2 and verdict["markers_inside"] == 2 and verdict["outside_marked"] == 1
    assert verdict["largest_error_m"] == pytest.approx(0.1414, abs=0.001) and verdict["errors_m"]["marker_3"] == pytest.approx(0.1118, abs=0.001)
    report = loc_check.build_report(a_survey(), boxes(), {"marker"}, tracks=GOOD, markers=MARKERS, corners=CORNERS)
    assert report["passed"] and report["tracks"]["listed"] == 2
    assert any("2 of 2 markers" in line and "0.14 m" in line for line in loc_check.summary_lines(report))
    # With the tracker's flag off the tracks are not asked about.
    assert "tracks" not in loc_check.build_report(a_survey(), boxes(), {"marker"}) or loc_check.build_report(a_survey(), boxes(), {"marker"})["tracks"] is None


@pytest.mark.parametrize(
    "tracks, said",
    [
        (GOOD[1:], "marker_3 has no listed track within 1 m"),
        (GOOD + [a_track(4, 35.4, 20.3)], "marker_3 has 2 listed tracks within 1 m"),
        (GOOD + [a_track(5, 50.0, 40.0)], "a listed track at 50.0, 40.0 has no marker within 1 m"),
        (GOOD[:2] + [a_track(3, 83.4, 30.0)], "marker_9 is outside the boundary markers and its track is on the list"),
        (GOOD[:2], "marker_9 is outside the boundary markers and no confirmed track of it is marked outside"),
        ([a_track(1, 35.1, 20.05, state=0), GOOD[1], GOOD[2]], "marker_3 has no listed track within 1 m"),  # tentative: not on the list
        (None, "no message was heard on /loc/tracks"),
    ],
)
def test_what_fails_the_list(tracks, said):
    verdict = loc_check.judge_tracks(tracks, MARKERS, CORNERS)
    assert any(said in problem for problem in verdict["problems"]), verdict["problems"]
    report = loc_check.build_report(a_survey(), boxes(), {"marker"}, tracks=tracks, markers=MARKERS, corners=CORNERS, tracks_wanted=True)
    assert not report["passed"]


def test_the_scenarios_true_markers_are_read_in_the_vehicles_local_frame(tmp_path):
    run = build_run(SURVEY, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    assert loc_check.tracks_wanted(manifest) and not loc_check.tracks_wanted({"node_flags": {"loc_node": {"loc_node": True}}})
    markers, corners = loc_check.true_markers(run)
    assert sorted(item["name"] for item in markers) == ["marker_1", "marker_3", "marker_7", "marker_9"]
    assert {item["name"]: (item["e"], item["n"]) for item in markers}["marker_9"] == (83.5, 30.0)
    assert sorted(corners) == [(20.0, 10.0), (20.0, 50.0), (80.0, 10.0), (80.0, 50.0)]


def test_a_scenario_that_asks_for_the_median_position_is_failed_if_the_node_says_it_reports_the_filters(tmp_path):
    """LOC-9: in a calm sim the median and the filter are centimetres apart, so the positions cannot show which was
    flown. The node's status says which, and the check holds the flight to what its scenario asked for."""
    manifest = json.loads((build_run(SURVEY, tmp_path / "run", variant="on") / "scenario.json").read_text())
    assert loc_check.median_wanted(manifest) and not loc_check.median_wanted({"node_flags": {"loc_node": {"loc_node": True, "marker_tracker": True}}})
    asked = dict(tracks=GOOD, markers=MARKERS, corners=CORNERS, tracks_wanted=True, median_wanted=True)
    told = loc_check.build_report(a_survey(track_positions="median"), boxes(), {"marker"}, **asked)
    assert told["passed"] and told["track_positions"] == "median"
    silent = loc_check.build_report(a_survey(track_positions="filter"), boxes(), {"marker"}, **asked)
    assert not silent["passed"] and any("weighted median" in problem for problem in silent["problems"])
    assert any("median" in line for line in loc_check.summary_lines(told))


def test_the_list_scenario_has_the_node_write_its_list_in_the_run_folder_where_the_runner_watches(tmp_path):
    """LOC-10: the scenario scored by the localisation judge. Its list goes where the runner's capture looks."""
    import yaml

    listing = SIM / "scenarios" / "localisation" / "survey_list.yaml"
    run = build_run(listing, tmp_path / "run", variant="on")
    manifest = json.loads((run / "scenario.json").read_text())
    settings = yaml.safe_load((run / "params.yaml").read_text())["loc_node"]["ros__parameters"]
    assert manifest["mission"] == "localisation" and manifest["list_file"] == str(run / "list" / "judges_list.csv")
    assert settings["judges_list_dir"] == str(run / "list") and (run / "list").parent.is_dir()
    on = {name for name, value in settings["features"].items() if value is True}
    assert on == {"loc_node", "sim_oracle_detector", "marker_tracker", "loc_median_position", "digit_reader", "template_digit_classifier", "digit_vote", "judges_list"}
    assert "LOC list: 3 markers" in manifest["expect_log"]
    truth = json.loads((run / "truth.json").read_text())
    assert sorted(marker["id"] for marker in truth["markers"]) == [1, 3, 7, 9] and truth["survey_area"] == [[20.0, 10.0], [80.0, 10.0], [80.0, 50.0], [20.0, 50.0]]
    # The judge can score the run's truth: a list with the three markers inside the area, each where it is.
    sys.path.insert(0, str(SIM))
    from judge.geometry import point_in_polygon

    assert [point_in_polygon(marker["e"], marker["n"], truth["survey_area"]) for marker in sorted(truth["markers"], key=lambda item: item["id"])] == [True, True, True, False]
    # Flags off: nothing writes a list, and the runner has none to watch for.
    off = build_run(listing, tmp_path / "off", variant="off")
    assert json.loads((off / "scenario.json").read_text())["list_file"] is None
    assert yaml.safe_load((off / "params.yaml").read_text())["loc_node"]["ros__parameters"]["judges_list_dir"] == ""
    # The survey pipeline's scenario, which keeps no list, is as it was.
    assert json.loads((build_run(SURVEY, tmp_path / "plain", variant="on") / "scenario.json").read_text())["list_file"] is None


def test_the_readers_the_votes_and_the_lists_own_word_is_kept_and_their_failures_fail_the_flight():
    digits = {"read": 80, "abstained": 30, "skipped": 0, "faults": 0, "last_ms": 2.1, "classifier": "template (stand-in)", "vote": {"tracks": 4, "decided": {"1": 3}, "contested": [], "faults": 0, "fault": ""}}
    listing = {"rows": 3, "unread": 0, "written": 40, "file": "judges_list.csv", "origin_fixes": 400, "error": "", "statustexts": 6, "statustext_error": ""}
    good = loc_check.build_report(a_survey(digits=digits, list=listing), boxes(), {"marker"})
    assert good["passed"] and good["digits"]["read"] == 80 and good["list"]["rows"] == 3
    plain = loc_check.build_report(a_survey(), boxes(), {"marker"})
    assert plain["passed"] and plain["digits"] is None and plain["list"] is None
    for status, said in (
        ({"digits": {"error": "the digit reader needs digit_model_path"}}, "the digit reader was asked for and did not run"),
        ({"digits": {**digits, "vote": {**digits["vote"], "faults": 2, "fault": "RuntimeError: x"}}}, "the digit vote faulted 2 time(s)"),
        ({"list": {**listing, "error": "the judges' list could not be written"}}, "the judges' list was asked for and was not kept"),
        ({"list": {**listing, "statustexts": 0, "statustext_error": "No module named 'mavros_msgs'"}}, "never went to the ground station as a STATUSTEXT: No module named"),
    ):
        report = loc_check.build_report(a_survey(**status), boxes(), {"marker"})
        assert not report["passed"] and any(said in problem for problem in report["problems"]), status
