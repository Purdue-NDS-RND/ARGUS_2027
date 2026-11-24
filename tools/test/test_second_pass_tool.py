"""LOC-12: tools/second_pass.py, the post-landing second pass as it is run: a logged survey in, the merged judges' list out."""

import math
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path[:0] = [str(TOOLS), str(REPO / "src" / "argus_localisation" / "test"), str(REPO / "src" / "argus_localisation"), str(REPO / "src" / "drone_mission_core"), str(REPO / "src" / "argus_camera")]

import second_pass as tool  # noqa: E402  (tools/second_pass.py)
import test_second_pass as made  # noqa: E402  (the logged survey and its plate detector)
from argus_localisation import marker_tracker  # noqa: E402
from argus_localisation import second_pass as sp  # noqa: E402
from drone_mission_core.frame_projection import load_camera  # noqa: E402

ORIGIN = (34.175, -118.4825)
CAMERA = ["--camera-mode", "sim_delivery", "--airframe", "sim_iris_downcam"]


@pytest.fixture(scope="module")
def logged(tmp_path_factory):
    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    folder = made.survey(tmp_path_factory.mktemp("survey") / "survey_000001000", camera, mount)
    live = [
        made.track(1, 4.3, 0.5, sightings=6, digit=4, votes=made.reading(4, weight=3.0), weight=3.0, readings=4),  # plate a, 0.3 m off, read as 4
        made.track(2, 60.0, 0.0, sightings=5),  # something the flight listed that no logged frame shows
    ]
    sp.save_live_tracks(folder / sp.LIVE_TRACKS_NAME, live, ORIGIN)
    return folder


def test_the_logged_survey_is_looked_at_again_and_the_merged_list_is_written_with_what_changed(logged, tmp_path, capsys):
    out = tmp_path / "second_list"
    assert tool.main([str(logged), *CAMERA, "--out", str(out)], detector=made.PlateDetector()) == 0
    said = capsys.readouterr().out
    assert "second pass: 40 of 40 logged frame(s) looked at" in said and "(every frame was looked at)" in said
    assert "merged with the flight's 2 marker track(s): 1 row(s) in the list, 3 marker(s) found and not read" in said
    assert "new: a marker at (12.0, -1.0) m, not read" in said and "new: a marker at (20.0, 1.5) m, not read" in said
    rows = (out / "judges_list.csv").read_text().splitlines()
    assert rows[0] == "ID,Latitude,Longitude" and len(rows) == 2 and rows[1].startswith("4,34.17500")
    # The row's place is between the flight's and the second pass's, nearer the second pass's (it saw the plate more often).
    latitude, longitude = (float(value) for value in rows[1].split(",")[1:])
    east = (longitude - ORIGIN[1]) * 111_319.49 * math.cos(math.radians(ORIGIN[0]))
    assert 4.0 < east < 4.15 and (out / "judges_list.kml").is_file()
    assert "tools/judges_list_view.py" in said and "--against" in said
    # A list already in the folder is kept under another name, not written over.
    assert tool.main([str(logged), *CAMERA, "--out", str(out)], detector=made.PlateDetector()) == 0
    assert len([path for path in out.iterdir() if path.name.startswith("judges_list") and path.suffix == ".csv"]) == 2


def test_no_marker_the_flight_listed_is_lost_and_tracks_keep_their_states(logged, tmp_path):
    chain, poses, ground, tracker, votes = tool.build_chain(*load_camera("sim_delivery", "sim_iris_downcam"), made.PlateDetector())
    assert votes is None and chain.tiling is True  # no digit model: nothing is read, and the frames are tiled
    sp.SecondPass(chain, poses, ground, __import__("cv2").imread).run(logged)
    live, _origin, _settings = sp.load_live_tracks(logged / sp.LIVE_TRACKS_NAME)
    merged = {item.track_id: item for item in sp.merge(live, sp.kept(tracker.tracks(), None, source="second pass"))}
    assert merged[2] == live[1] and merged[1].source == "both" and merged[1].digit == 4 and len(merged) == 4
    assert all(item.state in (marker_tracker.CONFIRMED, marker_tracker.COASTING) for item in merged.values())


def test_what_stops_the_pass_is_said_and_the_flights_list_stands(logged, tmp_path, capsys):
    for arguments, word in (
        ([str(tmp_path / "nowhere"), *CAMERA, "--out", str(tmp_path / "o")], "is not the flight's tracks"),
        ([str(logged), *CAMERA, "--out", str(tmp_path / "o")], "--engine is needed"),
        ([str(logged), *CAMERA, "--out", str(tmp_path / "o"), "--engine", str(tmp_path / "none.engine")], "none.engine"),
        ([str(logged), "--camera-mode", "no_such_mode", "--airframe", "sim_iris_downcam", "--out", str(tmp_path / "o"), "--engine", "x"], "no_such_mode"),
    ):
        assert tool.main(arguments) == 2
        said = capsys.readouterr().err
        assert word in said and "The flight's own list stands." in said and not (tmp_path / "o").exists()
    assert tool.main([str(logged), *CAMERA, "--out", str(tmp_path / "o"), "--budget-s", "1200"], detector=made.PlateDetector()) == 2
    assert "no more than 900" in capsys.readouterr().err
    assert tool.main([str(logged), *CAMERA, "--out", str(tmp_path / "missing" / "o")], detector=made.PlateDetector()) == 2
    assert "is not a folder that exists" in capsys.readouterr().err


def test_within_a_short_budget_fewer_frames_are_looked_at_and_a_list_is_still_written(logged, tmp_path, capsys):
    ticks = [0.0]

    def clock():
        ticks[0] += 0.5  # every look at the clock is half a second later: a slow computer
        return ticks[0]

    assert tool.main([str(logged), *CAMERA, "--out", str(tmp_path / "short"), "--budget-s", "60"], detector=made.PlateDetector(), clock=clock) == 0
    said = capsys.readouterr().out
    looked = int(said.split("second pass: ")[1].split(" of 40")[0])
    assert 1 <= looked < 40 and "(the budget" in said and (tmp_path / "short" / "judges_list.csv").is_file() and ticks[0] <= 60.0 + 5.0


def test_time_only_says_what_reading_and_tiling_cost_and_how_many_frames_fit_and_writes_nothing(logged, tmp_path, capsys):
    before = sorted(path.name for path in logged.iterdir())
    assert tool.main([str(logged), *CAMERA, "--time-only", "--tile-ms", "25"]) == 0
    said = capsys.readouterr().out.splitlines()
    assert said[0].startswith("second pass: 40 of 40 logged frame(s) looked at") and "0 box(es)" in said[0]
    assert "tile(s) a frame (640 x 360)" in said[1] and said[2].startswith("with a detector at 25 ms a tile:") and said[2].endswith(": all 40 of this survey")
    assert sorted(path.name for path in logged.iterdir()) == before
    # A detector slow enough that the survey does not fit: said as every nth frame.
    assert tool.main([str(logged), *CAMERA, "--time-only", "--tile-ms", "60000"]) == 0
    assert "th of this survey's 40" in capsys.readouterr().out
    assert tool.main([str(logged), *CAMERA]) == 2 and "--out is needed" in capsys.readouterr().err


class PlateBoxer:
    """A detector for a marker plate on the synthetic ground: the one large bright blob of the picture (or of the tile) it is handed."""

    name = "plate boxer"

    def detect(self, image, stamp_ns):
        import cv2
        import numpy as np

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        mask = cv2.morphologyEx((grey > 150).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))  # the plate with its digit closed over
        _count, _labels, stats, _centres = cv2.connectedComponentsWithStats(mask)
        from drone_mission_core import detection_contract as contract

        return [contract.Detection("marker", 0.9, x + w / 2.0, y + h / 2.0, float(w), float(h)) for x, y, w, h, area in stats[1:] if area >= 8000 and 0.7 <= w / h <= 1.4]


def test_a_digit_read_in_the_second_pass_reaches_the_merged_list(tmp_path, capsys):
    """The spec audit: the list is ID, latitude, longitude, so a digit is the only way the pass adds or changes a row, and no test read one."""
    import json

    import cv2
    import synth_targets
    from argus_localisation import digit_reader

    camera, mount = load_camera("sim_survey", "sim_iris_downcam")
    options = synth_targets.RenderOptions(width=camera.width, height=camera.height, mpp=0.0045, rotation_deg=25.0, background="noise", supersample=2)
    picture, _drawn = synth_targets.render(synth_targets.SurveyMarker(digit=7), options, seed=3)  # the marker under the camera, as the node's own vote test has it
    folder = tmp_path / "survey_000002000"
    folder.mkdir()
    height = 0.0045 * camera.fx
    record, index = sp.FlightRecord(folder), []
    for tick in range(40):
        stamp = 2000 * 1_000_000_000 + tick * 1_000_000_000 // 30
        record.pose(stamp, (5.0, 3.0, height), (0.0, 0.0, 0.0, 1.0))
        record.velocity(stamp, (0.0, 0.0, 0.0))
        if tick % 6 == 3:
            name = f"frame_{len(index):06d}.png"
            cv2.imwrite(str(folder / name), picture)
            index.append(json.dumps({"seq": len(index), "capture_ns": stamp, "mode": "sim_survey", "file": name, "width": camera.width, "height": camera.height}))
    record.close(wait_s=10.0)
    (folder / sp.INDEX_NAME).write_text("\n".join(index) + "\n")
    # The flight saw the marker three times and read nothing (its track is listed, with no digit).
    sp.save_live_tracks(folder / sp.LIVE_TRACKS_NAME, [made.track(1, 5.1, 3.0, sightings=3)], ORIGIN)
    out = tmp_path / "list"
    assert tool.main([str(folder), "--camera-mode", "sim_survey", "--airframe", "sim_iris_downcam", "--out", str(out)], detector=PlateBoxer(), digit_classifier=digit_reader.TemplateDigitClassifier()) == 0
    said = capsys.readouterr().out
    assert "1 row(s) in the list, 0 marker(s) found and not read" in said and "track 1: digit not read -> 7" in said
    rows = (out / "judges_list.csv").read_text().splitlines()
    assert len(rows) == 2 and rows[1].startswith("7,34.1750")
