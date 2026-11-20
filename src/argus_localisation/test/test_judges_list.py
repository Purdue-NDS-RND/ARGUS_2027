"""The judges' list (LOC-10): tracks to ID,Latitude,Longitude rows, written so that a crash never leaves half a file."""

import csv
import io
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC.parent / "sim")]

from argus_localisation import judges_list as jl  # noqa: E402
from drone_mission_core.target_access import latlon_to_local_enu  # noqa: E402

HOME = (40.4237000, -86.9212000)  # where the local frame's (0, 0) is
CONFIRMED, COASTING, TENTATIVE = 1, 2, 0


def state(track_id, digit, east, north, class_id="marker", status=CONFIRMED, outside=False):
    return SimpleNamespace(track_id=track_id, class_id=class_id, state=status, outside=outside, digit=digit, reported_east_m=east, reported_north_m=north, east_m=east + 5.0, north_m=north + 5.0)


def rows(*specs):
    return jl.rows_from_tracks([state(*spec) for spec in specs], HOME)[0]


def test_a_local_position_goes_to_latitude_and_longitude_and_back():
    latitude, longitude = jl.local_to_latlon(62.0, 31.0, *HOME)
    assert latlon_to_local_enu(latitude, longitude, HOME[0], HOME[1], 0.0, 0.0) == pytest.approx((62.0, 31.0), abs=1.0e-4)  # the mission's own conversion, backwards
    # The sim's judge reads the list with ArduPilot's arithmetic: the same place to a centimetre.
    import target_placement

    north, east = target_placement.offset_ne_m(HOME[0], HOME[1], latitude, longitude)
    assert (east, north) == pytest.approx((62.0, 31.0), abs=0.02)
    assert jl.local_to_latlon(0.0, 0.0, *HOME) == pytest.approx(HOME, abs=1.0e-12)
    for nonsense in ((float("nan"), 0.0, *HOME), (0.0, 0.0, 95.0, 0.0), (0.0, 0.0, 40.0, 200.0), (0.0, None, *HOME)):
        assert jl.local_to_latlon(*nonsense) is None


def test_the_local_frames_origin_is_the_median_of_what_the_fixes_say():
    origin = jl.OriginEstimate()
    assert origin.origin() is None
    rng = np.random.default_rng(3)
    for index in range(40):
        east, north = 4.0 * index, 10.0 + 0.5 * index  # flying away from home
        latitude, longitude = jl.local_to_latlon(east + float(rng.normal(0.0, 0.05)), north + float(rng.normal(0.0, 0.05)), *HOME)
        assert origin.add(latitude, longitude, east, north)
        if index + 1 < jl.MIN_ORIGIN_PAIRS:
            assert origin.origin() is None  # not on a fix or two
    for _ in range(4):  # a few fixes paired with the wrong moment's position, 3 m out
        latitude, longitude = jl.local_to_latlon(50.0 + 3.0, 20.0, *HOME)
        origin.add(latitude, longitude, 50.0, 20.0)
    found = origin.origin()
    assert latlon_to_local_enu(found[0], found[1], HOME[0], HOME[1], 0.0, 0.0) == pytest.approx((0.0, 0.0), abs=0.05) and origin.pairs == 44
    assert not origin.add(float("nan"), -86.9, 0.0, 0.0) and not origin.add(95.0, 0.0, 0.0, 0.0) and not origin.add(40.0, -86.9, None, 0.0) and origin.pairs == 44
    for _ in range(2 * jl.MAX_ORIGIN_PAIRS):
        origin.add(*HOME, 0.0, 0.0)
    assert origin.pairs == jl.MAX_ORIGIN_PAIRS


def test_the_list_is_the_listed_tracks_that_have_a_digit_at_their_reported_positions():
    tracks = [
        state(1, 3, 35.0, 20.0),
        state(2, 7, 62.0, 31.0, status=COASTING),  # confirmed, not seen lately: still on the list
        state(3, 1, 48.0, 44.0),
        state(4, -1, 30.0, 40.0),  # confirmed, no digit yet
        state(5, 9, 83.5, 30.0, outside=True),  # beyond the corner markers
        state(6, 4, 55.0, 25.0, status=TENTATIVE),  # seen once or twice
        state(7, -1, 20.0, 10.0, class_id="boundary_marker"),
    ]
    listed, unread = jl.rows_from_tracks(tracks, HOME)
    assert [(row.marker_id, row.track_id) for row in listed] == [("1", 3), ("3", 1), ("7", 2)]  # by ID
    assert (listed[1].east_m, listed[1].north_m) == (35.0, 20.0)  # the reported position (LOC-9), not the filter's
    assert (listed[1].latitude, listed[1].longitude) == pytest.approx(jl.local_to_latlon(35.0, 20.0, *HOME))
    assert [(item.track_id, item.east_m, item.north_m) for item in unread] == [(4, 30.0, 40.0)]
    # With no origin there is no latitude to give: no rows, and every listed track is unread.
    nowhere, waiting = jl.rows_from_tracks(tracks, None)
    assert nowhere == [] and [item.track_id for item in waiting] == [1, 2, 3, 4]
    # IDs that repeat (the vote's unique step switched off) are both rows, in track order.
    assert [(row.marker_id, row.track_id) for row in rows((9, 5, 1.0, 1.0), (8, 5, 9.0, 9.0))] == [("5", 8), ("5", 9)]


def test_the_csv_is_the_judges_header_and_seven_decimal_places():
    text = jl.format_csv(rows((1, 3, 35.0, 20.0), (2, 7, 62.0, 31.0)))
    lines = text.splitlines()
    assert lines[0] == "ID,Latitude,Longitude" == jl.LIST_HEADER and len(lines) == 3 and text.endswith("\n")
    for line, (east, north) in zip(lines[1:], ((35.0, 20.0), (62.0, 31.0))):
        marker_id, latitude, longitude = line.split(",")
        assert len(latitude.split(".")[1]) == 7 and len(longitude.split(".")[1]) == 7
        assert latlon_to_local_enu(float(latitude), float(longitude), HOME[0], HOME[1], 0.0, 0.0) == pytest.approx((east, north), abs=0.02)  # 7 places: a centimetre
    assert [line.split(",")[0] for line in lines[1:]] == ["3", "7"]
    assert jl.format_csv([]) == "ID,Latitude,Longitude\n"  # an empty list is still a list
    # The sim's judge reads it.
    from judge.missions import LIST_HEADER, parse_list

    assert LIST_HEADER == jl.LIST_HEADER
    parsed = parse_list(text, {"latitude_deg": HOME[0], "longitude_deg": HOME[1]})
    assert [entry[0] for entry in parsed] == ["3", "7"] and parsed[0][1:] == pytest.approx((35.0, 20.0), abs=0.02)


def test_the_kml_has_a_placemark_for_each_row():
    listed = rows((1, 3, 35.0, 20.0), (2, 7, 62.0, 31.0))
    root = ET.fromstring(jl.format_kml(listed))
    space = {"k": "http://www.opengis.net/kml/2.2"}
    marks = root.findall(".//k:Placemark", space)
    assert [mark.findtext("k:name", namespaces=space) for mark in marks] == ["3", "7"]
    longitude, latitude, height = marks[0].findtext("k:Point/k:coordinates", namespaces=space).split(",")
    assert (float(latitude), float(longitude), float(height)) == pytest.approx((listed[0].latitude, listed[0].longitude, 0.0), abs=1.0e-7)  # KML is longitude first
    assert ET.fromstring(jl.format_kml([])).findall(".//k:Placemark", space) == []


def test_the_map_shows_each_marker_where_it_is():
    listed, unread = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0), state(4, -1, 30.0, 40.0)], HOME)
    boundary = [(20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0)]
    picture, place = jl.render_map(listed, unread, boundary)
    assert picture.shape == (jl.MAP_PX[1], jl.MAP_PX[0], 3) and picture.dtype == np.uint8
    # `place` is the map's own scale: a marker's dot is at its place, north up and east right.
    (x3, y3), (x7, y7) = place(35.0, 20.0), place(62.0, 31.0)
    assert x7 > x3 and y7 < y3  # further east is further right; further north is further up
    for x, y in ((x3, y3), (x7, y7)):
        assert tuple(int(v) for v in picture[int(round(y)), int(round(x))]) == jl.MAP_MARKER_BGR
    assert tuple(int(v) for v in picture[int(round(place(30.0, 40.0)[1])), int(round(place(30.0, 40.0)[0]))]) == jl.MAP_UNREAD_BGR
    # Nothing found yet: still a picture, of the area.
    assert jl.render_map([], [], boundary)[0].shape == picture.shape and jl.render_map([], [], [])[0].shape == picture.shape


def test_the_status_text_is_the_count_and_fits_a_mavlink_statustext():
    listed, unread = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0), state(4, -1, 30.0, 40.0)], HOME)
    assert jl.status_text(listed, unread) == "LOC list: 2 markers, 1 more unread"
    assert jl.status_text(listed[:1], []) == "LOC list: 1 marker"
    assert jl.status_text([], []) == "LOC list: 0 markers"
    assert len(jl.status_text(listed * 60, unread * 60)) <= jl.STATUSTEXT_MAX_CHARS == 50
    # The ground station is told when the list is not reaching the disk, not shown a count as if it were.
    assert jl.status_text(listed, unread, not_written=True) == "LOC list NOT WRITTEN: 2 found" and jl.STATUSTEXT_PERIOD_S == 2.0


def test_a_file_is_replaced_whole_or_not_at_all(tmp_path, monkeypatch):
    target = tmp_path / "judges_list.csv"
    jl.write_atomic(target, b"ID,Latitude,Longitude\n3,40.4238797,-86.9207866\n")
    assert target.read_bytes().startswith(b"ID,") and [item.name for item in tmp_path.iterdir()] == ["judges_list.csv"]  # no temp file left
    # The write of the new text dies half way (the disk fills, the power goes): the old file is whole and still there.
    real_replace = os.replace
    monkeypatch.setattr(os, "replace", lambda source, destination: (_ for _ in ()).throw(OSError("no space left on device")))
    with pytest.raises(OSError):
        jl.write_atomic(target, b"ID,Latitude,Longitude\n")
    assert target.read_bytes().endswith(b"-86.9207866\n") and [item.name for item in tmp_path.iterdir()] == ["judges_list.csv"]
    # And it is written beside the file, then renamed onto it: a rename within one folder is all or nothing.
    seen = []
    monkeypatch.setattr(os, "replace", lambda source, destination: seen.append((Path(source).parent, Path(destination))) or real_replace(source, destination))
    jl.write_atomic(target, b"new")
    assert seen == [(tmp_path, target)] and target.read_bytes() == b"new"


def test_the_list_is_rewritten_when_it_changes_and_only_then(tmp_path):
    out = jl.JudgesList(tmp_path / "list")
    boundary = [(20.0, 10.0), (80.0, 10.0), (80.0, 50.0), (20.0, 50.0)]
    first, unread = jl.rows_from_tracks([state(1, 3, 35.0, 20.0)], HOME)
    assert out.write(first, unread, boundary) is True and out.written == 1 and out.error == ""
    names = sorted(item.name for item in (tmp_path / "list").iterdir())
    assert names == ["judges_list.csv", "judges_list.kml", "judges_list.png"] == sorted((jl.CSV_NAME, jl.KML_NAME, jl.MAP_NAME))
    assert out.csv_path.read_text() == jl.format_csv(first)
    assert cv2.imread(str(tmp_path / "list" / jl.MAP_NAME)).shape == (jl.MAP_PX[1], jl.MAP_PX[0], 3)
    stamp = out.csv_path.stat().st_mtime_ns
    assert out.write(first, unread, boundary) is False and out.written == 1 and out.csv_path.stat().st_mtime_ns == stamp  # nothing changed
    second, unread = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0)], HOME)
    assert out.write(second, unread, boundary) is True and out.written == 2 and list(csv.reader(io.StringIO(out.csv_path.read_text())))[2][0] == "7"
    # A folder that cannot be made, and a write that fails, are said and do not raise: the survey goes on.
    blocked = tmp_path / "file"
    blocked.write_text("in the way")
    bad = jl.JudgesList(blocked / "list")
    assert bad.write(first, [], boundary) is False and "judges' list" in bad.error and bad.written == 0
    with pytest.raises(ValueError, match="judges_list_dir"):
        jl.JudgesList("")


def test_the_writer_keeps_the_newest_list_and_writes_it_off_the_callers_thread(tmp_path):
    out = jl.JudgesList(tmp_path / "list")
    writer = jl.ListWriter(out, period_s=0.05)
    one, _ = jl.rows_from_tracks([state(1, 3, 35.0, 20.0)], HOME)
    two, _ = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0)], HOME)
    writer.offer(one, [], [])
    writer.offer(two, [], [])  # before the writer's turn: only the newest is written
    assert not out.csv_path.exists()  # offering writes nothing
    assert writer.flush() is True and out.written == 1 and out.csv_path.read_text() == jl.format_csv(two)
    assert writer.flush() is False  # nothing new
    # Its own thread does the same once a period, and a stop writes what is waiting before it returns.
    writer.start()
    three, _ = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0), state(3, 1, 48.0, 44.0)], HOME)
    writer.offer(three, [], [])
    writer.stop(wait_s=5.0)
    assert out.csv_path.read_text() == jl.format_csv(three) and not writer.running
    writer.stop(wait_s=1.0)  # twice is no fault


def test_the_running_writer_puts_a_changed_list_on_the_disk_within_its_period_with_no_stop(tmp_path):
    """The spec audit: the write in the middle of a survey is what is left if the node dies. Nothing here stops or wakes the writer."""
    import time

    out = jl.JudgesList(tmp_path / "list")
    writer = jl.ListWriter(out, period_s=0.05)
    writer.start()
    try:
        for count in (1, 2):
            listed, _ = jl.rows_from_tracks([state(index, index, 30.0 + index, 20.0) for index in range(1, count + 1)], HOME)
            writer.offer(listed, [], [])
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and out.written < count:  # (counted once the CSV, the KML and the map are all down)
                time.sleep(0.01)
            assert out.csv_path.read_text() == jl.format_csv(listed) and out.written == count and writer.running
    finally:
        writer.stop(wait_s=5.0)
    assert jl.WRITE_PERIOD_S == 1.0 and jl.ListWriter(out).period_s == 1.0  # the node's: the list on the disk is never more than a second old


def test_a_list_already_in_the_folder_is_put_aside_not_written_over(tmp_path):
    """The flight-safety review: a node restarted in the middle of a survey has no tracks, and its first list is a header alone."""
    folder = tmp_path / "list"
    first = jl.JudgesList(folder)
    three, _ = jl.rows_from_tracks([state(1, 3, 35.0, 20.0), state(2, 7, 62.0, 31.0), state(3, 1, 48.0, 44.0)], HOME)
    first.write(three, [], [])
    assert first.kept_earlier == ""  # nothing was there
    restarted = jl.JudgesList(folder)
    assert restarted.write([], [], []) is True and restarted.csv_path.read_text() == "ID,Latitude,Longitude\n"
    kept = Path(restarted.kept_earlier)
    assert kept.parent == folder and kept.name.startswith("judges_list.earlier-") and kept.read_text() == jl.format_csv(three)
    # Its own later writes put nothing aside, and an earlier list with no rows is not worth keeping.
    restarted.write(three[:1], [], [])
    again = jl.JudgesList(folder)
    again.write(three[:1], [], [])
    assert len(list(folder.glob("judges_list.earlier-*.csv"))) == 2  # the first run's three rows, and the restarted run's one
    empty = tmp_path / "empty"
    jl.JudgesList(empty).write([], [], [])
    jl.JudgesList(empty).write([], [], [])
    assert list(empty.glob("judges_list.earlier-*.csv")) == []
