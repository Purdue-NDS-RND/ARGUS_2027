"""PLT-14: the judges' list viewer (tools/judges_list_view.py). No ROS."""

import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import judges_list_view as view  # noqa: E402

SITE = REPO / "sim" / "sites" / "apollo_xi.yaml"
# Inside the sim site's survey area (34.1755390 to 34.1761678 north, -118.4820656 to -118.4813056 east).
GOOD = "ID,Latitude,Longitude\n3,34.1757000,-118.4818000\n7,34.1759000,-118.4815000\n0,34.1760500,-118.4819500\n"


def listed(tmp_path, text, name="judges_list.csv"):
    path = tmp_path / name
    path.write_text(text)
    return path


def test_a_clean_list_is_printed_with_each_marker_from_the_takeoff_point_and_exits_0(tmp_path, capsys):
    assert view.main([str(listed(tmp_path, GOOD)), "--site", str(SITE)]) == 0
    out = capsys.readouterr().out
    lines = out.splitlines()
    assert lines[0].startswith("ID  Latitude") and [line.split()[0] for line in lines[1:4]] == ["0", "3", "7"]  # by ID
    assert "34.1757000" in lines[2] and "yes" in lines[2] and "3 marker(s); site apollo_xi (NOT VERIFIED" in out and out.strip().endswith("LIST CLEAN: nothing found wrong")
    rows, problems = view.read_list(listed(tmp_path, GOOD))
    assert problems == [] and view.place_rows(rows, view.read_site(SITE)) == []
    three = next(row for row in rows if row["id"] == "3")
    assert three["north_m"] == pytest.approx(0.0007 * 111319.49, abs=0.05) and three["east_m"] == pytest.approx(0.0007 * 111319.49 * 0.8274, abs=0.2) and 30 < three["bearing_deg"] < 50
    # Without a site file the format is still checked, and the tool says the places were not.
    assert view.main([str(listed(tmp_path, GOOD))]) == 0 and "no site file given: the places were not checked" in capsys.readouterr().out


@pytest.mark.parametrize(
    "text, word",
    [
        ("Id,Lat,Lon\n3,34.1757,-118.4818\n", "line 1: the header must be exactly 'ID,Latitude,Longitude'"),
        (GOOD + "3,34.1758000,-118.4816000\n", "line 5: ID 3 is also on line 2"),
        (GOOD + "12,34.1758000,-118.4816000\n", "line 5: the ID '12' is not one digit"),
        (GOOD + "4,34.1758000\n", "line 5: expected ID,Latitude,Longitude and found 2 value(s)"),
        (GOOD + "4,north,west\n", "line 5: 'north,west' is not a latitude and a longitude"),
        (GOOD + "4,0.0,0.0\n", "line 5: 0.0,0.0 is not a place on the earth"),
        (GOOD + "4,134.0,-118.48\n", "line 5: 134.0,-118.48 is not a place on the earth"),
        (GOOD + "\n4,34.1758000,-118.4816000\n", "line 5: an empty line"),
        (GOOD + "4,34.1750000,-118.4825000\n", "line 5: ID 4 is outside the survey area (0 m from the takeoff point"),
        (GOOD + "4,34.1757030,-118.4818000\n", "lines 2 and 5: IDs 3 and 4 are 0.33 m apart: one marker listed twice?"),
    ],
)
def test_what_is_wrong_with_a_list_is_said_by_line_and_exits_1(tmp_path, capsys, text, word):
    assert view.main([str(listed(tmp_path, text)), "--site", str(SITE)]) == 1
    assert f"PROBLEM: {word}" in capsys.readouterr().out


def test_the_map_is_one_page_with_the_area_the_markers_and_what_is_wrong(tmp_path, capsys):
    page = tmp_path / "list.html"
    outside = GOOD + "4,34.1750000,-118.4825000\n"
    assert view.main([str(listed(tmp_path, outside)), "--site", str(SITE), "--map", str(page)]) == 1
    text = page.read_text()
    assert text.startswith("<!doctype html>") and "<svg" in text and "http://" not in text.replace('xmlns="http://www.w3.org/2000/svg"', "") and "<script" not in text  # nothing fetched
    assert text.count("<circle") == 1 + 4 and text.count('stroke="#b00" stroke-width="2"/><text') == 1  # the takeoff point and four markers, one of them in red
    assert "<li>line 5: ID 4 is outside the survey area" in text and "NOT VERIFIED" in text and ">10 m<" in text
    assert view.main([str(listed(tmp_path, GOOD)), "--map", str(page)]) == 2 and "--map needs --site" in capsys.readouterr().err


def test_an_earlier_list_shows_what_changed(tmp_path, capsys):
    earlier = listed(tmp_path, "ID,Latitude,Longitude\n3,34.1757000,-118.4818000\n7,34.1759100,-118.4815000\n5,34.1758000,-118.4816000\n", "earlier.csv")
    assert view.main([str(listed(tmp_path, GOOD)), "--against", str(earlier)]) == 0
    out = capsys.readouterr().out
    assert "Against the earlier list:\n  ID 0: new\n  ID 5: gone\n  ID 7: moved 1.11 m" in out
    assert view.main([str(listed(tmp_path, GOOD)), "--against", str(listed(tmp_path, GOOD, "same.csv"))]) == 0 and "Against the earlier list: no change" in capsys.readouterr().out


def test_a_file_that_is_not_there_or_not_a_site_is_exit_2_and_the_format_is_the_list_writers_own(tmp_path, capsys):
    assert view.main([str(tmp_path / "none.csv")]) == 2 and "No such file" in capsys.readouterr().err
    assert view.main([str(listed(tmp_path, GOOD)), "--site", str(listed(tmp_path, "name: x\n", "site.yaml"))]) == 2 and "not a site file" in capsys.readouterr().err
    writer = (REPO / "src/argus_localisation/argus_localisation/judges_list.py").read_text()
    assert f'LIST_HEADER = "{view.HEADER}"' in writer and "DECIMALS = 7" in writer
    # An empty list (the header alone) is a clean list of no markers.
    assert view.main([str(listed(tmp_path, "ID,Latitude,Longitude\n"))]) == 0 and "0 marker(s)" in capsys.readouterr().out
