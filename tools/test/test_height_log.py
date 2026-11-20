"""tools/height_log.py: the height estimator run over a recorded flight (CEN-03's bench and field check). No ROS."""

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "height_log.py"
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "sim" / "test"))

import height_log  # noqa: E402
import replay_flight as flight  # noqa: E402
from replay import bag  # noqa: E402

MS, S, T0 = flight.MS, flight.S, flight.T0


def records(heights, ground_up=0.0, range_bias_m=0.0, ranges=True):
    """Poses at 30 Hz through the heights given (above home), with a rangefinder reading beside each, over ground `ground_up` above home."""
    out = []
    for index, height in enumerate(heights):
        ns = T0 + index * 33 * MS
        out.append(flight.pose_record(ns, up=height))
        if ranges:
            out.append(flight.range_record(ns, range_m=height - ground_up + range_bias_m))
    return sorted(out, key=lambda record: record.log_time_ns)


def descent_then_hover(top=15.0, hover=2.4, hover_s=10.0):
    heights = [top] * 60
    while heights[-1] > hover:
        heights.append(max(hover, heights[-1] - 0.5 / 30.0))
    return heights + [hover] * int(hover_s * 30)


def write(tmp_path, name, recs):
    return bag.write_bag(tmp_path / name, recs, storage="sqlite3")


def test_a_recorded_descent_shows_the_source_by_height_and_why_the_barometer_was_used(tmp_path):
    report = height_log.analyse(write(tmp_path, "descent", records(descent_then_hover())))
    assert report["poses"] > 900 and report["range_readings"] > 900
    bands = {band["band"]: band for band in report["bands"]}
    assert bands["12 m and over"]["rangefinder_share"] == 0.0 and bands["12 m and over"]["heights"] >= 60
    assert bands["0 to 3 m"]["rangefinder_share"] == 1.0 and bands["6 to 9 m"]["rangefinder_share"] == 1.0
    assert report["baro_reasons"].get("out of range", 0) >= 60
    # Flat ground at home's height: the rangefinder and the height above home agree.
    assert abs(report["rangefinder_minus_home"]["median_m"]) < 0.01 and report["rangefinder_minus_home"]["spread_p95_m"] < 0.01
    assert report["tape"] is None and report["problems"] == []


def test_ground_that_is_not_at_homes_height_shows_as_the_difference_between_the_two_sources(tmp_path):
    report = height_log.analyse(write(tmp_path, "slope", records(descent_then_hover(hover=4.0), ground_up=1.5)))
    assert abs(report["rangefinder_minus_home"]["median_m"] + 1.5) < 0.01


def test_a_hover_over_a_taped_height_passes_within_10_cm_and_fails_beyond(tmp_path):
    good = height_log.analyse(write(tmp_path, "good", records(descent_then_hover(), range_bias_m=0.04)), tape_m=2.4, last_s=8.0)
    assert good["tape"]["passed"] is True and abs(good["tape"]["median_m"] - 2.44) < 0.01 and good["tape"]["heights"] > 200
    bad = height_log.analyse(write(tmp_path, "bad", records(descent_then_hover(), range_bias_m=0.2)), tape_m=2.4, last_s=8.0)
    assert bad["tape"]["passed"] is False
    assert any("0.20 m from the taped 2.4 m" in problem for problem in bad["problems"])


def test_a_taped_hover_on_the_barometer_is_a_failure_not_a_pass(tmp_path):
    report = height_log.analyse(write(tmp_path, "silent", records(descent_then_hover(), ranges=False)), tape_m=2.4, last_s=8.0)
    assert report["tape"]["passed"] is False and report["tape"]["heights"] == 0
    assert any("no height from the rangefinder" in problem for problem in report["problems"])
    assert report["baro_reasons"] == {"no reading": report["poses"]}


def test_a_bag_without_poses_says_so(tmp_path):
    report = height_log.analyse(write(tmp_path, "empty", [flight.range_record(T0)]))
    assert report["poses"] == 0 and "no pose" in report["problems"][0]


def test_the_command_line(tmp_path):
    path = write(tmp_path, "cli", records(descent_then_hover(), range_bias_m=0.04))
    out = tmp_path / "height.json"
    done = subprocess.run([sys.executable, str(TOOL), str(path), "--tape", "2.4", "--last", "8", "--out", str(out)], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    assert "0 to 3 m" in done.stdout and "100% from the rangefinder" in done.stdout
    assert "taped height 2.4 m: PASS, median 2.44 m" in done.stdout
    assert json.loads(out.read_text())["tape"]["passed"] is True
    failed = subprocess.run([sys.executable, str(TOOL), str(write(tmp_path, "cli_bad", records(descent_then_hover(), range_bias_m=0.2))), "--tape", "2.4", "--last", "8"], capture_output=True, text=True)
    assert failed.returncode == 1 and "FAIL" in failed.stdout
    assert subprocess.run([sys.executable, str(TOOL), str(tmp_path / "nothing_here")], capture_output=True, text=True).returncode == 2
