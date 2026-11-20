"""The bench tool for the survey's frame logging (LOC-11), on frames small enough for any machine."""

import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import frame_log_bench as bench  # noqa: E402


def test_a_short_run_logs_every_frame_and_says_what_it_cost(tmp_path):
    report = bench.run(tmp_path / "frames", width=320, height=180, rate_hz=30.0, seconds=1.0, workers=2, quality=90, min_free_gb=0.0)
    assert report["offered"] == 30 and report["written"] == 30 and report["dropped"] == 0 and report["thinned"] == 0
    assert report["every_frame_logged"] is True and report["keep_every_at_end"] == 1
    assert report["megabytes"] > 0.0 and report["megabytes_per_second"] == pytest.approx(report["megabytes"] / report["wall_s"], rel=1.0e-6)
    assert report["frame_megabytes"] == pytest.approx(report["megabytes"] / 30, rel=1.0e-6) and report["cpu_cores_used"] > 0.0
    folder = Path(report["folder"])
    assert len(list(folder.glob("frame_*.jpg"))) == 30
    lines = bench.summary(report)
    assert any("30 of 30 frames" in line for line in lines) and any("MB a second" in line for line in lines)
    assert report["jpeg_library"] and any("JPEG library" in line for line in lines)
    assert bench.run.__defaults__[:3] == (3840, 2160, 17.0) and bench.run.__defaults__[-1] == 20.0  # the 4K mode's own size and rate; the flight's free-space floor
    json.dumps(report)


def test_the_frames_are_textured_so_that_their_size_means_something():
    flat, textured = bench.test_frame(320, 180, 0, textured=False), bench.test_frame(320, 180, 0, textured=True)
    import cv2

    size = lambda image: len(cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1])  # noqa: E731
    assert size(textured) > 5 * size(flat) and textured.shape == (180, 320, 3)
    assert not (bench.test_frame(320, 180, 1) == textured).all()  # and they differ frame to frame


def test_the_command_line_writes_the_report_and_fails_a_run_that_lost_frames(tmp_path, capsys):
    out = tmp_path / "report.json"
    code = bench.main(["--width", "320", "--height", "180", "--rate", "30", "--seconds", "0.5", "--workers", "1", "--min-free-gb", "0", "--out", str(tmp_path / "frames"), "--report", str(out)])
    assert code == 0 and json.loads(out.read_text())["written"] == 15 and "every frame logged" in capsys.readouterr().out
    assert bench.verdict({"offered": 100, "written": 80, "dropped": 20, "thinned": 0}) == (False, "20 of 100 frames were not logged (20 dropped at the log's queue, 0 thinned)")
