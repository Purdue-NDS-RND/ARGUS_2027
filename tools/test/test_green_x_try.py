"""The green X detector's evaluation tool (REC-1), on a handful of renders."""

import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))

import green_x_try as tool  # noqa: E402


def steady(report):
    return {key: value for key, value in report.items() if key != "ms_a_frame"}


def test_a_small_run_counts_hits_misses_and_false_alarms_and_repeats_with_its_seed():
    report = tool.evaluate(count=24, negatives=16, seed=3)
    assert report["count"] == 24 and report["hits"] <= report["found"] <= 24 and report["hits"] + len(report["misses"]) == 24
    assert report["recall"] == report["hits"] / 24 and report["recall"] >= 0.9 and report["false_alarms"] == [] and report["negatives"] == 16
    assert report["buckets"] == 12 and 0 < report["buckets_found"] <= 12 and report["false_buckets"] == 0
    assert steady(report) == steady(tool.evaluate(count=24, negatives=16, seed=3))
    assert report["ms_a_frame"]["median"] > 0.0
    lines = tool.summary(report)
    assert "recall" in lines[0] and "false alarm" in lines[0] and "bucket" in lines[2]
    json.dumps(report)


def test_the_lines_are_the_specs():
    assert tool.MIN_RECALL == 0.98 and tool.MAX_ANGLE_DEG == 3.0
    assert tool.centre_limit_px(30.0) == 2.0 and abs(tool.centre_limit_px(3.0) - 0.01 * tool.ARM_SPAN_M * 778.0 / 3.0) < 1.0e-9  # max(2 px, 1% of the span)
    assert tool.verdict({"recall": 0.985, "false_alarms": []}) and not tool.verdict({"recall": 0.97, "false_alarms": []}) and not tool.verdict({"recall": 1.0, "false_alarms": ["bullseye #4"]})
    # The X repeats every quarter turn: 3 degrees off is 3 degrees off whichever arm is measured.
    assert abs(tool.angle_error_deg(0.0, 45.0)) < 1.0e-9 and abs(tool.angle_error_deg(0.0, 135.0)) < 1.0e-9 and abs(tool.angle_error_deg(0.05, 45.0) - 2.8648) < 1.0e-3


def test_the_command_line_writes_its_report(tmp_path, capsys):
    out = tmp_path / "report.json"
    code = tool.main(["--n", "10", "--negatives", "8", "--seed", "2", "--report", str(out)])
    report = json.loads(out.read_text())
    assert code == (0 if tool.verdict(report) else 1) and report["count"] == 10 and ("PASS" in capsys.readouterr().out) == tool.verdict(report)
