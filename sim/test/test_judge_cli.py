"""sim/judge/cli.py: the command the scenario runner calls after each flight.

It reads a run folder (judge.yaml, truth.json, trace.jsonl and, for
localisation, the judges' list) and writes score.json. No ROS needed.
"""

import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "sim" / "judge" / "cli.py"

TRUTH = {
    "origin": {"latitude_deg": 34.1760, "longitude_deg": -118.4810, "elevation_m": 216.0},
    "takeoff": {"e": 0.0, "n": 0.0},
    "target": {"name": "delivery_target", "e": 10.0, "n": 15.0},
    "payload": {"name": "package", "mass_kg": 0.5},
    "markers": [{"name": "m1", "id": 4, "e": 0.0, "n": 30.0}],
}


def trace_lines(vehicle_rest=0.19):
    """Delivery as the 2026 mission flies it: land on the target, fake the release, relaunch, RTL."""
    lines = [{"t": 1.0, "event": "armed"}]
    knots = [(0, 0, 0, 0), (10, 0, 0, 15), (30, 10.1, 15.0, 15), (60, 10.1, 15.0, 0), (70, 10.1, 15.0, 0), (80, 10.1, 15.0, 15), (100, 0, 0, 15), (115, 0, 0, 0)]
    for (t0, e0, n0, u0), (t1, e1, n1, u1) in zip(knots, knots[1:]):
        for i in range(10):
            f = i / 10
            lines.append(
                {"t": t0 + (t1 - t0) * f, "pose": "vehicle", "e": e0 + (e1 - e0) * f, "n": n0 + (n1 - n0) * f, "u": vehicle_rest + u0 + (u1 - u0) * f}
            )
    lines.append({"t": 115.0, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": vehicle_rest})
    lines.append({"t": 65.0, "event": "release", "fake": True})
    lines.append({"t": 100.0, "event": "list_written"})
    lines.append({"t": 118.0, "event": "disarmed"})
    return lines


def write_run(folder: Path, judge: dict, lines=None) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "judge.yaml").write_text(yaml.safe_dump(judge))
    (folder / "truth.json").write_text(json.dumps(TRUTH))
    with open(folder / "trace.jsonl", "w") as handle:
        for line in lines if lines is not None else trace_lines():
            handle.write(json.dumps(line) + "\n")
    return folder


def run_cli(*args):
    return subprocess.run([sys.executable, str(CLI), *map(str, args)], capture_output=True, text=True)


def test_cli_scores_a_delivery_run_and_writes_score_json(tmp_path):
    run = write_run(tmp_path / "run", {"mission": "delivery", "vehicle_ground_offset_m": 0.19})
    result = run_cli(run)
    assert result.returncode == 0, result.stderr
    score = json.loads((run / "score.json").read_text())
    assert score["mission"] == "delivery"
    assert score["details"]["scored_from"] == "vehicle at release"
    assert abs(score["details"]["distance_m"] - 0.1) < 1e-6
    # 13 - 0.1, a quarter point for the 500 g package, then the 25% autonomy bonus.
    assert abs(score["points"] - 1.25 * (13.0 - 0.1 + 0.25)) < 1e-6
    assert score["common"]["single_flight"] is True
    assert "delivery" in result.stdout and "points" in result.stdout


def test_cli_passes_settings_from_judge_yaml(tmp_path):
    run = write_run(
        tmp_path / "run",
        {"mission": "delivery", "vehicle_ground_offset_m": 0.19, "settings": {"max_points": 20.0, "autonomy_bonus_fraction": 0.0}},
    )
    assert run_cli(run).returncode == 0
    score = json.loads((run / "score.json").read_text())
    assert abs(score["points"] - (20.0 - 0.1 + 0.25)) < 1e-6


def test_cli_reads_the_judges_list_for_localisation(tmp_path):
    run = write_run(tmp_path / "run", {"mission": "localisation", "vehicle_ground_offset_m": 0.19, "settings": {"height_penalty_below_m": 0.0, "height_zero_below_m": 0.0}})
    # 30 m north of the origin, as ArduPilot's flat-earth formula has it.
    lat = TRUTH["origin"]["latitude_deg"] + 30.0 / 111318.84502145034
    (run / "judges_list.csv").write_text(f"ID,Latitude,Longitude\n4,{lat:.7f},{TRUTH['origin']['longitude_deg']:.7f}\n")
    result = run_cli(run)
    assert result.returncode == 0, result.stderr
    score = json.loads((run / "score.json").read_text())
    assert score["details"]["correct"] == 1
    assert score["details"]["merged"] is None


def test_cli_scores_a_missing_judges_list_as_zero(tmp_path):
    run = write_run(tmp_path / "run", {"mission": "localisation"})
    assert run_cli(run).returncode == 0
    score = json.loads((run / "score.json").read_text())
    assert score["points"] == 0.0
    assert any("judges_list.csv" in note for note in score["notes"])


def test_cli_fails_clearly_on_a_broken_run_folder(tmp_path):
    missing = run_cli(tmp_path / "nowhere")
    assert missing.returncode == 2
    assert "judge.yaml" in missing.stderr

    run = write_run(tmp_path / "bad", {"mission": "juggling"})
    bad = run_cli(run)
    assert bad.returncode == 2
    assert "unknown mission" in bad.stderr
    assert not (run / "score.json").exists()

    run = write_run(tmp_path / "typo", {"mission": "delivery", "setings": {}})
    typo = run_cli(run)
    assert typo.returncode == 2
    assert "setings" in typo.stderr


def test_cli_out_option_writes_the_score_elsewhere(tmp_path):
    run = write_run(tmp_path / "run", {"mission": "delivery"})
    out = tmp_path / "elsewhere.json"
    assert run_cli(run, "--out", out).returncode == 0
    assert json.loads(out.read_text())["mission"] == "delivery"
    assert not (run / "score.json").exists()
