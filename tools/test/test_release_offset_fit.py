"""CEN-08: tools/release_offset_fit.py, K fitted from sim runs' folders and from a field's CSV."""

import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TOOLS), str(TOOLS.parent / "src" / "drone_mission_core")]

import release_offset_fit as tool  # noqa: E402
from drone_mission_core import release_offset as ro  # noqa: E402


def run_folder(root, name, height, lean, k=1.5, released=True, logged=True, payload="parcel"):
    """A sim run's folder as the fit reads it: the mission's log with the lean at the release, and the trace with the package's places."""
    run = root / name
    run.mkdir(parents=True)
    drift = ro.predicted_drift(k, height, lean)
    lines = ["[mission_executor-2] [INFO] [100.0] [mission_executor]: [package_drop] State: FINAL_ALIGN"]
    if logged:
        lines += [f"[mission_executor-2] [INFO] [10{index}.0] [mission_executor]: [package_drop] release offset: height {height + 0.5 - 0.1 * index:.2f} m, lean ({lean[0]:.3f}, {lean[1]:.3f}), aim shifted (0.00, 0.00) m" for index in range(5)]
        lines.append(f"[mission_executor-2] [INFO] [106.0] [mission_executor]: [package_drop] release offset: height {height:.2f} m, lean ({lean[0]:.3f}, {lean[1]:.3f}), aim shifted (0.00, 0.00) m")
    if released:
        lines.append("[mission_executor-2] [INFO] [107.0] [mission_executor]: [package_drop] Closing sprayer for payload drop")
    (run / "launch.log").write_text("\n".join(lines) + "\n")
    trace = [{"t": 50.0, "pose": "vehicle", "e": 0.0, "n": 0.0, "u": 0.2, "yaw": 1.57}, {"t": 50.0, "pose": payload, "e": -0.13, "n": 0.0, "u": 0.02},
             {"t": 106.9, "pose": payload, "e": 10.0, "n": 15.0, "u": height - 0.1}]  # fmt: skip
    if released:
        trace += [{"t": 107.0, "event": "release", "fake": False, "what": "x"}, {"t": 107.4, "pose": payload, "e": 10.0 + 0.5 * drift[0], "n": 15.0 + 0.5 * drift[1], "u": 1.0},
                  {"t": 110.0, "pose": payload, "e": 10.0 + drift[0], "n": 15.0 + drift[1], "u": 0.05}]  # fmt: skip
    (run / "trace.jsonl").write_text("".join(json.dumps(event) + "\n" for event in trace))
    (run / "scenario.json").write_text(json.dumps({"payload": payload}))
    return run


def test_k_is_fitted_from_sim_runs_and_runs_that_gave_no_drop_are_said_and_left_out(tmp_path, capsys):
    runs = [run_folder(tmp_path, "wind-s1", 2.4, (-0.12, -0.09)), run_folder(tmp_path, "wind-s2", 2.5, (-0.10, -0.11)), run_folder(tmp_path, "wind-s3", 2.4, (-0.14, -0.07)),
            run_folder(tmp_path, "held", 2.4, (-0.1, -0.1), released=False), run_folder(tmp_path, "flag-off", 2.4, (-0.1, -0.1), logged=False)]  # fmt: skip
    drop = tool.drop_from_run(runs[0])
    assert drop.height_m == 2.4 and drop.lean == (-0.12, -0.09) and drop.drift_m == pytest.approx(ro.predicted_drift(1.5, 2.4, (-0.12, -0.09)))  # the last line before the release, the package's own places
    assert tool.main(["--runs", *map(str, runs)]) == 0
    said = capsys.readouterr().out
    assert "not used: held: nothing was released" in said and "not used: flag-off: the mission did not log its lean at the release" in said
    assert "K = 1.50 from 3 drop(s)" in said and "For the drop mission's config: release_offset_k: 1.50" in said and "| wind-s1 | 2.40 | (-0.120, -0.090) |" in said


def test_field_drops_come_from_a_csv_and_drops_that_cannot_be_fitted_are_said(tmp_path, capsys):
    path = tmp_path / "drops.csv"
    rows = [("d1", 3.0, -0.15, 0.02), ("d2", 3.0, -0.11, -0.03), ("d3", 4.0, -0.18, 0.0), ("d4", 2.5, -0.05, 0.04)]
    path.write_text("name,height_m,lean_east,lean_north,drift_east_m,drift_north_m\n" + "".join(f"{name},{h},{le},{ln},{-2.0 * h * le:.3f},{-2.0 * h * ln:.3f}\n" for name, h, le, ln in rows))
    assert tool.main(["--drops", str(path)]) == 0 and "K = 2.00 from 4 drop(s)" in capsys.readouterr().out
    # Too few, and still air: no fit, exit code 1.
    assert tool.main(["--runs", str(run_folder(tmp_path, "one", 2.4, (-0.1, 0.0)))]) == 1 and "no fit: 1 usable drop(s)" in capsys.readouterr().out
    still = tmp_path / "still.csv"
    still.write_text("name,height_m,lean_east,lean_north,drift_east_m,drift_north_m\na,2.5,0.004,0.0,0.02,0.01\nb,2.5,0.0,0.005,-0.02,0.0\nc,2.5,-0.003,0.0,0.0,0.03\n")
    assert tool.main(["--drops", str(still)]) == 1 and "made in still air" in capsys.readouterr().out
    # Drifts that are not along the lean: the model explains nothing, and K is to stay 0.
    across = tmp_path / "across.csv"
    across.write_text("name,height_m,lean_east,lean_north,drift_east_m,drift_north_m\na,2.5,-0.1,0.0,0.0,0.4\nb,2.5,-0.1,0.0,0.0,-0.4\nc,2.5,0.0,-0.1,0.4,0.0\n")
    assert tool.main(["--drops", str(across)]) == 1 and "Leave release_offset_k at 0" in capsys.readouterr().out
    for arguments, word in ((["--drops", str(tmp_path / "none.csv")], "No such file"), ([], "no drops"), (["--drops", str(tmp_path / "wind-none" / "x")], "")):
        assert tool.main(arguments) == 2
    bad = tmp_path / "bad.csv"
    bad.write_text("name,height_m\na,2\n")
    assert tool.main(["--drops", str(bad)]) == 2 and "expected the header" in capsys.readouterr().err
