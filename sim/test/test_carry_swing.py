"""The swing of a carried bucket, measured from a flight's trace (REC-4)."""

import json
import math
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import carry_swing  # noqa: E402


def trace(tmp_path, swing_m, residual_m, speed=2.0):
    """A flight: the bucket lifted at (12, 8), carried 10 m west at `speed`, held 3 s, set down. The bucket hangs
    0.02 m east of the vehicle, is `swing_m` further out mid-leg and swings by `residual_m` after the stop."""
    lines, t = [], 0.0

    def put(ve, vu, be, bu):
        lines.append(json.dumps({"t": round(t, 3), "pose": "vehicle", "e": ve, "n": 8.0, "u": vu, "yaw": 0.0}))
        lines.append(json.dumps({"t": round(t, 3), "pose": "bucket", "e": be, "n": 8.0, "u": bu}))

    for _ in range(20):  # on the ground
        put(12.0, 0.9, 12.02, 0.02)
        t += 0.05
    for step in range(60):  # the lift, and a hover
        up = min(2.0, step * 0.1)
        put(12.0, 0.9 + up, 12.02, 0.02 + up)
        t += 0.05
    lines.append("not json")  # a broken line is passed over
    leg = int(10.0 / speed / 0.05)
    for step in range(leg + 1):
        e = 12.0 - 10.0 * step / leg
        put(e, 2.9, e + 0.02 + swing_m * math.sin(math.pi * step / leg), 2.02)
        t += 0.05
    for step in range(60):  # stopped: what swing is left
        put(2.0, 2.9, 2.02 + residual_m * math.cos(step * 0.37), 2.02)
        t += 0.05
    for step in range(20):  # down
        put(2.0, 2.9 - 0.1 * step, 2.02, 2.02 - 0.1 * step)
        t += 0.05
    file = tmp_path / "trace.jsonl"
    file.write_text("\n".join(lines) + "\n")
    return file


def test_the_carrys_swing_and_what_is_left_after_the_stop_are_read_from_the_trace(tmp_path):
    got = carry_swing.measure(carry_swing.read_rows(trace(tmp_path, swing_m=0.15, residual_m=0.01)))
    assert got["distance_m"] == 10.0 and abs(got["top_speed_mps"] - 2.0) < 0.05 and abs(got["duration_s"] - 4.7) < 0.2
    assert got["hangs_m"] == 0.02 and abs(got["swing_m"] - 0.15) < 0.005 and abs(got["residual_m"] - 0.01) < 0.002
    assert 2.5 < got["residual_s"] <= 3.5
    # A swing left after the stop is seen as one.
    swinging = carry_swing.measure(carry_swing.read_rows(trace(tmp_path, swing_m=0.15, residual_m=0.2)))
    assert abs(swinging["residual_m"] - 0.2) < 0.005


def test_a_trace_with_no_carry_gives_nothing_and_the_tool_says_so(tmp_path, capsys):
    grounded = tmp_path / "trace.jsonl"
    grounded.write_text("\n".join(json.dumps({"t": 0.05 * i, "pose": kind, "e": 0.0, "n": 0.0, "u": 0.0}) for i in range(30) for kind in ("vehicle", "bucket")))
    assert carry_swing.measure(carry_swing.read_rows(grounded)) is None and carry_swing.measure([]) is None
    assert carry_swing.main([str(tmp_path)]) == 0 and "null" in capsys.readouterr().out
    assert carry_swing.main([str(tmp_path / "nowhere")]) == 1 and carry_swing.main([]) == 2
