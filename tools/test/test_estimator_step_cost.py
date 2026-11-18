"""tools/estimator_step_cost.py: the timing tool for the estimator's step runs, and the bounded worst case does what it says."""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "estimator_step_cost.py"
sys.path.insert(0, str(ROOT / "tools"))

import estimator_step_cost  # noqa: E402


def test_the_tool_prints_both_cases():
    done = subprocess.run([sys.executable, str(TOOL), "--steps", "5"], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    ordinary, worst = done.stdout.strip().splitlines()
    assert ordinary.startswith("one bullseye in one frame: median ") and ordinary.endswith("over 5 steps, 1 tracks (the tick is 50 ms)")
    assert worst.startswith("the worst the bounds allow (4 frames of 16 detections): median ") and "16 tracks" in worst


def test_the_worst_case_really_handles_four_frames_of_sixteen_every_step():
    result = estimator_step_cost.measure(detections_per_frame=16, frames_per_step=4, steps=6)
    assert result["measurements"] == 6 * 4 * 16 and result["tracks"] == 16
    assert 0.0 < result["median_ms"] <= result["p95_ms"] <= result["max_ms"]
    # Not a performance limit for CI machines, only a guard against the step becoming absurd: many ticks long.
    assert result["median_ms"] < 500.0
