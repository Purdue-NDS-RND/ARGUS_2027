"""PLT-02: is the Jetson's clock on the ground laptop's? (platform/jetson/clock_sync.py)"""

import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import clock_sync  # noqa: E402

TOOL = Path(__file__).resolve().parents[1] / "clock_sync.py"

# chronyc -n -c tracking: reference id, its name, stratum, reference time, system offset,
# last offset, RMS offset, frequency, residual frequency, skew, root delay, root dispersion,
# update interval, leap status.
SYNCED = "C0A80A02,192.168.10.2,9,1798970400.123456789,0.000412345,-0.000101000,0.000350000,-5.123,0.001,0.020,0.000300000,0.000500000,16.2,Normal"
FREE = "00000000,,0,0.000000000,0.000000000,0.000000000,0.000000000,0.000,0.000,0.000,1.000000000,1.000000000,0.0,Not synchronised"


def test_tracking_is_read_from_chronycs_csv():
    tracking = clock_sync.parse_tracking(SYNCED)
    assert tracking.source == "192.168.10.2"
    assert tracking.stratum == 9
    assert tracking.offset_s == pytest.approx(0.000412345)
    assert tracking.rms_offset_s == pytest.approx(0.00035)
    assert tracking.leap == "Normal"


@pytest.mark.parametrize("text", ["", "not,enough,fields", SYNCED.replace("0.000412345", "fast")])
def test_output_that_is_not_tracking_is_an_error_not_a_pass(text):
    with pytest.raises(ValueError):
        clock_sync.parse_tracking(text)


def test_a_clock_on_the_ground_laptop_within_the_limit_is_ok():
    ok, reason = clock_sync.verdict(clock_sync.parse_tracking(SYNCED), source="192.168.10.2", max_offset_s=0.020)
    assert ok
    assert "192.168.10.2" in reason and "0.4 ms" in reason


def test_an_unsynchronised_clock_is_not_ok():
    ok, reason = clock_sync.verdict(clock_sync.parse_tracking(FREE), source="192.168.10.2", max_offset_s=0.020)
    assert not ok
    assert "not synchronised" in reason.lower()


def test_a_clock_following_something_other_than_the_ground_laptop_is_not_ok():
    ok, reason = clock_sync.verdict(clock_sync.parse_tracking(SYNCED), source="192.168.10.9", max_offset_s=0.020)
    assert not ok
    assert "192.168.10.9" in reason and "192.168.10.2" in reason
    # Without a named source any synchronised source will do.
    assert clock_sync.verdict(clock_sync.parse_tracking(SYNCED), source=None, max_offset_s=0.020)[0]


@pytest.mark.parametrize("offset", ["0.020000001", "-0.250000000"])
def test_an_offset_over_the_limit_is_not_ok_in_either_direction(offset):
    tracking = clock_sync.parse_tracking(SYNCED.replace("0.000412345", offset))
    ok, reason = clock_sync.verdict(tracking, source="192.168.10.2", max_offset_s=0.020)
    assert not ok
    assert "20.0 ms" in reason


def test_the_limit_itself_is_ok():
    tracking = clock_sync.parse_tracking(SYNCED.replace("0.000412345", "0.020000000"))
    assert clock_sync.verdict(tracking, source=None, max_offset_s=0.020)[0]


def run_tool(tmp_path, chronyc_output, *args, code=0):
    fake = tmp_path / "chronyc"
    fake.write_text(f"#!/bin/bash\n[[ \"$*\" == '-n -c tracking' ]] || exit 64\ncat <<'EOF'\n{chronyc_output}\nEOF\nexit {code}\n")
    fake.chmod(0o755)
    return subprocess.run([sys.executable, str(TOOL), "--chronyc", str(fake), *args], capture_output=True, text=True, timeout=30)


def test_the_tool_says_ok_and_exits_0(tmp_path):
    result = run_tool(tmp_path, SYNCED, "--source", "192.168.10.2")
    assert result.returncode == 0
    assert result.stdout.startswith("CLOCK OK:")


def test_the_limit_is_20_ms_when_none_is_given(tmp_path):
    assert clock_sync.DEFAULT_MAX_OFFSET_S == 0.020
    assert run_tool(tmp_path, SYNCED.replace("0.000412345", "0.019000000")).returncode == 0
    over = run_tool(tmp_path, SYNCED.replace("0.000412345", "0.021000000"))
    assert over.returncode == 1 and "limit 20.0 ms" in over.stdout
    tracking = clock_sync.parse_tracking(SYNCED.replace("0.000412345", "-0.021000000"))
    assert not clock_sync.verdict(tracking, source=None)[0]


def test_the_tool_says_not_ok_and_exits_1(tmp_path):
    result = run_tool(tmp_path, SYNCED, "--source", "192.168.10.2", "--max-offset-ms", "0.1")
    assert result.returncode == 1
    assert result.stdout.startswith("CLOCK NOT OK:")
    assert run_tool(tmp_path, FREE).returncode == 1


def test_the_tool_does_not_pass_when_chrony_cannot_be_asked(tmp_path):
    result = run_tool(tmp_path, "506 Cannot talk to daemon", code=1)
    assert result.returncode == 1
    assert result.stdout.startswith("CLOCK NOT OK:")
    missing = subprocess.run([sys.executable, str(TOOL), "--chronyc", str(tmp_path / "nothing")], capture_output=True, text=True, timeout=30)
    assert missing.returncode == 1 and missing.stdout.startswith("CLOCK NOT OK:")
