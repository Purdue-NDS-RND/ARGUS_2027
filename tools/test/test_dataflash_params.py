"""PLT-08: parameters out of a dataflash log (tools/dataflash_params.py), on logs built here byte by byte."""

import struct
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import dataflash_params as dfp  # noqa: E402

REPO = TOOLS.parent
HEAD = b"\xa3\x95"


def fmt(type_id, name, layout, columns, length=None):
    size = 3 + struct.calcsize("<" + "".join(dfp.FIELD[c] for c in layout)) if length is None else length
    return HEAD + bytes([128]) + struct.pack("<BB4s16s64s", type_id, size, name.encode(), layout.encode(), columns.encode())


def parm(name, value, type_id=64, default=0.0):
    return HEAD + bytes([type_id]) + struct.pack("<Q16sff", 1000, name.encode(), value, default)


def msg(text, type_id=91):
    return HEAD + bytes([type_id]) + struct.pack("<Q64s", 1000, text.encode())


PARM_FMT = fmt(64, "PARM", "QNff", "TimeUS,Name,Value,Default")
MSG_FMT = fmt(91, "MSG", "QZ", "TimeUS,Message")
FMT_FMT = fmt(128, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns", length=89)


def log(tmp_path, *parts):
    path = tmp_path / "flight.BIN"
    path.write_bytes(b"".join(parts))
    return path


def test_every_parameter_and_the_firmware_are_read(tmp_path):
    path = log(tmp_path, FMT_FMT, PARM_FMT, MSG_FMT, msg("ArduCopter V4.6.3 (92b0cd78)"), msg("ChibiOS: 88b84600"), parm("FENCE_ENABLE", 0.0), parm("BATT_LOW_VOLT", 10.5), parm("FS_OPTIONS", 16.0))
    found = dfp.read_parameters(path)
    assert found.values == {"FENCE_ENABLE": 0.0, "BATT_LOW_VOLT": 10.5, "FS_OPTIONS": 16.0}
    assert found.firmware == "ArduCopter V4.6.3 (92b0cd78)"
    assert found.changed == {}


def test_parms_layout_comes_from_its_fmt_not_from_this_file(tmp_path):
    # An older firmware's PARM has no Default column and another type number.
    old_fmt = fmt(77, "PARM", "QNf", "TimeUS,Name,Value")
    old = HEAD + bytes([77]) + struct.pack("<Q16sf", 5, b"GPS2_TYPE", 2.0)
    assert dfp.read_parameters(log(tmp_path, old_fmt, old)).values == {"GPS2_TYPE": 2.0}


def test_a_parameter_changed_in_flight_keeps_its_last_value_and_is_listed(tmp_path):
    path = log(tmp_path, PARM_FMT, parm("MOT_THST_HOVER", 0.25), parm("GRIP_ENABLE", 0.0), parm("MOT_THST_HOVER", 0.5), parm("GRIP_ENABLE", 0.0))
    found = dfp.read_parameters(path)
    assert found.values["MOT_THST_HOVER"] == 0.5
    assert found.changed == {"MOT_THST_HOVER": [0.25, 0.5]}


def test_other_messages_torn_bytes_and_unknown_field_letters_are_stepped_over(tmp_path):
    gps_fmt = fmt(50, "GPS", "QBIHBcLLeffffB", "TimeUS,I,Status,GMS,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U")
    gps = HEAD + bytes([50]) + struct.pack("<QBIHBhiiiffffB", 1, 0, 3, 1, 12, 80, 1, 2, 3, 0.0, 0.0, 0.0, 0.0, 1)
    odd_fmt = fmt(60, "ODD", "Q?", "TimeUS,What", length=20)  # '?' is no field letter: skipped by its length
    odd = HEAD + bytes([60]) + bytes(17)
    path = log(tmp_path, PARM_FMT, gps_fmt, odd_fmt, parm("A_ONE", 1.0), gps, b"\x00\xa3\x17garbage", odd, parm("B_TWO", 2.0), HEAD + bytes([64]) + b"cut")
    assert dfp.read_parameters(path).values == {"A_ONE": 1.0, "B_TWO": 2.0}


def test_a_file_with_no_parameters_is_an_error(tmp_path):
    with pytest.raises(dfp.DataflashError, match="no parameters"):
        dfp.read_parameters(log(tmp_path, b"not a log at all" * 10))


def test_values_are_written_as_a_ground_station_writes_them():
    assert dfp.format_value(16.0) == "16"
    assert dfp.format_value(-10.0) == "-10"
    assert dfp.format_value(10.5) == "10.5"
    assert float(dfp.format_value(struct.unpack("<f", struct.pack("<f", 0.197104))[0])) == pytest.approx(0.197104, rel=1e-6)


def test_the_tool_writes_a_file_the_parameter_checker_reads(tmp_path):
    import fc_param_check

    path = log(tmp_path, PARM_FMT, MSG_FMT, msg("ArduCopter V4.6.3 (92b0cd78)"), parm("Z_LAST", 3.0), parm("A_FIRST", 1.5), parm("A_FIRST", 2.5))
    out = tmp_path / "set.param"
    result = subprocess.run([sys.executable, str(TOOLS / "dataflash_params.py"), str(path), "--out", str(out)], capture_output=True, text=True)
    assert result.returncode == 0 and "2 parameters (ArduCopter V4.6.3 (92b0cd78))" in result.stdout
    text = out.read_text()
    assert "# Firmware: ArduCopter V4.6.3 (92b0cd78)" in text and "# A_FIRST changed during the log: 1.5, 2.5" in text
    assert [line for line in text.splitlines() if not line.startswith("#")] == ["A_FIRST,2.5", "Z_LAST,3"]
    assert fc_param_check.read_params(out) == {"A_FIRST": 2.5, "Z_LAST": 3.0}
    printed = subprocess.run([sys.executable, str(TOOLS / "dataflash_params.py"), str(path)], capture_output=True, text=True)
    assert printed.stdout == text


def test_a_file_that_cannot_be_read_is_exit_2(tmp_path):
    result = subprocess.run([sys.executable, str(TOOLS / "dataflash_params.py"), str(tmp_path / "none.BIN")], capture_output=True, text=True)
    assert result.returncode == 2 and "dataflash_params:" in result.stderr


def test_the_2026_logs_in_the_repo_say_what_the_baseline_is_measured_against():
    """The aircraft's 2026 settings, as its own log has them: the facts PLT-08's file changes."""
    found = dfp.read_parameters(REPO / "src" / "vision_pipeline" / "vision_pipeline" / "LOGS" / "00000031.BIN")
    assert found.firmware.startswith("ArduCopter V4.6.3")
    assert len(found.values) > 1000
    expected = {"FENCE_ENABLE": 0, "FS_THR_ENABLE": 0, "FS_GCS_ENABLE": 0, "BATT_FS_LOW_ACT": 0, "BATT_FS_CRT_ACT": 0, "GRIP_ENABLE": 0, "FS_OPTIONS": 16,
                "FS_EKF_ACTION": 1, "EK3_SRC_OPTIONS": 1, "GPS2_TYPE": 2, "RNGFND1_TYPE": 0, "SCR_ENABLE": 0}  # fmt: skip
    assert {name: found.values[name] for name in expected} == expected
    assert [found.values[f"SERVO{n}_FUNCTION"] for n in (5, 6, 7, 8)] == [0, 0, 0, 0]  # the spare outputs
