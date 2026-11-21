"""The flight-controller profile checker (REC-9): docs/fc/recovery.parm and what it must hold."""

import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1]
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

import fc_param_check as tool  # noqa: E402

PROFILE = REPO / "docs" / "fc" / "recovery.parm"


def profile():
    return tool.read_params(PROFILE)


def failed(params):
    return {result.rule for result in tool.check(params) if not result.passed}


def test_the_committed_recovery_profile_passes_every_rule(capsys):
    results = tool.check(profile())
    assert [result.rule for result in results] == ["gripper", "failsafe", "rangefinder", "ekf_source_options", "source_set_2", "second_gps", "fence", "radio_switches"]
    assert all(result.passed for result in results), [(result.rule, result.found) for result in results if not result.passed]
    assert tool.main([str(PROFILE)]) == 0 and "PASS" in capsys.readouterr().out


def test_parameter_files_are_read_in_the_forms_ground_stations_write(tmp_path):
    file = tmp_path / "some.param"
    file.write_text("# a comment\nGRIP_ENABLE,1\nSERVO7_FUNCTION 28   # trailing\nFS_OPTIONS\t16\n\nEK3_SRC_OPTIONS, 0\n")
    assert tool.read_params(file) == {"GRIP_ENABLE": 1.0, "SERVO7_FUNCTION": 28.0, "FS_OPTIONS": 16.0, "EK3_SRC_OPTIONS": 0.0}
    file.write_text("GRIP_ENABLE one\n")
    with pytest.raises(ValueError, match="line 1"):
        tool.read_params(file)
    file.write_text("GRIP_ENABLE 1\nGRIP_ENABLE 0\n")
    with pytest.raises(ValueError, match="twice"):
        tool.read_params(file)
    # QGroundControl's saved parameters: a header in comments, then vehicle, component, name, value and type, tab-separated.
    file.write_text(
        "# Onboard parameters for Vehicle 1\n#\n# Stack: ArduPilot\n# Vehicle: Multi-Rotor\n# Version: 4.6.3\n#\n# Vehicle-Id Component-Id Name Value Type\n"
        "1\t1\tEK3_SRC_OPTIONS\t0\t2\n1\t1\tFS_OPTIONS\t16\t4\n1\t1\tWPNAV_SPEED\t500.000000000000000000\t9\n"
    )
    assert tool.read_params(file) == {"EK3_SRC_OPTIONS": 0.0, "FS_OPTIONS": 16.0, "WPNAV_SPEED": 500.0}


@pytest.mark.parametrize(
    "change, rule",
    [
        ({"GRIP_ENABLE": 0}, "gripper"),
        ({"SERVO7_FUNCTION": 0}, "gripper"),  # the gripper on no output
        ({"SERVO7_FUNCTION": 0, "SERVO3_FUNCTION": 28}, "gripper"),  # on a motor's output
        ({"SERVO8_FUNCTION": 28}, "gripper"),  # on two outputs
        ({"GRIP_GRAB": 1100}, "gripper"),  # grab and release the same
        ({"FS_OPTIONS": 16 + 32}, "failsafe"),  # bit 5: a failsafe lets the bucket go
        ({"FS_OPTIONS": None}, "failsafe"),
        ({"RNGFND1_TYPE": 0}, "rangefinder"),
        ({"RNGFND1_ORIENT": 0}, "rangefinder"),  # looking forward
        ({"EK3_SRC_OPTIONS": 1}, "ekf_source_options"),  # the 2026 value
        ({"EK3_SRC2_POSXY": 3}, "source_set_2"),  # GPS in the set that is for flying without it
        ({"EK3_SRC2_VELXY": 0}, "source_set_2"),
        ({"FLOW_TYPE": 0}, "source_set_2"),
        ({"FLOW_TYPE": 1}, "source_set_2"),  # a flow sensor of another kind: the companion's OPTICAL_FLOW is ignored
        ({"EK3_SRC2_YAW": 0}, "source_set_2"),  # the flight controller's default: no yaw source in the set
        ({"GPS2_TYPE": 2}, "second_gps"),  # the 2026 value, with no second GPS fitted
        ({"FENCE_ENABLE": 0}, "fence"),
        ({"FENCE_ACTION": 0}, "fence"),  # report only
        ({"FENCE_TYPE": 1}, "fence"),  # no polygon
        ({"RC7_OPTION": 19}, "radio_switches"),  # a switch that lets the bucket go
        ({"RC9_OPTION": 90}, "radio_switches"),  # a switch that changes the EKF's source set
    ],
)
def test_each_rule_fails_on_what_it_is_for(change, rule):
    params = profile()
    for name, value in change.items():
        if value is None:
            params.pop(name, None)
        else:
            params[name] = float(value)
    assert failed(params) == {rule}, (change, [(r.rule, r.found) for r in tool.check(params) if not r.passed])


def test_an_aircrafts_dump_is_held_to_the_profile_and_to_the_rules(tmp_path, capsys):
    """On the bench: the parameters read back from the flight controller are checked against the committed profile."""
    dump = tmp_path / "aircraft.param"
    everything = {**profile(), "RC7_OPTION": 0.0, "RC8_OPTION": 0.0, "WPNAV_SPEED": 500.0}
    dump.write_text("".join(f"{name},{value:g}\n" for name, value in everything.items()))
    assert tool.main([str(dump), "--profile", str(PROFILE)]) == 0
    capsys.readouterr()
    # One value on the aircraft is not the profile's, and a switch is mapped to the gripper.
    everything.update({"EK3_SRC2_VELXY": 3.0, "RC7_OPTION": 19.0})
    del everything["GPS2_TYPE"]
    dump.write_text("".join(f"{name},{value:g}\n" for name, value in everything.items()))
    assert tool.main([str(dump), "--profile", str(PROFILE)]) == 1
    out = capsys.readouterr().out
    assert "EK3_SRC2_VELXY is 3, the profile has 5" in out and "GPS2_TYPE is not in the file" in out and "RC7_OPTION is 19 (gripper)" in out and "FAIL" in out
    assert tool.main([str(tmp_path / "missing.param")]) == 2
    # The same aircraft, saved by QGroundControl: read, and passed.
    everything = {**profile(), "RC7_OPTION": 0.0, "RC8_OPTION": 0.0}
    dump.write_text("# Vehicle-Id Component-Id Name Value Type\n" + "".join(f"1\t1\t{name}\t{value:.18f}\t9\n" for name, value in everything.items()))
    assert tool.main([str(dump), "--profile", str(PROFILE)]) == 0
    capsys.readouterr()
    # A file with none of the radio's options is not a whole dump: the switches were not looked at, and it does not pass as one.
    assert tool.main([str(PROFILE), "--profile", str(PROFILE)]) == 1 and "not a whole dump" in capsys.readouterr().out


def test_the_sims_recovery_parameters_and_the_aircrafts_agree_where_they_are_the_same_setting():
    sim = tool.read_params(REPO / "sim" / "params" / "recovery.parm")
    aircraft = profile()
    for name in ("FS_OPTIONS", "GPS2_TYPE", "EK3_SRC_OPTIONS", "EK3_SRC2_POSXY", "EK3_SRC2_VELXY", "EK3_SRC2_VELZ", "EK3_SRC2_YAW", "GRIP_ENABLE", "GRIP_TYPE"):
        assert sim[name] == aircraft[name], name
    assert aircraft["FLOW_TYPE"] == tool.FLOW_TYPE_MAVLINK == 5  # REC-5's primary path: the companion sends OPTICAL_FLOW
