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


# --- the 2027 baseline (PLT-08): platform/fcu/argus_2027.param ---------------------------------------------

BASELINE = REPO / "platform" / "fcu" / "argus_2027.param"
COMP_2026 = REPO / "platform" / "fcu" / "comp_2026.param"
BASELINE_RULES = ["fence", "battery_failsafe", "rc_failsafe", "gcs_failsafe", "ekf_failsafe", "failsafe", "gripper", "sprayer", "rangefinder", "ekf_source_options", "second_gps", "radio_switches"]


def baseline():
    return tool.read_params(BASELINE)


def baseline_failed(params):
    return {result.rule for result in tool.check(params, tool.BASELINE_RULES) if not result.passed}


def test_the_committed_baseline_passes_every_baseline_rule(capsys):
    results = tool.check(baseline(), tool.BASELINE_RULES)
    assert [result.rule for result in results] == BASELINE_RULES
    assert all(result.passed for result in results), [(result.rule, result.found) for result in results if not result.passed]
    assert tool.main([str(BASELINE), "--rules", "baseline"]) == 0 and "PASS" in capsys.readouterr().out
    # The recovery rules ask for more (source set 2): the baseline alone is not a recovery profile, and says so.
    assert tool.main([str(BASELINE)]) == 1 and "FAIL source_set_2" in capsys.readouterr().out


def test_the_2026_aircraft_fails_exactly_the_rules_the_baseline_was_written_for():
    """In June 2026: no fence, no failsafe with an action but the EKF's, the gripper off, no rangefinder,
    the EKF free to change its source set, and a second GPS configured that was not fitted."""
    flown = tool.read_params(COMP_2026)
    assert baseline_failed(flown) == {"fence", "battery_failsafe", "rc_failsafe", "gcs_failsafe", "gripper", "sprayer", "rangefinder", "ekf_source_options", "second_gps"}
    # With the baseline loaded over it, every rule holds; and nothing else of the aircraft's is touched.
    assert baseline_failed({**flown, **baseline()}) == set()
    assert len(flown) > 1000 and len(baseline()) < 40


def test_the_2026_file_is_what_the_log_in_the_repo_says():
    import dataflash_params

    log = REPO / "src" / "vision_pipeline" / "vision_pipeline" / "LOGS" / "00000031.BIN"
    assert COMP_2026.read_text() == dataflash_params.as_param_file(dataflash_params.read_parameters(log), log.name)


@pytest.mark.parametrize(
    "change, rule",
    [
        ({"FENCE_ENABLE": 0}, "fence"),
        ({"FENCE_TYPE": 4}, "fence"),  # the polygon without the ceiling
        ({"FENCE_ALT_MAX": 0}, "fence"),
        ({"FENCE_ACTION": 0}, "fence"),
        ({"BATT_FS_LOW_ACT": 0}, "battery_failsafe"),
        ({"BATT_LOW_VOLT": 0}, "battery_failsafe"),
        ({"BATT_FS_CRT_ACT": 0}, "battery_failsafe"),
        ({"BATT_CRT_VOLT": 0}, "battery_failsafe"),
        ({"BATT_CRT_VOLT": 21.5}, "battery_failsafe"),  # critical above low
        ({"FS_THR_ENABLE": 0}, "rc_failsafe"),
        ({"FS_GCS_ENABLE": 0}, "gcs_failsafe"),
        ({"FS_EKF_ACTION": 0}, "ekf_failsafe"),
        ({"FS_OPTIONS": 48}, "failsafe"),  # bit 5: a failsafe would drop the payload
        ({"GRIP_ENABLE": 0}, "gripper"),
        ({"SERVO7_FUNCTION": 0}, "gripper"),
        ({"SPRAY_ENABLE": 0}, "sprayer"),  # the 2026 value: the drop's command is acknowledged and nothing moves
        ({"SERVO8_FUNCTION": 0}, "sprayer"),  # enabled, with no output to move
        ({"SERVO6_FUNCTION": 22}, "sprayer"),  # the pump on two outputs
        ({"SERVO8_FUNCTION": 0, "SERVO9_FUNCTION": 22}, "sprayer"),  # not one of the spare outputs
        ({"SPRAY_SPEED_MIN": 100}, "sprayer"),  # ArduPilot's default: no spraying under 1 m/s, and the drop is made from a hover
        ({"SPRAY_PUMP_MIN": 0}, "sprayer"),  # ArduPilot's default: in a hover the pump's output would not move
        ({"RNGFND1_TYPE": 0}, "rangefinder"),
        ({"EK3_SRC_OPTIONS": 1}, "ekf_source_options"),
        ({"GPS2_TYPE": 2}, "second_gps"),
        ({"RC9_OPTION": 19}, "radio_switches"),
    ],
)
def test_each_baseline_rule_fails_on_what_it_is_for(change, rule):
    assert baseline_failed({**baseline(), **change}) == {rule}


def test_a_baseline_with_a_line_missing_fails_that_rule():
    for name, rule in (("FS_GCS_ENABLE", "gcs_failsafe"), ("BATT_CRT_VOLT", "battery_failsafe"), ("FENCE_ALT_MAX", "fence"), ("FS_OPTIONS", "failsafe"), ("SPRAY_ENABLE", "sprayer"), ("SPRAY_SPEED_MIN", "sprayer"), ("SPRAY_PUMP_MIN", "sprayer"), ("SERVO8_FUNCTION", "sprayer")):
        assert baseline_failed({k: v for k, v in baseline().items() if k != name}) == {rule}


def test_the_baseline_sets_the_drops_sprayer_as_the_sim_does():
    """The drop releases with MAV_CMD_DO_SPRAYER (mission_context.py), which ArduPilot 4.6.3 acknowledges even with
    SPRAY_ENABLE 0. The sim's flights set the sprayer up before they drop; the aircraft's file must set the same."""
    sys.path.insert(0, str(REPO / "sim"))
    import payload_bridge

    sprayer = {name: float(value) for name, value in payload_bridge.SITL_PARAMS.items() if name.startswith("SPRAY_") or value == tool.SPRAYER_PUMP_FUNCTION}
    assert sprayer == {"SPRAY_ENABLE": 1.0, "SPRAY_SPEED_MIN": 0.0, "SPRAY_PUMP_MIN": 100.0, "SERVO8_FUNCTION": 22.0}
    assert {name: baseline()[name] for name in sprayer} == sprayer
    assert tool.read_params(COMP_2026)["SPRAY_ENABLE"] == 0  # how it flew in June 2026
    # The file says whose number the output is.
    text = BASELINE.read_text()
    block = text[text.index("# --- The drop's release") : text.index("SPRAY_ENABLE,1")]
    assert "mechanical team" in block and "MAV_CMD_DO_SPRAYER" in block
    # Gripper and sprayer are on different outputs, both spare.
    assert baseline()["SERVO7_FUNCTION"] == tool.GRIPPER_FUNCTION and baseline()["SERVO8_FUNCTION"] == tool.SPRAYER_PUMP_FUNCTION


def test_the_baseline_and_the_recovery_profile_agree_where_both_speak():
    both = set(baseline()) & set(profile())
    assert {"GRIP_ENABLE", "SERVO7_FUNCTION", "FS_OPTIONS", "RNGFND1_TYPE", "EK3_SRC_OPTIONS", "GPS2_TYPE", "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION"} <= both
    assert {name: baseline()[name] for name in both} == {name: profile()[name] for name in both}
    # Loaded one over the other, in either order, both sets of rules hold.
    loaded = {**baseline(), **profile()}
    assert baseline_failed(loaded) == set() and failed(loaded) == set()


def test_the_battery_thresholds_fit_the_pack_the_2026_logs_show():
    """A 6S pack: 3.5 V a cell low, 3.3 V critical, both under the lowest voltage seen in the 2026 flights (22.35 V)."""
    values = baseline()
    assert values["BATT_LOW_VOLT"] == pytest.approx(6 * 3.5) and values["BATT_CRT_VOLT"] == pytest.approx(6 * 3.3)
    assert values["BATT_CRT_VOLT"] < values["BATT_LOW_VOLT"] < 22.35
    assert (values["BATT_FS_LOW_ACT"], values["BATT_FS_CRT_ACT"]) == (2, 1)  # RTL, then land


def test_the_diff_lists_what_the_baseline_changes_from_2026(capsys):
    assert tool.main([str(BASELINE), "--rules", "baseline", "--diff", str(COMP_2026)]) == 0
    out = capsys.readouterr().out
    for line in ("diff FENCE_ENABLE: 0 -> 1", "diff FS_THR_ENABLE: 0 -> 1", "diff FS_GCS_ENABLE: 0 -> 1", "diff BATT_FS_LOW_ACT: 0 -> 2", "diff GRIP_ENABLE: 0 -> 1", "diff EK3_SRC_OPTIONS: 1 -> 0", "diff GPS2_TYPE: 2 -> 0", "diff GRIP_TYPE: not in the other file -> 1", "diff SPRAY_ENABLE: 0 -> 1", "diff SERVO8_FUNCTION: 0 -> 22"):
        assert line in out, line
    # What 2026 already had is not in the list: the EKF's action, and FS_OPTIONS.
    assert "diff FS_EKF_ACTION" not in out and "diff FS_OPTIONS" not in out
    assert "parameters differ from" in out
    lines = tool.changes({"A": 1.0, "B": 2.0, "C": 3.0}, {"A": 1.0, "B": 5.0})
    assert lines == ["B: 5 -> 2", "C: not in the other file -> 3"]


def test_an_aircraft_dump_is_held_to_the_baseline_with_its_rules(tmp_path, capsys):
    dump = tmp_path / "aircraft.param"
    loaded = {**tool.read_params(COMP_2026), **baseline()}
    dump.write_text("".join(f"{name},{value:g}\n" for name, value in sorted(loaded.items())))
    assert tool.main([str(dump), "--rules", "baseline", "--profile", str(BASELINE)]) == 0
    capsys.readouterr()
    dump.write_text("".join(f"{name},{value:g}\n" for name, value in sorted({**loaded, "FS_GCS_ENABLE": 0.0}.items())))
    assert tool.main([str(dump), "--rules", "baseline", "--profile", str(BASELINE)]) == 1
    out = capsys.readouterr().out
    assert "FAIL gcs_failsafe" in out and "FAIL profile: FS_GCS_ENABLE is 0, the profile has 1" in out
