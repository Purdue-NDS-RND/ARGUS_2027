"""PLT-10: the preflight go/no-go's rules (drone_utils/preflight.py). No ROS."""

import dataclasses
import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "drone_utils"))
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_utils import preflight as pf  # noqa: E402
from drone_utils import preflight_node as node  # noqa: E402
from drone_utils.preflight import Limits, Snapshot  # noqa: E402

HEALTHY_BITS = pf.SENSOR_AHRS | pf.SENSOR_GEOFENCE | pf.SENSOR_PREARM_CHECK | pf.SENSOR_LASER_POSITION | 0x1 | 0x20


def good(**changes) -> Snapshot:
    """An aircraft that is ready: every reading there, fresh and inside its limit."""
    base = Snapshot(
        now_s=1000.0,
        params_source="flight controller", params_problems=[], baseline_sha256="ab" * 32,
        sensors_present=HEALTHY_BITS, sensors_enabled=HEALTHY_BITS, sensors_health=HEALTHY_BITS, sys_status_age_s=0.3, fence_points=4,
        gps_fix_type=3, gps_hdop=0.9, gps_satellites=14, gps_age_s=0.1,
        range_m=0.12, range_min_m=0.3, range_max_m=12.0, range_age_s=0.05,
        camera_rates={"/argus/camera/down/image_raw": (59.2, 60.0)},
        engine=None,
        timesync_age_s=0.1, timesync_jitter_ms=1.5, clock=(True, "following 192.168.10.2 (stratum 9), 0.4 ms off (limit 20.0 ms)"), clock_wanted=True,
        disk_free_gb=412.0, log_dir="/mnt/argus-logs",
        battery_v=24.9, battery_age_s=0.4,
        profile=(True, "use_frame_time_pose,use_target_estimator"), profile_name="delivery_2027",
    )  # fmt: skip
    return dataclasses.replace(base, **changes)


def failed(snapshot, limits=Limits()):
    return [check.name for check in pf.run_checks(snapshot, limits) if not check.go]


def test_the_checks_are_the_specs_eleven_in_its_order():
    checks = pf.run_checks(good())
    assert [check.name for check in checks] == ["params", "fence", "gps", "ekf", "rangefinder", "cameras", "engine", "timesync", "disk", "battery", "profile"]
    assert pf.CHECKS == tuple(check.name for check in checks)


def test_a_ready_aircraft_is_go():
    checks = pf.run_checks(good())
    assert all(check.go for check in checks), [(c.name, c.detail) for c in checks if not c.go]
    result = pf.verdict(checks)
    assert result.go and result.clean and result.line == "GO" and result.failed == ()
    lines = pf.report_lines(checks, result)
    assert len(lines) == 12 and lines[-1] == "GO" and lines[0].startswith("[   GO] params")
    assert "sha256 abababababab" in lines[0] and "HDOP 0.90" in lines[2] and "24.9 V, 4.15 V a cell" in lines[9]


def test_a_rangefinder_that_sends_a_zero_and_has_no_data_is_not_a_reading():
    """The spec audit: the flight controller goes on sending a range of zero for a sensor that is unplugged."""
    dead = good(range_m=0.0, sensors_health=HEALTHY_BITS & ~pf.SENSOR_LASER_POSITION)
    assert failed(dead) == ["rangefinder"]
    assert failed(good(range_m=0.0)) == []  # on the ground, with the sensor healthy: under its minimum, and fine


def test_an_aircraft_nothing_was_read_from_fails_every_check_but_the_engine_it_does_not_use():
    assert failed(Snapshot()) == [name for name in pf.CHECKS if name != "engine"]
    result = pf.verdict(pf.run_checks(Snapshot()))
    assert not result.go and result.line.startswith("NO-GO: params, fence, gps, ekf")


@pytest.mark.parametrize(
    "changes, check, word",
    [
        ({"params_problems": None}, "params", "not compared"),
        ({"params_problems": ["FENCE_ENABLE is 0, the profile has 1", "FS_GCS_ENABLE is 0, the profile has 1"]}, "params", "FENCE_ENABLE is 0, the profile has 1 (and 1 more)"),
        ({"params_source": "aircraft.param", "params_age_h": 30.0}, "params", "30 h old"),
        ({"sensors_enabled": HEALTHY_BITS & ~pf.SENSOR_GEOFENCE}, "fence", "enabled, healthy fence"),
        ({"sensors_health": HEALTHY_BITS & ~pf.SENSOR_GEOFENCE}, "fence", "enabled, healthy fence"),
        ({"gps_fix_type": 2}, "gps", "no 3D fix"),
        ({"gps_hdop": 1.41}, "gps", "HDOP 1.41"),
        ({"gps_hdop": None}, "gps", "HDOP unknown"),
        ({"gps_satellites": 7}, "gps", "7 satellites"),
        ({"gps_age_s": 5.0}, "gps", "no GPS status"),
        ({"sensors_health": HEALTHY_BITS & ~pf.SENSOR_AHRS}, "ekf", "estimate is not healthy"),
        ({"sensors_health": HEALTHY_BITS & ~pf.SENSOR_PREARM_CHECK}, "ekf", "pre-arm checks do not pass"),
        ({"range_m": None}, "rangefinder", "no rangefinder reading"),
        ({"range_age_s": 3.0}, "rangefinder", "no rangefinder reading"),
        ({"range_m": float("inf")}, "rangefinder", "outside the sensor's range"),
        ({"range_m": 12.5}, "rangefinder", "outside the sensor's range"),
        ({"camera_rates": None}, "cameras", "no camera was named"),
        ({"fence_points": 0}, "fence", "0 fence points are stored"),
        ({"fence_points": 2}, "fence", "2 fence points are stored"),
        ({"sensors_health": HEALTHY_BITS & ~pf.SENSOR_LASER_POSITION}, "rangefinder", "not fitted or has no data"),
        ({"sensors_present": HEALTHY_BITS & ~pf.SENSOR_LASER_POSITION}, "rangefinder", "not fitted or has no data"),
        ({"camera_rates": {"/argus/camera/down/image_raw": (47.0, 60.0)}}, "cameras", "at 47.0 Hz (mode 60)"),
        ({"camera_rates": {"/argus/camera/down/image_raw": (0.0, 60.0), "/argus/camera/flow/image_raw": (60.0, 60.0)}}, "cameras", "down/image_raw at 0.0 Hz"),
        ({"engine": {"path": "/models/yolo.engine", "sha256": None, "manifest_sha256": "cd" * 32}}, "engine", "no engine file"),
        ({"engine": {"path": "/models/yolo.engine", "sha256": "ef" * 32, "manifest_sha256": "cd" * 32}}, "engine", "the manifest has cdcdcdcdcdcd"),
        ({"engine": {"path": "/models/yolo.engine", "sha256": "ef" * 32, "manifest_sha256": None}}, "engine", "not in a manifest"),
        ({"timesync_age_s": None}, "timesync", "no time-sync status"),
        ({"timesync_jitter_ms": 35.0}, "timesync", "wanders by 35.0 ms"),
        ({"timesync_jitter_ms": None}, "timesync", "too few time-sync exchanges"),
        ({"clock": (False, "following 10.0.0.9, not the ground laptop 192.168.10.2")}, "timesync", "not the ground laptop"),
        ({"clock": None}, "timesync", "was not compared with the ground laptop"),
        ({"disk_free_gb": None}, "disk", "could not be read"),
        ({"disk_free_gb": 19.9}, "disk", "19.9 GB free"),
        ({"battery_v": 23.3}, "battery", "3.88 V a cell"),
        ({"battery_v": None}, "battery", "no battery reading"),
        ({"battery_age_s": 9.0}, "battery", "no battery reading"),
        ({"profile": None}, "profile", "no flag profile was checked"),
        ({"profile": (False, "drop_release_gates requires centre_on_estimator")}, "profile", "requires centre_on_estimator"),
    ],
)
def test_one_thing_wrong_fails_its_check_and_only_its_check(changes, check, word):
    checks = pf.run_checks(good(**changes))
    assert [c.name for c in checks if not c.go] == [check]
    assert word in next(c.detail for c in checks if c.name == check)
    assert not pf.verdict(checks).go


def test_readings_at_their_limits_pass():
    edge = good(gps_hdop=1.4, gps_satellites=8, disk_free_gb=20.0, battery_v=6 * 3.9, timesync_jitter_ms=20.0, camera_rates={"/argus/camera/down/image_raw": (48.0, 60.0)}, range_m=0.0)
    assert failed(edge) == []
    assert failed(good(engine={"path": "/m/y.engine", "sha256": "cd" * 32, "manifest_sha256": "cd" * 32})) == []
    assert failed(good(camera_rates={})) == []  # a flight that was said to need no camera
    assert failed(good(fence_points=None)) == [] and failed(good(fence_points=3)) == []  # not read (no way to), or a triangle
    assert failed(good(clock=None, clock_wanted=False)) == []  # the sim, or a flight with no ground laptop's clock asked for
    assert "was not compared" in pf.run_checks(good(clock=None, clock_wanted=False))[7].detail  # and the line says so
    assert failed(good(timesync_largest_step_ms=100.0)) == [] and failed(good(timesync_largest_step_ms=100.1)) == ["timesync"]
    assert failed(good(params_source="aircraft.param", params_age_h=23.0)) == []


def test_the_limits_are_the_configs_to_set():
    assert failed(good(battery_v=14.9), Limits(cells=4, min_cell_v=3.7)) == []
    assert failed(good(battery_v=14.9)) == ["battery"]
    assert failed(good(gps_hdop=1.8), Limits(max_hdop=2.0)) == []


def test_no_go_blocks_unless_overridden_with_a_reason_and_the_override_is_named():
    checks = pf.run_checks(good(gps_hdop=2.5, battery_v=23.0))
    blocked = pf.verdict(checks)
    assert not blocked.go and blocked.line == "NO-GO: gps, battery"
    for empty in (None, "", "   "):
        assert not pf.verdict(checks, override_reason=empty).go  # no reason, no override
    overridden = pf.verdict(checks, override_reason="HDOP is high under the hangar roof; flying from the apron", who="safety pilot")
    assert overridden.go and not overridden.clean
    assert overridden.line == 'NO-GO OVERRIDDEN by safety pilot: gps, battery ("HDOP is high under the hangar roof; flying from the apron")'
    # An override of nothing is just GO.
    assert pf.verdict(pf.run_checks(good()), override_reason="habit").line == "GO"


def test_every_preflight_is_logged_and_an_override_with_who_and_why(tmp_path):
    log = tmp_path / "logs" / "preflight.jsonl"
    checks = pf.run_checks(good(gps_hdop=2.5))
    result = pf.verdict(checks, override_reason="apron", who="lead")
    pf.append_log(log, pf.log_record(checks, result, "2027-06-04T15:02:11Z", "apron", "lead", {"profile": "delivery_2027", "sequence": "package_delivery_2027.yaml"}))
    pf.append_log(log, pf.log_record(pf.run_checks(good()), pf.verdict(pf.run_checks(good())), "2027-06-04T15:20:00Z"))
    first, second = [json.loads(line) for line in log.read_text().splitlines()]
    assert first["verdict"] == "NO-GO OVERRIDDEN" and first["failed"] == ["gps"] and first["override"] == {"by": "lead", "reason": "apron"}
    assert first["profile"] == "delivery_2027" and len(first["checks"]) == 11 and first["checks"][2]["go"] is False
    assert second["verdict"] == "GO" and "override" not in second
    blocked = pf.log_record(checks, pf.verdict(checks), "2027-06-04T15:01:00Z")
    assert blocked["verdict"] == "NO-GO" and "override" not in blocked


def test_an_override_that_cannot_be_logged_is_an_error(tmp_path):
    target = tmp_path / "file"
    target.write_text("")
    with pytest.raises(OSError):
        pf.append_log(target / "preflight.jsonl", {"verdict": "NO-GO OVERRIDDEN"})


def test_the_hash_is_the_files_own(tmp_path):
    file = tmp_path / "baseline.param"
    file.write_text("FENCE_ENABLE,1\n")
    import hashlib

    assert pf.sha256_of(file) == hashlib.sha256(b"FENCE_ENABLE,1\n").hexdigest()
    assert pf.sha256_of(tmp_path / "none") is None


# --- the parts the node leans on ----------------------------------------------------------------------------------


def test_the_flight_controllers_values_are_held_to_the_baselines(tmp_path):
    baseline = tmp_path / "baseline.param"
    baseline.write_text("# the baseline\nFENCE_ENABLE,1\nBATT_LOW_VOLT,21\nFS_OPTIONS,16  # bit 5 clear\n")
    wanted = pf.read_param_file(baseline)
    assert wanted == {"FENCE_ENABLE": 1.0, "BATT_LOW_VOLT": 21.0, "FS_OPTIONS": 16.0}
    assert pf.compare_params({"FENCE_ENABLE": 1.0, "BATT_LOW_VOLT": 21.0, "FS_OPTIONS": 16.0, "SOMETHING_ELSE": 5.0}, wanted) == []
    assert pf.compare_params({"FENCE_ENABLE": 0.0, "BATT_LOW_VOLT": 21.0}, wanted) == ["FENCE_ENABLE is 0, the baseline has 1", "FS_OPTIONS was not read; the baseline has 16"]
    # A dump as the ground stations write it.
    dump = tmp_path / "dump.params"
    dump.write_text("1\t1\tFENCE_ENABLE\t1\t2\n1\t1\tBATT_LOW_VOLT\t21.0\t9\n1\t1\tFS_OPTIONS\t16\t6\n")
    assert pf.compare_params(pf.read_param_file(dump), wanted) == []
    bad = tmp_path / "bad.param"
    bad.write_text("FENCE_ENABLE\n")
    with pytest.raises(ValueError, match="line 1"):
        pf.read_param_file(bad)
    import hashlib

    assert pf.sha256_of(baseline) == hashlib.sha256(baseline.read_bytes()).hexdigest()


def test_the_time_sync_is_judged_on_most_of_its_steps_not_on_one():
    """The first sim flight: one 22 ms step among fifty exchanges read as a time base that wanders."""
    assert pf.timesync_steps_ms([]) == (None, None) and pf.timesync_steps_ms([5]) == (None, None)
    quiet = [1_000_000 * i for i in range(50)]  # 1 ms a step
    quiet[25:] = [value + 22_000_000 for value in quiet[25:]]  # and one step of 23 ms in the middle
    p90, largest = pf.timesync_steps_ms(quiet)
    assert p90 == pytest.approx(1.0) and largest == pytest.approx(23.0)
    assert [c.name for c in pf.run_checks(good(timesync_jitter_ms=p90, timesync_largest_step_ms=largest)) if not c.go] == []
    assert "largest step 23.0 ms" in pf.run_checks(good(timesync_jitter_ms=p90, timesync_largest_step_ms=largest))[7].detail
    wandering = [25_000_000 * (i % 2) for i in range(50)]  # every step 25 ms
    p90, largest = pf.timesync_steps_ms(wandering)
    assert p90 == pytest.approx(25.0)
    assert [c.name for c in pf.run_checks(good(timesync_jitter_ms=p90, timesync_largest_step_ms=largest)) if not c.go] == ["timesync"]


def test_a_profile_is_checked_against_the_sequence_it_is_to_be_flown_with(tmp_path):
    sequence = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_2027.yaml"
    assert pf.check_profile(REPO / "profiles" / "comp_2026.yaml", sequence) == (True, "")
    good_profile = tmp_path / "estimator.yaml"
    good_profile.write_text("description: test\nenable:\n  - use_frame_time_pose\n  - use_target_estimator\n")
    assert pf.check_profile(good_profile, sequence) == (True, "use_frame_time_pose,use_target_estimator")
    needs = tmp_path / "needs.yaml"
    needs.write_text("description: test\nenable:\n  - use_target_estimator\n")
    ok, why = pf.check_profile(needs, sequence)
    assert not ok and "use_frame_time_pose" in why
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text("description: test\nenable:\n  - survey_search\n")  # not a flag of the delivery's sequence
    assert pf.check_profile(unknown, sequence)[0] is False
    assert pf.check_profile(tmp_path / "none.yaml", sequence)[0] is False
    # A sequence that has a flag on by itself is not "the 2026 code", whatever the profile says: the line shows what will fly.
    edited = tmp_path / "edited.yaml"
    edited.write_text(sequence.read_text().replace("use_frame_time_pose: false", "use_frame_time_pose: true", 1))
    assert pf.check_profile(REPO / "profiles" / "comp_2026.yaml", edited) == (True, "use_frame_time_pose")
    assert pf.check_profile(good_profile, tmp_path / "none.yaml")[0] is False


# --- the command's side that needs no ROS (preflight_node.py) -------------------------------------------------------


def arguments(tmp_path, *extra):
    baseline = tmp_path / "baseline.param"
    baseline.write_text("FENCE_ENABLE,1\nFS_OPTIONS,16\n")
    return node.build_parser().parse_args(["--baseline", str(baseline), "--log-dir", str(tmp_path / "logs"), *extra])


def test_cameras_are_named_as_topic_and_rate():
    assert node.parse_cameras(["/camera/image=30", "/argus/camera/flow/image_raw=60"]) == {"/camera/image": 30.0, "/argus/camera/flow/image_raw": 60.0}
    assert node.parse_cameras([]) == {}
    for bad in ("/camera/image", "camera/image=30", "/camera/image=fast", "/camera/image=0"):
        with pytest.raises(ValueError, match="TOPIC=HZ"):
            node.parse_cameras([bad])


def test_the_offline_side_fills_the_parameters_from_a_dump_the_disk_the_engine_and_the_profile(tmp_path):
    (tmp_path / "logs").mkdir()
    dump = tmp_path / "aircraft.param"
    dump.write_text("FENCE_ENABLE,1\nFS_OPTIONS,16\nSOMETHING,3\n")
    engine = tmp_path / "yolo.engine"
    engine.write_bytes(b"engine")
    import hashlib

    sequence = REPO / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_2027.yaml"
    args = arguments(tmp_path, "--param-dump", str(dump), "--engine", str(engine), "--engine-sha256", hashlib.sha256(b"engine").hexdigest(),
                     "--profile", str(REPO / "profiles" / "comp_2026.yaml"), "--sequence", str(sequence))  # fmt: skip
    snapshot = node.fill_offline(good(params_problems=None, params_source=None, baseline_sha256=None, disk_free_gb=None, profile=None, clock=None, clock_wanted=False), args)
    assert snapshot.params_source == str(dump) and snapshot.params_problems == [] and 0.0 <= snapshot.params_age_h < 0.1
    assert snapshot.baseline_sha256 == hashlib.sha256(b"FENCE_ENABLE,1\nFS_OPTIONS,16\n").hexdigest()
    assert snapshot.disk_free_gb > 0.0 and snapshot.log_dir == str(tmp_path / "logs")
    assert snapshot.engine["sha256"] == snapshot.engine["manifest_sha256"] and snapshot.profile == (True, "") and snapshot.profile_name == "comp_2026"
    assert [c.name for c in pf.run_checks(snapshot, Limits(min_free_gb=0.001)) if not c.go] == []
    # A dump that is not the baseline, and an engine that is not the manifest's.
    dump.write_text("FENCE_ENABLE,0\nFS_OPTIONS,16\n")
    args = arguments(tmp_path, "--param-dump", str(dump), "--engine", str(engine), "--engine-sha256", "00" * 32)
    snapshot = node.fill_offline(good(params_problems=None, params_source=None, baseline_sha256=None), args)
    assert snapshot.params_problems == ["FENCE_ENABLE is 0, the baseline has 1"] and snapshot.engine["sha256"] != snapshot.engine["manifest_sha256"]
    # Parameters already read from the flight controller are not replaced by a dump.
    live = node.fill_offline(good(params_source="flight controller (MAVROS)", params_problems=[]), args)
    assert live.params_source == "flight controller (MAVROS)" and live.params_problems == [] and live.params_age_h is None


def test_with_no_way_to_read_the_parameters_the_check_is_no_go(tmp_path):
    snapshot = node.fill_offline(good(params_problems=None, params_source=None, baseline_sha256=None), arguments(tmp_path))
    assert snapshot.params_problems is None
    assert [c.name for c in pf.run_checks(snapshot, Limits(min_free_gb=0.0)) if not c.go] == ["params", "disk"]  # and the log folder does not exist


def clock_tool(tmp_path, line, code):
    tool = tmp_path / "clock_sync.py"
    tool.write_text(f"import sys\nassert sys.argv[1:] == ['--source', '192.168.10.2'], sys.argv\nprint({line!r})\nsys.exit({code})\n")
    return str(tool)


def test_the_clock_is_asked_through_the_clock_tool_only_when_a_source_is_named(tmp_path):
    assert node.clock_verdict(None, None) is None
    assert node.clock_verdict(clock_tool(tmp_path, "CLOCK OK: following 192.168.10.2 (stratum 9), 0.4 ms off (limit 20.0 ms)", 0), "192.168.10.2") == (True, "following 192.168.10.2 (stratum 9), 0.4 ms off (limit 20.0 ms)")
    ok, why = node.clock_verdict(clock_tool(tmp_path, "CLOCK NOT OK: the clock is not synchronised to anything", 1), "192.168.10.2")
    assert not ok and why == "the clock is not synchronised to anything"
    assert node.clock_verdict(None, "192.168.10.2")[0] is False
    assert node.clock_verdict(str(tmp_path / "none.py"), "192.168.10.2")[0] is False


def test_go_is_exit_0_no_go_is_exit_1_and_both_are_logged(tmp_path, capsys):
    args = arguments(tmp_path, "--min-free-gb", "0.001")
    assert node.run(args, good(log_dir=str(tmp_path / "logs"))) == 0
    assert capsys.readouterr().out.strip().splitlines()[-1] == "GO"
    assert node.run(args, good(gps_fix_type=2)) == 1
    out = capsys.readouterr().out
    assert "[NO-GO] gps" in out and out.strip().splitlines()[-1] == "NO-GO: gps"
    records = [json.loads(line) for line in (tmp_path / "logs" / "preflight.jsonl").read_text().splitlines()]
    assert [record["verdict"] for record in records] == ["GO", "NO-GO"] and records[1]["failed"] == ["gps"]


def test_an_override_goes_ahead_only_with_a_reason_and_only_if_it_is_logged(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(node.getpass, "getuser", lambda: "pilot")
    args = arguments(tmp_path, "--override", "GPS under the hangar roof; flying from the apron", "--out", str(tmp_path / "record.json"))
    assert node.run(args, good(gps_hdop=3.0)) == 0
    out = capsys.readouterr().out
    assert 'NO-GO OVERRIDDEN by pilot: gps ("GPS under the hangar roof; flying from the apron")' in out
    record = json.loads((tmp_path / "logs" / "preflight.jsonl").read_text().splitlines()[-1])
    assert record["verdict"] == "NO-GO OVERRIDDEN" and record["override"] == {"by": "pilot", "reason": "GPS under the hangar roof; flying from the apron"}
    assert json.loads((tmp_path / "record.json").read_text())["override"]["by"] == "pilot"
    # An empty reason is no override.
    assert node.run(arguments(tmp_path, "--override", "  "), good(gps_hdop=3.0)) == 1
    capsys.readouterr()
    # Nowhere to log it: the override does not stand.
    blocked = tmp_path / "file"
    blocked.write_text("")
    args = node.build_parser().parse_args(["--log-dir", str(blocked / "logs"), "--override", "apron"])
    assert node.run(args, good(gps_hdop=3.0)) == 1
    assert "could not be logged" in capsys.readouterr().out
    args = node.build_parser().parse_args(["--override", "apron"])  # no log folder at all
    assert node.run(args, good(gps_hdop=3.0)) == 1


def test_the_command_is_installed_as_preflight():
    setup = (REPO / "src" / "drone_utils" / "setup.py").read_text()
    assert "'preflight = drone_utils.preflight_node:main'" in setup


def test_only_what_was_not_heard_is_asked_for_and_at_the_stream_keepers_rates():
    """The spec audit: asking for the rangefinder at 10 Hz would have halved the rate the missions fly on."""
    from drone_utils.stream_keeper import MESSAGE_RATES_HZ

    assert node.to_ask_for({}) == {1: 2.0, 24: 5.0, 147: 2.0, 173: 20.0}
    assert all(node.to_ask_for({})[message_id] == MESSAGE_RATES_HZ[message_id][1] for message_id in node.to_ask_for({}))
    assert node.to_ask_for({"gps": (1.0, None), "range": (1.0, None)}) == {1: 2.0, 147: 2.0}
    assert node.to_ask_for({"sys_status": 1, "gps": 1, "battery": 1, "range": 1, "offsets": [], "frames": {}}) == {}


def test_the_fence_points_come_with_the_parameters(tmp_path):
    dump = tmp_path / "aircraft.param"
    dump.write_text("FENCE_ENABLE,1\nFS_OPTIONS,16\nFENCE_TOTAL,0\n")
    snapshot = node.fill_offline(good(params_problems=None, params_source=None, baseline_sha256=None, fence_points=None), arguments(tmp_path, "--param-dump", str(dump)))
    assert snapshot.fence_points == 0 and snapshot.params_problems == []
    assert "fence" in [c.name for c in pf.run_checks(snapshot) if not c.go]


def test_a_preflight_that_could_not_look_is_no_go_printed_and_logged_and_cannot_be_overridden(tmp_path, capsys):
    """Exit 2, as before, but no longer silent: a mistyped file is a NO-GO in the log like any other."""
    (tmp_path / "logs").mkdir()
    code = node.main(["--camera", "camera/image", "--log-dir", str(tmp_path / "logs"), "--override", "in a hurry"])
    out = capsys.readouterr()
    assert code == 2 and "TOPIC=HZ" in out.err
    lines = out.out.strip().splitlines()
    assert len(lines) == 12 and all(line.startswith("[NO-GO]") and "could not run" in line for line in lines[:11])
    assert lines[-1].startswith("NO-GO: params, fence") and "OVERRIDDEN" not in out.out
    record = json.loads((tmp_path / "logs" / "preflight.jsonl").read_text().splitlines()[-1])
    assert record["verdict"] == "NO-GO" and len(record["failed"]) == 11 and "override" not in record


def test_nothing_is_asked_of_a_flight_controller_that_is_armed_or_whose_state_is_not_known():
    from types import SimpleNamespace

    assert node.may_ask(SimpleNamespace(connected=True, armed=False)) is True
    assert node.may_ask(SimpleNamespace(connected=True, armed=True)) is False
    assert node.may_ask(SimpleNamespace(connected=False, armed=False)) is False
    assert node.may_ask(None) is False


def test_an_empty_baseline_holds_nothing_and_says_so(tmp_path):
    assert pf.compare_params({"FENCE_ENABLE": 1.0}, {}) == ["the baseline has no parameters in it: there is nothing to hold the flight controller to"]
    empty = tmp_path / "baseline.param"
    empty.write_text("# nothing here\n")
    dump = tmp_path / "dump.param"
    dump.write_text("FENCE_ENABLE,1\n")
    args = node.build_parser().parse_args(["--baseline", str(empty), "--param-dump", str(dump)])
    snapshot = node.fill_offline(good(params_problems=None, params_source=None, baseline_sha256=None), args)
    assert "params" in [c.name for c in pf.run_checks(snapshot) if not c.go]


def test_a_pymavlink_that_is_not_installed_is_looked_for_in_the_users_packages_and_then_said(monkeypatch):
    """The sim's image hides the user's packages from ROS; pymavlink is installed there (PLT-07 runs the command inside it)."""
    import site

    monkeypatch.setitem(sys.modules, "pymavlink", None)  # an import of it now fails, wherever it is
    monkeypatch.setattr(sys, "path", [item for item in sys.path if item != site.getusersitepackages()])
    with pytest.raises(ImportError):
        node._mavutil()
    assert sys.path[-1] == site.getusersitepackages()  # looked for there, after everything else
