"""The sim's time base (FND-03): MAVROS time-sync config, the sim clock switch, and the check that proves it. No ROS."""

import ast
import json
import sys
from pathlib import Path

import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
sys.path.insert(0, str(SIM))

import clock_bridge  # noqa: E402
import time_check  # noqa: E402
from runner import events, result  # noqa: E402
from scenario.build import BuildError, build_run  # noqa: E402

from test_runner import SCORE, make_run  # noqa: E402  (the run-folder fixture)
from test_scenario import DELIVERY, read, scenario_file  # noqa: E402

MAVROS_CONFIG = REPO / "config" / "mavros" / "argus_mavros.yaml"
MAVROS_LAUNCH = REPO / "config" / "mavros" / "argus_mavros.launch.py"
START = (SIM / "container" / "start.sh").read_text()
TIME_BASE = SIM / "scenarios" / "delivery" / "time_base.yaml"
LAUNCH_FILES = sorted(path for path in (REPO / "src").glob("*/launch/*.launch.py") if "mission_executor" in path.read_text())


# --- MAVROS: the time-sync requirement ---------------------------------------------------------


def test_argus_mavros_config_turns_time_sync_on_at_10_hz():
    data = yaml.safe_load(MAVROS_CONFIG.read_text())
    time_plugin = data["/**/time"]["ros__parameters"]  # the sys_time plugin's node is /mavros/time
    assert time_plugin["timesync_rate"] == 10.0
    assert time_plugin["timesync_mode"] == "MAVLINK"
    assert time_check.TIMESYNC_RATE_HZ == time_plugin["timesync_rate"]


def test_argus_mavros_config_is_a_params_file_without_feature_flags():
    data = yaml.safe_load(MAVROS_CONFIG.read_text())
    assert "features" not in data
    assert all(isinstance(block, dict) and set(block) == {"ros__parameters"} for block in data.values())


def test_argus_mavros_launch_loads_the_argus_config_last_and_takes_use_sim_time():
    source = MAVROS_LAUNCH.read_text()
    tree = ast.parse(source)
    node = next(
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "Node" and any(kw.arg == "executable" and getattr(kw.value, "value", "") == "mavros_node" for kw in n.keywords)
    )
    parameters = next(kw.value for kw in node.keywords if kw.arg == "parameters")
    order = [ast.unparse(item) for item in parameters.elts]
    assert "apm_pluginlists.yaml" in order[0] and "apm_config.yaml" in order[1]
    assert order[2] == "LaunchConfiguration('config')"  # after MAVROS's own defaults, so it wins
    assert "'use_sim_time'" in order[3] and "'fcu_url'" in order[3]
    assert not any(kw.arg == "name" for kw in node.keywords)  # unnamed: the parameters reach every node of the MAVROS process
    defaults = {
        call.args[0].value: next((kw.value for kw in call.keywords if kw.arg == "default_value"), None)
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and getattr(call.func, "id", "") == "DeclareLaunchArgument"
    }
    assert defaults["use_sim_time"].value == "false"
    assert "argus_mavros.yaml" in source


# --- the sim clock switch ------------------------------------------------------------------------


def test_clock_bridge_passes_one_message_per_4_ms_of_sim_time():
    throttle = clock_bridge.ClockThrottle()
    passed = [t for t in range(0, 1_000_000_000, 1_000_000) if throttle.forward(t)]  # one sim second of 1 ms steps
    assert len(passed) == 250 and passed[:3] == [0, 4_000_000, 8_000_000]
    assert clock_bridge.STEP_NS == 4_000_000


def test_clock_bridge_follows_a_clock_that_starts_late_skips_steps_or_resets():
    throttle = clock_bridge.ClockThrottle()
    assert throttle.forward(57_301_000_000)  # the first message, whatever the time
    assert not throttle.forward(57_302_000_000) and not throttle.forward(57_304_999_999)
    assert throttle.forward(57_305_000_000)
    assert throttle.forward(57_400_000_000)  # a gap
    assert not throttle.forward(57_403_000_000)
    assert throttle.forward(1_000_000)  # the world was reset: time went back
    assert not throttle.forward(2_000_000) and throttle.forward(5_000_000)


def test_the_camera_bridge_file_has_no_clock_in_it():
    # The flags-off sim bridges the camera only; the clock goes through sim/clock_bridge.py on sim time.
    assert "/clock" not in (SIM / "config" / "bridge.yaml").read_text()


def test_start_script_is_on_wall_time_unless_asked():
    assert "SIM_TIME=${SIM_TIME:-0}" in START
    # The 2026 start-up is still there, word for word, on the branch taken without sim time.
    assert 'start mavros "$ARGUS_REPO/mavros_boot.sh" sim' in START
    assert 'bridge_args=(-p "config_file:=$BRIDGE_FILE")' in START
    sim_time_only = START.count("if [[ $SIM_TIME == 1 ]]; then")
    assert sim_time_only == 3  # the clock bridge, MAVROS, and the wait for time sync
    assert 'start clock python3 "$ARGUS_SIM/clock_bridge.py"' in START
    assert "use_sim_time:=true" in START and "argus_mavros.launch.py" in START and "--wait-synced" in START
    # On sim time MAVROS is on SITL's own second serial port, with the stream keeper (PLT-06).
    assert "MAVROS_PORT=5762" in START
    assert '"fcu_url:=tcp://127.0.0.1:$MAVROS_PORT" "config:=$ARGUS_SIM/config/mavros_sim.yaml" use_sim_time:=true stream_keeper:=true' in START
    assert START.count("stream_keeper:=true") == 1 and START.count("tcp://") == 1
    assert "SIM_TIME: ${SIM_TIME:-0}" in (SIM / "compose.yaml").read_text()


def test_a_scenario_run_takes_sim_time_from_its_run_folder_only():
    # A scenario built without sim time must not be switched by a stray SIM_TIME=1 in the shell.
    line = next(line for line in START.splitlines() if line.strip().startswith("SIM_TIME=$(python3"))
    assert '.get("sim_time") is True' in line and "$RUN_DIR/scenario.json" in line


def launch_sim_time_problems(source: str, name: str = "<launch>") -> list:
    """What stops a launch file from running all its nodes on sim time when asked, and only then."""
    tree = ast.parse(source)

    def called(node) -> str:
        return getattr(node.func, "id", getattr(node.func, "attr", "")) if isinstance(node, ast.Call) else ""

    problems = []
    declared = [n for n in ast.walk(tree) if called(n) == "DeclareLaunchArgument" and n.args and getattr(n.args[0], "value", None) == "use_sim_time"]
    if len(declared) != 1:
        return [f"{name}: declares use_sim_time {len(declared)} times, not once"]
    default = next((kw.value for kw in declared[0].keywords if kw.arg == "default_value"), None)
    if not (isinstance(default, ast.Constant) and default.value == "false"):
        problems.append(f"{name}: use_sim_time must default to 'false'")
    description = next(n for n in ast.walk(tree) if called(n) == "LaunchDescription")
    items = description.args[0].elts
    setters = [index for index, item in enumerate(items) if called(item) == "SetParameter"]
    if len(setters) != 1:
        return problems + [f"{name}: needs exactly one SetParameter in the launch description"]
    setter = items[setters[0]]
    keywords = {kw.arg: ast.unparse(kw.value) for kw in setter.keywords}
    if keywords != {"name": "'use_sim_time'", "value": "True", "condition": "IfCondition(LaunchConfiguration('use_sim_time'))"}:
        problems.append(f"{name}: SetParameter must set use_sim_time to True only if the argument is true, got {keywords}")
    if any(called(item) != "DeclareLaunchArgument" for item in items[: setters[0]]):
        problems.append(f"{name}: SetParameter must come before every node, or the nodes before it stay on wall time")
    return problems


def test_thirteen_launch_files_start_the_executor():
    assert len(LAUNCH_FILES) == 13  # the survey's own launch, with the localisation node, is the thirteenth (LOC-5)


@pytest.mark.parametrize("path", LAUNCH_FILES, ids=lambda p: p.name)
def test_every_executor_launch_can_put_all_its_nodes_on_sim_time_and_defaults_to_wall_time(path):
    assert launch_sim_time_problems(path.read_text(), path.name) == []


def test_the_launch_rule_catches_each_way_of_getting_it_wrong():
    good = (REPO / "src" / "drone_mission_demo" / "launch" / "package_delivery_demo.launch.py").read_text()
    assert launch_sim_time_problems(good) == []
    assert launch_sim_time_problems(good.replace('default_value="false"', 'default_value="true"', 1)) != []
    assert launch_sim_time_problems(good.replace('condition=IfCondition(LaunchConfiguration("use_sim_time")),', "")) != []
    moved = good.replace("            takeoff_service_node,\n", "", 1).replace("            SetParameter(", "            takeoff_service_node,\n            SetParameter(", 1)
    assert moved != good and launch_sim_time_problems(moved) != []
    assert launch_sim_time_problems(good.replace('"use_sim_time",\n                default_value', '"sim_clock",\n                default_value', 1)) != []


# --- the time check's arithmetic and verdict -----------------------------------------------------------


def test_percentile_is_nearest_rank():
    values = [5.0, 1.0, 3.0, 2.0, 4.0]
    assert time_check.percentile(values, 0.5) == 3.0
    assert time_check.percentile(values, 0.95) == 5.0
    assert time_check.percentile(values, 0.2) == 1.0
    assert time_check.percentile(list(range(1, 101)), 0.95) == 95
    with pytest.raises(ValueError):
        time_check.percentile([], 0.5)


def test_stamp_agreement_summary_is_in_milliseconds():
    agreement = time_check.StampAgreement()
    assert agreement.summary() == {"count": 0, "median_ms": None, "p95_abs_ms": None, "max_abs_ms": None}
    for index in range(100):
        agreement.add(10.0 + index, 10.0 + index - 0.004)  # 4 ms behind the clock
    agreement.add(200.0, 200.030)  # one stamp 30 ms ahead
    summary = agreement.summary()
    assert summary["count"] == 101
    assert summary["median_ms"] == pytest.approx(4.0, abs=1e-6)
    assert summary["p95_abs_ms"] == pytest.approx(4.0, abs=1e-6)
    assert summary["max_abs_ms"] == pytest.approx(30.0, abs=1e-6)


def test_a_synchronised_stamp_is_whole_milliseconds_plus_a_recent_offset():
    offset = 43_720_117  # ns, MAVROS's estimate
    boot_ms = 184_701
    stamp = boot_ms * 1_000_000 + offset
    assert time_check.is_synced(stamp, [offset])
    assert time_check.is_synced(stamp, [43_707_000, offset, 43_733_500])  # made with the offset before the newest
    assert not time_check.is_synced(stamp + 1, [offset])
    assert not time_check.is_synced(184_745_000_000, [offset])  # a stamp taken from the 1 ms sim clock on arrival
    assert not time_check.is_synced(stamp, [])
    # MAVROS applies an offset only when it is above zero; a stamp cannot have been made with another.
    assert not time_check.is_synced(boot_ms * 1_000_000, [0])
    assert not time_check.is_synced(boot_ms * 1_000_000 - 5_000_000, [-5_000_000])


GOOD_POSE = {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}
GOOD_SYNC = {"count": 2000, "median_round_trip_ms": 3.0, "largest_offset_step_ms": 0.02}
GOOD_NODES = {"/mavros/time": True, "/mission_executor": True, "/target_cv": True, "/launch_ros_4242": False, "/mavros/param": None}


def check(pose=GOOD_POSE, synced=GOOD_POSE, sync=GOOD_SYNC, nodes=GOOD_NODES, rate=10.0):
    return time_check.verdict(pose, synced, sync, nodes, rate)


def test_verdict_passes_when_stamps_agree_time_sync_runs_and_every_node_is_on_sim_time():
    assert check() == {"passed": True, "problems": []}
    assert time_check.LIMIT_MS == 20.0


@pytest.mark.parametrize(
    "change, word",
    [
        ({"pose": {**GOOD_POSE, "p95_abs_ms": 20.5}}, "95th percentile"),
        ({"pose": {**GOOD_POSE, "count": 99}}, "only 99 poses"),
        ({"synced": {**GOOD_POSE, "count": 12}}, "arrival times"),
        ({"synced": {**GOOD_POSE, "p95_abs_ms": 31.0}}, "synchronised pose stamps are 31.0 ms"),
        ({"sync": {**GOOD_SYNC, "count": 3}}, "not running"),
        ({"rate": 0.0}, "timesync_rate is 0.0"),
        ({"rate": None}, "did not report its timesync_rate"),
        ({"nodes": {**GOOD_NODES, "/target_cv": False}}, "/target_cv is on wall time"),
        ({"nodes": {**GOOD_NODES, "/session_logger": None}}, "/session_logger did not report"),
        ({"nodes": {"/launch_ros_1": True, "/argus_rate_check": False}}, "no ROS nodes"),
    ],
)
def test_verdict_fails_and_says_why(change, word):
    report = check(**change)
    assert report["passed"] is False
    assert len(report["problems"]) == 1 and word in report["problems"][0]


def test_a_passing_p95_exactly_on_the_limit_passes():
    assert check(pose={**GOOD_POSE, "p95_abs_ms": 20.0})["passed"]


def test_report_file_round_trip(tmp_path):
    agreement, synced, sync = time_check.StampAgreement(), time_check.StampAgreement(), time_check.TimesyncLog()
    for index in range(150):
        agreement.add(1.0 + index * 0.05, 1.0 + index * 0.05 - 0.004)
        synced.add(1.0 + index * 0.05, 1.0 + index * 0.05 - 0.004)
        sync.add(43_000_000 + index * 13_000, 3.0)
    report = time_check.build_report(agreement, synced, sync, {"/mission_executor": True}, 10.0, 7.5)
    assert report["passed"] and report["pose"]["count"] == 150 and report["pose_synced"]["count"] == 150
    assert report["timesync"]["largest_offset_step_ms"] == pytest.approx(0.013)
    assert time_check.read_time_check(tmp_path) is None
    (tmp_path / "time_check.json").write_text(json.dumps(report))
    assert time_check.read_time_check(tmp_path)["passed"] is True
    (tmp_path / "time_check.json").write_text("{not json")
    assert time_check.read_time_check(tmp_path) is None
    (tmp_path / "time_check.json").write_text(json.dumps({"passed": "yes"}))
    assert time_check.read_time_check(tmp_path) is None


# --- frame stamps (FND-01) ----------------------------------------------------------------------------------


def stamps(count=300, lag_ms=9.0, detection_lag_ms=120.0, own_clock=False):
    """Frames at 30 Hz heard lag_ms after their stamp, and one detection per frame heard detection_lag_ms after it."""
    frames = time_check.FrameStamps()
    for index in range(count):
        stamp = 10_000_000_000 + index * 33_333_333
        frames.on_frame(stamp + int(lag_ms * 1e6), stamp)
        heard = stamp + int(detection_lag_ms * 1e6)
        frames.on_detection(heard, heard - 2_000_000 if own_clock else stamp)  # 2026: stamped by the detector as it publishes
    return frames


def test_frames_stamped_at_capture_and_detections_carrying_those_stamps_pass():
    summary = stamps().summary()
    assert summary["frames"]["count"] == 300 and summary["frames"]["p95_abs_ms"] == pytest.approx(9.0, abs=0.01)
    assert summary["detections"] == summary["detections_with_a_frame_stamp"] == 300
    assert summary["detection_age"]["median_ms"] == pytest.approx(120.0, abs=0.01)  # how old the frame is when the mission hears of it
    assert time_check.frame_problems(summary, require_image_stamps=True) == []
    assert time_check.FRAME_PERIOD_MS == pytest.approx(33.33, abs=0.01)


def test_a_frame_stamp_more_than_a_frame_period_from_the_clock_fails():
    problems = time_check.frame_problems(stamps(lag_ms=40.0).summary(), require_image_stamps=False)
    assert len(problems) == 1 and "one frame period" in problems[0]
    assert time_check.frame_problems(stamps(lag_ms=30.0).summary(), require_image_stamps=False) == []


def test_detections_stamped_with_the_detectors_own_clock_fail_only_where_the_flag_is_on():
    own = stamps(own_clock=True).summary()
    assert own["detections_with_a_frame_stamp"] == 0 and own["detection_age"]["median_ms"] == pytest.approx(2.0, abs=0.01)
    assert time_check.frame_problems(own, require_image_stamps=False) == []  # flags off: the 2026 stamps are not judged
    problems = time_check.frame_problems(own, require_image_stamps=True)
    assert len(problems) == 1 and "its own clock" in problems[0]


def test_too_few_frames_or_detections_to_judge_is_said():
    assert "only 10 frames" in time_check.frame_problems(stamps(count=10).summary(), True)[0]
    few = stamps()
    few.detections, few.from_a_frame = 5, 5
    assert "only 5 detections" in time_check.frame_problems(few.summary(), True)[0]
    assert time_check.frame_problems(time_check.FrameStamps().summary(), False) == []  # no camera heard and nothing required


def test_the_scenario_says_whether_detections_must_carry_frame_stamps(tmp_path):
    assert not time_check.detector_stamps_from_image(tmp_path)
    (tmp_path / "scenario.json").write_text(json.dumps({"node_flags": {"target_cv": {"stamp_from_image": True}}}))
    assert time_check.detector_stamps_from_image(tmp_path)
    (tmp_path / "scenario.json").write_text(json.dumps({"node_flags": {}}))
    assert not time_check.detector_stamps_from_image(tmp_path)
    run = build_run(TIME_BASE, tmp_path / "run", variant="on")
    assert time_check.detector_stamps_from_image(run)
    assert not time_check.detector_stamps_from_image(build_run(TIME_BASE, tmp_path / "off", variant="off"))


def test_the_report_carries_the_frame_section_and_its_problems_fail_it():
    agreement, synced, sync = time_check.StampAgreement(), time_check.StampAgreement(), time_check.TimesyncLog()
    for index in range(150):
        agreement.add(1.0 + index * 0.05, 1.0 + index * 0.05 - 0.004)
        synced.add(1.0 + index * 0.05, 1.0 + index * 0.05 - 0.004)
        sync.add(43_000_000 + index * 13_000, 3.0)
    good = time_check.build_report(agreement, synced, sync, {"/mission_executor": True}, 10.0, 7.5, stamps(), True)
    assert good["passed"] and good["frames"]["detections"] == 300
    bad = time_check.build_report(agreement, synced, sync, {"/mission_executor": True}, 10.0, 7.5, stamps(own_clock=True), True)
    assert not bad["passed"] and "its own clock" in bad["problems"][0]


# --- scenarios and the runner ---------------------------------------------------------------------------------


def test_sim_time_scenario_runs_on_sim_time_only_in_variant_on(tmp_path):
    scenario = scenario_file(tmp_path, sim_time=True, flags={"time_health_guard": True})
    on = build_run(scenario, tmp_path / "on", variant="on")
    off = build_run(scenario, tmp_path / "off", variant="off")
    assert read(on, "scenario.json")["sim_time"] is True
    assert read(on, "scenario.json")["launch"]["arguments"]["use_sim_time"] == "true"
    assert read(on, "sequence.yaml")["mission_sequence"]["features"]["time_health_guard"] is True
    assert read(off, "scenario.json")["sim_time"] is False
    assert "use_sim_time" not in read(off, "scenario.json")["launch"]["arguments"]
    assert read(off, "sequence.yaml")["mission_sequence"]["features"]["time_health_guard"] is False
    # Whatever the variant, the run folder's bridge file is the plain one: the clock has its own bridge.
    assert (on / "bridge.yaml").read_text() == (off / "bridge.yaml").read_text() == (SIM / "config" / "bridge.yaml").read_text()


def test_scenarios_without_the_key_stay_on_wall_time(tmp_path):
    run = build_run(DELIVERY, tmp_path / "run", variant="on")
    manifest = read(run, "scenario.json")
    assert manifest["sim_time"] is False and "use_sim_time" not in manifest["launch"]["arguments"]


@pytest.mark.parametrize("value", ["yes", 1, "true"])
def test_sim_time_must_be_true_or_false(tmp_path, value):
    with pytest.raises(BuildError, match="sim_time"):
        build_run(scenario_file(tmp_path, sim_time=value), tmp_path / "run", variant="on")


def test_shipped_time_base_scenario_is_the_delivery_baseline_on_one_clock(tmp_path):
    scenario, baseline = yaml.safe_load(TIME_BASE.read_text()), yaml.safe_load(DELIVERY.read_text())
    assert scenario["sim_time"] is True
    assert "flags" not in scenario  # the stale-data guard stays off until frames are fresh enough in the sim
    assert scenario["node_flags"] == {"target_cv": {"stamp_from_image": True}}
    for key in ("mission", "site", "sequence", "params", "targets", "objects", "wind"):
        assert scenario[key] == baseline[key], key
    run = build_run(TIME_BASE, tmp_path / "run", variant="on")
    assert read(run, "scenario.json")["sim_time"] is True
    assert read(run, "params.yaml")["target_cv"]["ros__parameters"]["features"]["stamp_from_image"] is True
    off = build_run(TIME_BASE, tmp_path / "off", variant="off")
    assert read(off, "params.yaml")["target_cv"]["ros__parameters"]["features"]["stamp_from_image"] is False


def test_a_sim_time_run_gets_the_time_check_beside_it():
    calm = {"wind": {"level": "calm", "mean_mps": 0.0, "gust_mps": 0.0, "discrete_gust_mps": 0.0}, "gps_events": []}
    assert events.tools_needed(calm) == set()
    assert events.tools_needed({**calm, "sim_time": False}) == set()
    assert events.tools_needed({**calm, "sim_time": True}) == {"time_check", "rate_check"}
    assert "time check" in events.unsupported({**calm, "sim_time": True}, {"rate_check"})
    assert "rate check" in events.unsupported({**calm, "sim_time": True}, {"time_check"})
    assert events.unsupported({**calm, "sim_time": True}, {"time_check", "rate_check"}) == ""
    fly = (SIM / "runner" / "fly.py").read_text()
    assert '"time_check": SIM_DIR / "time_check.py"' in fly and '"rate_check": SIM_DIR / "rate_check.py"' in fly


PASSED = {"passed": True, "problems": [], "pose": {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}}
FAILED = {"passed": False, "problems": ["node /target_cv is on wall time (use_sim_time is false)"], "pose": {"count": 4000, "median_ms": 4.0, "p95_abs_ms": 5.3, "max_abs_ms": 6.6}}


RATES_HELD = {"passed": True, "problems": [], "topics": {"/mavros/local_position/pose": {"measured_hz": 30.2, "asked_hz": 30.0, "status": "PASS"}}}
RATES_LOST = {
    "passed": False,
    "problems": ["/mavros/local_position/pose is at 4.2 Hz, under 90% of the 30 Hz asked (LOCAL_POSITION_NED)"],
    "topics": {"/mavros/local_position/pose": {"measured_hz": 4.2, "asked_hz": 30.0, "status": "FAIL"}},
}


def sim_time_run(tmp_path, report, rtf=0.95, rates=RATES_HELD):
    run = make_run(tmp_path, rtf=rtf, score=SCORE, manifest={"variant": "on", "sim_time": True})
    if report is not None:
        (run / "time_check.json").write_text(json.dumps(report))
    if rates is not None:
        (run / "rate_check.json").write_text(json.dumps(rates))
    return run


@pytest.mark.parametrize("rates, word", [(RATES_LOST, "pose is at 4.2 Hz"), (None, "no rate check")])
def test_sim_time_run_whose_message_rates_were_not_held_is_invalid(tmp_path, rates, word):
    summary = result.run_result(sim_time_run(tmp_path, PASSED, rates=rates))
    assert summary["status"] == "INVALID" and word in summary["reason"]


def test_sim_time_report_shows_the_measured_rates(tmp_path):
    report = result.render_report(result.run_result(sim_time_run(tmp_path, PASSED)))
    assert "| MAVROS message rates (sim time) | PASS: local_position 30.2 Hz |" in report


def test_sim_time_run_with_a_passing_time_check_is_ok_and_reports_it(tmp_path):
    summary = result.run_result(sim_time_run(tmp_path, PASSED))
    assert summary["status"] == "OK" and summary["sim_time"] is True
    assert summary["time_check"]["passed"] is True
    assert "Pose stamps against the sim clock" in result.render_report(summary) and "5.3 ms" in result.render_report(summary)


def test_sim_time_run_counts_even_when_the_sim_was_slow(tmp_path):
    # Every node ran on the sim clock, so a slow sim changes nothing the vehicle sees.
    summary = result.run_result(sim_time_run(tmp_path, PASSED, rtf=0.5))
    assert summary["status"] == "OK"
    assert summary["real_time_factor"] == pytest.approx(0.5)


def test_wall_time_run_that_was_slow_is_still_invalid(tmp_path):
    assert result.run_result(make_run(tmp_path, rtf=0.5, score=SCORE))["status"] == "INVALID"
    summary = result.run_result(make_run(tmp_path / "b", rtf=0.95, score=SCORE))
    assert summary["status"] == "OK" and summary["sim_time"] is False and summary["time_check"] is None
    assert "Pose stamps against the sim clock" not in result.render_report(summary)


@pytest.mark.parametrize("report, word", [(FAILED, "/target_cv is on wall time"), (None, "no time check")])
def test_sim_time_run_whose_time_check_did_not_pass_is_invalid(tmp_path, report, word):
    summary = result.run_result(sim_time_run(tmp_path, report))
    assert summary["status"] == "INVALID" and word in summary["reason"]
