"""The recorder's side of the replay harness (SIM-09): what is run, the taps' cutting, the launch files, the runner's --record. No ROS."""

import importlib.util
import json
import struct
import subprocess
import sys
import types
from pathlib import Path
from unittest import mock

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay import cdr, check_defs, msgdefs, record_run, taps, topics  # noqa: E402
from runner import events  # noqa: E402
from scenario.build import build_run  # noqa: E402

import replay_flight as flight  # noqa: E402

BASELINE = SIM / "scenarios" / "delivery" / "baseline.yaml"


# --- the taps ---


def test_the_tap_cuts_the_header_from_the_front_of_an_image_without_unpacking_it():
    header = flight.header(1234 * flight.S + 5, "camera_link")
    image = cdr.encode("std_msgs/msg/Header", header) + struct.pack("<II", 360, 640) + b"\xab" * 5000  # what follows is the image's own
    cut = taps.header_bytes(image)
    assert len(cut) == 4 + 8 + 4 + len("camera_link") + 1
    assert cdr.decode("std_msgs/msg/Header", cut) == header
    odd = flight.header(7, "ab")  # a frame id that does not end on a multiple of four: the cut is the header and no more
    assert cdr.decode("std_msgs/msg/Header", taps.header_bytes(cdr.encode("sensor_msgs/msg/Range", {"header": odd, "range": 3.0}))) == odd
    assert cdr.decode("std_msgs/msg/Header", taps.header_bytes(cdr.encode("geometry_msgs/msg/PoseStamped", {"header": header}))).frame_id == "camera_link"
    for bad in (b"", b"\x00\x01\x00\x00" + b"\x00" * 8 + struct.pack("<I", 999) + b"abc", b"\x07" * 40):
        with pytest.raises(ValueError):
            taps.header_bytes(bad)


def test_the_taps_arguments():
    assert taps.parse_args([]).image == list(topics.TAPPED_IMAGES) and taps.parse_args([]).sim_clock is False
    args = taps.parse_args(["--image", "/a", "--image", "/b", "--sim-clock"])
    assert args.image == ["/a", "/b"] and args.sim_clock and not args.use_sim_time


# --- what the recorder runs ---


def test_the_record_command_names_every_topic_and_the_storage():
    command = record_run.record_command(Path("/runs/x/bag"), "mcap", sim_time=False)
    assert command[:7] == ["ros2", "bag", "record", "--storage", "mcap", "--output", "/runs/x/bag"]
    assert command[7:9] == ["--storage-preset-profile", "fastwrite"] and "--use-sim-time" not in command  # plain records: read with the standard library
    recorded = command[9:]
    assert recorded == topics.record_topics() and "/replay/frame_seen/camera/image" in recorded and "/replay/sim_clock" in recorded
    for needed in ("/mavros/local_position/pose", "/mavros/imu/data", "/mavros/global_position/global", "/mavros/state", "/mavros/rangefinder/rangefinder",
                   "/mavros/setpoint_raw/local", "/camera/camera_info", "/perception/target_cv/detections", "/estimator/targets"):  # fmt: skip
        assert needed in recorded, needed
    assert not any(topic in ("/camera/image", "/target_cv/annotated") for topic in recorded)  # no images in the bag
    on_sim_time = record_run.record_command(Path("bag"), "sqlite3", sim_time=True)
    assert "--use-sim-time" in on_sim_time and "--storage-preset-profile" not in on_sim_time and on_sim_time[4] == "sqlite3"


def test_the_sim_clock_is_tapped_only_when_the_nodes_are_not_on_it():
    assert record_run.taps_command(sim_time=False)[-1] == "--sim-clock" and "--use-sim-time" not in record_run.taps_command(sim_time=False)
    assert record_run.taps_command(sim_time=True)[-1] == "--use-sim-time" and "--sim-clock" not in record_run.taps_command(sim_time=True)
    assert record_run.taps_command(False)[1].endswith("sim/replay/taps.py") and record_run.taps_command(False).count("--image") == 2
    assert record_run.choose_storage(True) == "mcap" and record_run.choose_storage(False) == "sqlite3"


def test_stop_interrupts_and_waits():
    sleeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    record_run.stop(sleeper, grace_s=5.0)
    assert sleeper.poll() is not None
    record_run.stop(sleeper)  # already gone: nothing to do
    record_run.stop(None)
    assert record_run.main([]) == 2


# --- the runner ---


def test_record_is_asked_for_on_the_command_line_and_runs_beside_the_flight(tmp_path):
    plain = json.loads((build_run(BASELINE, tmp_path / "plain") / "scenario.json").read_text())
    recorded = json.loads((build_run(BASELINE, tmp_path / "recorded", record=True) / "scenario.json").read_text())
    assert plain["record"] is False and recorded["record"] is True and record_run.wanted(recorded) and not record_run.wanted(plain)
    different = {key for key in recorded if key != "record" and recorded[key] != plain[key]}
    assert different <= {"vehicle", "launch"} and recorded["flags"] == plain["flags"] == {}  # only the run folder's own paths differ
    assert recorded["launch"]["file"] == plain["launch"]["file"] and recorded["sim_time"] == plain["sim_time"] and recorded["wind"] == plain["wind"]
    assert "record" in events.tools_needed(recorded) and "record" not in events.tools_needed(plain)
    assert "sim/replay/record_run.py" in events.unsupported(recorded, set())
    assert '"record": SIM_DIR / "replay" / "record_run.py"' in (SIM / "runner" / "fly.py").read_text()
    script = (SIM / "run_scenarios.sh").read_text()
    assert "--record) record=1 ;;" in script and "build_args+=(--record)" in script
    listed = subprocess.run([str(SIM / "run_scenarios.sh"), "--list", "delivery/baseline", "--record"], capture_output=True, text=True)
    assert listed.returncode == 0 and "baseline.yaml 1 off" in listed.stdout


# --- the launch files ---


class _Stub:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def load_launch(name, monkeypatch, arguments):
    """Load a launch file with `launch` replaced by stand-ins, and give its description's actions and processes."""

    class Configuration(_Stub):
        def perform(self, _context):
            return arguments[self.args[0]]

    launch = types.ModuleType("launch")
    launch.LaunchDescription = lambda actions: actions
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = type("DeclareLaunchArgument", (_Stub,), {})
    actions.ExecuteProcess = type("ExecuteProcess", (_Stub,), {})
    actions.OpaqueFunction = type("OpaqueFunction", (_Stub,), {})
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = Configuration
    for module_name, module in (("launch", launch), ("launch.actions", actions), ("launch.substitutions", substitutions)):
        monkeypatch.setitem(sys.modules, module_name, module)
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), SIM / "replay" / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declared = {action.args[0]: action.kwargs.get("default_value") for action in description if type(action).__name__ == "DeclareLaunchArgument"}
    (opaque,) = [action for action in description if type(action).__name__ == "OpaqueFunction"]
    return module, declared, [process.kwargs["cmd"] for process in opaque.kwargs["function"](mock.MagicMock())]


def test_record_launch_records_the_list_as_mcap_and_taps_nothing_unless_asked(monkeypatch):
    base = {"out": "/data/flight1", "storage": "mcap", "use_sim_time": "false", "sim_clock": "false", "tap_images": ""}
    _module, declared, commands = load_launch("record.launch.py", monkeypatch, base)
    assert declared == {"out": None, "storage": "mcap", "use_sim_time": "false", "sim_clock": "false", "tap_images": ""}
    (record,) = commands  # the aircraft's default: the recorder alone, which only listens
    assert record == record_run.record_command(Path("/data/flight1"), "mcap", sim_time=False, tapped_images=[])
    assert not any(topic.startswith("/replay/frame_seen") for topic in record)
    assert "/argus/camera/down/camera_info" in record and "/camera/camera_info" in record  # the 2027 driver's name and the 2026 one
    _module, _declared, commands = load_launch("record.launch.py", monkeypatch, {**base, "tap_images": "/camera/image, /argus/camera/down/image_raw"})
    record, tap = commands
    assert "/replay/frame_seen/camera/image" in record and "/replay/frame_seen/argus/camera/down/image_raw" in record
    assert tap[1].endswith("sim/replay/taps.py") and tap[2:6] == ["--image", "/camera/image", "--image", "/argus/camera/down/image_raw"]
    assert "--sim-clock" not in tap and "--use-sim-time" not in tap and "--no-images" not in tap
    _module, _declared, commands = load_launch("record.launch.py", monkeypatch, {**base, "storage": "sqlite3", "use_sim_time": "true", "tap_images": "/camera/image"})
    assert "--use-sim-time" in commands[0] and commands[0][4] == "sqlite3" and commands[1][-1] == "--use-sim-time"
    _module, _declared, commands = load_launch("record.launch.py", monkeypatch, {**base, "sim_clock": "true"})
    assert commands[1][-2:] == ["--no-images", "--sim-clock"]  # the sim's clock alone
    assert taps.parse_args(["--no-images", "--sim-clock"]).image == []


def test_play_launch_replays_with_the_bags_clock_and_keeps_the_flights_outputs_apart(monkeypatch):
    monkeypatch.setattr("subprocess.run", lambda *args, **kwargs: types.SimpleNamespace(stdout="Type: mavros_msgs/msg/State\nPublisher count: 0\nSubscription count: 0\n", stderr=""))
    arguments = {"bag": "/data/flight1", "rate": "0.5", "keep_outputs": "false", "beside_live_mavros": "false"}
    module, declared, (command,) = load_launch("play.launch.py", monkeypatch, arguments)
    assert declared == {"bag": None, "rate": "1.0", "keep_outputs": "false", "beside_live_mavros": "false"}
    assert command[:8] == ["ros2", "bag", "play", "/data/flight1", "--clock", "--rate", "0.5", "--topics"]
    played, remaps = command[8 : command.index("--remap")], command[command.index("--remap") + 1 :]
    assert "/clock" not in played and "/mavros/local_position/pose" in played and "/replay/sim_clock" in played  # the player's clock is the only /clock
    assert sorted(played + ["/clock"]) == sorted(topics.record_topics())
    assert "/estimator/targets:=/recorded/estimator/targets" in remaps and "/perception/target_cv/detections:=/recorded/perception/target_cv/detections" in remaps
    assert "/mavros/setpoint_raw/local:=/recorded/mavros/setpoint_raw/local" in remaps
    assert not any(remap.startswith(("/mavros/local_position", "/camera/camera_info", "/mavros/rangefinder", "/clock")) for remap in remaps)  # inputs keep their names
    _module, _declared, (kept,) = load_launch("play.launch.py", monkeypatch, {**arguments, "keep_outputs": "true"})
    kept_remaps = kept[kept.index("--remap") + 1 :]
    assert sorted(kept_remaps) == sorted(f"{topic}:=/recorded{topic}" for topic in ("/mavros/setpoint_raw/local", "/mavros/setpoint_raw/global", "/mavros/setpoint_position/local"))
    assert "/clock" not in kept  # detections and tracks under their own names; recorded setpoints never on MAVROS's topics
    # Whatever setpoint topic the recorder lists is moved aside: the list is found by name, not kept by hand.
    played_setpoints = [topic for topic in kept[kept.index("--topics") + 1 : kept.index("--remap")] if "/setpoint" in topic]
    assert sorted(played_setpoints) == sorted(module.SETPOINTS) and len(module.SETPOINTS) == 3


def test_play_launch_does_not_start_beside_a_live_mavros_or_when_it_cannot_tell(monkeypatch):
    arguments = {"bag": "b", "rate": "1.0", "keep_outputs": "false", "beside_live_mavros": "false"}

    def answer(stdout="", stderr="", error=None):
        def run(*args, **kwargs):
            if error is not None:
                raise error
            return types.SimpleNamespace(stdout=stdout, stderr=stderr)

        monkeypatch.setattr("subprocess.run", run)

    answer("Type: mavros_msgs/msg/State\nPublisher count: 1\nSubscription count: 2\n")
    with pytest.raises(RuntimeError, match="something is publishing /mavros/state in this ROS domain"):
        load_launch("play.launch.py", monkeypatch, arguments)
    module, _declared, commands = load_launch("play.launch.py", monkeypatch, {**arguments, "beside_live_mavros": "true"})
    assert len(commands) == 1  # the operator said so
    for silent in (dict(error=subprocess.TimeoutExpired("ros2", 20)), dict(error=OSError("no ros2")), dict(stdout="", stderr="something else went wrong")):
        answer(**silent)
        with pytest.raises(RuntimeError, match="could not find out whether a MAVROS is live"):  # not knowing is not "no"
            load_launch("play.launch.py", monkeypatch, arguments)
    answer(stderr="Unknown topic '/mavros/state'\n")  # nobody publishes or listens: that is a "no"
    assert len(load_launch("play.launch.py", monkeypatch, arguments)[2]) == 1
    assert module.publisher_count("Publisher count: 3\n") == 3 and module.publisher_count("Unknown topic '/mavros/state'") == 0 and module.publisher_count("") is None


# --- the check of the layouts against ROS (its plain parts; the check itself runs where ROS is) ---


def test_ros_type_names_are_read_as_the_layouts_write_them():
    assert check_defs.ros_field_text("double[9]") == ("float64", 9)
    assert check_defs.ros_field_text("sequence<vision_msgs/Detection2D>") == ("vision_msgs/Detection2D", -1)
    assert check_defs.ros_field_text("sequence<double>") == ("float64", -1) and check_defs.ros_field_text("boolean") == ("bool", None)
    assert check_defs.ros_field_text("string") == ("string", None) and check_defs.ros_field_text("std_msgs/Header") == ("std_msgs/Header", None)
    assert check_defs.ros_field_text("uint16") == ("uint16", None) and check_defs.ros_field_text("octet") == ("byte", None)


def test_the_filled_message_has_no_two_fields_alike_and_round_trips():
    layouts = msgdefs.Layouts()
    for type_name in sorted(msgdefs.DEFINITIONS):
        value = check_defs.fill(type_name, layouts, [0])
        assert cdr.decode(type_name, cdr.encode(type_name, value, layouts), layouts) == value, type_name
    pose = check_defs.fill("geometry_msgs/PoseStamped", layouts, [0])
    numbers = [pose.header.stamp.sec, pose.header.stamp.nanosec, pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w]
    assert len(set(numbers)) == len(numbers)
    assert len(check_defs.fill("sensor_msgs/CameraInfo", layouts, [0]).d) == 2 and "check_defs.py" in (SIM / "README.md").read_text()
