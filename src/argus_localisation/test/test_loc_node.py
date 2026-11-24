"""The localisation node through its real class (LOC-5), with its flags off and on.

rclpy and the message packages are the stand-ins of drone_target_cv's
test_contract_node.py, so this runs without ROS. The pipeline, the oracle
detector, the camera's calibration and the mount are the real ones.
"""

import json
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "argus_localisation"), str(SRC / "drone_mission_core"), str(SRC / "argus_camera"), str(SRC / "drone_target_cv"), str(SRC / "drone_target_cv" / "test"), str(SRC / "drone_mission_core" / "test")]

import test_contract_node as stand_ins  # noqa: E402
from drone_mission_core import detection_contract as contract  # noqa: E402
from test_contract_node import ros, with_vision_msgs  # noqa: E402, F401  (ros is a fixture)

S = 1_000_000_000
T0 = 700 * S
IMAGE_TOPIC, POSE_TOPIC, TRUTH_TOPIC = "camera/image", "/mavros/local_position/pose", "/sim/truth/poses"
SIM = {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "oracle", "sim_only": True, "use_sim_time": True}
FLAGS = {"features.loc_node": True, "features.sim_oracle_detector": True}
EXACT = {"oracle_pixel_sigma_px": 0.0, "oracle_miss_rate": 0.0, "oracle_false_alarms_per_frame": 0.0}


class RecordingNode(stand_ins.FakeNode):
    """FakeNode that also keeps each subscription's QoS, the service, the timers, and a clock the test sets."""

    now_ns = T0

    def __init__(self, name):
        super().__init__(name)
        self.qos, self.services, self.timers = {}, {}, []

    def has_parameter(self, name):
        return name in self.overrides or name in self.declared

    def get_parameter(self, name):
        if name == "use_sim_time":
            return types.SimpleNamespace(value=bool(self.overrides.get(name, False)))
        return super().get_parameter(name)

    def create_subscription(self, message_type, topic, callback, qos):
        self.qos[topic] = qos
        return super().create_subscription(message_type, topic, callback, qos)

    def create_service(self, service_type, name, callback):
        self.services[name] = callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def destroy_node(self):
        self.destroyed = True

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=type(self).now_ns))


class QoSProfile:
    def __init__(self, **settings):
        self.settings = settings


class String:
    def __init__(self):
        self.data = ""


def _header():
    return types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")


class MarkerTrack:
    TENTATIVE, CONFIRMED, COASTING, LOST = 0, 1, 2, 3

    def __init__(self):
        self.header, self.position_enu = _header(), types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.track_id, self.state, self.covariance_en, self.digit_log_prob, self.digit, self.sightings, self.outside = 0, 0, [], [], -1, 0, False


class MarkerTrackArray:
    def __init__(self):
        self.header, self.tracks = _header(), []


@pytest.fixture
def objects_file(tmp_path):
    path = tmp_path / "oracle_objects.json"
    path.write_text(json.dumps({"vehicle": "iris_survey", "objects": [{"name": "marker_3", "class_id": "marker", "size_m": 0.6, "top_m": 0.02}]}))
    return str(path)


def node_with(monkeypatch, **parameters):
    monkeypatch.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=RecordingNode))
    qos = types.SimpleNamespace(QoSProfile=QoSProfile, HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"), ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT="best_effort"), qos_profile_sensor_data="sensor_data")
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", types.SimpleNamespace(PoseStamped=object, TwistStamped=object))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", types.SimpleNamespace(Image=object, Range=object, NavSatFix=object))
    monkeypatch.setitem(sys.modules, "std_msgs", types.SimpleNamespace(msg=types.SimpleNamespace(String=String)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", types.SimpleNamespace(String=String))
    monkeypatch.setitem(sys.modules, "std_srvs.srv", types.SimpleNamespace(SetBool=object))
    monkeypatch.setitem(sys.modules, "tf2_msgs", types.SimpleNamespace(msg=types.SimpleNamespace(TFMessage=object)))
    monkeypatch.setitem(sys.modules, "tf2_msgs.msg", types.SimpleNamespace(TFMessage=object))
    interfaces = types.SimpleNamespace(MarkerTrack=MarkerTrack, MarkerTrackArray=MarkerTrackArray)
    monkeypatch.setitem(sys.modules, "argus_interfaces", types.SimpleNamespace(msg=interfaces))
    monkeypatch.setitem(sys.modules, "argus_interfaces.msg", interfaces)
    monkeypatch.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    monkeypatch.setattr(RecordingNode, "overrides", dict(parameters))
    monkeypatch.setattr(RecordingNode, "now_ns", T0)
    import importlib

    return importlib.import_module("argus_localisation.loc_node").LocNode()  # imported afresh, on this test's node class


def stamp(ns):
    return types.SimpleNamespace(sec=ns // S, nanosec=ns % S)


def pose(ns, east=0.0, north=0.0, up=10.0):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=stamp(ns)),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=east, y=north, z=up), orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


def truth(ns, vehicle=(0.0, 0.0, 10.0), marker=(2.0, 1.0, 0.0)):
    def transform(name, at):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp(ns)),
            child_frame_id=name,
            transform=types.SimpleNamespace(translation=types.SimpleNamespace(x=at[0], y=at[1], z=at[2]), rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
        )

    return types.SimpleNamespace(transforms=[transform("iris_survey", vehicle), transform("marker_3", marker), transform("parcel", (0.0, 0.0, 0.0))])


def image(ns, width=1280, height=720):
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns), frame_id="down_camera_optical"), width=width, height=height, encoding="bgr8", data=np.zeros((height, width, 3), dtype=np.uint8).tobytes())


def flying(node, ns=T0):
    for index in range(-30, 10):
        node.subscriptions[POSE_TOPIC](pose(ns + index * 33_000_000))
        node.subscriptions[TRUTH_TOPIC](truth(ns + index * 33_000_000))


def set_enabled(node, enabled):
    response = types.SimpleNamespace(success=False, message="")
    return node.services["/loc/set_enabled"](types.SimpleNamespace(data=enabled), response)


def tick(node):
    for period, callback in node.timers:
        if period < 0.5:
            callback()


def test_with_the_flag_off_the_node_makes_nothing(ros):
    node = node_with(ros, **SIM)
    assert node.subscriptions == {} and node.publishers == {} and node.services == {} and node.timers == [] and node.runtime is None
    assert "off" in node.logger.info.call_args[0][0]


@pytest.mark.parametrize(
    "change, said",
    [
        ({"detector": "engine"}, "needs engine_path"),
        ({"detector": "engine", "engine_path": "/nowhere/yolo.engine"}, "is not a file"),
        ({"features.loc_tiling": True}, "tiling has nothing to give the oracle"),
        ({"features.loc_median_position": True}, "features.loc_median_position needs features.marker_tracker"),
        ({"features.survey_frame_log": True}, "frame_log_dir must be set"),
        ({"detector": "yolo"}, "detector must be one of"),
        ({"use_sim_time": False}, "runs only in the sim"),
        ({"sim_only": False}, "runs only in the sim"),
        ({"features.sim_oracle_detector": False}, "features.sim_oracle_detector, which is off"),
        ({"camera_mode": ""}, "camera_mode and airframe must both be set"),
        ({"camera_mode": "imx519_4k15"}, ""),  # a real camera on the sim's airframe: projection v2's own refusal
        ({"oracle_objects_file": "/nowhere/objects.json"}, "oracle_objects_file"),
    ],
)
def test_a_pipeline_that_cannot_start_says_why_and_publishes_nothing(ros, objects_file, change, said):
    with_vision_msgs(ros)
    node = node_with(ros, **{**SIM, **FLAGS, "oracle_objects_file": objects_file, **change})
    assert node.runtime is None and node.publishers == {} and node.services == {} and node.timers == []
    error = node.logger.error.call_args[0][0]
    assert "could not start" in error and said in error


def test_in_the_sim_the_oracles_boxes_go_out_on_the_contract_once_the_pipeline_is_started(ros, objects_file):
    with_vision_msgs(ros)
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, "oracle_objects_file": objects_file})
    assert node.logger.error.call_args is None
    assert node.qos[IMAGE_TOPIC].settings == {"depth": 1, "history": "keep_last", "reliability": "best_effort"}  # the newest image only
    assert set(node.publishers) == {"/loc/detections", "/loc/status"} and set(node.services) == {"/loc/set_enabled"}
    flying(node)
    # Off until started: an image is not even looked at.
    node.subscriptions[IMAGE_TOPIC](image(T0))
    tick(node)
    assert node.publishers["/loc/detections"].sent == [] and node.runtime.status()["frames"] == 0
    response = set_enabled(node, True)
    assert response.success is True and response.message == "localisation pipeline started"
    node.subscriptions[IMAGE_TOPIC](image(T0))
    tick(node)
    sent = node.publishers["/loc/detections"].sent
    assert len(sent) == 1
    found = contract.decode(sent[0])
    assert found.stamp_ns == T0 and found.frame_id == "down_camera_optical" and [d.class_id for d in found.detections] == ["marker"]
    # The sim's survey camera: 778.33 px focal length at 1280 x 720. The marker is 2 m ahead (east) and 1 m left, 9.98 m below.
    marker = found.detections[0]
    assert marker.cx == pytest.approx(640.0 - 778.33 * 1.0 / 9.98, abs=1.0) and marker.cy == pytest.approx(360.0 - 778.33 * 2.0 / 9.98, abs=1.0)
    # The status, as JSON on a String.
    for period, callback in node.timers:
        if period >= 0.5:
            callback()
    status = json.loads(node.publishers["/loc/status"].sent[-1].data)
    assert status["enabled"] is True and status["frames"] == 1 and status["detector"] == "oracle" and status["camera_mode"] == "sim_survey"
    assert status["detections"] == 1 and status["placed"] == 1 and status["dropped"] == 0
    # Three images before the node's next turn: the newest is processed, the two that waited are dropped.
    for index in (1, 2, 3):
        node.subscriptions[IMAGE_TOPIC](image(T0 + index * 33_000_000))
    tick(node)
    tick(node)
    assert len(sent) == 2 and contract.decode(sent[1]).stamp_ns == T0 + 3 * 33_000_000 and node.runtime.status()["dropped"] == 2
    # Stopped again: nothing more goes out.
    assert set_enabled(node, False).message == "localisation pipeline stopped after 2 frames"
    node.subscriptions[IMAGE_TOPIC](image(T0 + 5 * 33_000_000))
    tick(node)
    assert len(sent) == 2


def test_a_frame_that_is_not_the_calibrated_modes_size_is_a_fault_not_a_detection(ros, objects_file):
    with_vision_msgs(ros)
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, "oracle_objects_file": objects_file, "start_enabled": True})
    flying(node)
    node.subscriptions[IMAGE_TOPIC](image(T0, width=640, height=360))
    tick(node)
    status = node.runtime.status()
    assert node.publishers["/loc/detections"].sent == [] and status["faults"] == 1 and "not the calibrated mode's 1280 x 720" in status["last_fault"]


class Model:
    """An engine that finds one marker at the centre of whatever it is shown."""

    def __init__(self):
        self.shapes = []

    def __call__(self, image, conf, verbose):
        self.shapes.append(image.shape)
        height, width = image.shape[:2]
        box = types.SimpleNamespace(xyxy=[[width / 2 - 10.0, height / 2 - 10.0, width / 2 + 10.0, height / 2 + 10.0]], conf=[0.9], cls=[0])
        return [types.SimpleNamespace(boxes=[box])]


def engine_node(monkeypatch, tmp_path, **parameters):
    file = tmp_path / "yolo26n.engine"
    file.write_bytes(b"stand-in")
    model = Model()
    monkeypatch.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: model))
    settings = {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "features.loc_node": True, "start_enabled": True}
    return node_with(monkeypatch, **{**settings, **parameters}), model


def test_the_engine_detector_runs_as_in_2026_with_no_sim_about_it(ros, tmp_path):
    """`detector: engine` needs neither sim time nor the oracle's flag: it is the aircraft's detector."""
    with_vision_msgs(ros)
    node, model = engine_node(ros, tmp_path)
    assert node.logger.error.call_args is None and node.runtime.status()["detector"] == "engine"
    for index in range(-30, 10):
        node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
    node.subscriptions[IMAGE_TOPIC](image(T0))
    tick(node)
    found = contract.decode(node.publishers["/loc/detections"].sent[0])
    assert model.shapes == [(720, 1280, 3)] * 2  # the blank frame at the load, then the camera's
    assert [(d.class_id, d.cx, d.cy) for d in found.detections] == [("marker", 640.0, 360.0)]


def test_tiling_is_behind_its_own_flag(ros, tmp_path):
    with_vision_msgs(ros)
    plain, _model = engine_node(ros, tmp_path)
    assert plain.runtime.pipeline.tiling is False
    tiled, model = engine_node(ros, tmp_path, **{"features.loc_tiling": True})
    assert tiled.runtime.pipeline.tiling is True
    for index in range(-30, 10):
        tiled.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
    tiled.subscriptions[IMAGE_TOPIC](image(T0))
    tick(tiled)
    assert model.shapes == [(720, 1280, 3)] * 2  # the load's blank frame, then one window: the sim's frame is one; a 4K frame is sixteen (test_loc_pipeline.py)


class Source:
    """A camera's frame source: a wrong frame first, then frames of the mode's size."""

    def __init__(self, clock):
        self.clock, self.reads, self.opened, self.closed = clock, 0, False, False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def read(self):
        import time

        from argus_camera.frame_source import Frame, FrameSourceError

        self.reads += 1
        time.sleep(0.005)
        if self.reads == 1:
            raise FrameSourceError("imx519_720p60: the camera gave a 640 x 480 frame")
        return Frame(image=np.zeros((720, 1280, 3), dtype=np.uint8), seq=self.reads, capture_ns=T0, mode="imx519_720p60", stamp_source="pts")


def test_a_real_cameras_frames_are_read_inside_the_node_and_never_come_over_a_topic(ros, tmp_path):
    """The aircraft's path: the camera's own FrameSource on a thread of the node. The calibration and the mount are
    the sim's here (the aircraft's modes are not calibrated yet); the mode's driver is a real camera's."""
    import time

    with_vision_msgs(ros)
    from argus_camera import frame_source
    from drone_mission_core import frame_projection

    sim_camera = frame_projection.load_camera("sim_survey", "sim_iris_downcam")
    ros.setattr(frame_projection, "load_camera", lambda mode, airframe: sim_camera)
    made = []
    ros.setattr(frame_source, "open_source", lambda mode, clock: made.append(Source(clock)) or made[-1])
    node, model = engine_node(ros, tmp_path, camera_mode="imx519_720p60", airframe="argus_2027")
    try:
        assert node.logger.error.call_args is None, node.logger.error.call_args
        assert IMAGE_TOPIC not in node.subscriptions and made and made[0].opened  # no image topic: the source is the way in
        for index in range(-30, 10):
            node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not node.publishers["/loc/detections"].sent:
            tick(node)
            time.sleep(0.01)
        assert node.publishers["/loc/detections"].sent, "no frame came through the reader thread"
        status = node.runtime.status()
        assert status["faults"] == 1 and "640 x 480" in status["last_fault"]  # the wrong frame was said, and reading went on
    finally:
        node.destroy_node()
    assert made[0].closed


def test_the_shipped_params_declare_every_flag_off_under_the_node():
    import yaml

    params = yaml.safe_load((SRC / "argus_localisation" / "config" / "loc_params.yaml").read_text())
    assert set(params) == {"loc_node"}  # no top-level `features:`, which would stop every node of the launch
    settings = params["loc_node"]["ros__parameters"]
    assert settings["features"] == {"loc_node": False, "sim_oracle_detector": False, "loc_tiling": False, "marker_tracker": False, "loc_median_position": False, "survey_frame_log": False, "digit_reader": False, "template_digit_classifier": False, "digit_vote": False, "judges_list": False, "post_landing_pass": False}
    assert settings["sim_only"] is False and settings["start_enabled"] is False and settings["detector"] == "engine"


def test_a_start_that_fails_late_leaves_no_service_behind_and_a_refused_box_does_not_end_the_node(ros, tmp_path):
    """The flight-safety review: the service used to be offered before the camera was opened, with nothing behind it."""
    with_vision_msgs(ros)
    from argus_camera import frame_source
    from drone_mission_core import frame_projection

    sim_camera = frame_projection.load_camera("sim_survey", "sim_iris_downcam")
    ros.setattr(frame_projection, "load_camera", lambda mode, airframe: sim_camera)

    def no_camera(mode, clock):
        raise frame_source.FrameSourceError("imx519_720p60: the camera did not open")

    ros.setattr(frame_source, "open_source", no_camera)
    node, _model = engine_node(ros, tmp_path, camera_mode="imx519_720p60", airframe="argus_2027")
    assert node.runtime is None and node.services == {} and node.publishers == {} and node.subscriptions == {} and node.timers == []
    assert "the camera did not open" in node.logger.error.call_args[0][0]

    # A box the contract refuses (a score over 1) is a fault in that frame, and the next frame still goes out.
    class Wild(Model):
        def __call__(self, image, conf, verbose):
            found = super().__call__(image, conf, verbose)
            found[0].boxes[0].conf = [1.7 if len(self.shapes) == 2 else 0.9]  # the first frame after the load's blank one
            return found

    file = tmp_path / "wild.engine"
    file.write_bytes(b"stand-in")
    model = Wild()
    ros.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: model))
    node = node_with(ros, **{"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "features.loc_node": True, "start_enabled": True})
    ros.setattr(frame_projection, "load_camera", lambda mode, airframe: sim_camera)
    for index in range(-30, 10):
        node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
    node.subscriptions[IMAGE_TOPIC](image(T0))
    tick(node)
    assert node.publishers["/loc/detections"].sent == [] and node.runtime.status()["faults"] == 1
    node.subscriptions[IMAGE_TOPIC](image(T0 + 33_000_000))
    tick(node)
    assert len(node.publishers["/loc/detections"].sent) == 1


def test_the_node_takes_the_sites_ground_and_the_vehicles_velocity(ros, tmp_path):
    with_vision_msgs(ros)
    node, _model = engine_node(ros, tmp_path, ground_below_home_m=-2.0)
    assert node.logger.error.call_args is None
    assert "/mavros/local_position/velocity_local" in node.subscriptions and "/mavros/rangefinder/rangefinder" in node.subscriptions
    for index in range(-30, 1):
        node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
        node.subscriptions["/mavros/local_position/velocity_local"](types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(T0 + index * 33_000_000)), twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=4.0, y=0.0, z=0.0))))
    # A frame 20 ms newer than the newest pose is placed at once: with the velocity the pose history carries on that far.
    RecordingNode.now_ns = T0 + 25_000_000
    node.subscriptions[IMAGE_TOPIC](image(T0 + 20_000_000))
    tick(node)
    status = node.runtime.status()
    assert status["frames"] == 1 and status["placed"] == 1 and status["waiting_for_pose"] is False
    refused, _ = engine_node(ros, tmp_path, ground_below_home_m=40.0)
    assert refused.runtime is None and "ground_below_home_m" in refused.logger.error.call_args[0][0]


def test_a_setting_of_the_wrong_kind_does_not_take_the_node_down(ros, tmp_path):
    """rclpy refuses an integer where a parameter was declared as a number with a point. The node must log it and
    stay out of the way, not exit: the survey would wait on a service that is not there."""
    with_vision_msgs(ros)

    class Strict(RecordingNode):
        def declare_parameter(self, name, default=None):
            if name == "ground_below_home_m" and isinstance(self.overrides.get(name), int):
                raise TypeError("ground_below_home_m: expected a double, got an integer")
            return super().declare_parameter(name, default)

    ros.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=Strict))
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    ros.setattr(Strict, "overrides", {"features.loc_node": True, "camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "ground_below_home_m": 3})
    import importlib

    loc_node = importlib.import_module("argus_localisation.loc_node")  # afresh, on the strict node class
    node = loc_node.LocNode()
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)  # the next test imports it on its own node class
    assert node.runtime is None and node.services == {} and node.publishers == {}
    assert "ground_below_home_m" in node.logger.error.call_args[0][0]


def test_the_tracker_is_behind_its_flag_and_its_tracks_go_out_as_marker_tracks(ros, objects_file):
    with_vision_msgs(ros)
    plain = node_with(ros, **{**SIM, **FLAGS, **EXACT, "oracle_objects_file": objects_file})
    assert plain.runtime.pipeline.tracker is None and "/loc/tracks" not in plain.publishers
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, "features.marker_tracker": True, "oracle_objects_file": objects_file, "start_enabled": True})
    assert node.logger.error.call_args is None and "/loc/tracks" in node.publishers
    for frame in range(4):  # the vehicle holds over the origin; the marker is 2 m ahead and 1 m to the left
        ns = T0 + frame * 100_000_000
        for index in range(-3, 1):
            node.subscriptions[POSE_TOPIC](pose(ns + index * 33_000_000))
            node.subscriptions[TRUTH_TOPIC](truth(ns + index * 33_000_000))
        RecordingNode.now_ns = ns + 10_000_000
        node.subscriptions[IMAGE_TOPIC](image(ns))
        tick(node)
    sent = node.publishers["/loc/tracks"].sent
    assert len(sent) == 4 and [len(message.tracks) for message in sent] == [1, 1, 1, 1]
    assert [message.tracks[0].state for message in sent] == [MarkerTrack.TENTATIVE, MarkerTrack.TENTATIVE, MarkerTrack.CONFIRMED, MarkerTrack.CONFIRMED]
    last = sent[-1].tracks[0]
    assert (last.position_enu.x, last.position_enu.y) == pytest.approx((2.0, 1.0), abs=0.05) and last.sightings == 4 and last.outside is False
    assert last.digit == -1 and len(last.digit_log_prob) == 10 and len(last.covariance_en) == 4 and last.header.frame_id == "map"
    assert sent[-1].header.stamp.sec == (T0 + 300_000_000) // S and last.track_id == sent[0].tracks[0].track_id
    status = node.runtime.status()
    assert status["tracks"] == 1 and status["confirmed_ids"] == [last.track_id]


def test_a_step_in_the_flight_controllers_position_steps_the_tracks_and_a_failed_publish_is_a_fault(ros, objects_file):
    with_vision_msgs(ros)
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, "features.marker_tracker": True, "oracle_objects_file": objects_file, "start_enabled": True})
    velocity_topic = "/mavros/local_position/velocity_local"

    def fly(ns, east=0.0):
        for index in range(-3, 1):
            at = ns + index * 33_000_000
            node.subscriptions[velocity_topic](types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(at)), twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))))
            node.subscriptions[POSE_TOPIC](pose(at, east=east))
            node.subscriptions[TRUTH_TOPIC](truth(at))
        RecordingNode.now_ns = ns + 10_000_000
        node.subscriptions[IMAGE_TOPIC](image(ns))
        tick(node)

    for frame in range(4):
        fly(T0 + frame * 100_000_000)
    before = node.publishers["/loc/tracks"].sent[-1].tracks[0]
    assert before.position_enu.x == pytest.approx(2.0, abs=0.05)
    fly(T0 + 400_000_000, east=1.5)  # standing still, the reported position jumps 1.5 m east: an EKF reset
    after = node.publishers["/loc/tracks"].sent[-1].tracks
    assert len(after) == 1 and after[0].track_id == before.track_id and after[0].position_enu.x == pytest.approx(3.5, abs=0.1)
    assert any("stepped" in call[0][0] for call in node.logger.warn.call_args_list)
    # A track message that cannot be made is that frame's fault; the node goes on and the next frame's goes out.
    sent = len(node.publishers["/loc/tracks"].sent)
    original = node._tracks_out.encode_tracks
    node._tracks_out.encode_tracks = lambda *args: (_ for _ in ()).throw(AssertionError("uint32 out of range"))
    fly(T0 + 500_000_000, east=1.5)
    assert node.runtime.status()["faults"] == 1 and len(node.publishers["/loc/tracks"].sent) == sent
    node._tracks_out.encode_tracks = original
    fly(T0 + 600_000_000, east=1.5)
    assert len(node.publishers["/loc/tracks"].sent) == sent + 1


def test_the_median_position_is_behind_its_flag_and_is_what_goes_out_with_it_on(ros, objects_file):
    with_vision_msgs(ros)
    from argus_localisation import final_position

    def survey(**flags):
        node = node_with(ros, **{**SIM, **FLAGS, "features.marker_tracker": True, "oracle_pixel_sigma_px": 6.0, "oracle_miss_rate": 0.0, "oracle_false_alarms_per_frame": 0.0, "oracle_objects_file": objects_file, "start_enabled": True, **flags})
        for frame in range(12):
            ns = T0 + frame * 100_000_000
            for index in range(-3, 1):
                node.subscriptions[POSE_TOPIC](pose(ns + index * 33_000_000))
                node.subscriptions[TRUTH_TOPIC](truth(ns + index * 33_000_000))
            RecordingNode.now_ns = ns + 10_000_000
            node.subscriptions[IMAGE_TOPIC](image(ns))
            tick(node)
        (state,) = node._tracker.tracks()
        return node, state, node.publishers["/loc/tracks"].sent[-1].tracks[0]

    node, state, sent = survey()
    assert node._tracker.median_positions is False and (sent.position_enu.x, sent.position_enu.y) == (state.east_m, state.north_m)
    node, state, sent = survey(**{"features.loc_median_position": True})
    median = final_position.final_position(node._tracker.sightings_of(state.track_id))
    assert node._tracker.median_positions is True and (sent.position_enu.x, sent.position_enu.y) == pytest.approx(median)
    assert (sent.position_enu.x, sent.position_enu.y) != (state.east_m, state.north_m)  # the median of noisy sightings is not their weighted mean
    assert (sent.position_enu.x, sent.position_enu.y) == pytest.approx((2.0, 1.0), abs=0.15)


LOGGING = {"features.survey_frame_log": True, "frame_log_workers": 1, "frame_log_min_free_gb": 0.0}


def status_of(node):
    for period, callback in node.timers:
        if period >= 0.5:
            callback()
    return json.loads(node.publishers["/loc/status"].sent[-1].data)


def test_the_surveys_frames_are_logged_between_the_pipelines_start_and_its_stop_with_the_flag_on(ros, objects_file, tmp_path):
    with_vision_msgs(ros)
    plain = node_with(ros, **{**SIM, **FLAGS, **EXACT, "oracle_objects_file": objects_file})
    assert plain._frame_log is None and "frame_log" not in plain.runtime.status()
    frames = tmp_path / "frames"
    frames.mkdir()
    # (Two workers: a queue of four, so that the three frames offered at once below fit whatever the writer is doing.)
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, **LOGGING, "frame_log_workers": 2, "frame_log_dir": str(frames), "oracle_objects_file": objects_file})
    assert node.logger.error.call_args is None and list(frames.iterdir()) == []  # nothing is written before the survey
    flying(node)
    node.subscriptions[IMAGE_TOPIC](image(T0))
    assert set_enabled(node, True).message == "localisation pipeline started"
    (folder,) = list(frames.iterdir())
    for index in range(3):
        node.subscriptions[IMAGE_TOPIC](image(T0 + index * 33_000_000))  # three frames before the node's turn: the detector takes the newest
    tick(node)
    status = status_of(node)
    assert status["frames"] == 1 and status["dropped"] == 2  # the detector's count
    assert status["frame_log"]["logging"] is True and status["frame_log"]["folder"] == str(folder) and status["frame_log"]["keep_every"] == 1
    set_enabled(node, False)
    node.subscriptions[IMAGE_TOPIC](image(T0 + 10 * 33_000_000))
    node._frame_log.stop(wait_s=5.0)  # (the stop does not wait for the disk; the test does)
    assert len(list(folder.glob("frame_*.jpg"))) == 3 and len((folder / "index.jsonl").read_text().splitlines()) == 3  # the log has all three
    assert node._frame_log.status()["written"] == 3 and node._frame_log.status()["logging"] is False
    # A detector that falls under nine tenths of its rate (the sim's camera gives 10 a second): the logger gives way.
    set_enabled(node, True)
    for second in range(1, 7):  # one frame a second for six seconds
        RecordingNode.now_ns = T0 + second * S
        flying(node, RecordingNode.now_ns)
        node.subscriptions[IMAGE_TOPIC](image(RecordingNode.now_ns - 10_000_000))
        tick(node)
        status = status_of(node)
    assert status["rate_hz"] < 2.0 and status["frame_log"]["keep_every"] > 1 and "the detector is" in status["frame_log"]["reason"]
    node.destroy_node()
    assert node._frame_log.status()["logging"] is False


def test_with_the_post_landing_flag_on_the_surveys_poses_ranges_and_final_tracks_are_kept_beside_its_frames(ros, objects_file, tmp_path):
    """LOC-12: what tools/second_pass.py needs after landing. With the flag off nothing of it is written or loaded."""
    with_vision_msgs(ros)
    frames = tmp_path / "frames"
    frames.mkdir()
    base = {**SIM, **FLAGS, **EXACT, **LOGGING, "features.marker_tracker": True, "frame_log_dir": str(frames), "oracle_objects_file": objects_file}
    ros.delitem(sys.modules, "argus_localisation.second_pass", raising=False)
    off = node_with(ros, **base)
    assert off.declared["features.post_landing_pass"] is False and off._second_pass is None and "argus_localisation.second_pass" not in sys.modules
    flying(off)
    set_enabled(off, True)
    set_enabled(off, False)
    off._frame_log.stop(wait_s=5.0)
    assert all(not (folder / "flight_record.jsonl").exists() and not (folder / "live_tracks.json").exists() for folder in frames.iterdir())
    off.destroy_node()

    kept = tmp_path / "kept"
    kept.mkdir()
    node = node_with(ros, **{**base, "frame_log_dir": str(kept), "features.post_landing_pass": True})
    assert node.logger.error.call_args is None
    flying(node)  # before the survey: nothing is recorded
    assert set_enabled(node, True).message == "localisation pipeline started"
    (folder,) = list(kept.iterdir())
    flying(node, T0 + S)
    node.subscriptions[IMAGE_TOPIC](image(T0 + S))
    tick(node)
    set_enabled(node, False)
    node._frame_log.stop(wait_s=5.0)
    assert node._last_record.wait(5.0) and node._last_record.failed == ""  # (the record's own thread writes it: the stop did not wait on the disk; the test does)
    from argus_localisation import second_pass

    events = second_pass.read_record(folder)
    assert len([event for event in events if event["k"] == "pose"]) == 40 and all(event["t"] >= T0 + S - 30 * 33_000_000 for event in events)  # the survey's own, not what came before it
    tracks, _origin, settings = second_pass.load_live_tracks(folder / "live_tracks.json")
    assert [track.source for track in tracks] == ["live"] * len(tracks) and len(tracks) == len(node._tracker.tracks())
    assert settings == {"ground_below_home_m": 0.0, "digit_min_confidence": 0.9, "median_positions": False, "digit_ids_unique": True}  # the flight's own, for the tool
    assert (folder / "index.jsonl").is_file()  # beside the frames' own index
    assert any("for the pass after landing" in call.args[0] for call in node.logger.info.call_args_list)
    node.destroy_node()
    # It needs the frame log and the tracker: without either the node says so and flies on without it (it is extra to the flight's own detections).
    for missing in ("features.survey_frame_log", "features.marker_tracker"):
        without = node_with(ros, **{**base, "features.post_landing_pass": True, missing: False})
        assert without.runtime is not None and without._second_pass is None
        assert "features.post_landing_pass needs features.survey_frame_log and features.marker_tracker" in without.logger.error.call_args[0][0]
        without.destroy_node()


def test_the_shipped_settings_of_the_frame_log_are_the_specs_and_a_wrong_one_is_refused(ros, objects_file, tmp_path):
    import yaml

    for file in (SRC / "argus_localisation" / "config" / "loc_params.yaml", SRC / "drone_mission_demo" / "config" / "params" / "sim_survey_mission.yaml"):
        settings = yaml.safe_load(file.read_text())["loc_node"]["ros__parameters"]
        assert (settings["frame_log_jpeg_quality"], settings["frame_log_workers"], settings["frame_log_dir"]) == (90, 3, "")
        assert settings["frame_log_max_gb"] == 400.0 and settings["frame_log_min_free_gb"] == 20.0 and settings["features"]["survey_frame_log"] is False
    with_vision_msgs(ros)
    frames = tmp_path / "frames"
    frames.mkdir()
    node = node_with(ros, **{**SIM, **FLAGS, **EXACT, "features.survey_frame_log": True, "frame_log_min_free_gb": 0.0, "frame_log_dir": str(frames), "oracle_objects_file": objects_file})
    assert node.logger.error.call_args is None and node._frame_log._settings["jpeg_quality"] == 90 and node._frame_log._settings["workers"] == 3  # the node's own defaults
    assert node._frame_log._expected_rate_hz == 10.0  # the sim's camera gives 10 a second, under the detector's target of 15
    for change, said in (({"frame_log_workers": 0}, "frame_log_workers must be 1 or more"), ({"frame_log_dir": ""}, "frame_log_dir must be set"), ({"frame_log_jpeg_quality": 0}, "JPEG quality")):
        refused = node_with(ros, **{**SIM, **FLAGS, **EXACT, **LOGGING, "frame_log_dir": str(frames), "oracle_objects_file": objects_file, **change})
        assert refused.runtime is None and said in refused.logger.error.call_args[0][0]


def test_a_frame_log_setting_of_the_wrong_kind_does_not_stop_the_node_while_the_logs_flag_is_off(ros, objects_file):
    with_vision_msgs(ros)

    class Strict(RecordingNode):
        def declare_parameter(self, name, default=None):
            if name == "frame_log_max_gb" and isinstance(self.overrides.get(name), int):
                raise TypeError("frame_log_max_gb: expected a double, got an integer")
            return super().declare_parameter(name, default)

    import importlib

    def start(**flags):
        ros.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=Strict))
        ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
        ros.setattr(Strict, "overrides", {**SIM, **FLAGS, **EXACT, "oracle_objects_file": objects_file, "frame_log_max_gb": 400, **flags})
        node = importlib.import_module("argus_localisation.loc_node").LocNode()
        ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
        return node

    node_with(ros, **SIM)  # (puts the message stand-ins in place; this node, with its flag off, is not the one under test)
    off = start()
    assert off.runtime is not None, [call[0][0] for call in off.logger.error.call_args_list]
    assert set(off.services) == {"/loc/set_enabled"}  # the pipeline runs; only the log could not have
    assert "frame log's parameters could not be declared" in off.logger.error.call_args[0][0]
    on = start(**{"features.survey_frame_log": True, "frame_log_dir": "/somewhere"})
    assert on.runtime is None and "features.survey_frame_log is on and the frame log's parameters could not be declared" in on.logger.error.call_args[0][0]


def test_a_real_cameras_frames_go_to_the_log_from_the_cameras_own_thread(ros, tmp_path):
    """The path the 4K frames take: read inside the node, offered to the detector and to the log."""
    import time

    with_vision_msgs(ros)
    from argus_camera import frame_source
    from drone_mission_core import frame_projection

    sim_camera = frame_projection.load_camera("sim_survey", "sim_iris_downcam")
    ros.setattr(frame_projection, "load_camera", lambda mode, airframe: sim_camera)
    ros.setattr(frame_source, "open_source", lambda mode, clock: Source(clock))
    frames = tmp_path / "frames"
    frames.mkdir()
    node, _model = engine_node(ros, tmp_path, camera_mode="imx519_720p60", airframe="argus_2027", **{**LOGGING, "frame_log_dir": str(frames)})
    try:
        assert node.logger.error.call_args is None, node.logger.error.call_args
        assert node._frame_log._expected_rate_hz == 15.0  # a 60 Hz camera: the detector's own target is the lower
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and node._frame_log.status()["written"] < 5:
            time.sleep(0.01)
        assert node._frame_log.status()["written"] >= 5, node._frame_log.status()
        set_enabled(node, False)
        node._frame_log.stop(wait_s=5.0)
        written = node._frame_log.status()["written"]
        time.sleep(0.1)  # the camera goes on giving frames; the survey is over and none is logged
        assert node._frame_log.status()["written"] == written and len(list(next(frames.iterdir()).glob("frame_*.jpg"))) == written
    finally:
        node.destroy_node()


def test_the_digit_reader_is_behind_its_flag_and_reads_the_markers_the_detector_boxes(ros, tmp_path):
    """The engine detector boxes a marker in the frame; the reader, with the stand-in classifier, reads its digit."""
    sys.path.insert(0, str(SRC.parent / "tools"))
    import synth_targets

    with_vision_msgs(ros)
    options = synth_targets.RenderOptions(width=1280, height=720, mpp=0.0045, rotation_deg=25.0, background="noise", supersample=2)
    picture, drawn = synth_targets.render(synth_targets.SurveyMarker(digit=7), options, seed=3)  # a 133 px marker in the middle of the frame

    class Boxer(Model):
        def __call__(self, image, conf, verbose):
            self.shapes.append(image.shape)
            box = types.SimpleNamespace(xyxy=[[640.0 - 95.0, 360.0 - 95.0, 640.0 + 95.0, 360.0 + 95.0]], conf=[0.9], cls=[0])
            return [types.SimpleNamespace(boxes=[box])]

    def start(**flags):
        file = tmp_path / "yolo.engine"
        file.write_bytes(b"stand-in")
        ros.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: Boxer()))
        settings = {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "sim_only": True, "use_sim_time": True}
        node = node_with(ros, **{**settings, "features.loc_node": True, "start_enabled": True, **flags})
        for index in range(-30, 10):
            node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
        message = image(T0)
        message.data = picture.tobytes()
        node.subscriptions[IMAGE_TOPIC](message)
        return node

    plain = start()
    assert plain.runtime.pipeline.digit_reader is None and plain.runtime.status()["digits"] is None
    node = start(**{"features.digit_reader": True, "features.template_digit_classifier": True})
    assert node.logger.error.call_args is None and "stand-in" in node.logger.warn.call_args_list[-1][0][0]
    result = node.runtime.step()
    assert len(result.crops) == 1 and [reading.digit for reading in result.readings] == [7] and result.timings_ms["digits"] > 0.0
    status = node.runtime.status()
    assert status["digits"]["last_ms"] > 0.0
    assert {key: value for key, value in status["digits"].items() if key != "last_ms"} == {"read": 1, "abstained": 0, "skipped": 0, "faults": 0, "classifier": "template (stand-in)"}
    json.dumps(status)
    # The stand-in knows the sim's marker model and nothing else: off the sim it does not start, and the boxes go on.
    for outside in ({"use_sim_time": False}, {"sim_only": False}):
        aircraft = start(**{"features.digit_reader": True, "features.template_digit_classifier": True, **outside})
        assert aircraft.runtime is not None and aircraft.runtime.pipeline.digit_reader is None
        assert "runs only in the sim" in aircraft.logger.error.call_args[0][0] and "runs only in the sim" in aircraft.runtime.status()["digits"]["error"]
    # The stand-in's flag without the reader's: said, and nothing reads digits.
    alone = start(**{"features.template_digit_classifier": True})
    assert alone.runtime is not None and alone.runtime.pipeline.digit_reader is None and "needs features.digit_reader" in alone.logger.error.call_args[0][0]


def test_a_digit_reader_that_cannot_start_does_not_take_the_detections_with_it(ros, tmp_path):
    """The flight-safety review: the reader is extra to the boxes and the tracks. With no model to load the node
    says so and runs without it; a digit setting of the wrong kind stops nothing while the reader's flag is off."""
    with_vision_msgs(ros)
    node, _model = engine_node(ros, tmp_path, **{"features.digit_reader": True})
    assert node.runtime is not None and set(node.services) == {"/loc/set_enabled"} and node.runtime.pipeline.digit_reader is None
    assert "needs digit_model_path" in node.logger.error.call_args[0][0] and "digits are not read" in node.logger.error.call_args[0][0]
    for index in range(-30, 10):
        node.subscriptions[POSE_TOPIC](pose(T0 + index * 33_000_000))
    node.subscriptions[IMAGE_TOPIC](image(T0))
    tick(node)
    assert len(node.publishers["/loc/detections"].sent) == 1
    assert "needs digit_model_path" in status_of(node)["digits"]["error"]
    low, _model = engine_node(ros, tmp_path, **{"features.digit_reader": True, "features.template_digit_classifier": True, "digit_min_confidence": 0.2, "sim_only": True, "use_sim_time": True})
    assert low.runtime is not None and low.runtime.pipeline.digit_reader is None and "min_confidence" in low.logger.error.call_args[0][0]

    class Strict(RecordingNode):
        def declare_parameter(self, name, default=None):
            if name == "digit_min_confidence" and isinstance(self.overrides.get(name), int):
                raise TypeError("digit_min_confidence: expected a double, got an integer")
            return super().declare_parameter(name, default)

    import importlib

    file = tmp_path / "yolo26n.engine"
    ros.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=Strict))
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    ros.setattr(Strict, "overrides", {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "features.loc_node": True, "digit_min_confidence": 1})
    file.write_bytes(b"stand-in")
    ros.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: Model()))
    off = importlib.import_module("argus_localisation.loc_node").LocNode()
    assert off.runtime is not None, [call[0][0] for call in off.logger.error.call_args_list]
    assert "the digit reader's parameters could not be declared" in off.logger.error.call_args[0][0] and off.runtime.status()["digits"] is None
    # With the reader's flag on, that setting keeps the reader from starting, and only the reader.
    ros.setattr(Strict, "overrides", {**Strict.overrides, "features.digit_reader": True, "features.template_digit_classifier": True, "sim_only": True, "use_sim_time": True})
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    on = importlib.import_module("argus_localisation.loc_node").LocNode()
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    assert on.runtime is not None and "could not be declared" in on.runtime.status()["digits"]["error"]


def test_the_vote_is_behind_its_flag_and_a_tracks_digit_goes_out_with_the_track(ros, tmp_path):
    """LOC-8 through the node: the detector boxes a marker in four frames, the reader reads each, and the track's digit is their vote."""
    sys.path.insert(0, str(SRC.parent / "tools"))
    import synth_targets

    with_vision_msgs(ros)
    options = synth_targets.RenderOptions(width=1280, height=720, mpp=0.0045, rotation_deg=25.0, background="noise", supersample=2)
    picture, _drawn = synth_targets.render(synth_targets.SurveyMarker(digit=7), options, seed=3)

    class Boxer(Model):
        def __call__(self, image, conf, verbose):
            box = types.SimpleNamespace(xyxy=[[640.0 - 95.0, 360.0 - 95.0, 640.0 + 95.0, 360.0 + 95.0]], conf=[0.9], cls=[0])
            return [types.SimpleNamespace(boxes=[box])]

    def fly(frames=4, **flags):
        file = tmp_path / "yolo.engine"
        file.write_bytes(b"stand-in")
        ros.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: Boxer()))
        settings = {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "sim_only": True, "use_sim_time": True}
        node = node_with(ros, **{**settings, "features.loc_node": True, "start_enabled": True, **flags})
        for frame in range(frames):
            ns = T0 + frame * 100_000_000
            for index in range(-3, 1):
                node.subscriptions[POSE_TOPIC](pose(ns + index * 33_000_000))
            RecordingNode.now_ns = ns + 10_000_000
            message = image(ns)
            message.data = picture.tobytes()
            node.subscriptions[IMAGE_TOPIC](message)
            tick(node)
        return node

    reading = {"features.marker_tracker": True, "features.digit_reader": True, "features.template_digit_classifier": True}
    plain = fly(**reading)
    assert plain.runtime.pipeline.digit_votes is None and plain.runtime.pipeline.tracker.digit_votes is None
    assert plain.publishers["/loc/tracks"].sent[-1].tracks[0].digit == -1 and "vote" not in status_of(plain)["digits"]
    node = fly(**reading, **{"features.digit_vote": True})
    said = lambda made: [call[0][0] for call in made.logger.info.call_args_list]  # noqa: E731
    assert node.logger.error.call_args is None and any("one digit to one track" in line for line in said(node))
    sent = node.publishers["/loc/tracks"].sent
    assert [message.tracks[0].state for message in sent] == [MarkerTrack.TENTATIVE, MarkerTrack.TENTATIVE, MarkerTrack.CONFIRMED, MarkerTrack.CONFIRMED]
    assert [message.tracks[0].digit for message in sent] == [-1, -1, 7, 7]  # a digit once the track is on the list
    last = sent[-1].tracks[0]
    assert int(np.argmax(last.digit_log_prob)) == 7 and min(last.digit_log_prob) < -10.0  # four readings, summed
    status = status_of(node)
    vote = status["digits"]["vote"]
    assert status["digits"]["read"] == 4 and (vote["tracks"], vote["decided"], vote["contested"], vote["unique_ids"]) == (1, {str(last.track_id): 7}, [], True)
    json.dumps(status)
    # IDs that may repeat: the assignment over tracks and digits is switched off, and the node says so.
    repeats = fly(**reading, **{"features.digit_vote": True, "digit_ids_unique": False})
    assert any("the assignment over tracks and digits is off" in line for line in said(repeats))
    assert status_of(repeats)["digits"]["vote"]["unique_ids"] is False and repeats.publishers["/loc/tracks"].sent[-1].tracks[0].digit == 7
    assert (vote["faults"], vote["fault"]) == (0, "") and vote["solves"] >= 1 and vote["last_solve_ms"] >= 0.0
    # The vote's setting of the wrong kind leaves the reader alone: the vote takes IDs to be unique, and says what happened.
    class Strict(RecordingNode):
        def declare_parameter(self, name, default=None):
            if name == "digit_ids_unique" and not isinstance(self.overrides.get(name, True), bool):
                raise TypeError("digit_ids_unique: expected a boolean")
            return super().declare_parameter(name, default)

    import importlib

    ros.setitem(sys.modules, "rclpy.node", types.SimpleNamespace(Node=Strict))
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    ros.setattr(Strict, "overrides", {**node.overrides, "digit_ids_unique": 1})
    slip = importlib.import_module("argus_localisation.loc_node").LocNode()
    ros.delitem(sys.modules, "argus_localisation.loc_node", raising=False)
    assert slip.runtime is not None and slip.runtime.pipeline.digit_reader is not None and slip.runtime.pipeline.digit_votes.unique_ids is True
    assert "digit_ids_unique could not be declared" in slip.logger.error.call_args[0][0]
    # The vote without the tracker, or without a reader that started: said, and the node runs as it would without the vote.
    for missing, needs in (({"features.digit_reader": True, "features.template_digit_classifier": True}, "features.marker_tracker"), ({"features.marker_tracker": True}, "a digit reader that started")):
        short = fly(frames=1, **missing, **{"features.digit_vote": True})
        assert short.runtime is not None and short.runtime.pipeline.digit_votes is None and needs in short.logger.error.call_args[0][0]
        assert len(short.publishers["/loc/detections"].sent) == 1

    # LOC-12: with the post-landing flag on, the flight's tracks are kept with their votes as they stand when the survey ends.
    from argus_localisation import second_pass

    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    keeping = fly(**reading, **{"features.digit_vote": True, "features.survey_frame_log": True, "features.post_landing_pass": True, "frame_log_dir": str(frames_dir), "frame_log_workers": 1, "frame_log_min_free_gb": 0.0})
    assert keeping.logger.error.call_args is None
    keeping.destroy_node()
    (folder,) = list(frames_dir.iterdir())
    tracks, _origin, settings = second_pass.load_live_tracks(folder / "live_tracks.json")
    (track,) = tracks
    votes = keeping.runtime.pipeline.digit_votes
    assert track.digit == 7 and track.vote_readings == votes.readings(track.track_id) >= 3 and track.vote_weight == pytest.approx(votes.weight(track.track_id)) and track.vote_weight > 0.0
    assert track.digit_log_prob == pytest.approx(votes.log_probabilities(track.track_id)) and settings["digit_ids_unique"] is True
    # ... and summed with a second pass's votes by the vote's own rule, they are still a 7.
    again = second_pass.merge(tracks, [second_pass.KeptTrack(9, "marker", track.state, track.reported_east_m + 0.2, track.reported_north_m, 20, digit_log_prob=track.digit_log_prob, vote_weight=track.vote_weight, vote_readings=track.vote_readings, source="second pass")])
    assert len(again) == 1 and again[0].digit == 7 and again[0].vote_readings == 2 * track.vote_readings


def test_the_judges_list_is_behind_its_flag_and_is_written_as_the_tracks_get_their_digits(ros, tmp_path):
    """LOC-10 through the node: four frames of a drawn 7, GPS fixes beside the poses, and the list on the disk."""
    sys.path.insert(0, str(SRC.parent / "tools"))
    import synth_targets

    from argus_localisation import judges_list as jl

    with_vision_msgs(ros)
    options = synth_targets.RenderOptions(width=1280, height=720, mpp=0.0045, rotation_deg=25.0, background="noise", supersample=2)
    picture, _drawn = synth_targets.render(synth_targets.SurveyMarker(digit=7), options, seed=3)
    home = (40.4237000, -86.9212000)

    class Boxer(Model):
        def __call__(self, image, conf, verbose):
            box = types.SimpleNamespace(xyxy=[[640.0 - 95.0, 360.0 - 95.0, 640.0 + 95.0, 360.0 + 95.0]], conf=[0.9], cls=[0])
            return [types.SimpleNamespace(boxes=[box])]

    class StatusText:
        def __init__(self):
            self.severity, self.text = 0, ""

    def fix(ns, east, north, status=0):
        latitude, longitude = jl.local_to_latlon(east, north, *home)
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(ns)), status=types.SimpleNamespace(status=status), latitude=latitude, longitude=longitude)

    def fly(frames=4, fixes=True, **flags):
        file = tmp_path / "yolo.engine"
        file.write_bytes(b"stand-in")
        ros.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=lambda path, task: Boxer()))
        ros.setitem(sys.modules, "mavros_msgs", types.SimpleNamespace(msg=types.SimpleNamespace(StatusText=StatusText)))
        ros.setitem(sys.modules, "mavros_msgs.msg", types.SimpleNamespace(StatusText=StatusText))
        settings = {"camera_mode": "sim_survey", "airframe": "sim_iris_downcam", "detector": "engine", "engine_path": str(file), "sim_only": True, "use_sim_time": True}
        node = node_with(ros, **{**settings, "features.loc_node": True, "start_enabled": True, **flags})
        for frame in range(frames):
            ns = T0 + frame * 100_000_000
            for index in range(-3, 1):
                node.subscriptions[POSE_TOPIC](pose(ns + index * 33_000_000, east=30.0, north=12.0))
                if fixes and "/mavros/global_position/global" in node.subscriptions:
                    node.subscriptions["/mavros/global_position/global"](fix(ns + index * 33_000_000, 30.0, 12.0))
            RecordingNode.now_ns = ns + 10_000_000
            message = image(ns)
            message.data = picture.tobytes()
            node.subscriptions[IMAGE_TOPIC](message)
            tick(node)
        return node

    everything = {"features.marker_tracker": True, "features.digit_reader": True, "features.template_digit_classifier": True, "features.digit_vote": True}
    plain = fly(**everything, judges_list_dir=str(tmp_path / "unused"))
    assert plain._list_writer is None and "list" not in status_of(plain) and not (tmp_path / "unused").exists()
    assert "/mavros/global_position/global" not in plain.subscriptions and "/mavros/statustext/send" not in plain.publishers
    folder = tmp_path / "list"
    node = fly(**everything, **{"features.judges_list": True, "judges_list_dir": str(folder)})
    assert node.logger.error.call_args is None
    node._list_writer.stop(wait_s=5.0)  # (the writer has its own thread; the test waits for it)
    rows = (folder / "judges_list.csv").read_text().splitlines()
    assert rows[0] == "ID,Latitude,Longitude" and len(rows) == 2 and rows[1].startswith("7,")
    from drone_mission_core.target_access import latlon_to_local_enu

    track = node.publishers["/loc/tracks"].sent[-1].tracks[0]
    _id, latitude, longitude = rows[1].split(",")
    assert latlon_to_local_enu(float(latitude), float(longitude), home[0], home[1], 0.0, 0.0) == pytest.approx((track.position_enu.x, track.position_enu.y), abs=0.05)
    assert (folder / "judges_list.kml").exists() and (folder / "judges_list.png").exists()
    # The count went to the ground station as it changed: found and unread, then read.
    # The count goes to the ground station when it changes, and no more often than every two seconds: the first at
    # once, the next (0.2 s later here) when the period is up, by the status timer.
    texts = lambda made: [(message.severity, message.text) for message in made.publishers["/mavros/statustext/send"].sent]  # noqa: E731
    assert texts(node) == [(6, "LOC list: 0 markers")]
    assert status_of(node)["list"]["statustexts"] == 1 and texts(node) == [(6, "LOC list: 0 markers")]  # still inside the period
    RecordingNode.now_ns += 3 * S
    listed = status_of(node)["list"]
    assert texts(node) == [(6, "LOC list: 0 markers"), (6, "LOC list: 1 marker")] and (listed["statustexts"], listed["statustext_error"], listed["kept_earlier"]) == (2, "", "")
    status_of(node)
    assert len(texts(node)) == 2  # nothing new to say
    assert (listed["rows"], listed["unread"], listed["error"], listed["file"]) == (1, 0, "", str(folder / "judges_list.csv")) and listed["written"] >= 1 and listed["origin_fixes"] == 16
    # With no GPS fix there is no latitude to give: the marker is found, unread, and the list has its header alone.
    blind = fly(fixes=False, **everything, **{"features.judges_list": True, "judges_list_dir": str(tmp_path / "blind")})
    blind._list_writer.stop(wait_s=5.0)
    assert (tmp_path / "blind" / "judges_list.csv").read_text() == "ID,Latitude,Longitude\n" and status_of(blind)["list"]["unread"] == 1
    RecordingNode.now_ns += 3 * S
    status_of(blind)
    assert blind.publishers["/mavros/statustext/send"].sent[-1].text == "LOC list: 0 markers, 1 more unread"
    # Stopping the pipeline writes the list as it finally stands.
    closing = fly(frames=3, **everything, **{"features.judges_list": True, "judges_list_dir": str(tmp_path / "closing")})
    set_enabled(closing, False)
    closing._list_writer.stop(wait_s=5.0)
    assert (tmp_path / "closing" / "judges_list.csv").read_text().splitlines()[1].startswith("7,")
    # A list that cannot be kept (no folder given, no tracker, a parent that is not there) is said; the node runs on.
    for wrong, why in (({"judges_list_dir": ""}, "judges_list_dir must be set"), ({"judges_list_dir": str(tmp_path / "no" / "such" / "list")}, "is not a folder that exists")):
        short = fly(frames=1, **everything, **{"features.judges_list": True, **wrong})
        assert short.runtime is not None and short._list_writer is None and why in short.logger.error.call_args[0][0] and why in status_of(short)["list"]["error"]
        assert len(short.publishers["/loc/detections"].sent) == 1
    alone = fly(frames=1, **{"features.judges_list": True, "judges_list_dir": str(folder)})
    assert alone.runtime is not None and "needs features.marker_tracker" in alone.logger.error.call_args[0][0]
