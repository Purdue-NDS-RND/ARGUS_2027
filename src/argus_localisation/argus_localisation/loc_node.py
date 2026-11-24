"""The localisation node (LOC-5): a thin wrapper round runtime.py and pipeline.py.

Behind the node flag `features.loc_node`, which defaults to false. With the
flag off the node makes no subscription, publisher or service, opens no
camera and does nothing.

With it on:

  - **Frames.** A mode whose driver is a real camera is read by the
    camera's own FrameSource inside this node, on a thread of its own, so
    full frames never cross a ROS topic. The sim's modes (driver
    `ros_topic`) come from `image_topic`, taken with a queue of one. Either
    way only the newest frame is kept: if the pipeline falls behind it
    drops frames, it never queues them.
  - **Detector.** `detector: engine` runs the TensorRT engine named by
    `engine_path` the way the 2026 node did, through Ultralytics, on the
    Jetson (engine.py); `engine_classes` says what each of the engine's
    class numbers is on the contract. With no engine file, or no
    Ultralytics, the node says so and does not start. `detector: oracle`
    is the sim's stand-in (oracle.py): it needs
    `features.sim_oracle_detector`, `use_sim_time` and `sim_only`, and
    refuses otherwise.
  - **On and off.** The pipeline is off until `/loc/set_enabled`
    (std_srvs/SetBool) starts it, unless `start_enabled`; the survey
    mission starts it at its first track and stops it at its end.
  - **Out.** `/loc/detections`: each frame's boxes on the detection
    contract (FND-02), stamped with the frame's capture time, in the full
    frame's pixels. `/loc/status`: once a second, a std_msgs/String of JSON
    (runtime.LocRuntime.status: frames, rate, dropped, detections, tracks,
    confirmed IDs).
  - The camera is named by `camera_mode` and `airframe`, loaded with the
    loader projection v2 uses, with its refusals. Boxes are put on the
    ground with MAVROS's pose at the frame's capture time, on the
    rangefinder's ground when it is believed and otherwise on the site's
    (`ground_below_home_m`, the survey mission's own setting).
  - `features.loc_tiling` gives the detector overlapping windows of the
    full frame instead of the shrunk frame.
  - `features.marker_tracker` runs the marker tracker (marker_tracker.py,
    LOC-7) on every frame's ground points and publishes its marker tracks
    on `/loc/tracks` (argus_interfaces/MarkerTrackArray) after each frame;
    the status then counts its tracks and lists the confirmed ones. With
    the flag off there is no tracker and no such topic.
  - `features.survey_frame_log` logs every frame the camera gives while
    the pipeline is on, as JPEG at quality 90, to a new folder for each
    survey under `frame_log_dir` (frame_logging.py, LOC-11; the writer is
    argus_camera's FrameLog). The logger gives way to the detector: when
    the pipeline runs under nine tenths of the rate it should have, or
    under its minimum rate, the logger keeps every second frame, then
    every fourth, and comes back when the pipeline is well. `/loc/status`
    carries its counts under `frame_log`.
  - `features.digit_reader` reads the digit of every `marker` crop
    (digit_reader.py, LOC-6): corners, a 64 x 64 patch stood by the X's
    arms, a classifier on its four quarter turns, and no answer rather
    than an unsure one. The classifier is the ONNX model at
    `digit_model_path`, run on the CPU, or with
    `features.template_digit_classifier` a stand-in for the sim's marker
    model (sim time and `sim_only` required, as for the oracle). A reader
    that cannot start is an error in the log and in `/loc/status`
    (`digits.error`); the node runs without it.
  - `features.digit_vote` (with the tracker and the digit reader on) gives
    a track its digit: the weighted vote of its readings (digit_vote.py,
    LOC-8), with one assignment over tracks and digits while
    `digit_ids_unique` is true. The digit and the summed log-probabilities
    go out in the track's own fields on `/loc/tracks`.
  - `features.post_landing_pass` (LOC-12; with the frame log and the
    tracker on) keeps, beside the survey's logged frames, what a second
    look at them after landing needs: the survey's poses and rangefinder
    readings as they come (`flight_record.jsonl`) and, when the survey
    stops, the tracks as the flight leaves them with their digit votes
    (`live_tracks.json`). The pass itself is `tools/second_pass.py`, run
    after landing; the node does not run it.
  - `features.judges_list` (with the tracker on) writes the judges' list
    (judges_list.py, LOC-10) in `judges_list_dir`: `judges_list.csv`
    (`ID,Latitude,Longitude`), a KML and a map image, each replaced whole
    as the list changes, on a thread of its own. The local frame is placed
    on the earth from the fixes on GPS_TOPIC, and the count goes to the
    ground station as a STATUSTEXT.
  - `features.loc_median_position` (with the tracker on) makes a track's
    published position the weighted median of its sightings
    (final_position.py, LOC-9) instead of the filter's position.

If the node cannot start with its flag on (a camera that is not calibrated,
an oracle outside the sim) it logs why and stays out of the way: nothing is
published and the service is not offered.
"""

from __future__ import annotations

import json
import threading
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node

DETECTIONS_TOPIC = "/loc/detections"
STATUS_TOPIC = "/loc/status"
ENABLE_SERVICE = "/loc/set_enabled"
POSE_TOPIC = "/mavros/local_position/pose"
VELOCITY_TOPIC = "/mavros/local_position/velocity_local"  # lets the pose history carry on a few milliseconds past its newest pose
RANGEFINDER_TOPIC = "/mavros/rangefinder/rangefinder"
GPS_TOPIC = "/mavros/global_position/global"  # the flight controller's own latitude and longitude, for the judges' list
STATUSTEXT_TOPIC = "/mavros/statustext/send"  # to the ground station, through the flight controller
STATUSTEXT_INFO = 6  # MAV_SEVERITY_INFO
TICK_S = 0.01  # how often the node looks for a new frame
STATUS_EVERY_S = 1.0
READ_RETRY_S = 0.2  # the pause after a failed read of the camera
NO_FRAME_PAUSE_S = 0.005  # and after a read that gave no frame
DETECTORS = ("oracle", "engine")


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class LocNode(Node):
    def __init__(self) -> None:
        super().__init__("loc_node")
        self.runtime: Any = None
        self._frame_log: Any = None
        self._source: Any = None
        self._reader: Optional[threading.Thread] = None
        self._stopping = threading.Event()
        self._seq = 0
        self._list_writer: Any = None  # the judges' list (LOC-10), with the flag on
        self._second_pass: Any = None  # the second_pass module, with features.post_landing_pass on (LOC-12)
        self._flight_record: Any = None  # the survey's poses and ranges, written beside its frames for the second pass
        self._record_folder: Any = None
        self._last_record: Any = None  # the record last closed (its `failed` says whether the disk took it)
        self._list_origin: Any = None
        self._list_text = ""  # the count as it now stands
        self._list_text_sent = ""  # and as it last went to the ground station
        self._list_text_sent_ns: Optional[int] = None
        self._statustexts = 0
        self._statustext_error = ""
        self._list_counts = (0, 0)
        self._statustext: Any = None
        try:
            self.declare_parameter("features.loc_node", False)
            self.declare_parameter("features.sim_oracle_detector", False)
            self.declare_parameter("features.loc_tiling", False)
            self.declare_parameter("features.marker_tracker", False)
            self.declare_parameter("features.loc_median_position", False)
            self.declare_parameter("features.survey_frame_log", False)
            self.declare_parameter("features.digit_reader", False)
            self.declare_parameter("features.template_digit_classifier", False)
            self.declare_parameter("features.digit_vote", False)
            self.declare_parameter("features.judges_list", False)
            self.declare_parameter("features.post_landing_pass", False)
            self.declare_parameter("camera_mode", "")
            self.declare_parameter("airframe", "")
            self.declare_parameter("image_topic", "camera/image")
            self.declare_parameter("detector", "engine")
            self.declare_parameter("engine_path", "")
            self.declare_parameter("engine_classes", ["marker"])
            self.declare_parameter("start_enabled", False)
            self.declare_parameter("sim_only", False)
            self.declare_parameter("ground_below_home_m", 0.0)
            self.declare_parameter("oracle_objects_file", "")
            self.declare_parameter("oracle_seed", 0)
            self.declare_parameter("oracle_pixel_sigma_px", 2.0)
            self.declare_parameter("oracle_miss_rate", 0.05)
            self.declare_parameter("oracle_false_alarms_per_frame", 0.02)
        except Exception as error:  # noqa: BLE001  (a setting of the wrong kind in the params file: say so and stay up, doing nothing)
            self.get_logger().error(f"the localisation node's parameters could not be declared ({error}): the node does nothing")
            return
        # The frame log's settings are declared by themselves: one of the wrong kind in the params file must not
        # stop the node when the log's flag is off. With the flag on it stops the start below, and says why.
        self._frame_log_settings_problem = ""
        try:
            self.declare_parameter("frame_log_dir", "")
            self.declare_parameter("frame_log_workers", 3)
            self.declare_parameter("frame_log_jpeg_quality", 90)
            self.declare_parameter("frame_log_max_gb", 400.0)
            self.declare_parameter("frame_log_min_free_gb", 20.0)
        except Exception as error:  # noqa: BLE001
            self._frame_log_settings_problem = str(error)
            self.get_logger().error(f"the frame log's parameters could not be declared ({error}): the survey's frames cannot be logged")
        # The digit reader's settings likewise. The reader is extra to the boxes and the tracks: whatever keeps it
        # from starting is said, and the node runs without it.
        self._digit_settings_problem = ""
        try:
            self.declare_parameter("digit_model_path", "")
            self.declare_parameter("digit_min_confidence", 0.9)
        except Exception as error:  # noqa: BLE001
            self._digit_settings_problem = str(error)
            self.get_logger().error(f"the digit reader's parameters could not be declared ({error}): digits cannot be read")
        # And the vote's one setting, apart again: one of the wrong kind leaves the reader alone, and the vote on its default.
        self._vote_settings_problem = ""
        try:
            self.declare_parameter("digit_ids_unique", True)
        except Exception as error:  # noqa: BLE001
            self._vote_settings_problem = str(error)
            self.get_logger().error(f"digit_ids_unique could not be declared ({error}): the digit vote, if on, takes IDs to be unique")
        self._list_settings_problem = ""
        try:
            self.declare_parameter("judges_list_dir", "")
        except Exception as error:  # noqa: BLE001
            self._list_settings_problem = str(error)
            self.get_logger().error(f"judges_list_dir could not be declared ({error}): the judges' list cannot be written")
        if not self._flag("features.loc_node"):
            self.get_logger().info("Localisation node off (features.loc_node): nothing subscribed, nothing published, no camera opened")
            return
        try:
            self._start()
        except Exception as error:  # noqa: BLE001  (a pipeline that cannot start must say so and stay out of the way)
            self.runtime = None
            self.get_logger().error(f"features.loc_node is on but the localisation pipeline could not start ({error}): nothing is published")

    def _flag(self, name: str) -> bool:
        return bool(self.get_parameter(name).get_parameter_value().bool_value)

    def _text(self, name: str) -> str:
        return str(self.get_parameter(name).get_parameter_value().string_value).strip()

    def _number(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _sim_time(self) -> bool:
        if not self.has_parameter("use_sim_time"):
            return False
        return bool(self.get_parameter("use_sim_time").value)

    def _detector(self, camera: Any, mount: Any) -> Any:
        """The detector the parameters ask for, or an error saying why it cannot run. Makes nothing on the ROS side."""
        name = self._text("detector")
        if name not in DETECTORS:
            raise ValueError(f"detector must be one of {', '.join(DETECTORS)}, got '{name}'")
        if name == "engine":
            from . import engine, pipeline

            return engine.load_engine(self._text("engine_path"), list(self.get_parameter("engine_classes").value or []), pipeline.MIN_SCORE)
        if self._flag("features.loc_tiling"):
            raise ValueError("features.loc_tiling is on with detector 'oracle': tiling has nothing to give the oracle, which does not look at the picture")
        from . import oracle

        if not self._flag(f"features.{oracle.FLAG}"):
            raise ValueError(f"detector 'oracle' needs the node flag features.{oracle.FLAG}, which is off")
        oracle.check_sim_only(self._sim_time(), self._flag("sim_only"))
        vehicle, objects = oracle.load_objects(self._text("oracle_objects_file"))
        self._truth = oracle.TruthHistory(vehicle)
        self.get_logger().warn(f"Sim oracle detector on (features.{oracle.FLAG}): reporting {len(objects)} true object(s) from {oracle.TRUTH_TOPIC}, not looking at the picture")
        return oracle.OracleDetector(
            camera,
            mount,
            objects,
            self._truth,
            seed=int(self._number("oracle_seed")),
            pixel_sigma_px=self._number("oracle_pixel_sigma_px"),
            miss_rate=self._number("oracle_miss_rate"),
            false_alarms_per_frame=self._number("oracle_false_alarms_per_frame"),
        )

    def _digit_reader(self, camera: Any) -> Tuple[Any, str]:
        """(the digit reader, "") when its flag is on and it can start; (None, why not) when it cannot; (None, "") when off.

        The reader is extra to the boxes and the tracks, so nothing here stops the node: a reader that cannot
        start is an error in the log and in `/loc/status`, and the survey's detections go on without digits.
        """
        stand_in = self._flag("features.template_digit_classifier")
        if not self._flag("features.digit_reader"):
            if stand_in:
                self.get_logger().error("features.template_digit_classifier needs features.digit_reader, which is off: it is the digit reader's stand-in classifier, and nothing reads digits")
            return None, ""
        try:
            if self._digit_settings_problem:
                raise ValueError(f"its parameters could not be declared ({self._digit_settings_problem})")
            from . import digit_reader

            if stand_in:
                digit_reader.check_stand_in_sim_only(self._sim_time(), self._flag("sim_only"))
                classifier = digit_reader.TemplateDigitClassifier()
            else:
                classifier = digit_reader.load_onnx(self._text("digit_model_path"))
            reader = digit_reader.DigitReader(classifier, camera=camera, min_confidence=self._number("digit_min_confidence"))
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"features.digit_reader is on but the digit reader could not start ({error}): digits are not read; the detections go on without them")
            return None, str(error)
        if stand_in:
            self.get_logger().warn("Template digit classifier on (features.template_digit_classifier): a stand-in that reads the seven-segment digit of the sim's marker model, not a trained model")
        return reader, ""

    def _digit_votes(self, reader: Any) -> Any:
        """The vote over a track's readings when its flag is on and there is something to vote on; otherwise None, and why is said.

        Like the reader it is extra to the boxes and the tracks: nothing here stops the node.
        """
        if not self._flag("features.digit_vote"):
            return None
        if self._tracker is None or reader is None:
            missing = "features.marker_tracker" if self._tracker is None else "a digit reader that started (features.digit_reader)"
            self.get_logger().error(f"features.digit_vote is on but needs {missing}: it sums a track's digit readings, and no track gets a digit")
            return None
        from . import digit_vote

        unique = True if self._vote_settings_problem else bool(self.get_parameter("digit_ids_unique").value)
        # The assignment over tracks and digits is solved at most twice a second: the vote shares the detector's thread.
        votes = digit_vote.DigitVotes(unique_ids=unique, solve_period_s=digit_vote.NODE_SOLVE_PERIOD_S)
        self._tracker.digit_votes = votes
        self.get_logger().info(
            "Digit vote on (features.digit_vote): a track's digit is the weighted vote of its readings"
            + ("; one digit to one track (digit_ids_unique)" if unique else "; the assignment over tracks and digits is off (digit_ids_unique false)")
        )
        return votes

    def _judges_list(self) -> None:
        """Make the judges' list's writer when its flag is on. Like the reader and the vote, nothing here stops the node:
        a list that cannot be kept is an error in the log and in `/loc/status`."""
        self._list_error = ""
        if not self._flag("features.judges_list"):
            return
        try:
            if self._tracker is None:
                raise ValueError("it needs features.marker_tracker: the list is the tracker's listed tracks")
            if self._list_settings_problem:
                raise ValueError(f"judges_list_dir could not be declared ({self._list_settings_problem})")
            from . import judges_list

            folder = judges_list.JudgesList(self._text("judges_list_dir"))
            if not folder.folder.parent.is_dir():
                raise ValueError(f"judges_list_dir is {folder.folder}, and {folder.folder.parent} is not a folder that exists")
        except Exception as error:  # noqa: BLE001
            self._list_error = str(error)
            self.get_logger().error(f"features.judges_list is on but the judges' list cannot be kept ({error}): no list is written")
            return
        self._judges_list_module = judges_list
        self._list_origin = judges_list.OriginEstimate()
        self._list_writer = judges_list.ListWriter(folder)
        self._list_writer.start()
        self.get_logger().info(f"Judges' list on (features.judges_list): {folder.csv_path}, rewritten as it changes")

    def _start(self) -> None:
        import vision_msgs.msg as vision_messages
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image, Range
        from std_msgs.msg import String
        from std_srvs.srv import SetBool

        from argus_camera import modes as camera_modes
        from argus_camera.bench import default_config_dir
        from drone_mission_core import detection_contract as contract
        from drone_mission_core.frame_projection import load_camera
        from drone_mission_core.pose_buffer import PoseBuffer
        from drone_mission_core.projection_v2 import GroundHeight, GroundProjector

        from . import frames, pipeline, runtime

        # First everything that can refuse: the camera's calibration, the detector, the camera itself. Nothing is
        # made on the ROS side until all of it stands, so a node that could not start offers no service.
        self._contract, self._vision_messages, self._frames, self._String = contract, vision_messages, frames, String
        mode_name, airframe = self._text("camera_mode"), self._text("airframe")
        if not mode_name or not airframe:
            raise ValueError("camera_mode and airframe must both be set: the sensor mode the frames come from, and the airframe in argus_camera's robot.yaml")
        camera, mount = load_camera(mode_name, airframe)  # refuses what projection v2 refuses
        self._mode_name = mode_name
        self._poses = PoseBuffer()
        self._ground = GroundHeight(self._poses.pose_at)
        ground = pipeline.SiteGround(self._ground, self.get_parameter("ground_below_home_m").value)
        self._truth = None
        mode = camera_modes.load_modes(default_config_dir() / "modes.yaml")[mode_name]
        frame_log = None
        if self._flag("features.survey_frame_log"):
            from . import frame_logging

            if self._frame_log_settings_problem:
                raise ValueError(f"features.survey_frame_log is on and the frame log's parameters could not be declared ({self._frame_log_settings_problem})")
            workers = int(self._number("frame_log_workers"))
            if workers < 1:
                raise ValueError("frame_log_workers must be 1 or more: the encoding runs on worker threads, not on the camera's or the detector's")
            frame_log = frame_logging.SurveyFrameLog(
                self._text("frame_log_dir"),
                jpeg_quality=int(self._number("frame_log_jpeg_quality")),
                workers=workers,
                max_bytes=int(self._number("frame_log_max_gb") * 1.0e9),
                # The detector cannot be faster than its camera: its rate is held to the lower of its target and the mode's.
                expected_rate_hz=min(runtime.TARGET_RATE_HZ, float(mode.fps)),
                min_free_bytes=self._number("frame_log_min_free_gb") * 1.0e9,
            )
        if self._flag("features.post_landing_pass"):
            # LOC-12: what the pass after landing needs from the flight is kept beside the frames. It needs the frames and the tracks.
            # Like the reader, the vote and the list it is extra to the flight's own detections: without what it needs it is said and left out.
            if frame_log is None or not self._flag("features.marker_tracker"):
                self.get_logger().error("features.post_landing_pass needs features.survey_frame_log and features.marker_tracker (it looks again at the logged frames and merges with the flight's tracks): nothing is kept for a pass after landing")
            else:
                from . import second_pass

                self._second_pass = second_pass
        detector = self._detector(camera, mount)
        projector = GroundProjector(camera, mount, self._poses.pose_at, ground)
        frame_id = str(getattr(mount, "optical_frame", "") or "down_camera_optical")
        self._tracker = None
        if self._flag("features.loc_median_position") and not self._flag("features.marker_tracker"):
            raise ValueError("features.loc_median_position needs features.marker_tracker: it is the position of the tracker's tracks")
        if self._flag("features.marker_tracker"):
            import argus_interfaces.msg as track_messages

            from . import marker_tracker
            from . import track_messages as tracks_out

            from drone_mission_core.estimator_feed import PositionResetDetector

            tracker = marker_tracker.MarkerTracker(median_positions=self._flag("features.loc_median_position"))
            self._tracker, self._track_messages, self._tracks_out = tracker, track_messages, tracks_out
            self._resets = PositionResetDetector()  # a step in the reported position that the reported velocity does not explain
        reader, digit_error = self._digit_reader(camera)
        votes = self._digit_votes(reader)
        self._judges_list()
        chain = pipeline.LocPipeline(
            detector,
            projector,
            frame_id,
            (int(camera.width), int(camera.height)),
            tracker=self._tracker,
            digit_reader=reader,
            digit_votes=votes,
            tiling=self._flag("features.loc_tiling"),
            pose_at=self._poses.pose_at,
        )
        from_topic = mode.driver == "ros_topic"
        if not from_topic:
            from argus_camera.frame_source import open_source

            source = open_source(mode, clock=lambda: int(self.get_clock().now().nanoseconds))
            source.open()  # raises when the camera does not open
            self._source = source
        self.runtime = runtime.LocRuntime(chain, lambda: int(self.get_clock().now().nanoseconds), start_enabled=self._flag("start_enabled"))
        self.runtime.digit_error = digit_error
        self._frame_log = frame_log
        if self.runtime.enabled:
            self._start_frame_log()

        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, VELOCITY_TOPIC, self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(Range, RANGEFINDER_TOPIC, self._on_range, qos_profile_sensor_data)
        if self._truth is not None:
            from tf2_msgs.msg import TFMessage

            from . import oracle

            self.create_subscription(TFMessage, oracle.TRUTH_TOPIC, self._on_truth, qos_profile_sensor_data)
        if self._list_writer is not None:
            # Where the local frame is on the earth, from the vehicle's own fixes; and the count, to the ground station.
            try:
                from sensor_msgs.msg import NavSatFix

                self.create_subscription(NavSatFix, GPS_TOPIC, self._on_fix, qos_profile_sensor_data)
            except Exception as error:  # noqa: BLE001
                self._list_error = f"no GPS fixes to place the list by ({error})"
                self.get_logger().error(f"the judges' list has {self._list_error}: its markers stay unread")
            try:
                from mavros_msgs.msg import StatusText

                self._StatusText = StatusText
                self._statustext = self.create_publisher(StatusText, STATUSTEXT_TOPIC, 10)
            except Exception as error:  # noqa: BLE001
                self._statustext_error = str(error)
                self.get_logger().error(f"the judges' list's count cannot be sent to the ground station ({error}); the list is still written")
        self._detections = self.create_publisher(vision_messages.Detection2DArray, DETECTIONS_TOPIC, 10)
        self._status = self.create_publisher(String, STATUS_TOPIC, 10)
        self._tracks = None
        if self._tracker is not None:
            self._tracks = self.create_publisher(self._track_messages.MarkerTrackArray, self._tracks_out.TRACKS_TOPIC, 10)
        if from_topic:
            # The sim's camera: the newest image only, never one that waited behind another.
            newest = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
            topic = self._text("image_topic")
            self.create_subscription(Image, topic, self._on_image, newest)
            frames_from = f"the topic {topic}"
        else:
            self._reader = threading.Thread(target=self._read_frames, name="loc_frames", daemon=True)
            self._reader.start()
            frames_from = f"the camera's own frame source ({mode.driver})"
        self.create_service(SetBool, ENABLE_SERVICE, self._on_set_enabled)
        self.create_timer(TICK_S, self._tick)
        self.create_timer(STATUS_EVERY_S, self._publish_status)
        self.get_logger().info(
            f"Localisation node on (features.loc_node): {mode_name} on {airframe}, frames from {frames_from}, detector '{detector.name}', "
            f"{'started' if self.runtime.enabled else f'waiting for {ENABLE_SERVICE}'}"
        )

    def _start_frame_log(self) -> None:
        if self._frame_log is None:
            return
        folder = self._frame_log.start(int(self.get_clock().now().nanoseconds))
        if folder is None:
            self.get_logger().error(f"localisation: the survey's frames are not being logged ({self._frame_log.status()['error']})")
        else:
            self.get_logger().info(f"localisation: logging the survey's frames to {folder}")
            if self._second_pass is not None:
                try:
                    self._flight_record, self._record_folder = self._second_pass.FlightRecord(folder), folder
                    self.get_logger().info(f"localisation: recording the survey's poses and ranges to {self._flight_record.path} for the pass after landing (features.post_landing_pass)")
                    if self._last_record is not None and self._last_record.failed:
                        self.get_logger().error(f"localisation: the last survey's record for the pass after landing was not written whole ({self._last_record.failed})")
                except OSError as error:
                    self._flight_record = None
                    self.get_logger().error(f"localisation: the survey's poses cannot be recorded ({error}): no pass after landing can be run on this survey")

    def _close_flight_record(self, wait_s: Optional[float] = None) -> None:
        """The survey has stopped: the record is closed and the tracks as the flight leaves them are written beside it (LOC-12).
        What is written is gathered here, from memory; the writing is the record's own thread's, so the stop does not wait on the disk."""
        record, self._flight_record = self._flight_record, None
        if record is None:
            return
        try:
            votes = getattr(self.runtime.pipeline, "digit_votes", None)
            if votes is not None:
                votes.settle()
            origin = self._list_origin.origin() if self._list_origin is not None else None
            tracks = self._second_pass.kept(self._tracker.tracks(), votes)
            settings = {
                "ground_below_home_m": float(self.get_parameter("ground_below_home_m").value),
                "digit_min_confidence": 0.9 if self._digit_settings_problem else float(self._number("digit_min_confidence")),
                "median_positions": self._flag("features.loc_median_position"),
                "digit_ids_unique": True if self._vote_settings_problem else bool(self.get_parameter("digit_ids_unique").value),
            }
            path, module = self._record_folder / self._second_pass.LIVE_TRACKS_NAME, self._second_pass
            record.close(then=lambda: module.save_live_tracks(path, tracks, origin, settings), wait_s=wait_s)
            self._last_record = record
            self.get_logger().info(f"localisation: the survey's poses and ranges and {len(tracks)} track(s) are being kept in {self._record_folder} for the pass after landing")
        except Exception as error:  # noqa: BLE001  (the pass after landing is extra to the flight's own list)
            record.close()
            self.get_logger().error(f"localisation: what the pass after landing needs could not be kept ({error}); the flight's own list stands")

    # --- inputs ----------------------------------------------------------------------

    def _on_pose(self, message: Any) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        stamp_ns = _stamp_ns(message.header.stamp)
        self._poses.add_pose(stamp_ns, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        if self._flight_record is not None:
            self._flight_record.pose(stamp_ns, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        if self._tracker is None:
            return
        # An EKF reset moves everything the vehicle will see from here on: the tracks made before it go with it.
        previous_ns = self._resets.last_stamp_ns
        velocity = None if previous_ns is None else (self._poses.velocity_at(previous_ns) or self._poses.velocity_at(stamp_ns))
        step = self._resets.add(stamp_ns, (position.x, position.y), velocity)
        if step is not None:
            self._tracker.shift_all(*step)
            self.get_logger().warn(f"localisation: the flight controller's position stepped {step[0]:+.2f} m east, {step[1]:+.2f} m north; the marker tracks stepped with it")

    def _on_velocity(self, message: Any) -> None:
        linear = message.twist.linear
        self._poses.add_velocity(_stamp_ns(message.header.stamp), (linear.x, linear.y, linear.z))
        if self._flight_record is not None:
            self._flight_record.velocity(_stamp_ns(message.header.stamp), (linear.x, linear.y, linear.z))

    def _on_range(self, message: Any) -> None:
        self._ground.add_range(_stamp_ns(message.header.stamp), message.range, message.min_range, message.max_range)
        if self._flight_record is not None:
            self._flight_record.range(_stamp_ns(message.header.stamp), message.range, message.min_range, message.max_range)

    def _on_truth(self, message: Any) -> None:
        poses, stamp_ns = {}, None
        for transform in message.transforms:
            stamp_ns = _stamp_ns(transform.header.stamp)
            at, turn = transform.transform.translation, transform.transform.rotation
            poses[str(transform.child_frame_id)] = (at.x, at.y, at.z, turn.x, turn.y, turn.z, turn.w)
        if stamp_ns is not None:
            self._truth.add(stamp_ns, poses)

    def _on_image(self, message: Any) -> None:
        if not self.runtime.enabled:
            return  # an image is not even unpacked while the pipeline is off
        self._seq += 1
        try:
            frame = self._frames.frame_from_message(message, self._seq, self._mode_name)
            self.runtime.offer(frame)
            if self._frame_log is not None:
                self._frame_log.offer(frame)  # every frame of the survey, whether or not the detector gets to it
        except ValueError as error:
            self.get_logger().error(f"localisation: {error}", throttle_duration_sec=10.0)

    def _read_frames(self) -> None:
        """The camera's own thread. A read that fails is counted and said, and reading goes on after a pause."""
        while not self._stopping.is_set():
            try:
                frame = self._source.read()
            except Exception as error:  # noqa: BLE001  (a camera fault must not end the thread without a word)
                self.runtime.note_fault(f"{type(error).__name__}: {error}")
                self.get_logger().error(f"localisation: the camera's frame source failed ({error})", throttle_duration_sec=10.0)
                self._stopping.wait(READ_RETRY_S)
                continue
            if frame is None:
                self._stopping.wait(NO_FRAME_PAUSE_S)  # a camera with nothing to give must not spin a core
                continue
            self.runtime.offer(frame)
            if self._frame_log is not None and self.runtime.enabled:
                self._frame_log.offer(frame)

    # --- the pipeline ----------------------------------------------------------------

    def _tick(self) -> None:
        result = self.runtime.step()
        if result is None:
            return
        try:
            message = self._contract.encode(result.detections, self._vision_messages)
        except Exception as error:  # noqa: BLE001  (a box the contract refuses is that frame's fault, not the node's end)
            self.runtime.note_fault(f"{type(error).__name__}: {error}")
            self.get_logger().error(f"localisation: a frame's boxes could not go out on the contract ({error})", throttle_duration_sec=10.0)
            return
        self._detections.publish(message)
        if self._tracks is None:
            return
        try:
            states = self._tracker.tracks()
            tracks = self._tracks_out.encode_tracks(states, result.stamp_ns, self._track_messages)
        except Exception as error:  # noqa: BLE001  (the tracks are extra to the boxes: a fault here is counted, and the node goes on)
            self.runtime.note_fault(f"{type(error).__name__}: {error}")
            self.get_logger().error(f"localisation: the marker tracks could not go out ({error})", throttle_duration_sec=10.0)
            return
        self._tracks.publish(tracks)
        self._offer_list(states)

    def _offer_list(self, states: Any) -> None:
        """The judges' list as the tracks now make it, to its writer; and its count to the ground station when it changes."""
        if self._list_writer is None:
            return
        try:
            module = self._judges_list_module
            rows, unread = module.rows_from_tracks(states, self._list_origin.origin())
            self._list_writer.offer(rows, unread, module.boundary_of(states))
            self._list_counts = (len(rows), len(unread))
            text = module.status_text(rows, unread, not_written=bool(self._list_writer.judges_list.error))
            if text != self._list_text:
                self._list_text = text
                self.get_logger().info(f"localisation: {text}")
            self._send_count()
        except Exception as error:  # noqa: BLE001  (the list is extra to the tracks)
            self.runtime.note_fault(f"{type(error).__name__}: {error}")
            self.get_logger().error(f"localisation: the judges' list could not be made up ({error})", throttle_duration_sec=10.0)

    def _send_count(self) -> None:
        """The list's count to the ground station when it has changed, and no more often than STATUSTEXT_PERIOD_S:
        a marker on the edge of the search area must not fill the flight controller's link."""
        if self._statustext is None or not self._list_text or self._list_text == self._list_text_sent:
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        since = None if self._list_text_sent_ns is None else now_ns - self._list_text_sent_ns
        if since is not None and 0 <= since < int(self._judges_list_module.STATUSTEXT_PERIOD_S * 1_000_000_000):
            return  # the newest count goes out when the period is up (the status timer asks again)
        message = self._StatusText()
        message.severity, message.text = STATUSTEXT_INFO, self._list_text
        self._statustext.publish(message)
        self._list_text_sent, self._list_text_sent_ns = self._list_text, now_ns
        self._statustexts += 1

    def _on_fix(self, message: Any) -> None:
        """A GPS fix, paired with the local position of its own moment: one more say in where the local frame is."""
        if getattr(getattr(message, "status", None), "status", 0) < 0:  # sensor_msgs/NavSatStatus: -1 is no fix
            return
        stamp = message.header.stamp
        at = self._poses.pose_at(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
        if at is not None:
            self._list_origin.add(message.latitude, message.longitude, at.position[0], at.position[1])

    def _publish_status(self) -> None:
        status = self.runtime.status()
        if self._frame_log is not None:
            # The logger gives way to the detector: it is shown the pipeline's status once a second.
            self._frame_log.review(status, int(self.get_clock().now().nanoseconds))
            status["frame_log"] = self._frame_log.status()
        if self._list_writer is not None or self._list_error:
            kept = None if self._list_writer is None else self._list_writer.judges_list
            if kept is not None:
                try:
                    self._send_count()  # a count held back by the period goes out now
                except Exception as error:  # noqa: BLE001
                    self._statustext_error = str(error)
            status["list"] = {
                "rows": self._list_counts[0],
                "unread": self._list_counts[1],  # listed markers with no digit yet, or no origin to place them by
                "written": 0 if kept is None else int(kept.written),
                "file": "" if kept is None else str(kept.csv_path),
                "origin_fixes": 0 if self._list_origin is None else int(self._list_origin.pairs),
                "statustexts": self._statustexts,  # counts sent to the ground station
                "statustext_error": self._statustext_error,  # why none can be sent, if none can
                "kept_earlier": "" if kept is None else kept.kept_earlier,  # a list found in the folder at the start, put aside
                "error": self._list_error or ("" if kept is None else kept.error),
            }
        message = self._String()
        message.data = json.dumps({"camera_mode": self._mode_name, **status})
        self._status.publish(message)
        if status["below_minimum_rate"]:
            self.get_logger().warn(f"localisation: {status['rate_hz']:.1f} frames a second, under the minimum of 8", throttle_duration_sec=10.0)

    def _on_set_enabled(self, request: Any, response: Any) -> Any:
        if self.runtime is None:  # (the service is made only once the pipeline stands; this is for a call during shutdown)
            response.success, response.message = False, "the localisation pipeline is not running"
            return response
        response.success = True
        was_on = self.runtime.enabled
        response.message = self.runtime.set_enabled(bool(request.data))
        if self.runtime.enabled and not was_on:
            self._start_frame_log()
        elif was_on and not self.runtime.enabled:
            if self._frame_log is not None:
                self._frame_log.stop()
            self._close_list()
            self._close_flight_record()
        self.get_logger().info(f"{ENABLE_SERVICE}: {response.message}")
        return response

    def _close_list(self) -> None:
        """The survey has stopped: the list as it finally stands, written now (on the writer's thread, not this one)."""
        if self._list_writer is None:
            return
        try:
            votes = getattr(self.runtime.pipeline, "digit_votes", None)
            if votes is not None:
                votes.settle()  # the digits are given out afresh, not as they stood half a second ago
            self._offer_list(self._tracker.tracks())
        except Exception as error:  # noqa: BLE001  (the list as it was last offered is still written)
            self.runtime.note_fault(f"{type(error).__name__}: {error}")
            self.get_logger().error(f"localisation: the judges' list could not be closed afresh ({error}); it is written as it last stood")
        finally:
            self._list_writer.wake()

    def destroy_node(self) -> None:
        self._stopping.set()
        self._close_flight_record(wait_s=2.0)  # a survey still running at shutdown leaves its record and tracks all the same; a stalled disk does not hold the shutdown
        if self._list_writer is not None:
            self._list_writer.stop(wait_s=2.0)  # what is waiting is written; a stalled disk does not hold the shutdown
        if self._frame_log is not None:
            self._frame_log.stop(wait_s=2.0)  # the frames still queued get two seconds; a stalled disk does not hold the shutdown
        if self._reader is not None:
            self._reader.join(timeout=2.0)
        if self._source is not None:
            self._source.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LocNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
