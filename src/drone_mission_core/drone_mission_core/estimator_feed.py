"""The target estimator inside the mission executor (FND-06): detections in, tracks out.

Behind the mission flag `use_target_estimator`, which defaults to false and
needs `use_frame_time_pose` (the pose history and projection v2). With the
flag off the executor makes none of this: no subscription, no timer, no
publisher.

`EstimatorFeed` has no ROS in it. It takes detection sets on the FND-02
contract, and for each frame

  1. waits until the pose history covers the frame's capture time (a frame
     can arrive before the pose of its own moment does);
  2. projects every detection to the ground with projection v2, onto the
     plane of its class, using the pose at the frame's stamp. A box that
     reaches the edge of the image is left out: it is the visible part of a
     target, and its centre is not the target's;
  3. turns the projection's Jacobians into measurement noise and gives the
     frame's measurements to the `TargetEstimator`.

It also watches the pose stream for a position reset of the flight
controller's estimator (a GPS glitch, a change of navigation source): a step
between two consecutive poses that the reported velocity does not explain.
ArduPilot reports no reset message that MAVROS passes on, so the step itself
is the report. Every track is moved by the same step.

`attach_target_estimator` is the thin ROS side: the subscriptions (one per
detector, on the contract's topics), a 10 Hz timer that publishes the tracks
as `vision_msgs/Detection3DArray` on /estimator/targets, and nothing else.
Missions read the feed in-process (FND-07 adds the context calls).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, fields
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from .detection_contract import ContractError, DetectionSet, decode, topic_for
from .features import get_feature_flag
from .frame_time import FRAME_TIME_FLAG, POSE_TOPIC

ESTIMATOR_FLAG = "use_target_estimator"
TARGETS_TOPIC = "/estimator/targets"
WORLD_FRAME = "map"  # MAVROS's local frame: east, north, up from the take-off point
DETECTORS_PARAMETER = "estimator_detectors"
LEGACY_DETECTOR = "target_cv"
LEGACY_IMAGE_SIZE_TOPIC = "/drone_package_drop/image_size"  # the 2026 detector's: (width, height) of the image it ran on
DEFAULT_DETECTORS = (LEGACY_DETECTOR,)  # the 2026 bullseye detector, on the contract with its features.detection_contract on
PUBLISH_RATE_HZ = 10.0

# Chosen here; the spec leaves them open (see PROGRESS.md, Decisions).
MAX_FRAME_AGE_S = 1.0  # a frame older than this is not worth placing: three hits must fall within a second anyway
MAX_STAMP_AHEAD_S = 0.05  # a stamp further ahead of the clock than this is on another clock
PENDING_LIMIT = 64  # frames waiting for their pose; two seconds' worth at 30 Hz
RESET_MIN_STEP_M = 0.5  # a smaller step is left to the filter's gate
RESET_MAX_PAIR_GAP_S = 0.12  # two poses further apart than this are not compared (as the pose history's gap)
RESET_MEMORY = 8  # recent resets remembered, for frames taken before one and handled after it
FORECAST_MAX_AHEAD_S = 0.2  # how far past the newest pose the vehicle's position is forecast
# The estimator shares the executor's one thread with the 20 Hz tick, so the work of one step is bounded.
# A detector that is not a box detector says how well it places a centre, in pixels (one standard deviation),
# and its detections are weighed by that instead of by their box's size. The ring fit's figure is its own
# PIXEL_SIGMA_PX (drone_target_cv/ring_fit.py; a test holds the two together).
# The green X and bucket detector (REC-1) places a centre by fitted lines and an ellipse, not by a box: 3 px, between
# its pass line on renders (2 px, or 1% of the X's span) and the 4.3 px median it showed from a vehicle at 3.5 m/s.
DETECTOR_PIXEL_SIGMA_PX: Dict[str, float] = {"ring_fit": 2.0, "recovery_cv": 3.0}
# The near-field ring fit (CEN-13) is listened to, beside the detectors of DETECTORS_PARAMETER, when a mission
# has this flag on.
RING_FIT_FLAG = "use_ring_fit"
RING_FIT_DETECTOR = "ring_fit"
MAX_DETECTIONS_PER_FRAME = 16  # more than this in one frame: the most confident of each class are kept, class by class in turn
MAX_FRAMES_PER_STEP = 4  # more than this ready at once: the rest wait for the next step
CLOCK_STEP_BACK_S = 0.5  # the clock this far behind where it was: it has stepped back


def _most_confident_of_each_class(detections: Sequence[Any], limit: int) -> Tuple[Any, ...]:
    """At most `limit` detections: the best of every class first, then the second best of every class, and so on.

    A crowd of one class (markers) can then never push the one detection of
    another (the bullseye) out of a frame.
    """
    by_class: Dict[str, List[Any]] = {}
    for detection in detections:
        by_class.setdefault(detection.class_id, []).append(detection)
    ranked = [sorted(group, key=lambda detection: detection.score, reverse=True) for _class_id, group in sorted(by_class.items())]
    kept: List[Any] = []
    depth = 0
    while len(kept) < limit and any(depth < len(group) for group in ranked):
        for group in ranked:
            if depth < len(group) and len(kept) < limit:
                kept.append(group[depth])
        depth += 1
    return tuple(kept)


class PositionResetDetector:
    """Finds a step in the reported position that the reported velocity does not explain."""

    def __init__(self, min_step_m: float = RESET_MIN_STEP_M, max_gap_s: float = RESET_MAX_PAIR_GAP_S) -> None:
        self._min_step_m = float(min_step_m)
        self._max_gap_ns = int(round(max_gap_s * 1.0e9))
        self._previous: Optional[Tuple[int, Tuple[float, float]]] = None

    @property
    def last_stamp_ns(self) -> Optional[int]:
        """The stamp of the pose last given, or None before the first."""
        return None if self._previous is None else self._previous[0]

    def add(self, stamp_ns: int, position: Sequence[float], velocity: Optional[Sequence[float]]) -> Optional[Tuple[float, float]]:
        """Give the next pose and the velocity reported around it. Returns the step (east, north) if this pose is a reset."""
        previous, self._previous = self._previous, (int(stamp_ns), (float(position[0]), float(position[1])))
        if previous is None or velocity is None:
            return None
        gap_ns = int(stamp_ns) - previous[0]
        if gap_ns <= 0 or gap_ns > self._max_gap_ns:
            return None
        gap_s = gap_ns * 1.0e-9
        step = (
            float(position[0]) - previous[1][0] - float(velocity[0]) * gap_s,
            float(position[1]) - previous[1][1] - float(velocity[1]) * gap_s,
        )
        if not all(math.isfinite(value) for value in step) or math.hypot(*step) < self._min_step_m:
            return None
        return step


class EstimatorFeed:
    """Detection sets in, tracks out. No ROS: `attach_target_estimator` feeds it."""

    def __init__(self, projector: Any, pose_buffer: Any, estimator: Any = None) -> None:
        from . import target_estimator  # needs numpy: imported only once the flag is on

        self._te = target_estimator
        self._projector = projector
        self._poses = pose_buffer
        self.estimator = estimator if estimator is not None else target_estimator.TargetEstimator()
        self._waiting: Deque[DetectionSet] = deque()
        self._resets: Deque[Tuple[int, int, Tuple[float, float]]] = deque(maxlen=RESET_MEMORY)  # (pose before, pose after, step)
        self._reset_detector = PositionResetDetector()
        self.counters: Dict[str, int] = {"frames": 0, "measurements": 0, "position resets": 0}
        self.last_fault: Optional[str] = None  # why the last frame that failed did
        self.on_report: Optional[Any] = None  # called with (FrameReport, stamp_ns) for every frame handled: the replay's hook (SIM-09)
        self._last_now_ns: Optional[int] = None
        self.totals = target_estimator.FrameReport()

    # --- in ----------------------------------------------------------------------------

    def _count(self, what: str) -> None:
        self.counters[what] = self.counters.get(what, 0) + 1

    @property
    def pending(self) -> int:
        """Frames waiting for the pose of their capture time."""
        return len(self._waiting)

    def add_detections(self, detection_set: DetectionSet, image_size: Optional[Sequence[int]] = None, pixel_sigma_px: Optional[float] = None) -> bool:
        """One frame's detections, as decoded from the contract. Handled at the next `step`. False if the frame was refused.

        `pixel_sigma_px` is the detector's own noise model, when it has one:
        how well it places a centre, in pixels. Without it a detection is
        weighed by the size of its box.

        `image_size` is the size of the image the detector says it ran on,
        when it says: the contract's pixels are those of the calibrated
        sensor mode, and a calibration is not scaled to another size, so
        detections from an image of another size are refused.
        """
        camera = self._projector.camera
        if image_size is not None and (int(image_size[0]), int(image_size[1])) != (camera.width, camera.height):
            self._count("dropped: the detector's image is not the calibrated mode's size")
            return False
        detections = detection_set.detections
        if not detections:
            return True  # a frame with nothing in it changes no track; silence is counted by the clock
        if len(detections) > MAX_DETECTIONS_PER_FRAME:
            self._count("trimmed: more detections in a frame than are handled")
            detection_set = DetectionSet(detection_set.stamp_ns, detection_set.frame_id, _most_confident_of_each_class(detections, MAX_DETECTIONS_PER_FRAME))
        if pixel_sigma_px is not None:
            detection_set = _WeighedSet(detection_set.stamp_ns, detection_set.frame_id, detection_set.detections, float(pixel_sigma_px))
        if len(self._waiting) >= PENDING_LIMIT:
            self._waiting.popleft()
            self._count("dropped: too many frames waiting")
        self._waiting.append(detection_set)
        return True

    def add_pose(self, stamp_ns: int, position: Sequence[float]) -> None:
        """Every pose from the flight controller, in order, to catch a position reset."""
        stamp_ns = int(stamp_ns)
        previous_ns = self._reset_detector.last_stamp_ns
        velocity = None
        if previous_ns is not None:
            velocity = self._poses.velocity_at(previous_ns) or self._poses.velocity_at(stamp_ns)
        step = self._reset_detector.add(stamp_ns, position, velocity)
        if step is None:
            return
        self.estimator.shift_all(*step)
        self._resets.append((previous_ns, stamp_ns, step))
        self._count("position resets")

    def _shift_for_resets(self, stamp_ns: int) -> Tuple[float, float]:
        """What the resets since a frame was taken add to a point projected with that frame's pose."""
        east = north = 0.0
        for before_ns, after_ns, step in self._resets:
            if stamp_ns >= after_ns:
                continue
            # Between the two poses the history is a straight line from one to
            # the other, so part of the step is already in the pose.
            part = 1.0 if stamp_ns <= before_ns else (after_ns - stamp_ns) / (after_ns - before_ns)
            east += part * step[0]
            north += part * step[1]
        return east, north

    # --- the work ----------------------------------------------------------------------

    def step(self, now_ns: int) -> None:
        """Handle waiting frames whose pose is known, oldest first and a few at a time, and move the tracks through their lives.

        A frame leaves the queue before it is handled, so a frame that makes
        the estimator fail is counted (`last_fault` says why) and gone: it is
        not tried again, and the frames handled before it are not replayed.
        """
        now_ns = int(now_ns)
        if self._last_now_ns is not None and self._last_now_ns - now_ns > int(CLOCK_STEP_BACK_S * 1.0e9):
            # Everything held is on the old clock.
            self._waiting.clear()
            self._resets.clear()
            self._reset_detector = PositionResetDetector()
            self.estimator.clock_stepped_back(now_ns)
            self._count("clock steps back")
        self._last_now_ns = now_ns
        ready = sorted(self._waiting, key=lambda found: found.stamp_ns)
        self._waiting = deque()
        handled = 0
        for detection_set in ready:
            stamp_ns = detection_set.stamp_ns
            if stamp_ns - now_ns > int(MAX_STAMP_AHEAD_S * 1.0e9):
                self._count("dropped: stamp ahead of the clock")
            elif now_ns - stamp_ns > int(MAX_FRAME_AGE_S * 1.0e9):
                self._count(f"dropped: frame older than {MAX_FRAME_AGE_S:g} s")
            elif self._poses.pose_at(stamp_ns) is not None:
                if handled >= MAX_FRAMES_PER_STEP:
                    self._waiting.append(detection_set)  # ready, but this step has done its share
                    continue
                handled += 1
                try:
                    self._handle(detection_set)
                except Exception as error:  # noqa: BLE001  (inside the executor: one bad frame must not stop the rest)
                    self._count("failed: handling a frame")
                    self.last_fault = f"{type(error).__name__}: {error}"
            elif (self._poses.newest_ns() or 0) < stamp_ns:
                self._waiting.append(detection_set)  # its pose has not arrived yet
            else:
                self._count("dropped: no pose for the frame's capture time")
        self.estimator.advance(now_ns)

    def _handle(self, detection_set: DetectionSet) -> None:
        te = self._te
        stamp_ns = detection_set.stamp_ns
        shift_east, shift_north = self._shift_for_resets(stamp_ns)
        measurements = []
        camera = self._projector.camera
        own_sigma = getattr(detection_set, "pixel_sigma_px", None)
        for detection in detection_set.detections:
            # A detector with its own noise model (the ring fit) found the centre some other way than as the
            # middle of its box: its box running over the picture's edge says nothing against the centre.
            if own_sigma is None and te.box_is_cut(detection.cx, detection.cy, detection.width, detection.height, camera.width, camera.height):
                self._count("skipped: box cut by the image's edge")  # the visible part of a target, not its centre
                continue
            point = self._projector.project_pixel_to_ground((detection.cx, detection.cy), stamp_ns, te.plane_height_m(detection.class_id))
            if point is None:
                self._count(f"refused: {self._projector.last_refusal or 'no reason given'}")
                continue
            measurement = te.measurement_from_point(
                detection.class_id, point, detection.width, detection.height, detection.score, pixel_sigma_px=own_sigma
            )
            if shift_east or shift_north:
                measurement = te.Measurement(
                    measurement.class_id, measurement.east_m + shift_east, measurement.north_m + shift_north, measurement.covariance, measurement.score, measurement.up_m
                )
            measurements.append(measurement)
        self.counters["frames"] += 1
        self.counters["measurements"] += len(measurements)
        if measurements:
            report = self.estimator.update(measurements, stamp_ns)
            for field in fields(report):
                if field.name != "nis":  # the totals count; the innovations go to whoever asked for the reports
                    setattr(self.totals, field.name, getattr(self.totals, field.name) + getattr(report, field.name))
            if self.on_report is not None:
                self.on_report(report, stamp_ns)

    # --- out ---------------------------------------------------------------------------

    def ready(self, now_ns: int) -> bool:
        """Whether a frame taken now could be placed: the pose history is live (a pose and a velocity within reach of now)."""
        return self._poses.predict_position(int(now_ns), FORECAST_MAX_AHEAD_S) is not None

    def actuation_time_ns(self, now_ns: int) -> int:
        """The time a command given now takes effect: now + 0.08 s. Outputs are given for this time."""
        return int(now_ns) + int(self._te.ACTUATION_DELAY_S * 1.0e9)

    def estimates(self, now_ns: int, class_id: Optional[str] = None) -> list:
        """Every track (of one class, if given), for the actuation time."""
        return self.estimator.estimates(int(now_ns), class_id)

    def primary(self, class_id: str, now_ns: int, near: Optional[Sequence[float]] = None):
        """The track of a class to act on (see TargetEstimator.primary), for the actuation time, or None."""
        return self.estimator.primary(class_id, int(now_ns), near)

    def vehicle_position_at(self, for_ns: int) -> Optional[Tuple[float, float, float]]:
        """Where the vehicle is expected to be at `for_ns` (the actuation time), from the pose history, or None if it cannot say."""
        return self._poses.predict_position(int(for_ns), FORECAST_MAX_AHEAD_S)


def encode_targets(estimates: Iterable[Any], stamp_ns: int, messages: Any, frame_id: str = WORLD_FRAME) -> Any:
    """A Detection3DArray of the tracks. `messages` gives Detection3DArray, Detection3D and ObjectHypothesisWithPose (vision_msgs.msg).

    The array is stamped with the time it was made. Each detection is stamped
    with its track's last hit, carries the class and the last hit's score, the
    position (east, north, up) with the 2x2 covariance in the pose's 6x6, a
    box of two standard deviations, and "<class>/<track id>/<STATE>" as its id.
    """

    def stamp(header: Any, ns: int) -> None:
        header.stamp.sec, header.stamp.nanosec = divmod(int(ns), 1_000_000_000)
        header.frame_id = frame_id

    array = messages.Detection3DArray()
    stamp(array.header, stamp_ns)
    for estimate in estimates:
        message = messages.Detection3D()
        stamp(message.header, estimate.last_hit_ns)
        message.id = f"{estimate.class_id}/{estimate.track_id}/{estimate.state}"
        hypothesis = messages.ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = estimate.class_id
        hypothesis.hypothesis.score = float(estimate.score)
        for position in (hypothesis.pose.pose.position, message.bbox.center.position):
            position.x, position.y, position.z = float(estimate.east_m), float(estimate.north_m), float(estimate.up_m)
        hypothesis.pose.pose.orientation.w = 1.0
        message.bbox.center.orientation.w = 1.0
        (ee, en), (ne, nn) = estimate.covariance
        covariance = hypothesis.pose.covariance  # 6x6 row by row: x, y, z, then rotations
        covariance[0], covariance[1], covariance[6], covariance[7] = float(ee), float(en), float(ne), float(nn)
        message.bbox.size.x = message.bbox.size.y = 2.0 * float(estimate.sigma_m)
        message.results.append(hypothesis)
        array.detections.append(message)
    return array


def estimator_camera(specs: Iterable[Any], config_dir: Any = None):
    """The one camera the sequence's missions name (`camera_mode` and `airframe`), as projection v2's model and mount.

    The estimator serves every mission of the sequence, so they must agree.
    Raises FrameProjectionError, and the executor does not start, when no
    mission names a camera, two missions name different ones, or the camera
    cannot be flown with (a stand-in calibration, an airframe not measured).
    """
    # Needs numpy (projection v2): imported only once the flag is on, so the
    # flags-off executor imports nothing it did not import in 2026.
    from .frame_projection import AIRFRAME_KEY, CAMERA_MODE_KEY, FrameProjectionError, load_camera

    named = set()
    for spec in specs:
        config = getattr(spec, "config", None) or {}
        mode, airframe = str(config.get(CAMERA_MODE_KEY, "") or "").strip(), str(config.get(AIRFRAME_KEY, "") or "").strip()
        if mode or airframe:
            if not (mode and airframe):
                raise FrameProjectionError(f"mission '{getattr(spec, 'name', '?')}': {CAMERA_MODE_KEY} and {AIRFRAME_KEY} go together, and one is missing")
            named.add((mode, airframe))
    if not named:
        raise FrameProjectionError(
            f"{ESTIMATOR_FLAG} is on but no mission's config names {CAMERA_MODE_KEY} and {AIRFRAME_KEY} "
            "(the sensor mode the detectors' images come from, and the airframe in argus_camera's robot.yaml)"
        )
    if len(named) > 1:
        raise FrameProjectionError(f"{ESTIMATOR_FLAG} is on and the missions name different cameras ({sorted(named)}): the one estimator needs one")
    ((mode, airframe),) = named
    return load_camera(mode, airframe, config_dir)


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class _WeighedSet(DetectionSet):
    """A frame's detections from a detector with its own noise model."""

    pixel_sigma_px: Optional[float] = None


def attach_target_estimator(node: Any, specs: Iterable[Any], pose_buffer: Any, ground_height: Any) -> Optional[EstimatorFeed]:
    """An EstimatorFeed running inside `node`, or None when the flag is off."""
    specs = list(specs)
    if not any(get_feature_flag(spec.features, ESTIMATOR_FLAG) for spec in specs):
        return None
    if pose_buffer is None:
        raise ValueError(f"{ESTIMATOR_FLAG} needs the pose history: turn {FRAME_TIME_FLAG} on with it")
    camera, mount = estimator_camera(specs)

    import vision_msgs.msg as vision_messages
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data

    from .projection_v2 import GroundProjector

    feed = EstimatorFeed(GroundProjector(camera, mount, pose_buffer.pose_at, ground_height), pose_buffer)
    logger = node.get_logger()
    # The estimator's own memory of the 2026 detector's image size: the
    # executor's copy is the missions', and they clear it between missions.
    legacy_size: Dict[str, Tuple[int, int]] = {}

    def now_ns() -> int:
        return int(node.get_clock().now().nanoseconds)

    def guarded(what: str, work) -> None:
        # The estimator runs inside the executor: whatever goes wrong in it
        # must not stop the missions' tick. It is counted and said, and the
        # tracks simply age (to COASTING and LOST) if it keeps happening.
        try:
            work()
        except Exception as error:  # noqa: BLE001
            feed._count(f"failed: {what}")
            logger.error(f"Estimator: {what} failed ({type(error).__name__}: {error})", throttle_duration_sec=5.0)

    def take_detections(detector: str, message: Any) -> None:
        try:
            found = decode(message)
        except ContractError as error:
            feed._count("dropped: not on the detection contract")
            logger.error(f"Estimator: a detection message is not on the contract ({error})", throttle_duration_sec=5.0)
            return
        # The 2026 detector says what size of image it ran on, once, when it
        # starts or the size changes. Its detections are used only while that
        # size is known and is the calibrated mode's; other detectors do not
        # say, and are taken at the contract's word.
        image_size = None
        if detector == LEGACY_DETECTOR:
            image_size = legacy_size.get("size")
            if image_size is None:
                feed._count("dropped: the detector has not said what size its image is")
                logger.error(
                    f"Estimator: {detector} has not said what size its image is (nothing heard on {LEGACY_IMAGE_SIZE_TOPIC}); its detections are not used",
                    throttle_duration_sec=5.0,
                )
                return
        if not feed.add_detections(found, image_size, DETECTOR_PIXEL_SIGMA_PX.get(detector)):
            logger.error(
                f"Estimator: {detector} ran on a {image_size[0]} x {image_size[1]} image but sensor mode {camera.name} is calibrated at "
                f"{camera.width} x {camera.height}; its detections are not used",
                throttle_duration_sec=5.0,
            )
            return
        step()

    def take_legacy_size(message: Any) -> None:
        legacy_size["size"] = (int(message.point.x), int(message.point.y))

    def step() -> None:
        faults = feed.counters.get("failed: handling a frame", 0)
        feed.step(now_ns())
        if feed.counters.get("failed: handling a frame", 0) != faults:
            logger.error(f"Estimator: a frame could not be handled and was dropped ({feed.last_fault})", throttle_duration_sec=5.0)

    def take_pose(message: Any) -> None:
        position = message.pose.position
        feed.add_pose(_stamp_ns(message.header.stamp), (position.x, position.y, position.z))

    def listener(detector: str):
        return lambda message: guarded("handling a detection message", lambda: take_detections(detector, message))

    def on_pose(message: Any) -> None:
        guarded("handling a pose", lambda: take_pose(message))

    node.declare_parameter(DETECTORS_PARAMETER, list(DEFAULT_DETECTORS))
    detectors = list(node.get_parameter(DETECTORS_PARAMETER).get_parameter_value().string_array_value)
    # use_ring_fit (CEN-13): the near-field centre finder as a second source of the bullseye. The flag is the
    # only way in: the detectors parameter cannot bring the ring fit past a flag that is off.
    if any(get_feature_flag(spec.features, RING_FIT_FLAG) for spec in specs):
        if RING_FIT_DETECTOR not in detectors:
            detectors.append(RING_FIT_DETECTOR)
    elif RING_FIT_DETECTOR in detectors:
        detectors = [detector for detector in detectors if detector != RING_FIT_DETECTOR]
        logger.error(f"Estimator: {DETECTORS_PARAMETER} names {RING_FIT_DETECTOR} but {RING_FIT_FLAG} is off: it is not listened to")
    for detector in detectors:
        node.create_subscription(vision_messages.Detection2DArray, topic_for(detector), listener(detector), 10)
    if LEGACY_DETECTOR in detectors:
        from geometry_msgs.msg import PointStamped

        node.create_subscription(
            PointStamped, LEGACY_IMAGE_SIZE_TOPIC, lambda message: guarded("handling an image size", lambda: take_legacy_size(message)), 10
        )
    # After the pose history's own subscription, so the history already holds the pose this callback is given.
    node.create_subscription(PoseStamped, POSE_TOPIC, on_pose, qos_profile_sensor_data)
    publisher = node.create_publisher(vision_messages.Detection3DArray, TARGETS_TOPIC, 10)

    def publish_tracks() -> None:
        step()
        now = now_ns()
        publisher.publish(encode_targets(feed.estimates(now), now, vision_messages))

    def on_timer() -> None:
        guarded("publishing the tracks", publish_tracks)

    node.create_timer(1.0 / PUBLISH_RATE_HZ, on_timer)
    logger.info(
        f"Target estimator on ({ESTIMATOR_FLAG}): camera {camera.name}, detectors {', '.join(detectors) or 'none'}, tracks on {TARGETS_TOPIC}"
    )
    return feed
