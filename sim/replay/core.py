#!/usr/bin/env python3
"""Replay a bag through the flight code without ROS (SIM-09, the `core` mode).

  python3 sim/replay/core.py <bag> [--camera-mode M --airframe A] [--truth east,north] [--out metrics.json]

The bag's poses, velocities, rangefinder readings and detections are given,
in the order and at the times the recorder received them, to the same
objects the mission executor runs: the pose history, the ground height,
projection v2 and the target estimator's feed. Nothing is published and no
clock is read: the time is the bag's, so the same bag gives the same answer
every time, on any machine. The estimator's tracks are read every 100 ms of
bag time, as the executor's own timer publishes them.

The marker tracker and the optical-flow code will be replayed the same way
when they exist (LOC-7, the flow lane); a handler is one more entry in
`Replay.handlers`.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[2]
for package in ("drone_mission_core", "argus_camera"):
    if str(REPO / "src" / package) not in sys.path:
        sys.path.insert(0, str(REPO / "src" / package))
if str(REPO / "sim") not in sys.path:
    sys.path.insert(0, str(REPO / "sim"))

from replay import topics  # noqa: E402
from replay.bag import open_bag  # noqa: E402
from replay.cdr import CdrError, stamp_ns  # noqa: E402

TICK_NS = 100_000_000  # the estimator's own timer in the executor: 10 Hz
DEFAULT_CAMERA_MODE, DEFAULT_AIRFRAME = "sim_delivery", "sim_iris_downcam"


class Replay:
    """The executor's estimator side, fed from a bag."""

    def __init__(self, camera: Any, mount: Any, detectors: Sequence[str] = topics.DETECTORS) -> None:
        from drone_mission_core import detection_contract
        from drone_mission_core.estimator_feed import LEGACY_DETECTOR, EstimatorFeed
        from drone_mission_core.pose_buffer import PoseBuffer
        from drone_mission_core.projection_v2 import GroundHeight, GroundProjector

        self._contract = detection_contract
        self._legacy_detector = LEGACY_DETECTOR
        self.camera = camera
        self.poses = PoseBuffer()
        self.ground = GroundHeight(self.poses.pose_at)
        self.feed = EstimatorFeed(GroundProjector(camera, mount, self.poses.pose_at, self.ground), self.poses)
        self.feed.on_report = lambda report, _stamp_ns: self.innovations.extend(report.nis)
        self.legacy_size: Optional[Tuple[int, int]] = None
        self.frames: List[dict] = []  # one per detection message taken: stamp, when it was heard, how many detections
        self.samples: List[dict] = []  # the tracks, every tick
        self.innovations: List[float] = []  # the normalised innovation squared of every measurement set against a track
        self.recorded: List[dict] = []  # what the flight's own estimator published, if the bag has it: for comparison
        self.clock_samples: List[Tuple[int, int]] = []  # (the sim's clock, when the recorder heard it)
        self._clock_map: Any = None
        self.undecodable: Dict[str, int] = {}
        self._next_tick_ns: Optional[int] = None
        self.handlers: Dict[str, Callable[[Any, int], None]] = {
            topics.POSE: self._on_pose,
            topics.VELOCITY: self._on_velocity,
            topics.RANGEFINDER: self._on_range,
            topics.LEGACY_IMAGE_SIZE: self._on_legacy_size,
            topics.ESTIMATOR_TARGETS: self._on_recorded_targets,
            topics.SIM_CLOCK_TAP: self._on_clock,
            topics.CLOCK: self._on_clock,
            **{topics.contract_topic(name): self._detections_from(name) for name in detectors},
        }

    # --- the bag's messages ------------------------------------------------------------

    def _on_pose(self, message: Any, _log_ns: int) -> None:
        position, orientation = message.pose.position, message.pose.orientation
        stamp = stamp_ns(message.header)
        self.poses.add_pose(stamp, (position.x, position.y, position.z), (orientation.x, orientation.y, orientation.z, orientation.w))
        self.feed.add_pose(stamp, (position.x, position.y, position.z))

    def _on_velocity(self, message: Any, _log_ns: int) -> None:
        linear = message.twist.linear
        self.poses.add_velocity(stamp_ns(message.header), (linear.x, linear.y, linear.z))

    def _on_range(self, message: Any, _log_ns: int) -> None:
        self.ground.add_range(stamp_ns(message.header), message.range, message.min_range, message.max_range)

    def _on_legacy_size(self, message: Any, _log_ns: int) -> None:
        self.legacy_size = (int(message.point.x), int(message.point.y))

    def _on_clock(self, message: Any, log_ns: int) -> None:
        self.clock_samples.append((int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec), log_ns))

    def to_recorder_time(self, stamp_ns: int) -> Optional[int]:
        """A stamp on the frames' clock as a time on the recorder's (the same number when they are one clock)."""
        if self._clock_map is None:
            from replay.latency import ClockMap

            self._clock_map = ClockMap(self.clock_samples)
        return self._clock_map.to_recorder(stamp_ns)

    def _on_recorded_targets(self, message: Any, log_ns: int) -> None:
        tracks = []
        for detection in message.detections:
            parts = str(detection.id).split("/")  # "<class>/<track id>/<STATE>", as estimator_feed.encode_targets writes it
            if len(parts) == 3:
                position = detection.bbox.center.position
                tracks.append({"class": parts[0], "id": int(parts[1]) if parts[1].isdigit() else -1, "state": parts[2], "east_m": position.x, "north_m": position.y, "hits": 0})
        self.recorded.append({"t_ns": log_ns, "tracks": tracks})

    def _detections_from(self, detector: str) -> Callable[[Any, int], None]:
        def take(message: Any, log_ns: int) -> None:
            try:
                found = self._contract.decode(message)
            except self._contract.ContractError:
                self.feed._count("dropped: not on the detection contract")
                return
            image_size = None
            if detector == self._legacy_detector:  # as the executor: the 2026 detector is used once it has said its image's size
                if self.legacy_size is None:
                    self.feed._count("dropped: the detector has not said what size its image is")
                    return
                image_size = self.legacy_size
            if not self.feed.add_detections(found, image_size):
                return
            self.frames.append({"detector": detector, "stamp_ns": found.stamp_ns, "heard_ns": log_ns, "detections": len(found.detections)})
            self._step(log_ns)

        return take

    # --- time --------------------------------------------------------------------------

    def _step(self, now_ns: int) -> None:
        self.feed.step(now_ns)

    def _tick(self, now_ns: int) -> None:
        self._step(now_ns)
        tracks = [
            {
                "id": estimate.track_id,
                "class": estimate.class_id,
                "state": estimate.state,
                "east_m": estimate.east_m,
                "north_m": estimate.north_m,
                "sigma_m": estimate.sigma_m,
                "age_s": estimate.age_s,
                "hits": estimate.hits,
            }
            for estimate in self.feed.estimates(now_ns)
        ]
        self.samples.append({"t_ns": now_ns, "tracks": tracks})

    def advance_to(self, now_ns: int) -> None:
        """Run the 10 Hz ticks that fall before `now_ns`."""
        if self._next_tick_ns is None:
            self._next_tick_ns = now_ns + TICK_NS
            return
        while self._next_tick_ns <= now_ns:
            self._tick(self._next_tick_ns)
            self._next_tick_ns += TICK_NS

    def run(self, bag_path) -> "Replay":
        last_ns = None
        for message in open_bag(bag_path):
            handler = self.handlers.get(message.topic)
            if handler is None:
                continue
            self.advance_to(message.log_time_ns)
            try:
                decoded = message.decode()
            except CdrError:
                self.undecodable[message.topic] = self.undecodable.get(message.topic, 0) + 1
                continue
            handler(decoded, message.log_time_ns)
            last_ns = message.log_time_ns
        if last_ns is not None and self._next_tick_ns is not None:
            self.advance_to(last_ns + TICK_NS)  # one more tick, after the last message
        return self


def replay_bag(bag_path, camera_mode: str = DEFAULT_CAMERA_MODE, airframe: str = DEFAULT_AIRFRAME, truth_en: Optional[Sequence[float]] = None, release_ns: Optional[int] = None) -> dict:
    """Replay a bag and give its metrics (metrics.py), as plain data."""
    from drone_mission_core.frame_projection import load_camera

    from replay import metrics

    camera, mount = load_camera(camera_mode, airframe)
    replay = Replay(camera, mount).run(bag_path)
    return metrics.summarise(replay, truth_en, release_ns)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a bag through the estimator without ROS and print its metrics.")
    parser.add_argument("bag", type=Path, help="a bag folder, a .mcap file or a .db3 file")
    parser.add_argument("--camera-mode", default=DEFAULT_CAMERA_MODE, help="the sensor mode the detections' pixels are in (argus_camera modes.yaml)")
    parser.add_argument("--airframe", default=DEFAULT_AIRFRAME, help="the airframe whose camera mount is used (argus_camera robot.yaml)")
    parser.add_argument("--truth", help="east,north of the bullseye in the local frame, in metres: turns on the error metrics")
    parser.add_argument("--truth-file", type=Path, help="a sim run's truth.json, in place of --truth")
    parser.add_argument("--release-ns", type=int, help="the bag time of the release, for the error at release")
    parser.add_argument("--out", type=Path, help="write the metrics here as JSON (always printed in short)")
    args = parser.parse_args(argv)
    truth = None
    if args.truth:
        truth = tuple(float(value) for value in args.truth.split(","))
    elif args.truth_file:
        from projection_check import target_in_local_frame

        truth = target_in_local_frame(json.loads(args.truth_file.read_text()))
    result = replay_bag(args.bag, args.camera_mode, args.airframe, truth, args.release_ns)
    text = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.out:
        args.out.write_text(text)
    from replay import metrics

    for line in metrics.short_lines(result):
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
