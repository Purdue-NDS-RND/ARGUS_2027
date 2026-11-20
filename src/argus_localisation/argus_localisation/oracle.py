"""The sim's oracle detector (LOC-5): a stand-in for the trained detector while training data is on hold. No ROS.

Behind the node flag `features.sim_oracle_detector`, off by default, and for
the sim only: `check_sim_only` refuses unless the node runs on sim time and
its `sim_only` parameter is set, so it can never run on the aircraft. It
does not look at the picture. It takes the true poses of the vehicle and of
the scenario's objects (sim/truth_poses.py, on TRUTH_TOPIC), puts each
object through the camera model at the frame's capture time, and reports
what a detector would: a box on the detection contract (FND-02), with pixel
noise, misses and false alarms as it is told.

  - The vehicle's true pose is interpolated to the frame's time between the
    two true poses either side of it (a 30 Hz truth and a vehicle at 4 m/s
    are 13 cm apart). With no true pose within MAX_TRUTH_GAP_S nothing is
    reported, and `last_refusal` says so.
  - An object is reported when its centre is in the picture. Its box is its
    size at its distance below the lens. A true object's score is 0.95; a
    false alarm's is between 0.25 and 0.45.
  - Noise, misses and false alarms come from one seeded generator, so a run
    repeats.

Which objects, their classes and their sizes come from a file the scenario
build writes into the run folder (the node's `oracle_objects_file`).
"""

from __future__ import annotations

import bisect
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from drone_mission_core import detection_contract as contract
from drone_mission_core.projection_v2 import ProjectionError, pixel_of_ground_point

FLAG = "sim_oracle_detector"
TRUTH_TOPIC = "/sim/truth/poses"
MAX_TRUTH_GAP_S = 0.1  # a true pose further than this from a frame's time is not that frame's
TRUE_SCORE = 0.95
FALSE_SCORE = (0.25, 0.45)
FALSE_SIZE_PX = (20.0, 80.0)
S = 1_000_000_000

Pose = Tuple[float, float, float, float, float, float, float]  # x, y, z, then the quaternion x, y, z, w


class OracleRefused(RuntimeError):
    """The oracle will not run here. The message says why."""


def check_sim_only(use_sim_time: bool, sim_only: bool) -> None:
    """Raise unless this is the sim: the node on sim time and its sim-only parameter set."""
    if not use_sim_time or not sim_only:
        raise OracleRefused(
            "the oracle detector reports the sim's true object poses and runs only in the sim: it needs use_sim_time true and the node's "
            f"sim_only parameter true (use_sim_time is {bool(use_sim_time)}, sim_only is {bool(sim_only)})"
        )


@dataclass(frozen=True)
class OracleObject:
    name: str  # the Gazebo model's name, as the truth topic gives it
    class_id: str  # one of the contract's classes
    size_m: float  # the side of its box on the ground
    top_m: float = 0.0  # how far its top is above its pose


def objects_from(data: Mapping[str, Any]) -> List[OracleObject]:
    objects = []
    for item in data.get("objects") or []:
        if item.get("class_id") not in contract.CLASSES:
            raise ValueError(f"oracle object '{item.get('name')}': '{item.get('class_id')}' is not a class of the detection contract ({', '.join(contract.CLASSES)})")
        size = float(item["size_m"])
        if not size > 0.0:
            raise ValueError(f"oracle object '{item.get('name')}': size_m must be above 0")
        objects.append(OracleObject(str(item["name"]), str(item["class_id"]), size, float(item.get("top_m", 0.0))))
    return objects


def load_objects(path: Any) -> Tuple[str, List[OracleObject]]:
    """(the vehicle's model name, the objects) from the run's file."""
    try:
        data = json.loads(Path(path).read_text())
        return str(data["vehicle"]), objects_from(data)
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise ValueError(f"oracle_objects_file '{path}' cannot be read as the oracle's objects ({error})") from error


class TruthHistory:
    """The last few seconds of true poses, by sim time."""

    def __init__(self, vehicle: str, keep: int = 150) -> None:
        self.vehicle = vehicle
        self._keep = int(keep)
        self._stamps: List[int] = []
        self._poses: List[Dict[str, Pose]] = []

    def add(self, stamp_ns: int, poses: Mapping[str, Sequence[float]]) -> None:
        if self._stamps and stamp_ns <= self._stamps[-1]:
            return
        self._stamps.append(int(stamp_ns))
        self._poses.append({name: tuple(float(v) for v in pose) for name, pose in poses.items()})
        if len(self._stamps) > self._keep:
            del self._stamps[0], self._poses[0]

    def at(self, stamp_ns: int) -> Optional[Tuple[np.ndarray, Tuple[float, float, float, float], Dict[str, Pose]]]:
        """(the vehicle's position, its orientation, every pose) at a time, or None with nothing near enough."""
        index = bisect.bisect_left(self._stamps, int(stamp_ns))
        before = index - 1 if index > 0 else None
        after = index if index < len(self._stamps) else None
        near = [i for i in (before, after) if i is not None and abs(self._stamps[i] - stamp_ns) <= MAX_TRUTH_GAP_S * S]
        if not near:
            return None
        nearest = min(near, key=lambda i: abs(self._stamps[i] - stamp_ns))
        poses = self._poses[nearest]
        if self.vehicle not in poses:
            return None
        position = np.array(poses[self.vehicle][:3])
        if before is not None and after is not None and len(near) == 2 and self.vehicle in self._poses[before] and self.vehicle in self._poses[after]:
            share = (stamp_ns - self._stamps[before]) / float(self._stamps[after] - self._stamps[before])
            position = (1.0 - share) * np.array(self._poses[before][self.vehicle][:3]) + share * np.array(self._poses[after][self.vehicle][:3])
        return position, tuple(poses[self.vehicle][3:7]), poses


class OracleDetector:
    name = "oracle"

    def __init__(
        self,
        camera: Any,
        mount: Any,
        objects: Sequence[OracleObject],
        truth: TruthHistory,
        seed: int = 0,
        pixel_sigma_px: float = 2.0,
        miss_rate: float = 0.05,
        false_alarms_per_frame: float = 0.02,
    ) -> None:
        if not 0.0 <= miss_rate <= 1.0:
            raise ValueError(f"miss_rate is a share of sightings from 0 to 1, got {miss_rate!r}")
        if pixel_sigma_px < 0.0 or false_alarms_per_frame < 0.0:
            raise ValueError("pixel_sigma_px and false_alarms_per_frame cannot be negative")
        self._camera, self._mount, self._objects, self._truth = camera, mount, list(objects), truth
        self._rng = np.random.default_rng(int(seed))
        self._sigma, self._miss, self._false = float(pixel_sigma_px), float(miss_rate), float(false_alarms_per_frame)
        self._classes = sorted({item.class_id for item in self._objects}) or list(contract.CLASSES)
        self.last_refusal: Optional[str] = None

    def detect_full(self, stamp_ns: int) -> Tuple[contract.Detection, ...]:
        """What is in the frame captured at `stamp_ns`, in the full frame's pixels."""
        self.last_refusal = None
        state = self._truth.at(int(stamp_ns))
        if state is None:
            self.last_refusal = f"no true poses within {MAX_TRUTH_GAP_S:g} s of the frame's time"
            return ()
        position, orientation, poses = state
        found: List[contract.Detection] = []
        for item in self._objects:
            pose = poses.get(item.name)
            if pose is None:
                continue
            point = (pose[0], pose[1], pose[2] + item.top_m)
            below = float(position[2]) - point[2]
            if below <= 0.2:
                continue
            try:
                u, v = pixel_of_ground_point(self._camera, self._mount, point, position, orientation)
            except ProjectionError:
                continue
            if not (0.0 <= u < self._camera.width and 0.0 <= v < self._camera.height):
                continue
            # The draws are made for every object in the picture, seen or missed, so that one setting does not shift another's.
            missed = self._rng.random() < self._miss
            noise = self._rng.normal(0.0, 1.0, size=2) * self._sigma
            if missed:
                continue
            side = item.size_m * float(self._camera.fx) / below
            found.append(contract.Detection(item.class_id, TRUE_SCORE, float(u + noise[0]), float(v + noise[1]), float(side), float(side)))
        for _ in range(int(self._rng.poisson(self._false)) if self._false > 0.0 else 0):
            side = float(self._rng.uniform(*FALSE_SIZE_PX))
            found.append(
                contract.Detection(
                    str(self._classes[int(self._rng.integers(len(self._classes)))]),
                    float(self._rng.uniform(*FALSE_SCORE)),
                    float(self._rng.uniform(0.0, self._camera.width)),
                    float(self._rng.uniform(0.0, self._camera.height)),
                    side,
                    side,
                )
            )
        return tuple(found)

    def detect(self, image: Any, stamp_ns: int) -> Tuple[contract.Detection, ...]:
        """The same, in the pixels of the (shrunk) image the pipeline hands a detector."""
        scale = float(image.shape[1]) / float(self._camera.width)
        return tuple(
            contract.Detection(d.class_id, d.score, d.cx * scale, d.cy * scale, d.width * scale, d.height * scale) for d in self.detect_full(stamp_ns)
        )
