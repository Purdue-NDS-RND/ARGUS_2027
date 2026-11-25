#!/usr/bin/env python3
"""The Gaussian-splat twin's replay (SIM-12): rendered frames of the real field, synthetic targets put into them, the detector and the estimator run over the result. Offline only.

  python3 sim/splat_twin.py standin <folder> [--frames 60]         # a render set of a made-up field, to try the rest on
  python3 sim/splat_twin.py composite <render set> --targets targets.yaml --out <folder>
  python3 sim/splat_twin.py replay <composited folder>             # the bullseye detector and the target estimator, against the truth

What a splat gives the team is pictures of its own field from any place: the
real grass, runway paint and shadows, along a trajectory a flight really
flew. What is here is everything after the pictures exist:

  render set   a folder as a splat renderer must write it (the contract):
                 camera.yaml   width, height, fx, fy, cx, cy in pixels; the
                               camera looks straight down, the image's top
                               towards the vehicle's nose
                 poses.csv     index,t_s,east_m,north_m,up_m,yaw_deg: the
                               camera in the local frame (east, north, up;
                               yaw from east, anticlockwise, as MAVROS has it)
                 frames/<index as six digits>.png
  composite    paints the targets of a targets file (a bullseye or a numbered
               marker, each at a place on the ground) into every frame with
               the frame's own pose, antialiased, and writes truth.json: where
               each target's centre is in each frame;
  replay       runs the 2026 bullseye detector on every composited frame,
               puts each detection on the ground with the frame's pose, feeds
               the target estimator, and reports the detection rate, the
               pixel and ground errors and the estimator's final error.

Not here: capturing the field and training the splat. Those need the field
and a training run (the training-data hold), and are a person's. `standin`
renders a made-up field (a noise texture with stripes) through the same
contract so that the rest can be proven; it is a stand-in and says so in
its camera.yaml.

The ground is taken as flat at up = 0 of the local frame. No ROS imports;
numpy, cv2 and yaml.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

SIM = Path(__file__).resolve().parent
REPO = SIM.parent
for path in (REPO / "tools", REPO / "src" / "drone_target_cv", REPO / "src" / "drone_mission_core"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import synth_targets  # noqa: E402

FRAME_NAME = "{index:06d}.png"
PIXEL_SIGMA_PX = 3.0  # the detector's centre, for the estimator's measurement noise


class TwinError(ValueError):
    """A render set or a targets file that cannot be used. The message says why."""


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Pose:
    index: int
    t_s: float
    east_m: float
    north_m: float
    up_m: float
    yaw_deg: float


def ground_to_image(camera: Camera, pose: Pose) -> np.ndarray:
    """The homography from the ground plane (east, north, 1) to the image (u, v, 1) for a camera looking straight down.

    The image's top is towards the nose: a point ahead of the vehicle is above the image's centre, a point to its right is right of it.
    """
    if pose.up_m <= 0:
        raise TwinError(f"frame {pose.index}: the camera is not above the ground (up {pose.up_m:g} m)")
    yaw = math.radians(pose.yaw_deg)
    forward, right = (math.cos(yaw), math.sin(yaw)), (math.sin(yaw), -math.cos(yaw))
    kx, ky = camera.fx / pose.up_m, camera.fy / pose.up_m
    return np.array([
        [kx * right[0], kx * right[1], camera.cx - kx * (right[0] * pose.east_m + right[1] * pose.north_m)],
        [-ky * forward[0], -ky * forward[1], camera.cy + ky * (forward[0] * pose.east_m + forward[1] * pose.north_m)],
        [0.0, 0.0, 1.0],
    ])  # fmt: skip


def image_to_ground(camera: Camera, pose: Pose, u: float, v: float) -> Tuple[float, float]:
    east, north, w = np.linalg.inv(ground_to_image(camera, pose)) @ np.array([u, v, 1.0])
    return float(east / w), float(north / w)


def read_set(folder: Path) -> Tuple[Camera, List[Pose], Dict[str, Any]]:
    """A render set's camera and poses, checked against its frames."""
    folder = Path(folder)
    try:
        raw = yaml.safe_load((folder / "camera.yaml").read_text(encoding="utf-8"))
        camera = Camera(int(raw["width"]), int(raw["height"]), float(raw["fx"]), float(raw["fy"]), float(raw["cx"]), float(raw["cy"]))
        with open(folder / "poses.csv", newline="", encoding="utf-8") as handle:
            poses = [Pose(int(row["index"]), float(row["t_s"]), float(row["east_m"]), float(row["north_m"]), float(row["up_m"]), float(row["yaw_deg"])) for row in csv.DictReader(handle)]
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as problem:
        raise TwinError(f"{folder} is not a render set (camera.yaml, poses.csv, frames/): {problem}") from problem
    if not poses:
        raise TwinError(f"{folder}: poses.csv has no poses")
    missing = [pose.index for pose in poses if not (folder / "frames" / FRAME_NAME.format(index=pose.index)).is_file()]
    if missing:
        raise TwinError(f"{folder}: {len(missing)} pose(s) have no frame (the first is {FRAME_NAME.format(index=missing[0])})")
    if any(later.t_s <= earlier.t_s for earlier, later in zip(poses, poses[1:])):
        raise TwinError(f"{folder}: the poses' times do not go forward")
    return camera, poses, {key: value for key, value in raw.items() if key not in ("width", "height", "fx", "fy", "cx", "cy")}


def read_targets(path: Path) -> List[Dict[str, Any]]:
    """targets: [{name, kind: bullseye | marker, east_m, north_m, size_m, digit (a marker's)}]"""
    try:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        items = raw["targets"]
        targets = []
        for item in items:
            kind = item["kind"]
            if kind not in ("bullseye", "marker"):
                raise ValueError(f"kind '{kind}' is not bullseye or marker")
            size = float(item["size_m"])
            if not size > 0:
                raise ValueError("size_m must be above zero")
            targets.append({"name": str(item["name"]), "kind": kind, "east_m": float(item["east_m"]), "north_m": float(item["north_m"]), "size_m": size, "digit": int(item["digit"]) if kind == "marker" else None})
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as problem:
        raise TwinError(f"{path}: not a targets file: {problem}") from problem
    if len({target["name"] for target in targets}) != len(targets) or not targets:
        raise TwinError(f"{path}: the targets need names, each once")
    return targets


def _shape(target: Dict[str, Any]) -> Any:
    if target["kind"] == "bullseye":
        return synth_targets.Bullseye(size_m=target["size_m"])
    return synth_targets.NumberedMarker(digit=target["digit"], size_m=target["size_m"])


def composite(folder: Path, targets: Sequence[Dict[str, Any]], out: Path, supersample: int = 3) -> Dict[str, Any]:
    """Paint the targets into every frame of a render set; write the frames, the set's files and truth.json to `out`."""
    camera, poses, more = read_set(folder)
    out = Path(out)
    (out / "frames").mkdir(parents=True, exist_ok=True)
    shapes = [(target, _shape(target)) for target in targets]
    truth = []
    for pose in poses:
        name = FRAME_NAME.format(index=pose.index)
        image = cv2.imread(str(Path(folder) / "frames" / name), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (camera.height, camera.width):
            raise TwinError(f"{folder}/frames/{name}: not a {camera.width} x {camera.height} picture")
        canvas = image.astype(np.float32)
        h = ground_to_image(camera, pose)
        seen = {}
        for target, shape in shapes:
            place = np.array([[1.0, 0.0, target["east_m"]], [0.0, 1.0, target["north_m"]], [0.0, 0.0, 1.0]])  # the target's own metres to the field's
            synth_targets._composite_target(canvas, shape, h @ place, supersample)
            u, v, w = h @ np.array([target["east_m"], target["north_m"], 1.0])
            half_px = 0.5 * target["size_m"] * camera.fx / pose.up_m
            whole = half_px <= u / w <= camera.width - half_px and half_px <= v / w <= camera.height - half_px
            seen[target["name"]] = {"center_px": [float(u / w), float(v / w)], "size_px": 2.0 * half_px, "whole_in_view": bool(whole)}
        cv2.imwrite(str(out / "frames" / name), np.clip(np.rint(canvas), 0, 255).astype(np.uint8))
        truth.append({"index": pose.index, "targets": seen})
    (out / "camera.yaml").write_text(yaml.safe_dump({"width": camera.width, "height": camera.height, "fx": camera.fx, "fy": camera.fy, "cx": camera.cx, "cy": camera.cy, **more}), encoding="utf-8")
    (out / "poses.csv").write_text((Path(folder) / "poses.csv").read_text(encoding="utf-8"), encoding="utf-8")
    record = {"targets": list(targets), "frames": truth, "from": str(folder)}
    (out / "truth.json").write_text(json.dumps(record) + "\n", encoding="utf-8")
    return record


def replay(folder: Path, target_name: Optional[str] = None) -> Dict[str, Any]:
    """The bullseye detector and the target estimator over a composited set, against its truth."""
    from drone_mission_core import target_estimator
    from drone_target_cv.target_detector import RedTargetDetector

    camera, poses, more = read_set(folder)
    try:
        truth = json.loads((Path(folder) / "truth.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as problem:
        raise TwinError(f"{folder} has no truth.json: composite the targets into the render set first ({problem})") from problem
    bullseyes = [target for target in truth["targets"] if target["kind"] == "bullseye" and target_name in (None, target["name"])]
    if len(bullseyes) != 1:
        raise TwinError(f"{folder}: the replay follows one bullseye, and the set has {len(bullseyes)}" + ("" if target_name else " (name one with --target)"))
    target = bullseyes[0]
    by_index = {item["index"]: item["targets"][target["name"]] for item in truth["frames"]}
    detector, estimator = RedTargetDetector(), target_estimator.TargetEstimator()
    in_view = found = 0
    false_hits = 0
    pixel_errors: List[float] = []
    ground_errors: List[float] = []
    stamp_ns = 0
    for pose in poses:
        image = cv2.imread(str(Path(folder) / "frames" / FRAME_NAME.format(index=pose.index)), cv2.IMREAD_COLOR)
        seen = by_index[pose.index]
        result = detector.detect(image)
        stamp_ns = int(round(pose.t_s * 1e9))
        measurements = []
        if seen["whole_in_view"]:
            in_view += 1
        if result.center is not None:
            u, v = float(result.center[0]), float(result.center[1])
            pixel = math.hypot(u - seen["center_px"][0], v - seen["center_px"][1])
            if pixel <= max(0.5 * seen["size_px"], 5.0):  # on the target: anything else is a hit on something that is not it
                east, north = image_to_ground(camera, pose, u, v)
                if seen["whole_in_view"]:
                    found += 1
                    pixel_errors.append(pixel)
                    ground_errors.append(math.hypot(east - target["east_m"], north - target["north_m"]))
                sigma = PIXEL_SIGMA_PX * pose.up_m / camera.fx
                measurements.append(target_estimator.Measurement("bullseye", east, north, [[sigma * sigma, 0.0], [0.0, sigma * sigma]], score=float(result.confidence)))
            else:
                false_hits += 1
        estimator.update(measurements, stamp_ns)
    estimate = estimator.primary("bullseye", stamp_ns)

    def p(values: Sequence[float], q: float) -> Optional[float]:
        return None if not values else float(np.percentile(np.asarray(values), q))

    return {"target": target["name"], "frames": len(poses), "in_view": in_view, "detected": found, "detection_rate": None if in_view == 0 else found / in_view, "false_hits": false_hits,
            "pixel_error_median": p(pixel_errors, 50), "pixel_error_p95": p(pixel_errors, 95), "ground_error_median_m": p(ground_errors, 50), "ground_error_p95_m": p(ground_errors, 95),
            "estimator_error_m": None if estimate is None else math.hypot(estimate.east_m - target["east_m"], estimate.north_m - target["north_m"]),
            "estimator_hits": None if estimate is None else estimate.hits, "stand_in": bool(more.get("stand_in", False))}  # fmt: skip


def camera_of_mode(mode: str, airframe: str) -> Camera:
    """The render contract's camera for one of the repo's calibrated sensor modes (argus_camera's modes and airframes): its size, focal lengths and centre.

    A renderer is to render at these. The mode's lens distortion is not part of the contract: a render is an
    undistorted pinhole picture, and a mode with distortion terms is refused here rather than rendered as if it had none.
    """
    for path in (Path(__file__).resolve().parents[1] / "src" / "drone_mission_core", Path(__file__).resolve().parents[1] / "src" / "argus_camera"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from drone_mission_core.frame_projection import load_camera

    try:
        model, _mount = load_camera(mode, airframe)
    except Exception as problem:  # noqa: BLE001  (whatever the loader refuses is said as the twin's own refusal)
        raise TwinError(f"the camera mode {mode} on {airframe} cannot be loaded: {problem}") from problem
    distortion = [float(value) for value in model.d]  # projection_v2.CameraModel's distortion terms
    if any(abs(value) > 1.0e-9 for value in distortion):
        raise TwinError(f"the camera mode {mode} has lens distortion ({distortion}): the render contract is an undistorted pinhole picture, and distortion is not modelled")
    return Camera(int(model.width), int(model.height), float(model.fx), float(model.fy), float(model.cx), float(model.cy))


def standin(folder: Path, frames: int = 60, seed: int = 1, width: int = 640, height: int = 360, camera: Optional[Camera] = None) -> Path:
    """A render set of a made-up field, flown along a straight line with a slow turn: for proving the rest. Not a splat, and marked so."""
    rng = np.random.default_rng(seed)
    folder = Path(folder)
    (folder / "frames").mkdir(parents=True, exist_ok=True)
    if camera is None:
        camera = Camera(width, height, 0.6 * width, 0.6 * width, width / 2.0, height / 2.0)
    width, height = camera.width, camera.height
    # The field: 0.05 m a pixel over 80 m x 80 m around the origin: grass-coloured noise at two scales, a pale runway stripe.
    size, mpp = 1600, 0.05
    coarse = cv2.resize(rng.normal(0.0, 1.0, (40, 40)).astype(np.float32), (size, size), interpolation=cv2.INTER_CUBIC)
    fine = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    field = np.stack([70 + 14 * coarse + 6 * fine, 105 + 18 * coarse + 8 * fine, 60 + 12 * coarse + 6 * fine], axis=-1)
    field[760:840, :, :] = 150 + 10 * fine[760:840, :, None]
    field = np.clip(field, 0, 255).astype(np.uint8)
    field_to_ground = np.array([[mpp, 0.0, -size * mpp / 2.0], [0.0, -mpp, size * mpp / 2.0], [0.0, 0.0, 1.0]])  # the field's pixel (column, row) to (east, north)
    rows = ["index,t_s,east_m,north_m,up_m,yaw_deg"]
    for index in range(frames):
        along = index / max(frames - 1, 1)
        pose = Pose(index, index / 10.0, -6.0 + 12.0 * along, 2.0 * math.sin(2.0 * math.pi * along), 12.0, 20.0 + 30.0 * along)
        h = ground_to_image(camera, pose) @ field_to_ground
        cv2.imwrite(str(folder / "frames" / FRAME_NAME.format(index=index)), cv2.warpPerspective(field, h, (width, height), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT))
        rows.append(f"{index},{pose.t_s:.3f},{pose.east_m:.4f},{pose.north_m:.4f},{pose.up_m:.4f},{pose.yaw_deg:.3f}")
    (folder / "poses.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")
    (folder / "camera.yaml").write_text(yaml.safe_dump({"width": width, "height": height, "fx": camera.fx, "fy": camera.fy, "cx": camera.cx, "cy": camera.cy, "stand_in": True,
                                                        "source": "sim/splat_twin.py standin: a made-up field, not a splat of the test field"}), encoding="utf-8")  # fmt: skip
    return folder


def report(found: Dict[str, Any]) -> str:
    def number(value: Optional[float], unit: str, digits: int = 2) -> str:
        return "none" if value is None else f"{value:.{digits}f} {unit}"

    rate = "no frame had the whole target in view" if found["detection_rate"] is None else f"{found['detected']} of {found['in_view']} frames with the whole target in view ({found['detection_rate']:.0%})"
    lines = [f"target {found['target']}: detected in {rate}; {found['false_hits']} hit(s) on something else",
             f"pixel error: median {number(found['pixel_error_median'], 'px')}, 95th percentile {number(found['pixel_error_p95'], 'px')}",
             f"ground error: median {number(found['ground_error_median_m'], 'm', 3)}, 95th percentile {number(found['ground_error_p95_m'], 'm', 3)}",
             f"estimator: {number(found['estimator_error_m'], 'm', 3)} from the truth after {found['estimator_hits'] or 0} hit(s)"]  # fmt: skip
    if found["stand_in"]:
        lines.append("STAND-IN: these frames are a made-up field, not a splat of the test field. The numbers prove the harness, not the detector on the real field.")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="The splat twin's replay: targets composited into rendered frames, the detector and estimator against the truth.")
    actions = parser.add_subparsers(dest="action", required=True)
    make = actions.add_parser("standin")
    make.add_argument("folder")
    make.add_argument("--frames", type=int, default=60)
    make.add_argument("--seed", type=int, default=1)
    make.add_argument("--camera-mode", help="render at one of the repo's calibrated sensor modes (argus_camera modes.yaml) instead of the made-up 640 x 360 camera; needs --airframe")
    make.add_argument("--airframe", help="the airframe of --camera-mode (argus_camera robot.yaml)")
    paint = actions.add_parser("composite")
    paint.add_argument("set")
    paint.add_argument("--targets", required=True)
    paint.add_argument("--out", required=True)
    run = actions.add_parser("replay")
    run.add_argument("folder")
    run.add_argument("--target")
    run.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.action == "standin":
            if bool(args.camera_mode) != bool(args.airframe):
                raise TwinError("--camera-mode and --airframe go together")
            camera = camera_of_mode(args.camera_mode, args.airframe) if args.camera_mode else None
            print(f"wrote the stand-in render set {standin(Path(args.folder), args.frames, args.seed, camera=camera)} ({args.frames} frames)")
        elif args.action == "composite":
            record = composite(Path(args.set), read_targets(Path(args.targets)), Path(args.out))
            print(f"wrote {len(record['frames'])} frames with {len(record['targets'])} target(s) to {args.out}")
        else:
            found = replay(Path(args.folder), args.target)
            print(json.dumps(found, indent=2) if args.json else report(found))
    except TwinError as problem:
        print(f"splat_twin: {problem}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
