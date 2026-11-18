#!/usr/bin/env python3
"""Blow wind on the sim's vehicle and payloads (SIM-03). Runs inside the sim container.

  python3 sim/wind.py <run folder>

Started by sim/runner/fly.py beside every flight whose scenario has wind,
and interrupted when the flight ends. Twenty times a second of sim time it
takes each body's true velocity from Gazebo's poses, works out the wind
there and then (wind_model.py: a mean, gusts, one discrete gust) and the
drag force on the body, F = 1/2 rho CdA |v_rel| v_rel, and gives the force
to Gazebo's ApplyLinkWrench system. The world of a windy run is built with
that system in it (sim/scenario/build.py).

The run folder gets wind.jsonl (every update: time, wind, each body's
velocity and force, the vehicle's tilt) and, at the end, wind.json: what
the wind was over the flight and, from the stretches where the vehicle
hovered, its tilt against the tilt a steady wind of that force should
give, atan(F / mg). sim/runner/result.py reads wind.json.

The arithmetic is in wind_model.py and in the plain functions here; only
`blow` touches Gazebo.
"""

from __future__ import annotations

import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SIM_DIR = Path(__file__).resolve().parent
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

import wind_model  # noqa: E402
from wind_model import ForcePlan, Wind, WindSettings  # noqa: E402

VELOCITY_WINDOW_S = 0.1  # a body's velocity is its move over about this long: true poses, differenced
HOVER_MAX_SPEED_MPS = 0.5  # "hovering": slower than this over the ground,
HOVER_MIN_HEIGHT_M = 2.0  # and this far above where it stood
TILT_TOLERANCE = 0.20  # the pass line: the hover's tilt within 20% of atan(F / mg)
MIN_HOVER_SAMPLES = 100  # five seconds of hover at 20 Hz
BOX_DRAG_COEFFICIENT = 1.05  # a box, face on
CYLINDER_DRAG_COEFFICIENT = 1.0  # a bucket, side on
# The size the scenario builder gives a bucket or a payload that names none (sim/scenario/build.py, sdf_objects.py).
BUCKET_DIAMETER_M, BUCKET_HEIGHT_M = 0.19, 0.20
PAYLOAD_SIZE_M = (0.15, 0.15, 0.10)


def tilt_deg(orientation: Tuple[float, float, float, float]) -> float:
    """How far a body's up axis is from the world's, from its quaternion (x, y, z, w)."""
    x, y, _z, _w = orientation
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def body_cda(item: dict) -> float:
    """The drag area of a scenario object that the wind moves: a payload box or a bucket."""
    if item.get("kind") == "bucket":
        return CYLINDER_DRAG_COEFFICIENT * float(item.get("diameter_m", BUCKET_DIAMETER_M)) * float(item.get("height_m", BUCKET_HEIGHT_M))
    size = item.get("size_m") or PAYLOAD_SIZE_M
    if isinstance(size, (int, float)):
        size = (size, size, size)
    sides = sorted(float(value) for value in size)
    return BOX_DRAG_COEFFICIENT * sides[-1] * sides[-2]  # its largest face


def bodies(manifest: dict, truth: dict) -> Dict[str, dict]:
    """Every body the wind acts on: the vehicle (drag area and mass from its profile) and each payload and bucket."""
    vehicle = manifest["vehicle"]
    found = {vehicle["model"]: {"cda_m2": float(vehicle.get("cda_m2", wind_model.IRIS_CDA_M2)), "mass_kg": float(vehicle.get("mass_kg", 0.0)), "vehicle": True}}
    for item in truth.get("objects", []):
        if item.get("kind") in ("payload", "bucket"):
            found[item["name"]] = {"cda_m2": body_cda(item), "mass_kg": float(item.get("mass_kg", 0.0)), "vehicle": False}
    return found


class Velocity:
    """A body's velocity from its poses: the move over the last tenth of a second."""

    def __init__(self) -> None:
        self._history: List[Tuple[float, Tuple[float, float, float]]] = []

    def add(self, t_s: float, position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        self._history.append((t_s, position))
        while len(self._history) > 2 and t_s - self._history[1][0] >= VELOCITY_WINDOW_S:
            self._history.pop(0)
        t0, p0 = self._history[0]
        if t_s - t0 <= 1.0e-6:
            return (0.0, 0.0, 0.0)
        return tuple((now - before) / (t_s - t0) for now, before in zip(position, p0))


def summarise(samples: List[dict], settings: WindSettings, mass_kg: float) -> dict:
    """wind.json: what the wind was, and the hover's tilt against the tilt that force should give.

    Each sample is one update for the vehicle: `speed_mps` of the wind,
    `force_n` on the vehicle (horizontal), `ground_speed_mps`, `height_m`
    and `tilt_deg`. `passed` says the wind blew. `hover` is there when
    the vehicle hovered for five seconds or more, and `hover.within_tolerance`
    is SIM-03's pass line: the tilt within 20% of atan(F / mg).
    """
    result = {
        "settings": {"mean_mps": settings.mean_mps, "gust_mps": settings.gust_mps, "discrete_gust_mps": settings.discrete_gust_mps, "direction_deg": settings.direction_deg},
        "updates": len(samples),
        "wind_speed_mps": None,
        "hover": None,
        "passed": bool(samples),
        "problems": [] if samples else ["the wind never blew: no pose of the vehicle was heard"],
    }
    if not samples:
        return result
    speeds = sorted(sample["speed_mps"] for sample in samples)
    result["wind_speed_mps"] = {"mean": sum(speeds) / len(speeds), "max": speeds[-1], "min": speeds[0]}
    hover = [s for s in samples if s["ground_speed_mps"] <= HOVER_MAX_SPEED_MPS and s["height_m"] >= HOVER_MIN_HEIGHT_M]
    if len(hover) < MIN_HOVER_SAMPLES or mass_kg <= 0.0:
        return result
    mean_force = sum(s["force_n"] for s in hover) / len(hover)
    mean_tilt = sum(s["tilt_deg"] for s in hover) / len(hover)
    expected = math.degrees(math.atan2(mean_force, mass_kg * wind_model.GRAVITY_MPS2))
    result["hover"] = {
        "updates": len(hover),
        "mean_force_n": mean_force,
        "mean_tilt_deg": mean_tilt,
        "expected_tilt_deg": expected,
        "mass_kg": mass_kg,
        "within_tolerance": expected > 0.0 and abs(mean_tilt - expected) <= TILT_TOLERANCE * expected,
    }
    return result


def tilt_problem(report: Optional[dict]) -> str:
    """Why a flight's hover does not meet SIM-03's pass line, or "" if it does."""
    if report is None:
        return "the run left no wind.json"
    hover = report.get("hover")
    if not hover:
        return f"the vehicle did not hover in the wind for {MIN_HOVER_SAMPLES / wind_model.UPDATE_HZ:g} s, so there is no tilt to judge"
    if not hover["within_tolerance"]:
        return (
            f"the hover's tilt is {hover['mean_tilt_deg']:.2f} degrees where atan(F / mg) gives {hover['expected_tilt_deg']:.2f} "
            f"(more than {int(100 * TILT_TOLERANCE)}% apart)"
        )
    return ""


def read_wind(folder) -> Optional[dict]:
    """The summary a run folder holds, or None if there is none that can be read."""
    try:
        report = json.loads((Path(folder) / "wind.json").read_text())
    except (OSError, json.JSONDecodeError):
        return None
    return report if isinstance(report, dict) and isinstance(report.get("passed"), bool) else None


def wanted(manifest: dict) -> bool:
    """Whether a built scenario has wind, so that its flight is blown on."""
    return not WindSettings.from_manifest(manifest.get("wind")).calm


# --- Gazebo ---------------------------------------------------------------------------


def blow(run_dir: Path) -> dict:
    """Blow until interrupted; write wind.jsonl as it goes and wind.json at the end."""
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    manifest = json.loads((run_dir / "scenario.json").read_text())
    truth = json.loads((run_dir / "truth.json").read_text())
    world = manifest["site"]
    settings = WindSettings.from_manifest(manifest.get("wind"))
    wind = Wind(settings, seed=int(manifest.get("seed") or 1))
    targets = bodies(manifest, truth)
    vehicle_name = manifest["vehicle"]["model"]
    velocities = {name: Velocity() for name in targets}
    plans = {name: ForcePlan() for name in targets}
    samples: List[dict] = []
    lock = threading.Lock()
    state = {"t0": None, "last": None, "start_up": None}
    node = Node()
    add = node.advertise(f"/world/{world}/wrench/persistent", EntityWrench)
    clear = node.advertise(f"/world/{world}/wrench/clear", Entity)
    log = open(run_dir / "wind.jsonl", "w")

    def publish(name: str, kind: str, force) -> None:
        entity = Entity()
        entity.name, entity.type = name, Entity.MODEL  # the force goes to the model's canonical link, at its origin
        if kind == "clear":
            clear.publish(entity)
            return
        message = EntityWrench()
        message.entity.CopyFrom(entity)
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        add.publish(message)

    def on_poses(message) -> None:
        t_s = message.header.stamp.sec + message.header.stamp.nsec * 1.0e-9
        with lock:
            if state["t0"] is None:
                state["t0"] = t_s
            due = state["last"] is None or t_s - state["last"] >= 1.0 / wind_model.UPDATE_HZ - 1.0e-6
            poses = {pose.name: pose for pose in message.pose if pose.name in targets}
            moving = {name: velocities[name].add(t_s, (pose.position.x, pose.position.y, pose.position.z)) for name, pose in poses.items()}
            if not due:
                # Between updates only a body whose pile was just cleared is served: its whole force goes back
                # as soon as the clear has been taken, not a twentieth of a second later.
                for name in moving:
                    if plans[name].settling:
                        for kind, value in plans[name].update(plans[name].wanted, t_s):
                            publish(name, kind, value)
                return
            if vehicle_name not in poses:
                return
            state["last"] = t_s
            air = wind.at(t_s - state["t0"])
            line = {"t": round(t_s, 3), "wind": [round(value, 3) for value in air], "bodies": {}}
            for name, velocity in moving.items():
                force = wind_model.drag_force(air, velocity, targets[name]["cda_m2"])
                for kind, value in plans[name].update(force, t_s):
                    publish(name, kind, value)
                line["bodies"][name] = {"v": [round(value, 3) for value in velocity], "f": [round(value, 4) for value in force]}
            pose = poses[vehicle_name]
            if state["start_up"] is None:
                state["start_up"] = pose.position.z
            velocity = moving[vehicle_name]
            force = wind_model.drag_force(air, velocity, targets[vehicle_name]["cda_m2"])
            sample = {
                "speed_mps": math.hypot(air[0], air[1]),
                "force_n": math.hypot(force[0], force[1]),
                "ground_speed_mps": math.hypot(velocity[0], velocity[1]),
                "height_m": pose.position.z - state["start_up"],
                "tilt_deg": tilt_deg((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
            }
            samples.append(sample)
            line["tilt_deg"] = round(sample["tilt_deg"], 3)
            log.write(json.dumps(line) + "\n")

    if not node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_poses):
        raise RuntimeError(f"could not subscribe to the poses of Gazebo world '{world}'")
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    print(f"wind: {settings.mean_mps:g} m/s from {settings.direction_deg:g} degrees, gusts {settings.gust_mps:g} m/s, on {', '.join(targets)}", flush=True)
    while not stopping.is_set():
        time.sleep(0.2)
    with lock:
        for name in targets:  # leave no force behind
            publish(name, "clear", (0.0, 0.0, 0.0))
        log.close()
        report = summarise(samples, settings, targets[vehicle_name]["mass_kg"])
    (run_dir / "wind.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 2
    report = blow(Path(argv[0]))
    hover = report.get("hover")
    if hover:
        print(f"hover: tilt {hover['mean_tilt_deg']:.2f} degrees, atan(F / mg) {hover['expected_tilt_deg']:.2f} degrees, force {hover['mean_force_n']:.2f} N over {hover['updates']} updates")
    for problem in report["problems"]:
        print(f"  {problem}")
    print(f"wind: {report['updates']} updates" if report["passed"] else "wind: never blew", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
