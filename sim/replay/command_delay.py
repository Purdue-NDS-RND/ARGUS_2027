#!/usr/bin/env python3
"""Capture to command, and command to response, from a recorded bag (PLT-05's second test). No ROS imports.

  python3 sim/replay/command_delay.py <bag> [--out delay.json]

Two numbers from one flight's bag (a sim run's, or the aircraft's):

  capture to command   a frame's capture stamp -> the first setpoint the
                       mission sent after it heard the detection made from
                       that frame. The whole chain the companion owns. The
                       line is 150 ms at the 95th percentile.
  actuation delay      a step in the commanded velocity or position -> the
                       vehicle's attitude beginning to leave where it was
                       (the start of its tilt, carried back from where it
                       is first seen above the noise). What the estimator
                       forecasts over (FND-06 uses 0.08 s).

A detection counts only if a setpoint followed it within 0.2 s: otherwise
the mission was not flying on vision then. Detections are followed by their
own frame stamp (the detection contract, or the 2026 topic with
features.stamp_from_image on); a 2026 detection stamped with the detector's
clock is not, and the count says how many those were.

The attitude is the IMU topic's, at the moment the recorder heard it, so the
actuation delay includes the telemetry link, as it does for the mission.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from replay import topics  # noqa: E402
from replay.bag import open_bag  # noqa: E402
from replay.cdr import CdrError  # noqa: E402
from replay.latency import ClockMap, spread_ms, stamp_ns  # noqa: E402

MS, S = 1_000_000, 1_000_000_000
CAPTURE_TO_COMMAND_P95_S = 0.150  # PLT-05's target
COMMAND_FOLLOWS_WITHIN_NS = 200 * MS  # a setpoint this soon after a detection is heard: the mission is flying on vision
STEP = {"velocity": 0.3, "position": 0.5}  # m/s and m: a change of command this large, after a steady one, is a step
STEADY_NS = 500 * MS
BASELINE_NS = 300 * MS
RESPONSE_WITHIN_NS = 1 * S
FIT_NS = 150 * MS  # of tilt after it is first seen out: the line that is carried back to the level
MIN_TILT_RAD = math.radians(0.3)
NOISE_SIGMAS = 5.0
SETPOINT_TOPICS = (topics.SETPOINT_LOCAL, topics.SETPOINT_POSITION)
Command = Tuple[int, str, float, float]  # heard, "velocity" or "position", east, north


def _roll_pitch(q) -> Tuple[float, float]:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def read_chain(bag_path) -> dict:
    """Detections (frame stamp on the recorder's clock, heard), commands and tilt samples, each in time order."""
    contract = {topics.contract_topic(name) for name in topics.DETECTORS}
    frames = set()  # stamps the camera's own messages carry: what a frame-stamped detection must match
    detections: List[Tuple[int, int]] = []
    legacy: List[Tuple[int, int]] = []
    commands: List[Command] = []
    tilt: List[Tuple[int, float, float]] = []
    clocks = {topics.SIM_CLOCK_TAP: [], topics.CLOCK: []}
    info_topics = {topics.CAMERA_INFO, *(topics.driver_info_topic(camera) for camera in topics.DRIVER_CAMERAS)}
    wanted = contract | info_topics | set(SETPOINT_TOPICS) | set(clocks) | {topics.LEGACY_DETECTION, topics.IMU}
    for message in open_bag(bag_path):
        topic, heard = message.topic, message.log_time_ns
        if topic not in wanted:
            continue
        try:
            decoded = message.decode()
        except CdrError:
            continue
        if topic in contract:
            detections.append((stamp_ns(decoded.header), heard))
        elif topic in info_topics:
            frames.add(stamp_ns(decoded.header))
        elif topic == topics.LEGACY_DETECTION:
            if not (decoded.point.x == -1.0 and decoded.point.y == -1.0):
                legacy.append((stamp_ns(decoded.header), heard))
        elif topic == topics.SETPOINT_LOCAL:
            velocity_only = (int(decoded.type_mask) & 0b111) == 0b111 and (int(decoded.type_mask) & 0b111000) != 0b111000
            vector = decoded.velocity if velocity_only else decoded.position
            commands.append((heard, "velocity" if velocity_only else "position", float(vector.x), float(vector.y)))
        elif topic == topics.SETPOINT_POSITION:
            commands.append((heard, "position", float(decoded.pose.position.x), float(decoded.pose.position.y)))
        elif topic == topics.IMU:
            tilt.append((heard, *_roll_pitch(decoded.orientation)))
        else:
            clocks[topic].append((int(decoded.clock.sec) * S + int(decoded.clock.nanosec), heard))
    clock = ClockMap(clocks[topics.SIM_CLOCK_TAP] or clocks[topics.CLOCK])
    unstamped = 0
    for stamp, heard in legacy:
        if stamp in frames:
            detections.append((stamp, heard))
        else:
            unstamped += 1
    placed = [(clock.to_recorder(stamp), heard) for stamp, heard in detections]
    return {
        "detections": sorted((stamp, heard) for stamp, heard in placed if stamp is not None),
        "commands": sorted(commands),
        "tilt": sorted(tilt),
        "not_frame_stamped": unstamped,
    }


def capture_to_command(detections: List[Tuple[int, int]], command_times: List[int], left_out: Optional[List[int]] = None) -> List[int]:
    """For each detection the mission was commanding on: its first setpoint after it was heard, less the frame's stamp.

    A detection whose next setpoint came later than COMMAND_FOLLOWS_WITHIN_NS is not counted; its wait goes in `left_out`.
    """
    delays: List[int] = []
    index = 0
    for stamp, heard in detections:
        while index < len(command_times) and command_times[index] < heard:
            index += 1
        if index == len(command_times):
            break
        if command_times[index] - heard > COMMAND_FOLLOWS_WITHIN_NS:
            if left_out is not None:
                left_out.append(command_times[index] - heard)
        elif command_times[index] >= stamp:
            delays.append(command_times[index] - stamp)
    return delays


def command_steps(commands: List[Command]) -> List[int]:
    """When the command jumped: a change of STEP or more from a command that had been steady for STEADY_NS."""
    steps: List[int] = []
    for kind, size in STEP.items():
        steady: List[Command] = []  # the commands since the last change, all within a third of a step of the first
        for command in (c for c in commands if c[1] == kind):
            heard, _, east, north = command
            if steady:
                moved = math.hypot(east - steady[0][2], north - steady[0][3])
                if moved >= size and heard - steady[0][0] >= STEADY_NS:
                    steps.append(heard)
                if moved >= size / 3.0:
                    steady = []
            steady.append(command)
    return sorted(steps)


def response_delays(steps: List[int], tilt: List[Tuple[int, float, float]]) -> List[int]:
    """For each step: when the tilt began to leave its level before the step.

    The tilt is first seen out when it is past its own noise (and 0.3 degrees)
    for two samples running. By then it has been moving for a while, the longer
    the noisier the hover, so the start is read off the way a dead time is read
    off a step response: a line through the next FIT_NS of tilt, taken along
    the direction it went, carried back to the level it left.
    """
    delays: List[int] = []
    for step in steps:
        before = [(roll, pitch) for heard, roll, pitch in tilt if step - BASELINE_NS <= heard < step]
        after = [(heard, roll, pitch) for heard, roll, pitch in tilt if step <= heard <= step + RESPONSE_WITHIN_NS]
        if len(before) < 5 or len(after) < 2:
            continue
        roll0 = sum(roll for roll, _ in before) / len(before)
        pitch0 = sum(pitch for _, pitch in before) / len(before)
        noise = math.sqrt(sum((roll - roll0) ** 2 + (pitch - pitch0) ** 2 for roll, pitch in before) / len(before))
        limit = max(MIN_TILT_RAD, NOISE_SIGMAS * noise)
        out = [math.hypot(roll - roll0, pitch - pitch0) > limit for _, roll, pitch in after]
        seen = next((index for index in range(len(after) - 1) if out[index] and out[index + 1]), None)
        if seen is None:
            continue
        window = [(heard, roll - roll0, pitch - pitch0) for heard, roll, pitch in after if after[seen][0] <= heard <= after[seen][0] + FIT_NS]
        delays.append(_onset(window, step, after[seen][0]) - step)
    return delays


def _onset(window: List[Tuple[int, float, float]], step: int, seen: int) -> int:
    """Where the line through `window` (time, roll and pitch off their level) meets the level: between the step and the first sighting."""
    if len(window) < 3:
        return seen
    mean_roll = sum(roll for _, roll, _ in window) / len(window)
    mean_pitch = sum(pitch for _, _, pitch in window) / len(window)
    size = math.hypot(mean_roll, mean_pitch)
    along = [(heard - seen, (roll * mean_roll + pitch * mean_pitch) / size) for heard, roll, pitch in window]  # signed, so noise does not bias it
    mean_t = sum(t for t, _ in along) / len(along)
    mean_y = sum(y for _, y in along) / len(along)
    spread = sum((t - mean_t) ** 2 for t, _ in along)
    slope = sum((t - mean_t) * (y - mean_y) for t, y in along) / spread if spread else 0.0
    if slope <= 0.0:
        return seen
    return max(step, min(seen, seen + round(mean_t - mean_y / slope)))


def analyse(chain: dict) -> dict:
    left_out: List[int] = []
    chain_delays = capture_to_command(chain["detections"], [command[0] for command in chain["commands"]], left_out)
    steps = command_steps(chain["commands"])
    actuation = response_delays(steps, chain["tilt"])
    spread = spread_ms(chain_delays)
    return {
        "capture_to_command": spread,
        "capture_to_command_passed": None if spread is None else spread["p95_ms"] <= CAPTURE_TO_COMMAND_P95_S * 1000.0,
        "actuation": spread_ms(actuation),
        "command_steps": len(steps),
        "detections": len(chain["detections"]),
        "no_setpoint_within_200_ms": len(left_out),
        "not_frame_stamped": chain["not_frame_stamped"],
    }


def lines(result: dict) -> List[str]:
    out: List[str] = []
    chain = result["capture_to_command"]
    if chain is None:
        out.append(f"capture to command: not measured ({result['detections']} frame-stamped detections, none followed by a setpoint)")
    else:
        verdict = "PASS" if result["capture_to_command_passed"] else "FAIL"
        out.append(
            f"capture to command: median {chain['median_ms']:.0f} ms, p95 {chain['p95_ms']:.0f} ms, max {chain['max_ms']:.0f} ms over {chain['count']} detections "
            f"(line: p95 within {CAPTURE_TO_COMMAND_P95_S * 1000.0:.0f} ms): {verdict}"
        )
    if chain is not None and result["no_setpoint_within_200_ms"]:
        out.append(
            f"  {result['no_setpoint_within_200_ms']} detections were not counted: no setpoint within {COMMAND_FOLLOWS_WITHIN_NS // MS} ms of their being heard "
            "(the mission was not sending setpoints then, or the executor stalled: look at where in the flight they are)"
        )
    if result["not_frame_stamped"]:
        out.append(f"  {result['not_frame_stamped']} detections carried the detector's clock, not their frame's stamp, and were not followed")
    actuation = result["actuation"]
    if actuation is None:
        out.append(f"actuation delay: not measured ({result['command_steps']} command steps with a tilt that answered)")
    else:
        out.append(f"actuation delay: median {actuation['median_ms']:.0f} ms, max {actuation['max_ms']:.0f} ms over {actuation['count']} of {result['command_steps']} command steps (FND-06 forecasts over 80 ms)")
    return out


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Capture to command and command to response from a recorded bag.")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, help="write the numbers as JSON")
    args = parser.parse_args(argv)
    result = analyse(read_chain(args.bag))
    print("\n".join(lines(result)))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2))
    # A bag may hold one of the two (the step probe's has no detections): pass unless the 150 ms line failed or nothing was measured.
    measured = result["capture_to_command"] is not None or result["actuation"] is not None
    return 0 if measured and result["capture_to_command_passed"] is not False else 1


if __name__ == "__main__":
    sys.exit(main())
