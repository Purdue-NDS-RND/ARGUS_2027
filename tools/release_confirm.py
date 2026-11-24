#!/usr/bin/env python3
"""Release confirmation (CEN-14): after a release command, was it acknowledged, and did the payload leave?

Read after the flight from the flight controller's own log, which every
flight has (tools/post_flight.py calls this for an attempt's bundle and puts
the result in the report and in `release_confirmation.json`):

  acknowledged   the release command's result in the log (MAVC): accepted, or
                 the reason it was not;
  the output     the servo output that carries the gripper or the drop servo
                 (the SERVOn_FUNCTION the parameters give it) moved after the
                 command (RCOU). No output with that function means the
                 command can have moved nothing;
  it left        any one of:
      the step   released in the air: with no climb asked for, the vehicle
                 rises and the throttle falls, at once (the vehicle is
                 lighter);
      the flight the throttle steady flight takes after the release is lower
                 than before it (a delivery lets go on the ground: there is no
                 step, but the climb away is lighter);
      the fall   the rangefinder reads something shorter than the ground that
                 gets farther and is gone: the payload passing through its
                 beam, where the payload hangs in it.

CONFIRMED needs all three: acknowledged, the output moved, and it left.
Anything less is NOT CONFIRMED, with what is missing. Nothing here acts on the
flight; it is evidence for the report.

`Thresholds` are the sim's working values (a 0.5 kg payload on the sim's
vehicle steps the throttle by 11%). The aircraft's are to be measured with a
dummy payload: the report gives the measured numbers beside the verdict so
that they can be.

No ROS imports; the standard library only.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Any, Dict, List, Optional, Sequence, Tuple

GRIPPER, SPRAYER, SET_SERVO = 211, 216, 183  # MAV_CMD_DO_GRIPPER, DO_SPRAYER (the 2026 drop servo), DO_SET_SERVO
SERVO_FUNCTIONS = {GRIPPER: (28,), SPRAYER: (22, 23)}  # ArduPilot's SERVOn_FUNCTION: gripper; sprayer pump and spinner
MAV_RESULT = {0: "accepted", 1: "temporarily rejected", 2: "denied", 3: "unsupported", 4: "failed", 5: "in progress", 6: "cancelled"}
RANGEFINDER_GOOD = 4

Outputs = Sequence[Tuple[float, Tuple[int, ...]]]  # (s, the PWM of outputs 1 to 14)
Tuning = Sequence[Tuple[float, float, float, float]]  # (s, throttle 0 to 1, climb asked for m/s, climb m/s)
Ranges = Sequence[Tuple[float, float, int]]  # (s, metres, status)


@dataclass(frozen=True)
class Thresholds:
    output_within_s: float = 1.5  # the output must move within this long of the command
    output_min_step_us: int = 100
    same_loop_s: float = 0.05  # an output sample this close before the command's time stamp is the command's own loop
    airborne_throttle: float = 0.05  # under this the vehicle is on the ground or its motors are idling
    steady_error_mps: float = 0.15  # steady flight: the climb is the one asked for, to this
    steady_max_climb_mps: float = 1.0  # and slow: a fast climb takes more throttle than a hover
    before_s: float = 3.0  # the steady flight just before the command that "before" is taken from
    look_back_s: float = 20.0
    min_samples: int = 5
    min_throttle_drop: float = 0.04  # of the throttle before. To calibrate with a dummy payload
    step_window_s: float = 1.5  # the step is looked for this long after the output moved
    no_climb_asked_mps: float = 0.05
    min_rise_mps: float = 0.05
    after_within_s: float = 60.0
    fall_window_s: float = 2.5
    fall_margin_m: float = 0.3  # shorter than the ground by this much is something under the vehicle
    fall_min_samples: int = 2


def release_channel(command: Dict[str, Any], params: Dict[str, float]) -> Optional[int]:
    """The servo output (1 to 14) a release command moves, from the parameters; None when no output has that function."""
    if command["command"] == SET_SERVO:
        channel = int(command.get("p1", 0))
        return channel if 1 <= channel <= 14 else None
    wanted = SERVO_FUNCTIONS.get(command["command"], ())
    for channel in range(1, 15):
        if int(params.get(f"SERVO{channel}_FUNCTION", 0)) in wanted:
            return channel
    return None


def output_moved(command: Dict[str, Any], outputs: Outputs, params: Dict[str, float], limits: Thresholds = Thresholds()) -> Dict[str, Any]:
    channel = release_channel(command, params)
    if channel is None:
        name = "gripper" if command["command"] == GRIPPER else "drop servo (sprayer)"
        return {"moved": False, "channel": None, "why": f"no servo output has the {name}'s function in the flight controller's parameters: the command can have moved nothing"}
    # The flight controller acts on the command in the loop that logs it: an output sample with the command's own
    # time stamp can already hold the new value. "Before" is a sample clearly earlier than the command.
    edge = command["t"] - limits.same_loop_s
    before = [values[channel - 1] for t, values in outputs if t < edge and len(values) >= channel]
    after = [(t, values[channel - 1]) for t, values in outputs if edge <= t <= command["t"] + limits.output_within_s and len(values) >= channel]
    if not before or not after:
        return {"moved": None, "channel": channel, "why": f"the log has no servo outputs (RCOU) around the command for output {channel}"}
    for t, value in after:
        if abs(value - before[-1]) >= limits.output_min_step_us:
            last = after[-1][1]
            return {"moved": True, "channel": channel, "before_us": before[-1], "after_us": last, "t": max(t, command["t"]),
                    "why": f"output {channel} went from {before[-1]} to {last} us, {max(0.0, t - command['t']):.2f} s after the command"}  # fmt: skip
    return {"moved": False, "channel": channel, "before_us": before[-1], "after_us": after[-1][1], "why": f"output {channel} stayed at {before[-1]} us for {limits.output_within_s:g} s after the command"}


def _steady(row: Tuple[float, float, float, float], limits: Thresholds) -> bool:
    _t, throttle, asked, climb = row
    return throttle > limits.airborne_throttle and abs(climb - asked) <= limits.steady_error_mps and abs(climb) <= limits.steady_max_climb_mps


def throttle_before(t_command: float, tuning: Tuning, limits: Thresholds = Thresholds()) -> Optional[float]:
    """The throttle of the last steady flight before the command: the median of its last `before_s`. None when there is too little."""
    steady = [row for row in tuning if t_command - limits.look_back_s <= row[0] <= t_command and _steady(row, limits)]
    if len(steady) < limits.min_samples:
        return None
    last = [row[1] for row in steady if row[0] >= steady[-1][0] - limits.before_s]
    return median(last) if len(last) >= limits.min_samples else None


def step(t_command: float, t_output: float, tuning: Tuning, limits: Thresholds = Thresholds()) -> Dict[str, Any]:
    """Released in the air: with no climb asked for, the vehicle rises and the throttle falls."""
    at_command = [row for row in tuning if t_command - 1.0 <= row[0] <= t_command]
    if not at_command or at_command[-1][1] <= limits.airborne_throttle:
        return {"seen": None, "why": "the vehicle was not flying at the command (a release on the ground has no step)"}
    before = throttle_before(t_command, tuning, limits)
    if before is None:
        return {"seen": None, "why": "too little steady flight before the command to compare with"}
    rows = []
    for row in tuning:
        if row[0] <= t_output:
            continue
        if row[0] > t_output + limits.step_window_s or row[2] > limits.no_climb_asked_mps:
            break  # a climb is asked for from here: more throttle is the controller's, not the load's
        rows.append(row)
    if not rows:
        return {"seen": None, "throttle_before": round(before, 4), "why": "a climb was asked for at once after the release: no sample to look at"}
    drop, rise = 1.0 - min(row[1] for row in rows) / before, max(row[3] for row in rows)
    seen = drop >= limits.min_throttle_drop and rise >= limits.min_rise_mps
    return {"seen": seen, "throttle_before": round(before, 4), "throttle_drop": round(drop, 4), "rise_mps": round(rise, 3), "samples": len(rows),
            "why": f"with no climb asked for, the throttle fell {drop:.1%} and the vehicle rose at {rise:.2f} m/s within {rows[-1][0] - t_output:.1f} s"
                   + ("" if seen else f" (a release shows {limits.min_throttle_drop:.0%} and {limits.min_rise_mps:g} m/s or more)")}  # fmt: skip


def flight(t_command: float, t_output: float, tuning: Tuning, limits: Thresholds = Thresholds()) -> Dict[str, Any]:
    """The throttle steady flight takes after the release against before it."""
    before = throttle_before(t_command, tuning, limits)
    if before is None:
        return {"seen": None, "why": "too little steady flight before the command to compare with"}
    steady = [row for row in tuning if t_output < row[0] <= t_output + limits.after_within_s and _steady(row, limits)]
    first = [row[1] for row in steady if row[0] <= steady[0][0] + limits.before_s] if steady else []
    if len(first) < limits.min_samples:
        return {"seen": None, "throttle_before": round(before, 4), "why": f"too little steady flight in the {limits.after_within_s:g} s after the release to compare"}
    after = median(first)
    drop = 1.0 - after / before
    seen = drop >= limits.min_throttle_drop
    return {"seen": seen, "throttle_before": round(before, 4), "throttle_after": round(after, 4), "throttle_drop": round(drop, 4),
            "why": f"steady flight took {before:.3f} of the throttle before the release and {after:.3f} after: a {'fall' if drop >= 0 else 'rise'} of {abs(drop):.1%}"
                   + ("" if seen else f" (a release shows a fall of {limits.min_throttle_drop:.0%} or more)")}  # fmt: skip


def fall(t_output: float, ranges: Ranges, limits: Thresholds = Thresholds()) -> Dict[str, Any]:
    """The rangefinder seeing the payload fall away: readings shorter than the ground that get longer, and then the ground again."""
    ground = [metres for t, metres, status in ranges if t_output - 1.0 <= t <= t_output and status == RANGEFINDER_GOOD]
    after = [(t, metres) for t, metres, status in ranges if t_output < t <= t_output + limits.fall_window_s and status == RANGEFINDER_GOOD]
    if len(ground) < 2 or not after:
        return {"seen": None, "why": "no good rangefinder readings around the release"}
    level = median(ground)
    short = [(t, metres) for t, metres in after if metres <= level - limits.fall_margin_m]
    falling = all(later[1] >= earlier[1] - 0.02 for earlier, later in zip(short, short[1:]))
    back = bool(short) and any(t > short[-1][0] and metres > level - limits.fall_margin_m for t, metres in after)
    seen = len(short) >= limits.fall_min_samples and falling and back
    if seen:
        why = f"{len(short)} readings from {short[0][1]:.2f} m out to {short[-1][1]:.2f} m, under a ground at {level:.2f} m, and then the ground again: something fell through the beam"
    elif short:
        why = f"{len(short)} reading(s) shorter than the ground at {level:.2f} m, but not something falling away and gone"
    else:
        why = f"nothing passed under the rangefinder (the ground stayed at {level:.2f} m): it sees a payload only where one hangs or falls in its beam"
    return {"seen": seen, "ground_m": round(level, 2), "short_readings": len(short), "why": why}


def confirm(command: Optional[Dict[str, Any]], outputs: Outputs, tuning: Tuning, ranges: Ranges, params: Dict[str, float], limits: Thresholds = Thresholds()) -> Dict[str, Any]:
    """The whole check for one release command (tools/post_flight.py's `first_release`). `command` None: nothing was released."""
    if command is None:
        return {"verdict": "NO RELEASE", "lines": ["no release command in the flight"]}
    name = {GRIPPER: "DO_GRIPPER (release)", SPRAYER: "DO_SPRAYER (the drop servo)", SET_SERVO: "DO_SET_SERVO"}.get(command["command"], str(command["command"]))
    result = int(command.get("result", -1))
    acknowledged = result == 0
    output = output_moved(command, outputs, params, limits)
    t_output = output.get("t", command["t"])
    evidence = {"step": step(command["t"], t_output, tuning, limits), "flight": flight(command["t"], t_output, tuning, limits), "fall": fall(t_output, ranges, limits)}
    left_by = [kind for kind, found in evidence.items() if found["seen"]]
    missing = []
    if not acknowledged:
        missing.append(f"the command was {MAV_RESULT.get(result, f'answered with result {result}')}")
    if output["moved"] is not True:
        missing.append("the output is not seen to move")
    if not left_by:
        missing.append("nothing shows the payload leaving")
    verdict = "CONFIRMED" if not missing else "NOT CONFIRMED"
    lines = [f"command: {name}, {MAV_RESULT.get(result, f'result {result}')} by the flight controller",
             f"output: {output['why']}"] + [f"{kind}: {found['why']}" for kind, found in evidence.items()]  # fmt: skip
    return {"verdict": verdict, "command": name, "t": command["t"], "acknowledged": acknowledged, "result": result, "output": output, "left_by": left_by, "evidence": evidence,
            "missing": missing, "lines": lines, "thresholds": {"min_throttle_drop": limits.min_throttle_drop, "min_rise_mps": limits.min_rise_mps, "fall_margin_m": limits.fall_margin_m}}  # fmt: skip
