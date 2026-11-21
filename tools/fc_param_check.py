#!/usr/bin/env python3
"""Check a flight-controller parameter file against what the recovery mission needs (REC-9). No ROS.

  python3 tools/fc_param_check.py docs/fc/recovery.parm
  python3 tools/fc_param_check.py aircraft_dump.param --profile docs/fc/recovery.parm

The first form checks the committed profile. The second is for the bench: a
dump of the aircraft's own parameters is held to the same rules, every
value in the profile must be in the dump with the same number, and the dump
must be a whole one (it has the radio's RCn_OPTION lines, which the last
rule looks at). Three forms are read: "NAME,VALUE" (Mission Planner),
"NAME VALUE" (MAVProxy), and QGroundControl's five tab-separated columns
(vehicle, component, name, value, type).

The rules, each from the spec's list:

  gripper             enabled, a servo gripper, on exactly one output, and that
                      one of the spare outputs 5 to 8; grab and release differ
  failsafe            FS_OPTIONS bit 5 clear: no failsafe releases the gripper
  rangefinder         one fitted and looking down
  ekf_source_options  EK3_SRC_OPTIONS 0: the EKF does not change source set itself
  source_set_2        no horizontal position, velocity from optical flow, a
                      yaw source, and FLOW_TYPE 5 (MAVLink: the companion's
                      OPTICAL_FLOW, REC-5): the set for flying without GPS
  second_gps          GPS2_TYPE 0: none is fitted
  fence               enabled, with the polygon, and an action that acts
  radio_switches      no RCn_OPTION is the gripper (19) or the EKF source set (90)

Exit code 0 when everything holds, 1 when a rule or a profile value does not,
2 when a file cannot be read.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

GRIPPER_FUNCTION = 28  # SERVOn_FUNCTION
SPARE_OUTPUTS = (5, 6, 7, 8)  # unassigned on the 2026 aircraft; 1 to 4 are the motors
FS_RELEASE_GRIPPER_BIT = 5
RC_OPTION_GRIPPER, RC_OPTION_EKF_SOURCE = 19, 90
RANGEFINDER_DOWN = 25  # RNGFNDn_ORIENT
SRC_NONE, SRC_OPTICAL_FLOW = 0, 5
FLOW_TYPE_MAVLINK = 5  # the flow comes from the companion as OPTICAL_FLOW; any other type and the flight controller ignores it
FENCE_POLYGON_BIT = 2
_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")
_SERVO, _RC = re.compile(r"^SERVO(\d+)_FUNCTION$"), re.compile(r"^RC(\d+)_OPTION$")


def read_params(path) -> Dict[str, float]:
    """A parameter file as {name: value}. `#` starts a comment.

    "NAME,VALUE", "NAME VALUE" (spaces or tabs), or QGroundControl's saved
    form: five columns, vehicle id, component id, name, value, type.
    """
    params: Dict[str, float] = {}
    for number, raw in enumerate(Path(path).read_text().splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = [part for part in re.split(r"[,\s]+", line) if part]
        if len(parts) == 5 and parts[0].isdigit() and parts[1].isdigit() and _NAME.match(parts[2]):
            parts = parts[2:4]  # QGroundControl: the name and the value are the third and fourth columns
        try:
            name, value = parts[0], float(parts[1])
        except (IndexError, ValueError):
            raise ValueError(f"{path}, line {number}: expected NAME,VALUE, got {raw.strip()!r}") from None
        if len(parts) > 2 or not _NAME.match(name):
            raise ValueError(f"{path}, line {number}: expected NAME,VALUE, got {raw.strip()!r}")
        if name in params:
            raise ValueError(f"{path}, line {number}: {name} is set twice")
        params[name] = value
    return params


@dataclass(frozen=True)
class Result:
    rule: str
    passed: bool
    found: str  # what the file has, and when it fails, what is wrong with it


def _whole(params: Dict[str, float], name: str) -> Optional[int]:
    value = params.get(name)
    return None if value is None or value != int(value) else int(value)


def _gripper(params) -> Optional[str]:
    if _whole(params, "GRIP_ENABLE") != 1:
        return f"GRIP_ENABLE is {params.get('GRIP_ENABLE', 'not set')}: the gripper is off"
    if _whole(params, "GRIP_TYPE") != 1:
        return f"GRIP_TYPE is {params.get('GRIP_TYPE', 'not set')}, not 1 (a servo gripper)"
    outputs = sorted(int(_SERVO.match(name).group(1)) for name, value in params.items() if _SERVO.match(name) and value == GRIPPER_FUNCTION)
    if len(outputs) != 1:
        return f"the gripper (SERVOn_FUNCTION {GRIPPER_FUNCTION}) is on {'no output' if not outputs else 'outputs ' + ', '.join(map(str, outputs))}: it must be on exactly one"
    if outputs[0] not in SPARE_OUTPUTS:
        return f"the gripper is on output {outputs[0]}, which is not one of the spare outputs {SPARE_OUTPUTS[0]} to {SPARE_OUTPUTS[-1]}"
    grab, release = params.get("GRIP_GRAB"), params.get("GRIP_RELEASE")
    if grab is None or release is None or grab == release:
        return f"GRIP_GRAB ({grab}) and GRIP_RELEASE ({release}) must both be set and differ"
    return None


def _failsafe(params) -> Optional[str]:
    options = _whole(params, "FS_OPTIONS")
    if options is None:
        return "FS_OPTIONS is not in the file: bit 5 (release the gripper on a failsafe) cannot be seen to be clear"
    if options & (1 << FS_RELEASE_GRIPPER_BIT):
        return f"FS_OPTIONS is {options}: bit {FS_RELEASE_GRIPPER_BIT} is set, so a failsafe releases the gripper"
    return None


def _rangefinder(params) -> Optional[str]:
    if not params.get("RNGFND1_TYPE"):
        return f"RNGFND1_TYPE is {params.get('RNGFND1_TYPE', 'not set')}: no rangefinder"
    if _whole(params, "RNGFND1_ORIENT") != RANGEFINDER_DOWN:
        return f"RNGFND1_ORIENT is {params.get('RNGFND1_ORIENT', 'not set')}, not {RANGEFINDER_DOWN} (down)"
    return None


def _ekf_source_options(params) -> Optional[str]:
    if _whole(params, "EK3_SRC_OPTIONS") != 0:
        return f"EK3_SRC_OPTIONS is {params.get('EK3_SRC_OPTIONS', 'not set')}, not 0: the EKF may change its source set by itself"
    return None


def _source_set_2(params) -> Optional[str]:
    if _whole(params, "EK3_SRC2_POSXY") != SRC_NONE:
        return f"EK3_SRC2_POSXY is {params.get('EK3_SRC2_POSXY', 'not set')}, not 0: source set 2 is for flying without GPS and must take no horizontal position"
    if _whole(params, "EK3_SRC2_VELXY") != SRC_OPTICAL_FLOW:
        return f"EK3_SRC2_VELXY is {params.get('EK3_SRC2_VELXY', 'not set')}, not {SRC_OPTICAL_FLOW} (optical flow)"
    if not params.get("EK3_SRC2_YAW"):
        return f"EK3_SRC2_YAW is {params.get('EK3_SRC2_YAW', 'not set')}: source set 2 has no yaw source"
    if _whole(params, "FLOW_TYPE") != FLOW_TYPE_MAVLINK:
        return f"FLOW_TYPE is {params.get('FLOW_TYPE', 'not set')}, not {FLOW_TYPE_MAVLINK} (MAVLink): the companion's OPTICAL_FLOW would be ignored, and source set 2 would have no velocity"
    return None


def _second_gps(params) -> Optional[str]:
    if _whole(params, "GPS2_TYPE") != 0:
        return f"GPS2_TYPE is {params.get('GPS2_TYPE', 'not set')}, not 0: no second GPS is fitted"
    return None


def _fence(params) -> Optional[str]:
    if _whole(params, "FENCE_ENABLE") != 1:
        return f"FENCE_ENABLE is {params.get('FENCE_ENABLE', 'not set')}: the fence is off"
    if not params.get("FENCE_ACTION"):
        return f"FENCE_ACTION is {params.get('FENCE_ACTION', 'not set')}: a breach is reported and nothing is done"
    kinds = _whole(params, "FENCE_TYPE")
    if kinds is None or not kinds & (1 << FENCE_POLYGON_BIT):
        return f"FENCE_TYPE is {params.get('FENCE_TYPE', 'not set')}: the polygon (bit {FENCE_POLYGON_BIT}) is not in it"
    return None


def _radio_switches(params) -> Optional[str]:
    names = {RC_OPTION_GRIPPER: "gripper", RC_OPTION_EKF_SOURCE: "EKF source set"}
    mapped = [f"{name} is {int(value)} ({names[int(value)]})" for name, value in sorted(params.items()) if _RC.match(name) and value in names]
    return ("; ".join(mapped) + ": only the mission releases the gripper and switches the source set") if mapped else None


RULES: List[tuple] = [
    ("gripper", _gripper),
    ("failsafe", _failsafe),
    ("rangefinder", _rangefinder),
    ("ekf_source_options", _ekf_source_options),
    ("source_set_2", _source_set_2),
    ("second_gps", _second_gps),
    ("fence", _fence),
    ("radio_switches", _radio_switches),
]


def check(params: Dict[str, float]) -> List[Result]:
    """Every rule's verdict on a set of parameters."""
    results = []
    for name, rule in RULES:
        problem: Optional[str] = rule(params)
        results.append(Result(name, problem is None, problem or "holds"))
    return results


def against_profile(params: Dict[str, float], profile: Dict[str, float]) -> List[str]:
    """Where a file's values are not the profile's."""
    problems = []
    for name, wanted in profile.items():
        if name not in params:
            problems.append(f"{name} is not in the file; the profile has {wanted:g}")
        elif abs(params[name] - wanted) > 1.0e-6:
            problems.append(f"{name} is {params[name]:g}, the profile has {wanted:g}")
    return problems


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Check a flight-controller parameter file against the recovery mission's needs.")
    parser.add_argument("file", type=Path, help="a parameter file: the profile, or a dump from the aircraft")
    parser.add_argument("--profile", type=Path, help="also require every value of this profile to be in the file")
    args = parser.parse_args(argv)
    try:
        params = read_params(args.file)
        profile = read_params(args.profile) if args.profile else {}
    except (OSError, ValueError) as error:
        print(f"fc_param_check: {error}", file=sys.stderr)
        return 2
    results = check(params)
    for result in results:
        print(f"{'ok  ' if result.passed else 'FAIL'} {result.rule}: {result.found}")
    differences = against_profile(params, profile)
    if args.profile and not any(_RC.match(name) for name in params):
        # A dump is the whole aircraft: with no RCn_OPTION in it, the radio's switches were not looked at.
        differences.append("the file has no RCn_OPTION lines: it is not a whole dump of the aircraft's parameters, and the radio's switches cannot be checked")
    for line in differences:
        print(f"FAIL profile: {line}")
    passed = all(result.passed for result in results) and not differences
    print(f"{'PASS' if passed else 'FAIL'}: {args.file} ({len(params)} parameters)")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
