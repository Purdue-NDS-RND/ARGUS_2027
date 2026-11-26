#!/usr/bin/env python3
"""Check a flight-controller parameter file against what the recovery mission needs (REC-9), or against the
2027 baseline (PLT-08). No ROS.

  python3 tools/fc_param_check.py docs/fc/recovery.parm
  python3 tools/fc_param_check.py aircraft_dump.param --profile docs/fc/recovery.parm
  python3 tools/fc_param_check.py platform/fcu/argus_2027.param --rules baseline
  python3 tools/fc_param_check.py platform/fcu/argus_2027.param --diff platform/fcu/comp_2026.param
  python3 tools/fc_param_check.py aircraft_dump.param --rules baseline --profile platform/fcu/argus_2027.param

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

`--rules baseline` holds a file to the baseline's rules instead (the spec's
list for PLT-08):

  fence               as above, and with the ceiling: FENCE_TYPE bit 0 and a
                      FENCE_ALT_MAX above zero
  battery_failsafe    a low voltage and a critical voltage below it, each
                      with an action
  rc_failsafe         FS_THR_ENABLE not 0: losing the radio does something
  gcs_failsafe        FS_GCS_ENABLE not 0: losing the ground station does
  ekf_failsafe        FS_EKF_ACTION not 0
  sprayer             the drop's release (MAV_CMD_DO_SPRAYER): enabled, the pump
                      on exactly one spare output, on at any speed
                      (SPRAY_SPEED_MIN 0) and fully (SPRAY_PUMP_MIN 100).
                      ArduPilot acknowledges the command with the sprayer
                      disabled, and then nothing moves
  rangefinder         as above, and a TFmini (RNGFND1_TYPE 20) has a serial
                      port: a SERIALn_PROTOCOL 9 whose SERIALn_BAUD is 115
  failsafe, gripper, ekf_source_options, second_gps,
  radio_switches      as above

With `--profile` the file is a dump of the aircraft, and a group that is
still hidden is named ("FAIL hidden"): GRIP_ENABLE, RNGFND1_TYPE, FLOW_TYPE,
PLND_ENABLED, SCR_ENABLE and SPRAY_ENABLE each hide the rest of their group
until they are set and the flight controller is rebooted, so a first load
sets the switch and nothing behind it. A switch with nothing of its group
beside it, when the switch is on or the profile sets something behind it,
means: reboot, refresh the parameters, load the file again, save a new dump.

`--diff OTHER` lists every parameter of the file that OTHER has with another
value, or does not have: what loading the file onto OTHER would change. It
is information and does not change the exit code.

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
SPRAYER_PUMP_FUNCTION = 22  # SERVOn_FUNCTION: the output MAV_CMD_DO_SPRAYER moves, which is the drop's release
SPARE_OUTPUTS = (5, 6, 7, 8)  # unassigned on the 2026 aircraft; 1 to 4 are the motors
FS_RELEASE_GRIPPER_BIT = 5
RC_OPTION_GRIPPER, RC_OPTION_EKF_SOURCE = 19, 90
RANGEFINDER_DOWN = 25  # RNGFNDn_ORIENT
RANGEFINDER_TFMINI = 20  # RNGFNDn_TYPE: a Benewake TFmini, on a serial port
SERIAL_PROTOCOL_RANGEFINDER = 9  # SERIALn_PROTOCOL
SERIAL_BAUD_115200 = (115, 115200)  # SERIALn_BAUD: ArduPilot takes either spelling
# A parameter that hides the rest of its group until it is set and the flight controller rebooted: {the switch: the group's prefix}.
HIDDEN_GROUPS = {"GRIP_ENABLE": "GRIP_", "RNGFND1_TYPE": "RNGFND1_", "FLOW_TYPE": "FLOW_", "PLND_ENABLED": "PLND_", "SCR_ENABLE": "SCR_", "SPRAY_ENABLE": "SPRAY_"}
SRC_NONE, SRC_OPTICAL_FLOW = 0, 5
FLOW_TYPE_MAVLINK = 5  # the flow comes from the companion as OPTICAL_FLOW; any other type and the flight controller ignores it
FENCE_POLYGON_BIT = 2
FENCE_CEILING_BIT = 0
_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")
_SERVO, _RC = re.compile(r"^SERVO(\d+)_FUNCTION$"), re.compile(r"^RC(\d+)_OPTION$")
_SERIAL_PROTOCOL = re.compile(r"^SERIAL(\d+)_PROTOCOL$")


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


def _sprayer(params) -> Optional[str]:
    if _whole(params, "SPRAY_ENABLE") != 1:
        return f"SPRAY_ENABLE is {params.get('SPRAY_ENABLE', 'not set')}: the drop's MAV_CMD_DO_SPRAYER is acknowledged and nothing moves"
    outputs = sorted(int(_SERVO.match(name).group(1)) for name, value in params.items() if _SERVO.match(name) and value == SPRAYER_PUMP_FUNCTION)
    if len(outputs) != 1:
        return f"the sprayer's pump (SERVOn_FUNCTION {SPRAYER_PUMP_FUNCTION}) is on {'no output' if not outputs else 'outputs ' + ', '.join(map(str, outputs))}: it must be on exactly one"
    if outputs[0] not in SPARE_OUTPUTS:
        return f"the sprayer's pump is on output {outputs[0]}, which is not one of the spare outputs {SPARE_OUTPUTS[0]} to {SPARE_OUTPUTS[-1]}"
    if params.get("SPRAY_SPEED_MIN") != 0:
        return f"SPRAY_SPEED_MIN is {params.get('SPRAY_SPEED_MIN', 'not set')}, not 0: the drop is made from a hover, below the speed the sprayer waits for"
    if params.get("SPRAY_PUMP_MIN") != 100:
        return f"SPRAY_PUMP_MIN is {params.get('SPRAY_PUMP_MIN', 'not set')}, not 100: in a hover the pump's output would not go to its end"
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


def _rangefinder_with_port(params) -> Optional[str]:
    problem = _rangefinder(params)
    if problem or _whole(params, "RNGFND1_TYPE") != RANGEFINDER_TFMINI:
        return problem
    ports = sorted(int(_SERIAL_PROTOCOL.match(name).group(1)) for name, value in params.items() if _SERIAL_PROTOCOL.match(name) and value == SERIAL_PROTOCOL_RANGEFINDER)
    if not ports:
        return f"RNGFND1_TYPE is {RANGEFINDER_TFMINI} (a TFmini on a serial port) and no SERIALn_PROTOCOL is {SERIAL_PROTOCOL_RANGEFINDER} (rangefinder): it has no port to be read on"
    slow = [port for port in ports if params.get(f"SERIAL{port}_BAUD") not in SERIAL_BAUD_115200]
    if slow:
        return f"SERIAL{slow[0]}_BAUD is {params.get(f'SERIAL{slow[0]}_BAUD', 'not set')}, not 115: the TFmini on that port talks at 115200"
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


def _fence_with_ceiling(params) -> Optional[str]:
    problem = _fence(params)
    if problem:
        return problem
    if not _whole(params, "FENCE_TYPE") & (1 << FENCE_CEILING_BIT):
        return f"FENCE_TYPE is {params['FENCE_TYPE']:g}: the maximum height (bit {FENCE_CEILING_BIT}) is not in it"
    if not params.get("FENCE_ALT_MAX", 0.0) > 0.0:
        return f"FENCE_ALT_MAX is {params.get('FENCE_ALT_MAX', 'not set')}: the fence has no ceiling"
    return None


def _battery_failsafe(params) -> Optional[str]:
    low, critical = params.get("BATT_LOW_VOLT", 0.0), params.get("BATT_CRT_VOLT", 0.0)
    if not low > 0.0 or not params.get("BATT_FS_LOW_ACT"):
        return f"BATT_LOW_VOLT is {params.get('BATT_LOW_VOLT', 'not set')} and BATT_FS_LOW_ACT {params.get('BATT_FS_LOW_ACT', 'not set')}: a low battery does nothing"
    if not critical > 0.0 or not params.get("BATT_FS_CRT_ACT"):
        return f"BATT_CRT_VOLT is {params.get('BATT_CRT_VOLT', 'not set')} and BATT_FS_CRT_ACT {params.get('BATT_FS_CRT_ACT', 'not set')}: a critical battery does nothing"
    if critical >= low:
        return f"BATT_CRT_VOLT ({critical:g}) is not below BATT_LOW_VOLT ({low:g})"
    return None


def _acts(name: str, what: str) -> Callable[[Dict[str, float]], Optional[str]]:
    def rule(params) -> Optional[str]:
        return None if params.get(name) else f"{name} is {params.get(name, 'not set')}: {what} does nothing"

    return rule


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


BASELINE_RULES: List[tuple] = [
    ("fence", _fence_with_ceiling),
    ("battery_failsafe", _battery_failsafe),
    ("rc_failsafe", _acts("FS_THR_ENABLE", "losing the radio")),
    ("gcs_failsafe", _acts("FS_GCS_ENABLE", "losing the ground station")),
    ("ekf_failsafe", _acts("FS_EKF_ACTION", "an EKF failure")),
    ("failsafe", _failsafe),
    ("gripper", _gripper),
    ("sprayer", _sprayer),
    ("rangefinder", _rangefinder_with_port),
    ("ekf_source_options", _ekf_source_options),
    ("second_gps", _second_gps),
    ("radio_switches", _radio_switches),
]
RULE_SETS = {"recovery": RULES, "baseline": BASELINE_RULES}


def check(params: Dict[str, float], rules: Optional[List[tuple]] = None) -> List[Result]:
    """Every rule's verdict on a set of parameters (the recovery rules unless another set is given)."""
    results = []
    for name, rule in RULES if rules is None else rules:
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


def hidden_groups(params: Dict[str, float], profile: Dict[str, float]) -> Dict[str, List[str]]:
    """The groups a dump still has hidden: {the switch: the profile's parameters behind it, which cannot be in the dump}.

    A group counts as hidden when the dump has nothing of it but its switch, and either the switch is on or the
    profile sets something behind it. With the switch off and nothing asked of the group there is nothing to say.
    """
    hidden: Dict[str, List[str]] = {}
    for switch, prefix in HIDDEN_GROUPS.items():
        if any(name.startswith(prefix) and name != switch for name in params):
            continue
        wanted = sorted(name for name in profile if name.startswith(prefix) and name != switch)
        if wanted or params.get(switch):
            hidden[switch] = wanted
    return hidden


def changes(params: Dict[str, float], other: Dict[str, float]) -> List[str]:
    """What loading `params` onto a flight controller set as `other` would change, a line a parameter."""
    lines = []
    for name in sorted(params):
        if name not in other:
            lines.append(f"{name}: not in the other file -> {params[name]:g}")
        elif abs(params[name] - other[name]) > 1.0e-6:
            lines.append(f"{name}: {other[name]:g} -> {params[name]:g}")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Check a flight-controller parameter file against the recovery mission's needs, or the 2027 baseline's.")
    parser.add_argument("file", type=Path, help="a parameter file: a profile, or a dump from the aircraft")
    parser.add_argument("--profile", type=Path, help="also require every value of this profile to be in the file")
    parser.add_argument("--rules", choices=sorted(RULE_SETS), default="recovery", help="which rules the file is held to (default: recovery's)")
    parser.add_argument("--diff", type=Path, help="also list what the file changes in this other parameter file")
    args = parser.parse_args(argv)
    try:
        params = read_params(args.file)
        profile = read_params(args.profile) if args.profile else {}
        other = read_params(args.diff) if args.diff else None
    except (OSError, ValueError) as error:
        print(f"fc_param_check: {error}", file=sys.stderr)
        return 2
    if other is not None:
        changed = changes(params, other)
        for line in changed:
            print(f"diff {line}")
        print(f"diff: {len(changed)} of {len(params)} parameters differ from {args.diff}")
    results = check(params, RULE_SETS[args.rules])
    for result in results:
        print(f"{'ok  ' if result.passed else 'FAIL'} {result.rule}: {result.found}")
    differences = against_profile(params, profile)
    hidden = hidden_groups(params, profile) if args.profile else {}
    for switch, wanted in hidden.items():
        # Said once, as the group: its parameters are not a line each under "profile".
        differences = [line for line in differences if not any(line.startswith(f"{name} is not in the file") for name in wanted)]
        behind = f" (the profile sets {', '.join(wanted)})" if wanted else ""
        value = f"{params[switch]:g}" if switch in params else "not set"
        print(f"FAIL hidden: {switch} is {value} and the rest of {HIDDEN_GROUPS[switch]}* is not in the file{behind}: "
              "the group is hidden until its switch is set and the flight controller rebooted. "
              "To show it: reboot the flight controller, refresh the parameters, load the file again and save a new dump")  # fmt: skip
    if args.profile and not any(_RC.match(name) for name in params):
        # A dump is the whole aircraft: with no RCn_OPTION in it, the radio's switches were not looked at.
        differences.append("the file has no RCn_OPTION lines: it is not a whole dump of the aircraft's parameters, and the radio's switches cannot be checked")
    for line in differences:
        print(f"FAIL profile: {line}")
    passed = all(result.passed for result in results) and not differences and not hidden
    print(f"{'PASS' if passed else 'FAIL'}: {args.file} ({len(params)} parameters)")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
