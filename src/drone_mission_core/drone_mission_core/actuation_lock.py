"""The actuation lock (PLT-09): the gripper and the drop servo answer only to a release state whose gates hold.

Behind the mission flag `actuation_lock`, which defaults to false. With the
flag off the executor has no lock and `MissionContext.command_gripper` and
`command_sprayer` are 2026's.

With it on, a command to an actuator goes out only if, on that tick, the
mission has said from which state it is commanding and that the gates of
that state hold:

    context.permit_actuation("gripper", state="GROUND_DWELL", gates_hold=True, why="set down: contact and the height under the ceiling")
    context.command_gripper(release=True, ...)

A permit is refused unless the state is a release state (RELEASE_STATES) and
`gates_hold` is exactly True; it is for one command of its own actuator and
is stale after PERMIT_LIFE_S. A command with no permit behind it is not
sent: the context logs the refusal and hands back a future whose result is
None, which every mission already reads as "not accepted".

The lock does not judge the gates: each release state has its own (the drop's
release gates, the delivery's set-down rule, the recovery's engage and
release rules), and the mission passes their verdict. What the lock adds is
that no other state, and no other code path, can move an actuator: a bug
that calls `command_gripper` from a transit state drops nothing.

The 2026 release paths ask for no permit, so a mission that actuates can be
locked only with its gated release path on (GATED_RELEASE_FLAG): the
executor refuses to start otherwise (`lock_violations`).

Turning the drop servo off (`command_sprayer(enable=False)`) ends an
actuation and is never locked.

`ActuationLock` is the rule, with no ROS in it; `attach_actuation_lock` is
the executor's side.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Tuple

from .features import get_feature_flag

FLAG = "actuation_lock"
ACTUATORS = ("gripper", "sprayer")  # the sprayer's output is the drop servo
PERMIT_LIFE_S = 0.11  # the tick a permit is given on, and the one after (20 Hz)
# The states an actuator may be commanded from: the drop's and the delivery's release, the recovery's engage and set-down.
RELEASE_STATES = ("DROP_PAYLOAD", "GROUND_DWELL", "ENGAGE", "SET_DOWN")
# The flag that gives each actuating mission a release path that asks the lock.
GATED_RELEASE_FLAG = {"package_drop": "drop_release_gates", "package_delivery": "delivery_soft_set_down", "package_recovery": "package_recovery"}


class ActuationLock:
    def __init__(self) -> None:
        self._permits: Dict[str, Tuple[float, bool, str]] = {}  # actuator -> (when, granted, why not)
        self.refused: List[Tuple[str, float, str]] = []  # every command the lock stopped: actuator, when, why
        self.allowed = 0

    def permit(self, actuator: str, state: Optional[str], gates_hold: Any, now_s: float, why: str = "") -> bool:
        """A mission says it is about to command `actuator` from `state` with its gates at `gates_hold`. True if that is a permit."""
        if actuator not in ACTUATORS:
            raise ValueError(f"the actuation lock knows {', '.join(ACTUATORS)}, not '{actuator}'")
        if state not in RELEASE_STATES:
            refusal = f"{state or 'no state'} is not a release state ({', '.join(RELEASE_STATES)})"
        elif gates_hold is not True:
            refusal = f"the gates do not hold in {state}" + (f": {why}" if why else "")
        else:
            refusal = ""
        self._permits[actuator] = (float(now_s), not refusal, refusal)
        return not refusal

    def allows(self, actuator: str, now_s: float) -> Tuple[bool, str]:
        """Whether a command to `actuator` may go out now. A permit is used up by the command it allows."""
        given = self._permits.pop(actuator, None)
        if given is None:
            why = "no release state has permitted it"
        elif not given[1]:
            why = given[2]
        elif not 0.0 <= now_s - given[0] <= PERMIT_LIFE_S:
            why = f"its permit is {now_s - given[0]:.2f} s old: the gates are read on the tick of the command"
        else:
            self.allowed += 1
            return True, ""
        self.refused.append((actuator, float(now_s), why))
        return False, why


def lock_requested(specs: Iterable[Any]) -> bool:
    """Whether any mission of the sequence has the actuation_lock flag on."""
    return any(get_feature_flag(spec.features, FLAG) for spec in specs)


def lock_violations(specs: Iterable[Any]) -> List[str]:
    """Why a sequence cannot be flown locked: an actuating mission whose release path would not ask for a permit."""
    problems = []
    for spec in specs:
        needed = GATED_RELEASE_FLAG.get(getattr(spec, "type_name", ""))
        if needed and get_feature_flag(spec.features, FLAG) and not get_feature_flag(spec.features, needed):
            problems.append(f"mission '{spec.name}' ({spec.type_name}) needs {needed} on: its 2026 release asks the lock for nothing and would be refused")
    return problems


def attach_actuation_lock(node: Any, specs: Iterable[Any]) -> Optional[ActuationLock]:
    """An ActuationLock for the executor, or None when the flag is off."""
    specs = list(specs)
    if not lock_requested(specs):
        return None
    node.get_logger().info("Actuation lock on: the gripper and the drop servo answer only to a release state whose gates hold")
    return ActuationLock()
