"""The takeover latch (PLT-09): a mode nobody here asked for means somebody else has the vehicle.

Behind the mission flag `takeover_latch`, which defaults to false. With the
flag off the executor has no latch and its tick is 2026's.

With it on, the executor watches the flight mode. A change to a mode that
autonomy did not ask for (the safety pilot's switch, or a failsafe's RTL or
LAND) latches autonomy out: the executor clears its setpoints, sends none,
ticks no mission and asks for no mode, not even its own abort's, until a
person re-arms it by hand:

    ros2 service call /mission_executor/rearm_autonomy std_srvs/srv/Trigger

which is refused unless the vehicle is in GUIDED again. Re-armed, autonomy
acts again, and what it does is end the interrupted mission by its failure
policy: with `abort_and_rtl` the vehicle comes home (or lands, where a
mission has named a landing). The mission is neither resumed nor entered
again: after a takeover the vehicle is not in a state any mission knows. An
abort that was under way asks for its mode again. Whether to re-arm at all
is the decision of whoever calls the service.

"Asked for" is a mode named in a request of ours within the last
OURS_WITHIN_S: `MissionContext.request_mode_change` (a mission's, or the
abort's) and the take-off service's GUIDED. A change while disarmed, or
before autonomy has asked for anything, is the crew setting up and latches
nothing. No state from the flight controller teaches nothing either way.

At the takeover the executor also tells the take-off service (a node of its
own, which asks for GUIDED, arms and launches on its own timer once a
take-off has been asked of it) to drop a take-off it has pending, and goes
on telling it every second until it has answered. That service is behind the
take-off node's own flag, `features.takeoff_cancel`, and an executor with the
latch on starts no mission until the service is there.

`TakeoverLatch` is the rule, with no ROS in it. `attach_takeover` and
`takeover_tick` are the executor's side.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Tuple

from .features import get_feature_flag

FLAG = "takeover_latch"
OURS_WITHIN_S = 10.0  # a mode we asked for is ours if it comes within this long of the asking
REARM_MODE = "GUIDED"
REARM_SERVICE = "~/rearm_autonomy"
TAKEOFF_CANCEL_SERVICE = "drone_utils/cancel_takeoff"  # simple_takeoff_service.py, with its node flag features.takeoff_cancel on
CANCEL_RETRY_S = 1.0


class TakeoverLatch:
    def __init__(self) -> None:
        self.latched = False
        self.reason = ""
        self.takeovers = 0
        self._mode: Optional[str] = None
        self._asked: Dict[str, float] = {}
        self._engaged = False  # autonomy has asked for a mode at least once

    def commanded(self, mode: str, now_s: float) -> None:
        """Autonomy has just asked the flight controller for `mode` (or for something that brings it)."""
        self._asked[str(mode)] = float(now_s)
        self._engaged = True

    def observe(self, mode: Optional[str], armed: bool, now_s: float) -> bool:
        """The flight controller's mode this tick. Returns whether autonomy is latched out."""
        if not mode:
            return self.latched
        previous, self._mode = self._mode, mode
        if self.latched or previous is None or mode == previous:
            return self.latched
        asked = self._asked.get(mode)
        if asked is not None and 0.0 <= now_s - asked <= OURS_WITHIN_S:
            return False
        if not self._engaged or not armed:
            return False
        self.latched, self.takeovers = True, self.takeovers + 1
        self.reason = f"the mode went from {previous} to {mode}, which autonomy did not ask for"
        return True

    def rearm(self, mode: Optional[str], now_s: float) -> Tuple[bool, str]:
        """A person hands the vehicle back. Only from GUIDED."""
        if not self.latched:
            return False, "autonomy is not latched out: there is nothing to re-arm"
        if mode != REARM_MODE:
            return False, f"the vehicle is in {mode or 'no known mode'}: put it in {REARM_MODE} first, then re-arm"
        self.latched, self._mode = False, mode
        self._asked[REARM_MODE] = float(now_s)
        return True, f"autonomy re-armed in {REARM_MODE}"


def latch_requested(specs: Iterable[Any]) -> bool:
    """Whether any mission of the sequence has the takeover_latch flag on."""
    return any(get_feature_flag(spec.features, FLAG) for spec in specs)


def _now_s(node: Any) -> float:
    return node.get_clock().now().nanoseconds * 1.0e-9


def attach_takeover(node: Any, specs: Iterable[Any]) -> Optional[TakeoverLatch]:
    """A TakeoverLatch with its re-arm service on `node`, or None when the flag is off."""
    if not latch_requested(specs):
        return None
    from std_srvs.srv import Trigger

    latch = TakeoverLatch()

    def rearm(_request: Any, response: Any) -> Any:
        state = getattr(node, "_mavros_state", None)
        response.success, response.message = latch.rearm(getattr(state, "mode", None), _now_s(node))
        if response.success:
            node._takeover_rearmed = True  # the next tick enters the running mission again (_handed_back)
        (node.get_logger().warn if response.success else node.get_logger().error)(f"Takeover latch: {response.message}")
        return response

    node._takeover_rearm = rearm  # kept for the tests: the service's callback
    node.create_service(Trigger, REARM_SERVICE, rearm)
    # The take-off service is a node of its own, and between "take off" and the take-off it asks for GUIDED, arms
    # and launches on its own timer: at a takeover it is told to drop what it has pending.
    node._takeoff_cancel_client = node.create_client(Trigger, TAKEOFF_CANCEL_SERVICE)
    node._takeoff_cancel_request = Trigger.Request
    node.get_logger().info("Takeover latch on: a mode autonomy did not ask for stops the executor until it is re-armed by hand")
    return latch


def _cancel_takeoff(node: Any, now_s: float) -> None:
    """Tell the take-off service to drop a pending take-off, again every CANCEL_RETRY_S until it has answered yes."""
    state = node._takeoff_cancel
    future = state.get("future")
    if future is not None and future.done():
        result = future.result()
        state["future"] = None
        if result is not None and getattr(result, "success", False):
            state["done"] = True
            return
    if state["done"] or state.get("future") is not None or now_s - state["asked_s"] < CANCEL_RETRY_S:
        return
    state["asked_s"] = now_s
    client = node._takeoff_cancel_client
    if client.service_is_ready():
        state["future"] = client.call_async(node._takeoff_cancel_request())
    elif not state["warned"]:
        state["warned"] = True
        node.get_logger().warn(
            "Takeover latch: the take-off service's cancel is not there (its node flag features.takeoff_cancel is off?): "
            "a take-off it has pending is not dropped"
        )


def _handed_back(node: Any) -> None:
    """After a re-arm: autonomy acts again, and what it does is end the interrupted mission by its failure policy.

    The mission is not resumed (a drop that was a tick from releasing would
    release where the pilot left the vehicle) and not entered again (a
    recovery with the bucket already on its hook, or a take-off in mid-air,
    would start from a state the vehicle is not in). It is treated as
    failed: `abort_and_rtl` brings the vehicle home, or lands it where a
    mission has named a landing; `continue_to_next` goes on to the next
    mission. An abort that was already under way asks for its mode again.
    """
    mission = node._active_mission
    if mission is not None:
        node.get_logger().warn(f"Takeover latch: re-armed; mission '{mission.name}' was interrupted and ends by its failure policy")
        node._handle_failure_policy(mission, mission.spec.failure_policy)
    if node._abort_requested:
        node._abort_mode_requested = False  # the abort's mode is asked for (again)


def _cancel_offered(node: Any, now_s: float) -> bool:
    """Whether the take-off service's cancel is there. Until it is, an executor with the latch on flies nothing.

    The latch and the cancel are two flags in two files (the sequence's, and
    the take-off node's `features.takeoff_cancel`). A latch without the
    cancel would stop the executor at a takeover on the pad and leave the
    take-off node to ask for GUIDED again and launch: that is found out here,
    on the ground, and not at the takeover.
    """
    if node._takeoff_cancel_client.service_is_ready():
        return True
    if now_s - getattr(node, "_takeover_waiting_said_s", -1.0e9) >= 5.0:
        node._takeover_waiting_said_s = now_s
        node.get_logger().error(
            "Takeover latch: waiting for the take-off service's cancel (drone_utils/cancel_takeoff). No mission is started "
            "until it is there: turn the take-off node's flag features.takeoff_cancel on with takeover_latch"
        )
    return False


def takeover_tick(node: Any, latch: TakeoverLatch) -> bool:
    """The executor's first step each tick with the latch on. True: autonomy is out, and the tick does nothing more."""
    state = getattr(node, "_mavros_state", None)
    was = latch.latched
    now_s = _now_s(node)
    if not getattr(node, "_takeover_cancel_seen", False):
        if not _cancel_offered(node, now_s):
            return True  # nothing is flown with the latch on and no way to stop a pending take-off
        node._takeover_cancel_seen = True
    out = latch.observe(getattr(state, "mode", None), bool(getattr(state, "armed", False)), now_s)
    if out and not was:
        node._mission_context.clear_all_setpoints()
        node._takeoff_cancel = {"done": False, "future": None, "asked_s": -1.0e9, "warned": False}
        node.get_logger().error(
            f"TAKEOVER: {latch.reason}. The executor sends nothing and asks for nothing until autonomy is re-armed by hand "
            f"(in {REARM_MODE}: ros2 service call /mission_executor/rearm_autonomy std_srvs/srv/Trigger)"
        )
    if out:
        _cancel_takeoff(node, now_s)
    elif getattr(node, "_takeover_rearmed", False):
        node._takeover_rearmed = False
        _handed_back(node)
    return out
