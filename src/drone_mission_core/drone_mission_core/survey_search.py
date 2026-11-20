"""The executor's side of the survey mission (LOC-3): the flag, and the service that starts and stops the localisation pipeline.

Behind the mission flag `survey_search`, which defaults to false and needs
`use_height_estimator` (the survey's height guard watches `height_agl()`).
The mission is drone_mission_demo/missions/search_pattern_mission.py.

attach_localisation_control is the thin ROS part: with the flag on in the
sequence the executor gets a client for LOCALISATION_SERVICE (std_srvs/
SetBool: true starts the pipeline, false stops it), which the localisation
node (LOC-5) serves. With the flag off no client is made.

No ROS at import time; the standard library only.
"""

from __future__ import annotations

from typing import Any, Iterable, Optional

from .features import get_feature_flag

SEARCH_FLAG = "survey_search"
LOCALISATION_SERVICE = "/loc/set_enabled"
DO_CHANGE_SPEED = 178  # MAV_CMD_DO_CHANGE_SPEED: param1 speed type (1: ground speed), param2 metres a second, param3 throttle (-1: leave)


def attach_localisation_control(node: Any, specs: Iterable[Any]) -> Optional[Any]:
    """A client for the localisation pipeline's on/off service, or None when no mission has the flag on."""
    if not any(get_feature_flag(spec.features, SEARCH_FLAG) for spec in specs):
        return None
    from std_srvs.srv import SetBool

    client = node.create_client(SetBool, LOCALISATION_SERVICE)
    node.get_logger().info(f"Survey mission on ({SEARCH_FLAG}): the localisation pipeline is started and stopped through {LOCALISATION_SERVICE}")
    return client
