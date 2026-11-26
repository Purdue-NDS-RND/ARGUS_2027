"""Mission type registrations for the demo mission package."""

from drone_mission_core.registry import import_optional_mission

from .gps_waypoint_mission import GpsWaypointMission
from .local_waypoint_mission import LocalWaypointMission
from .package_delivery_mission import PackageDeliveryMission
from .package_drop_mission import PackageDropMission
from .rtl_mission import RTLMission
from .takeoff_mission import TakeoffMission

__all__ = [
    "GpsWaypointMission",
    "LocalWaypointMission",
    "PackageDeliveryMission",
    "PackageDropMission",
    "RTLMission",
    "TakeoffMission",
]

# The 2027 missions. The fallback: one that cannot be imported on the aircraft must not take the June 2026
# missions above with it, so each is imported by itself. A failure is recorded (registry.IMPORT_FAILURES), the
# executor says it when it starts, and a sequence that names that mission does not start.
for _module, _name in (
    ("fence_demo_mission", "FenceDemoMission"),
    ("package_recovery_mission", "PackageRecoveryMission"),
    ("search_pattern_mission", "SearchPatternMission"),
    ("step_probe_mission", "CommandStepProbeMission"),
):
    _loaded = import_optional_mission(f"{__name__}.{_module}")
    if _loaded is not None:
        globals()[_name] = getattr(_loaded, _name)
        __all__.append(_name)
del _module, _name, _loaded
