"""Mission type registrations for the demo mission package."""

from .gps_waypoint_mission import GpsWaypointMission
from .local_waypoint_mission import LocalWaypointMission
from .package_delivery_mission import PackageDeliveryMission
from .package_drop_mission import PackageDropMission
from .rtl_mission import RTLMission
from .search_pattern_mission import SearchPatternMission
from .takeoff_mission import TakeoffMission

__all__ = [
    "GpsWaypointMission",
    "LocalWaypointMission",
    "PackageDeliveryMission",
    "PackageDropMission",
    "RTLMission",
    "SearchPatternMission",
    "TakeoffMission",
]
