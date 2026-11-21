"""What the recorder records (SIM-09): one list, used by the launch file, the sim's recorder and the replay. No ROS imports.

MAVROS pose, velocity, IMU, GPS, state, rangefinder and setpoints; camera
info; detections (the 2026 topics and every detector on the contract); the
estimator's tracks; and the recorder's own taps. Camera info is recorded
under the 2026 name and under the 2027 driver's. Servo outputs are not in
the list yet: MAVROS publishes them from a plugin (rc_io) that the 2027
plugin list does not load, and whether it should is SIM-06's question (the
payload bridge is what reads them). Images are
not recorded: a bag of frames is too big to carry and slows what it
records. Frames are meant to stay beside the bag as JPEG files with an
index, written by argus_camera's frame log (frame_log.py); nothing writes
them during a recording yet. What the bag does get is the taps: one small
message for every frame seen on an image topic, so it still says when each
frame arrived.
"""

from __future__ import annotations

from typing import Dict, List

POSE = "/mavros/local_position/pose"
VELOCITY = "/mavros/local_position/velocity_local"
RANGEFINDER = "/mavros/rangefinder/rangefinder"
IMU = "/mavros/imu/data"
GPS = "/mavros/global_position/global"
STATE = "/mavros/state"
EXTENDED_STATE = "/mavros/extended_state"
SETPOINT_LOCAL = "/mavros/setpoint_raw/local"
SETPOINT_GLOBAL = "/mavros/setpoint_raw/global"
SETPOINT_POSITION = "/mavros/setpoint_position/local"
CAMERA_INFO = "/camera/camera_info"
DRIVER_CAMERAS = ("down", "flow")  # the cameras of argus_camera's robot description
LEGACY_DETECTION = "/drone_package_drop/target_detection"
LEGACY_IMAGE_SIZE = "/drone_package_drop/image_size"
ESTIMATOR_TARGETS = "/estimator/targets"
CLOCK = "/clock"
SIM_CLOCK_TAP = "/replay/sim_clock"  # Gazebo's clock, for a sim run whose nodes are on wall time
FRAME_TAP_PREFIX = "/replay/frame_seen"  # + the image topic: its header, when a frame arrived

DETECTORS = ("target_cv", "yolo", "recovery_cv", "ring_fit", "sim_oracle")  # every name a detector publishes under on the contract


def contract_topic(detector: str) -> str:
    return f"/perception/{detector}/detections"


def driver_info_topic(camera: str) -> str:
    """The 2027 camera driver's camera_info topic for a camera (argus_camera publishing.topic_names)."""
    return f"/argus/camera/{camera}/camera_info"


def driver_image_topic(camera: str) -> str:
    return f"/argus/camera/{camera}/image_raw"


def frame_tap_topic(image_topic: str) -> str:
    """The tap's topic for an image topic: /camera/image -> /replay/frame_seen/camera/image."""
    return FRAME_TAP_PREFIX + "/" + image_topic.strip("/")


# Topic -> type, for everything recorded. A topic nobody publishes costs nothing to list.
RECORDED: Dict[str, str] = {
    POSE: "geometry_msgs/msg/PoseStamped",
    VELOCITY: "geometry_msgs/msg/TwistStamped",
    RANGEFINDER: "sensor_msgs/msg/Range",
    IMU: "sensor_msgs/msg/Imu",
    GPS: "sensor_msgs/msg/NavSatFix",
    STATE: "mavros_msgs/msg/State",
    EXTENDED_STATE: "mavros_msgs/msg/ExtendedState",
    SETPOINT_LOCAL: "mavros_msgs/msg/PositionTarget",
    SETPOINT_GLOBAL: "mavros_msgs/msg/GlobalPositionTarget",
    SETPOINT_POSITION: "geometry_msgs/msg/PoseStamped",
    CAMERA_INFO: "sensor_msgs/msg/CameraInfo",
    **{driver_info_topic(camera): "sensor_msgs/msg/CameraInfo" for camera in DRIVER_CAMERAS},
    LEGACY_DETECTION: "geometry_msgs/msg/PointStamped",
    LEGACY_IMAGE_SIZE: "geometry_msgs/msg/PointStamped",
    ESTIMATOR_TARGETS: "vision_msgs/msg/Detection3DArray",
    CLOCK: "rosgraph_msgs/msg/Clock",
    SIM_CLOCK_TAP: "rosgraph_msgs/msg/Clock",
    **{contract_topic(detector): "vision_msgs/msg/Detection2DArray" for detector in DETECTORS},
}

# Image topics the sim's recorder taps: the camera's frames, and the 2026 detector's annotated copy of each
# (which carries the frame's own header, so it says which frame a detection came from and when the detector was done).
# A tap is one more subscriber to an image topic, and on the aircraft that is not free: nothing else there listens
# to the raw /camera/image, so a tap makes the 2026 grabber deliver every raw frame, before it publishes the
# compressed frame the detector reads. So the launch file taps nothing unless it is told to (tap_images:=...).
TAPPED_IMAGES = ("/camera/image", "/target_cv/annotated")


def record_topics(tapped_images=TAPPED_IMAGES) -> List[str]:
    """Every topic to give `ros2 bag record`, in a fixed order."""
    return sorted(RECORDED) + [frame_tap_topic(topic) for topic in tapped_images]
