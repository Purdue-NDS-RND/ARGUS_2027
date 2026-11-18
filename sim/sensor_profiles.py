#!/usr/bin/env python3
"""The sim vehicle's camera profiles, generated from one template (SIM-05). No ROS, no Gazebo.

  python3 sim/sensor_profiles.py            write every profile's model, bridge file and calibration
  python3 sim/sensor_profiles.py --check    exit 1 if a generated file is not what the template gives now

One template (sim/models/iris_camera.sdf.template) is the Iris with a
downward camera; a profile fills in the camera's size and rate:

  delivery   640 x 360 at 30 Hz    today's (sim/models/iris_downcam, unchanged)
  survey     1280 x 720 at 10 Hz
  still4k    3840 x 2160 at 1 Hz   headless only: for digit crops
  flow       delivery's camera plus a global-shutter flow camera, 640 x 400 at 60 Hz, mono

Every downward camera has the IMX519's field of view as calibrated: 1.37635
rad across, from fx = 2334.99647 px at 3840 px wide. The generated files
are committed, so the sim needs no generation step, and a test holds them
to the template.

For each profile this writes the model (sim/models/<model>/), the
ros_gz_bridge file (sim/config/bridge_<profile>.yaml; delivery keeps
bridge.yaml) and, for the camera driver's side, the sensor mode's
calibration (src/argus_camera/config/calibration/sim_<profile>.yaml), an
ideal pinhole computed from the field of view; sim_flow.yaml is the flow
camera's.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

SIM_DIR = Path(__file__).resolve().parent
REPO = SIM_DIR.parent
TEMPLATE = SIM_DIR / "models" / "iris_camera.sdf.template"
CALIBRATION_DIR = REPO / "src" / "argus_camera" / "config" / "calibration"

# The IMX519 as calibrated at 4K (package_delivery_demo.yaml: camera_fx_px at camera_calibration_width_px).
IMX519_FX_PX, IMX519_WIDTH_PX = 2334.99647, 3840
HORIZONTAL_FOV_RAD = 1.37635  # 2 atan(1920 / 2334.99647), as the template has it
FLOW_HORIZONTAL_FOV_RAD = 1.2  # the flow camera's lens is not calibrated yet: a 69 degree stand-in
VEHICLE_MASS_KG, VEHICLE_CDA_M2 = 1.755, 0.08

PROFILES: Dict[str, dict] = {
    "delivery": {"model": "iris_downcam", "width": 640, "height": 360, "rate_hz": 30, "bridge": "config/bridge.yaml", "flow": False, "headless_only": False},
    "survey": {"model": "iris_survey", "width": 1280, "height": 720, "rate_hz": 10, "bridge": "config/bridge_survey.yaml", "flow": False, "headless_only": False},
    "still4k": {"model": "iris_still4k", "width": 3840, "height": 2160, "rate_hz": 1, "bridge": "config/bridge_still4k.yaml", "flow": False, "headless_only": True},
    "flow": {"model": "iris_flow", "width": 640, "height": 360, "rate_hz": 30, "bridge": "config/bridge_flow.yaml", "flow": True, "headless_only": False},
}
FLOW_CAMERA = {"width": 640, "height": 400, "rate_hz": 60}

_BRIDGE_HEAD = """# ros_gz_bridge: the downward camera into ROS, where target_cv and the
# session logger expect it (sim_target_mission.yaml: image_topic camera/image).
"""
_BRIDGE_CAMERA = """- ros_topic_name: "/camera/image"
  gz_topic_name: "/downcam/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/downcam/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
"""
_BRIDGE_FLOW = """
# The flow camera, under the 2027 camera driver's names for the "flow" camera.
- ros_topic_name: "/argus/camera/flow/image_raw"
  gz_topic_name: "/flowcam/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/argus/camera/flow/camera_info"
  gz_topic_name: "/flowcam/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
"""

_FLOW_SENSOR = """
    <link name="flowcam_link">
      <!-- The optical-flow camera: beside the downward camera, looking down, global shutter (Gazebo's cameras all are). -->
      <pose relative_to="iris_with_ardupilot::iris_with_standoffs::base_link">0.05 0 -0.08 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia>
          <ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="flowcam" type="camera">
        <pose>0 0 0 0 1.5707963 0</pose>
        <gz_frame_id>flowcam_link</gz_frame_id>
        <topic>flowcam/image</topic>
        <always_on>1</always_on>
        <update_rate>@FLOW_RATE@</update_rate>
        <visualize>0</visualize>
        <camera>
          <horizontal_fov>@FLOW_FOV@</horizontal_fov>
          <image>
            <width>@FLOW_WIDTH@</width>
            <height>@FLOW_HEIGHT@</height>
            <format>L8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>500</far>
          </clip>
          <camera_info_topic>flowcam/camera_info</camera_info_topic>
        </camera>
      </sensor>
    </link>

    <joint name="flowcam_joint" type="fixed">
      <parent>iris_with_ardupilot::iris_with_standoffs::base_link</parent>
      <child>flowcam_link</child>
    </joint>
"""


def focal_px(width_px: int, horizontal_fov_rad: float = HORIZONTAL_FOV_RAD) -> float:
    """The focal length, in pixels, of an ideal camera this wide with this field of view."""
    return width_px / (2.0 * math.tan(horizontal_fov_rad / 2.0))


def _about(name: str, profile: dict) -> str:
    """The comment at the head of a generated model. Delivery's is the hand-written one the model has always had."""
    if name == "delivery":
        return (SIM_DIR / "models" / "iris_downcam.about.txt").read_text()
    lines = [
        "  <!--",
        f"    Generated by sim/sensor_profiles.py from iris_camera.sdf.template: the '{name}' camera profile. Do not edit.",
        "",
        f"    iris_with_ardupilot plus a fixed downward camera, {profile['width']}x{profile['height']} at {profile['rate_hz']} Hz,",
        f"    horizontal_fov {HORIZONTAL_FOV_RAD} rad: the IMX519's as calibrated, as in iris_downcam.",
    ]
    if profile["flow"]:
        lines.append(f"    And a flow camera: {FLOW_CAMERA['width']}x{FLOW_CAMERA['height']} mono at {FLOW_CAMERA['rate_hz']} Hz, horizontal_fov {FLOW_HORIZONTAL_FOV_RAD} rad.")
    if profile["headless_only"]:
        lines.append("    Headless only: at this size a frame a second is all a club laptop renders beside the flight.")
    return "\n".join(lines + ["  -->", ""])


def render_model(name: str) -> str:
    """The model.sdf of a profile, from the template."""
    profile = PROFILES[name]
    extra = ""
    if profile["flow"]:
        extra = _FLOW_SENSOR
        for key, value in (("@FLOW_RATE@", FLOW_CAMERA["rate_hz"]), ("@FLOW_WIDTH@", FLOW_CAMERA["width"]), ("@FLOW_HEIGHT@", FLOW_CAMERA["height"]), ("@FLOW_FOV@", FLOW_HORIZONTAL_FOV_RAD)):
            extra = extra.replace(key, str(value))
    text = TEMPLATE.read_text()
    for key, value in (
        ("@ABOUT@", _about(name, profile)),
        ("@MODEL@", profile["model"]),
        ("@RATE@", profile["rate_hz"]),
        ("@WIDTH@", profile["width"]),
        ("@HEIGHT@", profile["height"]),
        ("@EXTRA@", extra),
    ):
        text = text.replace(key, str(value))
    if "@" in text:
        raise ValueError(f"profile {name}: the template has a placeholder nothing filled")
    return text


def render_config(name: str) -> str:
    profile = PROFILES[name]
    if name == "delivery":
        return (SIM_DIR / "models" / "iris_downcam" / "model.config").read_text()  # hand-written, kept as it is
    what = f"a downward camera of the '{name}' profile ({profile['width']}x{profile['height']} at {profile['rate_hz']} Hz)"
    if profile["flow"]:
        what += " and a flow camera"
    return (
        '<?xml version="1.0"?>\n<model>\n'
        f"  <name>Iris with ArduPilot, {name} camera profile</name>\n  <version>1.0</version>\n"
        '  <sdf version="1.9">model.sdf</sdf>\n  <description>\n'
        f"    Generated by sim/sensor_profiles.py: the ardupilot_gazebo iris_with_ardupilot model with {what}.\n"
        "  </description>\n  <depend>\n    <model>\n      <uri>model://iris_with_ardupilot</uri>\n    </model>\n  </depend>\n</model>\n"
    )


def render_bridge(name: str) -> str:
    profile = PROFILES[name]
    head = _BRIDGE_HEAD if name == "delivery" else f"# Generated by sim/sensor_profiles.py for the '{name}' camera profile. Do not edit.\n" + _BRIDGE_HEAD
    return head + _BRIDGE_CAMERA + (_BRIDGE_FLOW if profile["flow"] else "")


def render_calibration(name: str) -> Optional[str]:
    """The calibration file of the profile's downward camera, or None for delivery (sim_delivery.yaml is hand-written and kept)."""
    if name in ("delivery", "flow"):  # flow's downward camera is delivery's
        return None
    profile = PROFILES[name]
    fx = focal_px(profile["width"])
    return (
        f"# Generated by sim/sensor_profiles.py. The sim's downward camera in the '{name}' profile\n"
        f"# (sim/models/{profile['model']}/model.sdf): an ideal pinhole, {profile['width']} x {profile['height']},\n"
        f"# horizontal field of view {HORIZONTAL_FOV_RAD} rad, no distortion. fx = fy = {profile['width']} / (2 tan({HORIZONTAL_FOV_RAD} / 2)).\n"
        f"width: {profile['width']}\nheight: {profile['height']}\nmeasured: true\nbasis: computed from the sim model's field of view\n"
        f"distortion_model: plumb_bob\n"
        f"k: [{fx:.5f}, 0.0, {profile['width'] / 2.0:.1f}, 0.0, {fx:.5f}, {profile['height'] / 2.0:.1f}, 0.0, 0.0, 1.0]\n"
        "d: [0.0, 0.0, 0.0, 0.0, 0.0]\n"
    )


def render_flow_calibration() -> str:
    """The flow camera's calibration: an ideal pinhole of its stand-in field of view."""
    width, height = FLOW_CAMERA["width"], FLOW_CAMERA["height"]
    fx = focal_px(width, FLOW_HORIZONTAL_FOV_RAD)
    return (
        "# Generated by sim/sensor_profiles.py. The sim's flow camera (sim/models/iris_flow/model.sdf): an ideal\n"
        f"# pinhole, {width} x {height}, horizontal field of view {FLOW_HORIZONTAL_FOV_RAD} rad, no distortion. The field of view is a\n"
        "# stand-in: the real flow camera's lens has not been calibrated (modes.yaml, ov9782_flow).\n"
        f"width: {width}\nheight: {height}\nmeasured: true\nbasis: computed from the sim model's field of view\n"
        f"distortion_model: plumb_bob\n"
        f"k: [{fx:.5f}, 0.0, {width / 2.0:.1f}, 0.0, {fx:.5f}, {height / 2.0:.1f}, 0.0, 0.0, 1.0]\n"
        "d: [0.0, 0.0, 0.0, 0.0, 0.0]\n"
    )


def generated_files() -> Dict[Path, str]:
    """Every generated file and what it should hold."""
    files: Dict[Path, str] = {}
    for name, profile in PROFILES.items():
        folder = SIM_DIR / "models" / profile["model"]
        files[folder / "model.sdf"] = render_model(name)
        files[folder / "model.config"] = render_config(name)
        files[SIM_DIR / profile["bridge"]] = render_bridge(name)
        calibration = render_calibration(name)
        if calibration is not None:
            files[CALIBRATION_DIR / f"sim_{name}.yaml"] = calibration
    files[CALIBRATION_DIR / "sim_flow.yaml"] = render_flow_calibration()
    return files


def stale() -> List[Path]:
    """Generated files that are missing or not what the template gives now."""
    return [path for path, text in generated_files().items() if not path.is_file() or path.read_text() != text]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Generate the sim vehicle's camera profiles from the template.")
    parser.add_argument("--check", action="store_true", help="write nothing; exit 1 if a generated file is out of date")
    args = parser.parse_args(argv)
    if args.check:
        out_of_date = stale()
        for path in out_of_date:
            print(f"out of date: {path.relative_to(REPO)}")
        return 1 if out_of_date else 0
    for path, text in generated_files().items():
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.is_file() or path.read_text() != text:
            path.write_text(text)
            print(f"wrote {path.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
