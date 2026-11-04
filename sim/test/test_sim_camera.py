"""The sim's downward camera matches what the delivery mission assumes.

The mission projects pixels to the ground with camera_fx_px scaled to the
image width, and assumes image top = forward, image right = right, optical
axis down (tracking_projection.camera_ray_to_body_flu). If the Gazebo camera's
field of view or mount drifts from that, centring steers the wrong way or by
the wrong amount. Needs yaml.
"""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "sim" / "models" / "iris_downcam" / "model.sdf"
SEQUENCE = (
    ROOT / "src" / "drone_mission_demo" / "config" / "sequences" / "package_delivery_demo.yaml"
)


def delivery_config():
    data = yaml.safe_load(SEQUENCE.read_text())
    for mission in data["mission_sequence"]["missions"]:
        if mission["type"] == "package_delivery":
            return mission["config"]
    raise AssertionError("no package_delivery mission")


def camera_sensor():
    root = ET.parse(MODEL).getroot()
    sensors = [s for s in root.iter("sensor") if s.get("type") == "camera"]
    assert len(sensors) == 1, sensors
    return sensors[0]


class SimCameraTest(unittest.TestCase):
    def test_fov_matches_mission_intrinsics(self):
        cfg = delivery_config()
        expected = 2.0 * math.atan(
            0.5 * cfg["camera_calibration_width_px"] / cfg["camera_fx_px"]
        )
        hfov = float(camera_sensor().find("camera/horizontal_fov").text)
        self.assertAlmostEqual(hfov, expected, delta=1.0e-4)

    def test_aspect_matches_calibration(self):
        cfg = delivery_config()
        image = camera_sensor().find("camera/image")
        width = int(image.find("width").text)
        height = int(image.find("height").text)
        self.assertAlmostEqual(
            width / height,
            cfg["camera_calibration_width_px"] / cfg["camera_calibration_height_px"],
            places=3,
        )

    def test_rate_is_30_hz(self):
        self.assertEqual(float(camera_sensor().find("update_rate").text), 30.0)

    def test_mount_looks_down_with_image_top_forward(self):
        # Sensor pose roll pitch yaw = 0, +90 deg, 0 in a link with no rotation.
        roll, pitch, yaw = (float(v) for v in camera_sensor().find("pose").text.split()[3:])
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, math.pi / 2, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)
        root = ET.parse(MODEL).getroot()
        link = next(l for l in root.iter("link") if l.get("name") == "downcam_link")
        self.assertEqual([float(v) for v in link.find("pose").text.split()[3:]], [0.0] * 3)

    def test_camera_frame_axes_map_to_flu(self):
        # Rotate the Gazebo camera axes (look +X, image up +Z, image right -Y)
        # by pitch +90 deg and check where they point in the body FLU frame.
        pitch = float(camera_sensor().find("pose").text.split()[4])
        c, s = math.cos(pitch), math.sin(pitch)

        def rot(v):
            x, y, z = v
            return (c * x + s * z, y, -s * x + c * z)

        look = rot((1, 0, 0))
        up = rot((0, 0, 1))
        right = rot((0, -1, 0))
        for got, want in ((look, (0, 0, -1)), (up, (1, 0, 0)), (right, (0, -1, 0))):
            for g, w in zip(got, want):
                self.assertAlmostEqual(g, w, places=6)


if __name__ == "__main__":
    unittest.main()
