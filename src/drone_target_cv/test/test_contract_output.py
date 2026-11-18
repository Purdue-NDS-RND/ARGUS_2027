"""The 2026 detector's output on the detection contract (FND-02), through the real detector. No ROS."""

import sys
from pathlib import Path

import cv2
import numpy as np

CORE = Path(__file__).resolve().parents[2] / "drone_mission_core"
sys.path[:0] = [str(CORE), str(CORE / "test")]

from drone_mission_core import detection_contract as contract  # noqa: E402
from drone_target_cv.contract_output import bullseye_detection, legacy_messages, to_detection_set  # noqa: E402
from drone_target_cv.target_detector import RedTargetDetector, RedTargetDetectorConfig  # noqa: E402
from test_detection_contract import FakeMessages  # noqa: E402


def frame_with_bullseye(centre=(400, 200), radius=60, size=(640, 360)):
    image = np.full((size[1], size[0], 3), (60, 140, 60), dtype=np.uint8)  # grass
    for index, ring in enumerate(range(radius, 0, -radius // 5)):
        colour = (30, 30, 220) if index % 2 == 0 else (240, 240, 240)  # red and white rings, BGR
        cv2.circle(image, centre, ring, colour, -1)
    return image


def test_a_bullseye_the_real_detector_finds_becomes_a_contract_detection():
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(frame_with_bullseye())
    assert result.accepted
    detection = bullseye_detection(result)
    assert detection.class_id == "bullseye" and 0.0 < detection.score <= 1.0
    assert abs(detection.cx - 400) <= 3 and abs(detection.cy - 200) <= 3
    assert 100 <= detection.width <= 130 and 100 <= detection.height <= 130  # the 120 px target
    message = contract.encode(contract.DetectionSet(1_000_000_000, "down_camera_optical", (detection,)), FakeMessages)
    assert contract.decode(message).detections[0] == detection


def test_no_target_is_no_detection():
    empty = np.full((360, 640, 3), (60, 140, 60), dtype=np.uint8)
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(empty)
    assert not result.accepted and bullseye_detection(result) is None


def test_the_legacy_point_made_from_the_contract_detection_is_the_one_the_2026_node_publishes():
    # target_cv.py: x_norm = cx / width * 2 - 1, y_norm likewise.
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(frame_with_bullseye())
    detection = bullseye_detection(result)
    x, y, _area = contract.to_legacy_point(detection, (640, 360))
    assert x == (result.center[0] / 640.0) * 2.0 - 1.0 and y == (result.center[1] / 360.0) * 2.0 - 1.0


def test_a_score_outside_0_to_1_is_clamped_and_a_result_with_no_mask_still_has_a_box():
    from types import SimpleNamespace

    result = SimpleNamespace(center=(10, 20), confidence=1.7, selected_red_mask=None, cluster_area_px=400.0)
    detection = bullseye_detection(result)
    assert detection.score == 1.0 and (detection.width, detection.height) == (20.0, 20.0)
    assert bullseye_detection(SimpleNamespace(center=(10, 20), confidence=-0.2, selected_red_mask=None, cluster_area_px=0.0)).score == 0.0


def stamp(sec, nanosec):
    from types import SimpleNamespace

    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_a_frames_detections_become_one_set_with_the_frames_stamp_and_optical_frame():
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(frame_with_bullseye())
    found = to_detection_set(result, stamp(184, 701_000_000), "down_camera_optical")
    assert found.stamp_ns == 184_701_000_000 and found.frame_id == "down_camera_optical" and len(found.detections) == 1
    empty = RedTargetDetector(RedTargetDetectorConfig()).detect(np.full((360, 640, 3), (60, 140, 60), dtype=np.uint8))
    nothing = to_detection_set(empty, stamp(184, 734_000_000), "down_camera_optical")
    assert nothing.detections == () and nothing.stamp_ns == 184_734_000_000  # nothing found still says when it looked


def test_an_image_with_no_stamp_or_no_frame_cannot_go_out_on_the_contract():
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(frame_with_bullseye())
    assert to_detection_set(result, stamp(0, 0), "down_camera_optical") is None
    assert to_detection_set(result, stamp(184, 0), "") is None


def test_the_adapter_turns_a_contract_set_back_into_the_two_2026_messages():
    result = RedTargetDetector(RedTargetDetectorConfig()).detect(frame_with_bullseye())
    found = to_detection_set(result, stamp(184, 701_000_000), "down_camera_optical")
    point, size = legacy_messages(found, (640, 360))
    assert point == contract.to_legacy_point(found.detections[0], (640, 360)) and size == (640.0, 360.0, 0.0)
    empty = contract.DetectionSet(184_734_000_000, "down_camera_optical", ())
    assert legacy_messages(empty, (640, 360))[0] == (-1.0, -1.0, 0.0)  # the 2026 "nothing found"
    # Markers and buckets are not what the 2026 missions track: only the bullseye is passed on.
    others = contract.DetectionSet(1, "down_camera_optical", (contract.Detection("marker", 0.9, 5.0, 5.0, 2.0, 2.0),))
    assert legacy_messages(others, (640, 360))[0] == (-1.0, -1.0, 0.0)


def test_the_adapter_passes_on_nothing_the_2026_detector_would_not_have():
    confident = contract.Detection("bullseye", 0.9, 320.0, 180.0, 50.0, 50.0)
    weak = contract.Detection("bullseye", 0.2, 100.0, 100.0, 50.0, 50.0)  # under the 2026 detector's 0.25
    outside = contract.Detection("bullseye", 0.99, 700.0, 180.0, 50.0, 50.0)  # centred off a 640 x 360 image

    def point(*detections, **options):
        return legacy_messages(contract.DetectionSet(1, "down_camera_optical", detections), (640, 360), **options)[0]

    assert point(weak) == (-1.0, -1.0, 0.0) and point(outside) == (-1.0, -1.0, 0.0)
    assert point(weak, outside, confident) == contract.to_legacy_point(confident, (640, 360))
    assert point(weak, min_score=0.1) == contract.to_legacy_point(weak, (640, 360))
    assert point(contract.Detection("bullseye", 0.25, 0.0, 359.9, 5.0, 5.0)) != (-1.0, -1.0, 0.0)  # at the floor and at the edge: in


def test_a_stamp_from_the_future_is_not_a_detection_time_for_the_missions():
    from drone_target_cv.contract_output import stamp_is_usable

    now = 500 * 1_000_000_000
    assert stamp_is_usable(now - 40_000_000, now) and stamp_is_usable(now + 100_000_000, now)  # 40 ms old; clocks 100 ms apart
    assert stamp_is_usable(now - 5_000_000_000, now)  # old is the missions' own timeout's business
    assert not stamp_is_usable(now + 101_000_000, now) and not stamp_is_usable(1_794_966_718 * 1_000_000_000, now)  # wall time against sim time


def test_the_detector_node_and_the_adapter_use_the_contract_behind_a_flag_that_is_off():
    import ast

    package = Path(__file__).resolve().parents[1]
    node = (package / "drone_target_cv" / "target_cv.py").read_text()
    declared = [
        call
        for call in ast.walk(ast.parse(node))
        if isinstance(call, ast.Call) and getattr(call.func, "attr", "") == "declare_parameter" and call.args and getattr(call.args[0], "value", None) == "features.detection_contract"
    ]
    assert len(declared) == 1 and declared[0].args[1].value is False
    assert 'contract.topic_for("target_cv")' in node and "if self._detection_contract:" in node
    # Flag off, the node imports nothing it did not import in 2026: the contract's modules come in only with the flag on.
    header = node.split("class TargetCV")[0]
    assert "vision_msgs" not in header and "drone_mission_core" not in header and "contract_output" not in header
    # And with the flag on, a failure on the contract side cannot stop the 2026 topics: it is caught and logged,
    # and the contract's publish comes after the 2026 publishes in the frame handler.
    publish = node.split("def _publish_on_contract")[1].split("\n    def ")[0]
    assert "try:" in publish and "except Exception" in publish and "Detection contract output failed" in publish
    handler = node.split("def _process_frame")[1]
    assert handler.index("self._publish_on_contract(result, header)") > handler.index("self._publish_not_found_detection(header.stamp)") > handler.index("self._detection_pub.publish(det)")
    adapter = (package / "drone_target_cv" / "legacy_adapter.py").read_text()
    assert '"/drone_package_drop/target_detection"' in adapter and '"/drone_package_drop/image_size"' in adapter
    assert "contract.decode(" in adapter and "legacy_messages(" in adapter and len(adapter.splitlines()) < 110
    setup = (package / "setup.py").read_text()
    assert "detection_legacy_adapter = drone_target_cv.legacy_adapter:main" in setup
    import yaml

    for params in sorted((package.parent / "drone_mission_demo" / "config" / "params").glob("*target_mission.yaml")):
        assert yaml.safe_load(params.read_text())["target_cv"]["ros__parameters"]["features"]["detection_contract"] is False, params.name
