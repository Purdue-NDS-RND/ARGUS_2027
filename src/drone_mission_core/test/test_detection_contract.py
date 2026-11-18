"""The one detection contract (FND-02): classes, pixel space, encode and decode. No ROS.

Spec: every detector publishes vision_msgs/Detection2DArray on
/perception/<detector>/detections, stamped with the image's capture time in
the camera's optical frame; class ids are strings from one list; boxes are in
pixels of the full-resolution calibrated image, so a detector that ran on a
shrunk frame scales back up before publishing.
"""

import ast
from pathlib import Path

import pytest

from drone_mission_core import detection_contract as contract
from drone_mission_core.detection_contract import Detection, DetectionSet


class Message:
    """A stand-in for a ROS message that has only its real fields, so a misspelt field fails here as it would in ROS."""

    FIELDS: dict = {}

    def __init__(self):
        for name, make in self.FIELDS.items():
            object.__setattr__(self, name, make())

    def __setattr__(self, name, value):
        if name not in self.FIELDS:
            raise AttributeError(f"{type(self).__name__} has no field '{name}'")
        object.__setattr__(self, name, value)


def message(name, **fields):
    return type(name, (Message,), {"FIELDS": fields})


Time = message("Time", sec=int, nanosec=int)
Header = message("Header", stamp=Time, frame_id=str)
Point2D = message("Point2D", x=float, y=float)
Pose2D = message("Pose2D", position=Point2D, theta=float)
BoundingBox2D = message("BoundingBox2D", center=Pose2D, size_x=float, size_y=float)
ObjectHypothesis = message("ObjectHypothesis", class_id=str, score=float)


class FakeMessages:
    """The vision_msgs types the contract uses, with exactly the fields of vision_msgs 4.1 (ROS 2 Humble)."""

    Detection2DArray = message("Detection2DArray", header=Header, detections=list)
    Detection2D = message("Detection2D", header=Header, results=list, bbox=BoundingBox2D, id=str)
    ObjectHypothesisWithPose = message("ObjectHypothesisWithPose", hypothesis=ObjectHypothesis, pose=object)


def test_the_stand_in_messages_refuse_a_field_vision_msgs_does_not_have():
    box = FakeMessages.Detection2D().bbox
    box.center.position.x = 4.0
    with pytest.raises(AttributeError, match="no field"):
        box.center.x = 4.0  # vision_msgs 4 moved x and y under center.position
    with pytest.raises(AttributeError):
        FakeMessages.ObjectHypothesisWithPose().id  # the class id is under .hypothesis


def sample_set():
    return DetectionSet(
        stamp_ns=184_701_043_720_117,
        frame_id="down_camera_optical",
        detections=(
            Detection("bullseye", 0.93, 1920.5, 1080.25, 300.0, 280.0),
            Detection("marker", 0.41, 100.0, 2000.0, 64.0, 60.0),
        ),
    )


def test_the_class_list_is_the_one_in_the_spec():
    assert contract.CLASSES == ("bullseye", "green_x", "marker", "boundary_marker", "bucket")


def test_the_topic_is_perception_detector_detections():
    assert contract.topic_for("ring_fit") == "/perception/ring_fit/detections"
    assert contract.topic_for("target_cv") == "/perception/target_cv/detections"
    for bad in ("", "Ring Fit", "ring/fit", "9lives"):
        with pytest.raises(contract.ContractError):
            contract.topic_for(bad)


def test_encode_then_decode_gives_the_same_detections_back():
    message = contract.encode(sample_set(), FakeMessages)
    assert message.header.stamp.sec == 184_701 and message.header.stamp.nanosec == 43_720_117
    assert message.header.frame_id == "down_camera_optical"
    first = message.detections[0]
    assert first.header.frame_id == "down_camera_optical" and first.header.stamp.nanosec == 43_720_117
    assert first.results[0].hypothesis.class_id == "bullseye" and first.results[0].hypothesis.score == 0.93
    assert (first.bbox.center.position.x, first.bbox.center.position.y) == (1920.5, 1080.25)
    assert (first.bbox.size_x, first.bbox.size_y) == (300.0, 280.0)
    assert contract.decode(message) == sample_set()


def test_an_empty_set_still_says_when_and_in_which_frame():
    empty = DetectionSet(5_000_000_123, "down_camera_optical", ())
    message = contract.encode(empty, FakeMessages)
    assert message.detections == [] and message.header.stamp.sec == 5 and message.header.stamp.nanosec == 123
    assert contract.decode(message) == empty


@pytest.mark.parametrize(
    "detection, word",
    [
        (Detection("red_circle", 0.9, 10.0, 10.0, 5.0, 5.0), "class"),
        (Detection("bullseye", 1.2, 10.0, 10.0, 5.0, 5.0), "score"),
        (Detection("bullseye", -0.1, 10.0, 10.0, 5.0, 5.0), "score"),
        (Detection("bullseye", 0.9, float("nan"), 10.0, 5.0, 5.0), "centre"),
        (Detection("bullseye", 0.9, 10.0, 10.0, 0.0, 5.0), "size"),
        (Detection("bullseye", 0.9, 10.0, 10.0, 5.0, -1.0), "size"),
    ],
)
def test_a_detection_that_breaks_the_contract_is_refused_when_it_is_encoded(detection, word):
    with pytest.raises(contract.ContractError, match=word):
        contract.encode(DetectionSet(1, "down_camera_optical", (detection,)), FakeMessages)


def test_a_set_with_no_frame_or_a_bad_stamp_is_refused():
    with pytest.raises(contract.ContractError, match="frame"):
        contract.encode(DetectionSet(1, "", ()), FakeMessages)
    with pytest.raises(contract.ContractError, match="stamp"):
        contract.encode(DetectionSet(-5, "down_camera_optical", ()), FakeMessages)
    with pytest.raises(contract.ContractError, match="stamp"):
        contract.encode(DetectionSet(1.5, "down_camera_optical", ()), FakeMessages)


def test_decode_refuses_a_message_from_outside_the_contract():
    message = contract.encode(sample_set(), FakeMessages)
    message.detections[0].results[0].hypothesis.class_id = "42"  # a numeric id, as some detectors publish
    with pytest.raises(contract.ContractError, match="class"):
        contract.decode(message)
    message = contract.encode(sample_set(), FakeMessages)
    message.detections[0].results = []
    with pytest.raises(contract.ContractError, match="hypothes"):
        contract.decode(message)


def test_a_detector_that_ran_on_a_1280_frame_scales_back_up_to_4k():
    # 3840 x 2160 shrunk to 1280 x 720 for the detector: a factor of three each way.
    small = Detection("marker", 0.8, 640.0, 360.0, 20.0, 18.0)
    full = contract.scale_to_full_resolution(small, ran_on=(1280, 720), full=(3840, 2160))
    assert full == Detection("marker", 0.8, 1920.0, 1080.0, 60.0, 54.0)
    corner = contract.scale_to_full_resolution(Detection("marker", 0.8, 0.0, 720.0, 10.0, 10.0), (1280, 720), (3840, 2160))
    assert (corner.cx, corner.cy) == (0.0, 2160.0)


def test_scaling_handles_a_letterboxed_1280_by_736_input():
    # The detector's input is 1280 x 736 (a multiple of 32): the 4K frame scaled by 1/3 with 8 rows of padding top and bottom.
    in_padded = Detection("bullseye", 0.9, 640.0, 368.0, 30.0, 30.0)  # the middle of the padded input
    full = contract.scale_to_full_resolution(in_padded, ran_on=(1280, 736), full=(3840, 2160), letterbox=True)
    assert (full.cx, full.cy) == pytest.approx((1920.0, 1080.0)) and (full.width, full.height) == pytest.approx((90.0, 90.0))
    top = contract.scale_to_full_resolution(Detection("bullseye", 0.9, 0.0, 8.0, 3.0, 3.0), (1280, 736), (3840, 2160), letterbox=True)
    assert (top.cx, top.cy) == pytest.approx((0.0, 0.0))


def test_scaling_to_the_same_size_changes_nothing_and_a_bad_size_is_refused():
    detection = Detection("bucket", 0.5, 12.0, 34.0, 5.0, 6.0)
    assert contract.scale_to_full_resolution(detection, (1280, 720), (1280, 720)) == detection
    with pytest.raises(contract.ContractError):
        contract.scale_to_full_resolution(detection, (0, 720), (3840, 2160))
    with pytest.raises(contract.ContractError, match="shape"):
        contract.scale_to_full_resolution(detection, (1280, 736), (3840, 2160))  # not the same shape, and no letterbox said


def test_the_legacy_point_is_what_the_2026_detector_publishes():
    # 2026: x and y in -1..1 across the image, z the red area in pixels; (-1, -1, 0) for "not found".
    detection = Detection("bullseye", 0.9, 960.0, 180.0, 200.0, 100.0)
    assert contract.to_legacy_point(detection, image_size=(1280, 720)) == pytest.approx((0.5, -0.5, 20000.0))
    assert contract.to_legacy_point(None, image_size=(1280, 720)) == (-1.0, -1.0, 0.0)
    assert contract.to_legacy_point(Detection("bullseye", 0.9, 0.0, 720.0, 4.0, 4.0), (1280, 720))[:2] == pytest.approx((-1.0, 1.0))


def test_the_best_bullseye_for_the_legacy_topic_is_the_most_confident_one():
    detections = (
        Detection("marker", 0.99, 1.0, 1.0, 2.0, 2.0),
        Detection("bullseye", 0.6, 10.0, 10.0, 5.0, 5.0),
        Detection("bullseye", 0.8, 20.0, 20.0, 5.0, 5.0),
    )
    assert contract.best_of_class(detections, "bullseye").cx == 20.0
    assert contract.best_of_class(detections, "bucket") is None
    with pytest.raises(contract.ContractError):
        contract.best_of_class(detections, "red_circle")


def test_the_module_has_no_ros_in_it():
    tree = ast.parse(Path(contract.__file__).read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
    assert imported <= {"__future__", "dataclasses", "math", "re", "typing"}
