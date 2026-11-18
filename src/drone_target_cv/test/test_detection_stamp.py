"""Which stamp the red-target detector puts on its output (the `features.stamp_from_image` node flag). No ROS."""

import ast
from pathlib import Path
from types import SimpleNamespace

from drone_target_cv.detection_stamp import detection_stamp

NODE = Path(__file__).resolve().parents[1] / "drone_target_cv" / "target_cv.py"


def stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


IMAGE, NOW = stamp(184, 701_000_000), stamp(184, 745_000_000)


def test_flag_off_stamps_with_the_detector_clock_as_in_2026():
    assert detection_stamp(IMAGE, NOW, False) is NOW


def test_flag_on_copies_the_image_stamp():
    assert detection_stamp(IMAGE, NOW, True) is IMAGE


def test_flag_on_with_an_unstamped_image_falls_back_to_the_clock():
    assert detection_stamp(stamp(0, 0), NOW, True) is NOW
    assert detection_stamp(None, NOW, True) is NOW
    assert detection_stamp(stamp(0, 5), NOW, True).nanosec == 5  # any non-zero stamp is a stamp


def test_only_a_real_true_turns_it_on():
    for value in (None, 0, "", "true", 1):
        assert detection_stamp(IMAGE, NOW, value) is NOW


def test_the_node_declares_the_flag_off_and_uses_it_for_every_per_frame_detection():
    source = NODE.read_text()
    tree = ast.parse(source)
    declared = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "declare_parameter" and node.args and getattr(node.args[0], "value", None) == "features.stamp_from_image"
    ]
    assert len(declared) == 1 and declared[0].args[1].value is False
    functions = {node.name: node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)}
    body = ast.unparse(functions["_process_frame"])
    choose = "detection_stamp(header.stamp, self.get_clock().now().to_msg(), self._stamp_from_image)"
    assert f"det.header.stamp = {choose}" in body and "self._publish_not_found_detection(header.stamp)" in body
    not_found = ast.unparse(functions["_publish_not_found_detection"])
    assert "det.header.stamp = detection_stamp(image_stamp, self.get_clock().now().to_msg(), self._stamp_from_image)" in not_found
    # The image-size message is made from a frame too, so it follows the same rule (FND-01).
    assert f"dims.header.stamp = {choose}" in body
    # Only the one "not found" sent when detection is switched off, which has no image, keeps the clock.
    assert "self._publish_not_found_detection()" in source


def test_no_vision_node_stamps_what_it_makes_from_a_frame_with_its_own_clock():
    # FND-01: every message made from an image copies that image's stamp. In the 2027 detector the only
    # way to a stamp is detection_stamp (which gives the clock only with the flag off, the 2026 behaviour);
    # the YOLO node and the mission logger copy the source stamp outright. The grabbers are sources: they
    # are where a stamp is born. vision_pipeline/target_cv.py is the 2026 copy of the detector, kept as flown.
    import re

    repo = NODE.parents[3]
    own_clock = re.compile(r"\.header\.stamp\s*=\s*(self\.get_clock\(\)\.now\(\)|now\b)")
    checked = [NODE, repo / "src" / "vision_pipeline" / "vision_pipeline" / "yolo_node.py", repo / "src" / "vision_pipeline" / "vision_pipeline" / "mission_logger.py"]
    for path in checked:
        offending = [line.strip() for line in path.read_text().splitlines() if own_clock.search(line)]
        assert offending == [], f"{path.name}: {offending}"
    assert "det_array_msg.header.stamp = msg.header.stamp" in checked[1].read_text()


def test_flag_off_reads_the_clock_where_2026_read_it():
    # In comp-2026 the stamp was the clock at the publish, after the debug
    # images had gone out. The clock must still be read there, not earlier in
    # the frame, or every flags-off detection would be stamped earlier.
    lines = NODE.read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if "def _process_frame(" in line)
    end = next(i for i in range(start + 1, len(lines)) if lines[i].startswith("    def "))
    body = lines[start:end]
    mask_publish = max(i for i, line in enumerate(body) if "self._mask_pub.publish(" in line)
    clock_reads = [i for i, line in enumerate(body) if "self.get_clock().now()" in line]
    assert len(clock_reads) == 2  # the image-size message, and the detection; each read where 2026 read it
    size_publish = next(i for i, line in enumerate(body) if "self._image_size_pub.publish(" in line)
    assert clock_reads[0] < size_publish < clock_reads[1]
    assert clock_reads[1] > mask_publish
    assert any("det = PointStamped()" in line for line in body[clock_reads[1] - 6 : clock_reads[1]])  # it is the detection's own read
