"""SIM-12: the splat twin's replay (sim/splat_twin.py), on its stand-in field. No ROS; numpy, cv2, yaml."""

import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))

import splat_twin as twin  # noqa: E402

CAMERA = twin.Camera(640, 360, 384.0, 384.0, 320.0, 180.0)
TARGETS = {"targets": [{"name": "drop_target", "kind": "bullseye", "east_m": 1.0, "north_m": 0.5, "size_m": 2.0}, {"name": "marker_3", "kind": "marker", "east_m": -3.0, "north_m": 2.0, "size_m": 0.61, "digit": 3}]}


@pytest.fixture(scope="module")
def render_set(tmp_path_factory):
    return twin.standin(tmp_path_factory.mktemp("twin") / "set", frames=16)


def targets_file(tmp_path, content=TARGETS):
    path = tmp_path / "targets.yaml"
    path.write_text(yaml.safe_dump(content))
    return path


def test_the_ground_goes_into_the_picture_nose_up_and_comes_back_out():
    """East is the nose at yaw 0: a point ahead is above the centre, a point to the right (south) is right of it."""
    level = twin.Pose(0, 0.0, 10.0, 5.0, 12.0, 0.0)
    h = twin.ground_to_image(CAMERA, level)

    def pixel(east, north, homography=h):
        u, v, w = homography @ np.array([east, north, 1.0])
        return u / w, v / w

    assert pixel(10.0, 5.0) == pytest.approx((320.0, 180.0))
    assert pixel(13.0, 5.0) == pytest.approx((320.0, 180.0 - 3.0 * 384.0 / 12.0))  # 3 m ahead: 96 px up
    assert pixel(10.0, 2.0) == pytest.approx((320.0 + 96.0, 180.0))  # 3 m south is to the right of a vehicle facing east
    north_up = twin.ground_to_image(CAMERA, twin.Pose(0, 0.0, 10.0, 5.0, 12.0, 90.0))
    assert pixel(10.0, 8.0, north_up) == pytest.approx((320.0, 84.0)) and pixel(13.0, 5.0, north_up) == pytest.approx((416.0, 180.0))  # facing north: east is right
    for pose in (level, twin.Pose(1, 0.1, -4.0, 2.5, 7.0, 133.0)):
        east, north = twin.image_to_ground(CAMERA, pose, *pixel(1.5, -2.5, twin.ground_to_image(CAMERA, pose)))
        assert (east, north) == pytest.approx((1.5, -2.5), abs=1e-9)
    with pytest.raises(twin.TwinError, match="not above the ground"):
        twin.ground_to_image(CAMERA, twin.Pose(0, 0.0, 0.0, 0.0, 0.0, 0.0))


def test_the_stand_in_is_a_render_set_by_the_contract_and_says_it_is_not_a_splat(render_set):
    camera, poses, more = twin.read_set(render_set)
    assert (camera.width, camera.height) == (640, 360) and len(poses) == 16 and more["stand_in"] is True and "not a splat" in more["source"]
    assert sorted(path.name for path in (render_set / "frames").iterdir())[:2] == ["000000.png", "000001.png"]
    first, last = (cv2.imread(str(render_set / "frames" / f"{index:06d}.png")) for index in (0, 15))
    assert first.shape == (360, 640, 3) and float(np.abs(first.astype(float) - last.astype(float)).mean()) > 3.0  # the picture moves with the camera
    assert first[:, :, 1].mean() > first[:, :, 2].mean()  # a green field


def test_targets_are_painted_where_the_pose_puts_them_and_the_truth_says_where(tmp_path, render_set):
    record = twin.composite(render_set, twin.read_targets(targets_file(tmp_path)), tmp_path / "with")
    assert len(record["frames"]) == 16 and json.loads((tmp_path / "with" / "truth.json").read_text())["targets"][0]["name"] == "drop_target"
    camera, poses, _more = twin.read_set(tmp_path / "with")
    seen = record["frames"][8]["targets"]["drop_target"]
    u, v = seen["center_px"]
    image = cv2.imread(str(tmp_path / "with" / "frames" / "000008.png"))
    before = cv2.imread(str(render_set / "frames" / "000008.png"))
    blue, green, red = (int(value) for value in image[int(round(v)), int(round(u))])
    assert red > 180 and green < 90 and blue < 90  # the bullseye's red centre is at the truth's pixel
    assert seen["size_px"] == pytest.approx(2.0 * 384.0 / 12.0) and seen["whole_in_view"] is True
    far = (int(round(v)) + 150) % 360, (int(round(u)) + 250) % 640
    assert np.array_equal(image[far], before[far])  # the field away from the targets is the render's own
    # The numbered marker: its truth is where the pose puts it, and what is painted there is a marker (dark and light, not the field's green).
    marker_truth = record["frames"][8]["targets"]["marker_3"]
    expected = twin.ground_to_image(camera, poses[8]) @ np.array([-3.0, 2.0, 1.0])
    assert marker_truth["center_px"] == pytest.approx([expected[0] / expected[2], expected[1] / expected[2]], abs=0.01)
    assert marker_truth["size_px"] == pytest.approx(0.61 * camera.fx / poses[8].up_m, rel=0.01)
    mu, mv, half = int(round(marker_truth["center_px"][0])), int(round(marker_truth["center_px"][1])), int(marker_truth["size_px"] // 2) - 1
    if marker_truth["whole_in_view"]:
        patch, patch_before = image[mv - half : mv + half, mu - half : mu + half].astype(float), before[mv - half : mv + half, mu - half : mu + half].astype(float)
        assert np.abs(patch - patch_before).mean() > 20.0 and patch.min() < 60 and patch.max() > 190  # painted over the field: the plate's black and white
    assert any(frame["targets"]["marker_3"]["whole_in_view"] for frame in record["frames"])
    # A target that is out of the picture is said to be.
    off = twin.composite(render_set, [{"name": "far", "kind": "bullseye", "east_m": 300.0, "north_m": 0.0, "size_m": 2.0, "digit": None}], tmp_path / "off")
    assert all(frame["targets"]["far"]["whole_in_view"] is False for frame in off["frames"])


def test_the_replay_finds_the_bullseye_and_gives_its_errors_and_is_marked_as_a_stand_in(tmp_path, render_set, capsys):
    twin.composite(render_set, twin.read_targets(targets_file(tmp_path)), tmp_path / "with")
    found = twin.replay(tmp_path / "with")
    assert found["target"] == "drop_target" and found["frames"] == 16 and found["in_view"] >= 12 and found["detection_rate"] == 1.0 and found["false_hits"] == 0
    assert found["pixel_error_p95"] < 2.0 and found["ground_error_p95_m"] < 0.06 and found["estimator_error_m"] < 0.03 and found["estimator_hits"] >= 5
    assert found["stand_in"] is True
    assert twin.main(["replay", str(tmp_path / "with")]) == 0
    out = capsys.readouterr().out
    assert "detected in" in out and "(100%)" in out and out.strip().splitlines()[-1].startswith("STAND-IN: these frames are a made-up field, not a splat of the test field")
    # The harness measures: a pose that is wrong by half a metre shows as half a metre on the ground.
    lines = (tmp_path / "with" / "poses.csv").read_text().splitlines()
    shifted = [lines[0]] + [",".join([row.split(",")[0], row.split(",")[1], f"{float(row.split(',')[2]) + 0.5:.4f}", *row.split(",")[3:]]) for row in lines[1:]]
    (tmp_path / "with" / "poses.csv").write_text("\n".join(shifted) + "\n")
    assert twin.replay(tmp_path / "with")["ground_error_median_m"] == pytest.approx(0.5, abs=0.05)


def test_the_command_makes_composites_and_replays_and_refuses_what_is_not_a_render_set(tmp_path, capsys):
    made = tmp_path / "set"
    assert twin.main(["standin", str(made), "--frames", "4"]) == 0 and twin.main(["composite", str(made), "--targets", str(targets_file(tmp_path)), "--out", str(tmp_path / "with")]) == 0
    assert "wrote 4 frames with 2 target(s)" in capsys.readouterr().out
    assert twin.main(["replay", str(made)]) == 2 and "has no truth.json: composite the targets into the render set first" in capsys.readouterr().err
    assert twin.main(["replay", str(tmp_path / "none")]) == 2 and "is not a render set" in capsys.readouterr().err
    (made / "frames" / "000002.png").rename(made / "frames" / "kept.png")
    assert twin.main(["composite", str(made), "--targets", str(targets_file(tmp_path)), "--out", str(tmp_path / "x")]) == 2 and "1 pose(s) have no frame (the first is 000002.png)" in capsys.readouterr().err
    for content, word in (({"targets": [{"name": "a", "kind": "tree", "east_m": 0, "north_m": 0, "size_m": 1}]}, "kind 'tree' is not bullseye or marker"), ({"targets": []}, "the targets need names"),
                          ({"targets": [{"name": "a", "kind": "marker", "east_m": 0, "north_m": 0, "size_m": 0.6}]}, "not a targets file")):  # fmt: skip
        with pytest.raises(twin.TwinError, match=word):
            twin.read_targets(targets_file(tmp_path, content))
    two = {"targets": [dict(TARGETS["targets"][0]), {**TARGETS["targets"][0], "name": "second", "east_m": -2.0}]}
    (made / "frames" / "kept.png").rename(made / "frames" / "000002.png")
    twin.composite(made, twin.read_targets(targets_file(tmp_path, two)), tmp_path / "two")
    with pytest.raises(twin.TwinError, match="follows one bullseye, and the set has 2"):
        twin.replay(tmp_path / "two")
    assert twin.replay(tmp_path / "two", "second")["target"] == "second"
    assert math.isfinite(twin.PIXEL_SIGMA_PX)


def test_the_stand_in_can_be_rendered_at_a_calibrated_modes_own_size_and_focal_length(tmp_path, capsys, monkeypatch):
    """The spec audit: "at our camera intrinsics" was only ever a made-up camera. The contract's camera can be one of the repo's calibrated modes."""
    camera = twin.camera_of_mode("sim_delivery", "sim_iris_downcam")
    assert (camera.width, camera.height) == (640, 360) and camera.fx == pytest.approx(389.0, abs=1.0) and camera.cx == pytest.approx(320.0, abs=2.0)
    assert twin.main(["standin", str(tmp_path / "set"), "--frames", "3", "--camera-mode", "sim_delivery", "--airframe", "sim_iris_downcam"]) == 0
    read, _poses, more = twin.read_set(tmp_path / "set")
    assert read == camera and more["stand_in"] is True
    capsys.readouterr()
    assert twin.main(["standin", str(tmp_path / "other"), "--camera-mode", "sim_delivery"]) != 0 and "go together" in capsys.readouterr().err
    with pytest.raises(twin.TwinError, match="cannot be loaded"):
        twin.camera_of_mode("no_such_mode", "sim_iris_downcam")
    # The aircraft's own 4K mode is not loaded at all until its airframe's mount has been measured.
    with pytest.raises(twin.TwinError, match="cannot be loaded.*not verified"):
        twin.camera_of_mode("imx519_4k", "argus_2027")
    # And a mode with a real lens (that mode's own terms) is refused: distortion is not in the contract, and it is not rendered as a pinhole.
    import types

    from drone_mission_core import frame_projection

    lens = types.SimpleNamespace(width=3840, height=2160, fx=2400.0, fy=2400.0, cx=1920.0, cy=1080.0, d=(-0.233303, 0.051830, 0.0, 0.0, 0.0))
    monkeypatch.setattr(frame_projection, "load_camera", lambda mode, airframe: (lens, None))
    with pytest.raises(twin.TwinError, match="has lens distortion"):
        twin.camera_of_mode("imx519_4k", "argus_2027")
    monkeypatch.setattr(frame_projection, "load_camera", lambda mode, airframe: (types.SimpleNamespace(**{**vars(lens), "d": (0.0,) * 5}), None))
    assert twin.camera_of_mode("imx519_4k", "argus_2027") == twin.Camera(3840, 2160, 2400.0, 2400.0, 1920.0, 1080.0)
