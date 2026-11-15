"""drone_utils/sdf_objects.py: the 2027 sim objects (marker, X mat, bucket, payload).

The scenario builder embeds these models in the world, the judges score
against where they are, and recovery needs the bucket to be a real body with
the mass it is given. No ROS needed: the builders only return SDF text.
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "drone_utils"))

from drone_utils import sdf_objects  # noqa: E402
from drone_utils.sdf_objects import (  # noqa: E402
    build_bucket_sdf,
    build_bw_target_sdf,
    build_marker_sdf,
    build_payload_sdf,
    build_red_white_target_sdf,
    build_x_mat_sdf,
)

SEGMENTS = {0: 6, 1: 2, 2: 5, 3: 5, 4: 4, 5: 5, 6: 6, 7: 3, 8: 7, 9: 6}


def model(sdf: str) -> ET.Element:
    root = ET.fromstring(sdf)
    assert root.tag == "sdf"
    found = root.find("model")
    assert found is not None
    return found


def pose(element: ET.Element) -> list[float]:
    text = element.findtext("pose")
    return [float(v) for v in text.split()] if text else [0.0] * 6


def box(link: ET.Element, kind: str = "visual") -> list[float]:
    return [float(v) for v in link.find(f"{kind}/geometry/box/size").text.split()]


def links(element: ET.Element, prefix: str) -> list[ET.Element]:
    return [link for link in element.findall("link") if link.get("name").startswith(prefix)]


# --- marker -------------------------------------------------------------------


@pytest.mark.parametrize("size", [0.6, 1.2])
def test_marker_is_a_static_plate_of_the_given_size(size):
    marker = model(build_marker_sdf("marker_3", size=size, digit=3))
    assert marker.get("name") == "marker_3"
    assert marker.findtext("static") == "true"
    base = marker.find("link[@name='base']")
    assert box(base)[:2] == pytest.approx([size, size])
    assert box(base, "collision")[:2] == pytest.approx([size, size])
    assert len(marker.findall("link/collision")) == 1  # only the plate collides


@pytest.mark.parametrize("digit", range(10))
def test_marker_digit_is_centred_on_the_plate(digit):
    marker = model(build_marker_sdf("m", size=0.6, digit=digit))
    segments = links(marker, "seg_")
    assert len(segments) == SEGMENTS[digit]
    # The outline of the lit segments: centred left to right for every digit
    # that lights both sides, and the digit's own box is centred on the plate.
    field = marker.find("link[@name='digit_field']")
    assert pose(field)[:2] == pytest.approx([0.0, 0.0])
    half_w, half_h = box(field)[0] / 2, box(field)[1] / 2
    for segment in segments:
        x, y = pose(segment)[:2]
        w, h = box(segment)[:2]
        assert abs(x) + w / 2 <= half_w + 1e-9 and abs(y) + h / 2 <= half_h + 1e-9
    if digit == 8:
        xs = [pose(s)[0] for s in segments]
        ys = [pose(s)[1] for s in segments]
        assert min(xs) == pytest.approx(-max(xs)) and min(ys) == pytest.approx(-max(ys))


def test_marker_has_the_black_x_triangles_under_the_digit_field():
    marker = model(build_marker_sdf("m", size=0.6, digit=5))
    strips = links(marker, "top_strip_") + links(marker, "bottom_strip_")
    assert len(strips) >= 10
    field_z = pose(marker.find("link[@name='digit_field']"))[2]
    assert all(pose(strip)[2] < field_z for strip in strips)
    assert all(pose(segment)[2] > field_z for segment in links(marker, "seg_"))
    assert links(marker, "underline") == []  # the 2027 design has no underline


def test_marker_without_a_digit_is_a_boundary_marker():
    marker = model(build_marker_sdf("boundary_1", size=1.2, digit=None))
    assert links(marker, "seg_") == []
    assert marker.find("link[@name='digit_field']") is None


def test_marker_refuses_a_bad_digit_or_size():
    with pytest.raises(ValueError):
        build_marker_sdf("m", size=0.6, digit=12)
    with pytest.raises(ValueError):
        build_marker_sdf("m", size=0.0, digit=1)


# --- X mat --------------------------------------------------------------------


def test_x_mat_is_a_white_mat_with_two_green_diagonal_arms():
    mat = model(build_x_mat_sdf("x_mat", size=5.0, arm_width=0.5))
    assert mat.findtext("static") == "true"
    assert box(mat.find("link[@name='base']"))[:2] == pytest.approx([5.0, 5.0])
    arms = links(mat, "arm_")
    assert len(arms) == 2
    yaws = sorted(pose(arm)[5] for arm in arms)
    assert yaws == pytest.approx([-math.pi / 4, math.pi / 4])
    for arm in arms:
        length, width = box(arm)[:2]
        assert width == pytest.approx(0.5)
        assert length == pytest.approx(2 * (2.5 * math.sqrt(2) - 0.25), abs=1e-4)  # stops 0.25 m short of the corners
        r, g, b = (float(v) for v in arm.findtext("visual/material/diffuse").split()[:3])
        assert g > 0.5 and r < 0.1 and b < 0.1


# --- bucket -------------------------------------------------------------------


def test_bucket_is_a_dynamic_body_with_the_given_mass():
    bucket = model(build_bucket_sdf("bucket", mass_kg=1.5))
    assert bucket.findtext("static") == "false"
    assert float(bucket.findtext("link/inertial/mass")) == pytest.approx(1.5)
    inertia = bucket.find("link/inertial/inertia")
    # A solid cylinder of that mass: izz = m r^2 / 2.
    assert float(inertia.findtext("izz")) == pytest.approx(1.5 * 0.095**2 / 2)
    assert float(inertia.findtext("ixx")) == pytest.approx(float(inertia.findtext("iyy")))
    assert float(inertia.findtext("ixx")) > 0.0


@pytest.mark.parametrize("mass", [1.0, 2.0])
def test_bucket_mass_is_a_parameter(mass):
    assert float(model(build_bucket_sdf("b", mass_kg=mass)).findtext("link/inertial/mass")) == pytest.approx(mass)


def test_bucket_stands_on_the_ground_with_its_handle_upright_above_the_rim():
    bucket = model(build_bucket_sdf("bucket", mass_kg=1.5, diameter=0.19, height=0.20, handle_rise=0.12))
    link = bucket.find("link")
    body = link.find("collision[@name='body']")
    radius = float(body.findtext("geometry/cylinder/radius"))
    length = float(body.findtext("geometry/cylinder/length"))
    assert (radius, length) == pytest.approx((0.095, 0.20))
    # Model origin on the ground: the body's centre is half its height up.
    assert pose(link)[2] + pose(body)[2] == pytest.approx(0.10)
    bar = link.find("collision[@name='handle_bar']")
    posts = [link.find(f"collision[@name='handle_post_{side}']") for side in ("left", "right")]
    assert bar is not None and all(post is not None for post in posts)
    bar_z = pose(link)[2] + pose(bar)[2]
    assert bar_z == pytest.approx(0.20 + 0.12, abs=0.01)  # the bar is above the rim: a hook can pass under it
    for post in posts:
        assert abs(pose(post)[0]) == pytest.approx(0.095, abs=0.01)  # at the rim, either side
        assert pose(post)[3:5] == pytest.approx([0.0, 0.0])  # upright
    assert link.find("visual[@name='handle_bar']") is not None


def test_bucket_refuses_a_mass_that_is_not_positive():
    with pytest.raises(ValueError):
        build_bucket_sdf("b", mass_kg=0.0)


# --- payload ------------------------------------------------------------------


def test_payload_is_a_dynamic_box_with_the_given_mass():
    payload = model(build_payload_sdf("payload", mass_kg=0.5, size=(0.15, 0.10, 0.08)))
    assert payload.findtext("static") == "false"
    link = payload.find("link")
    assert float(link.findtext("inertial/mass")) == pytest.approx(0.5)
    assert box(link, "collision") == pytest.approx([0.15, 0.10, 0.08])
    assert pose(link)[2] == pytest.approx(0.04)  # resting on the ground
    # A solid box: ixx = m (y^2 + z^2) / 12.
    assert float(link.findtext("inertial/inertia/ixx")) == pytest.approx(0.5 * (0.10**2 + 0.08**2) / 12)


# --- the 2026 builders are as they were ---------------------------------------


def test_2026_builders_are_unchanged():
    """Flags off flies the 2026 sim: its target models must not move under us."""
    import hashlib

    def digest(text: str) -> str:
        return hashlib.sha256(text.encode()).hexdigest()[:16]

    assert digest(build_red_white_target_sdf("delivery_target", size=5.0)) == "75a9d5dfb4ab67f8"
    assert digest(build_bw_target_sdf("gcp", size=0.61, digit=7)) == "34a0b71c3aa0fec6"
    assert callable(sdf_objects.spawn_sdf)
