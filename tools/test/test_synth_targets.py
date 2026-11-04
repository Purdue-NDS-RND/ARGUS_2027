import json
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "tools"), str(ROOT / "src" / "drone_utils")]

import synth_targets as st  # noqa: E402
from drone_utils.sdf_objects import (  # noqa: E402
    build_bw_target_sdf,
    build_red_white_target_sdf,
)

GREY = (128, 128, 128)
BLUE = (0, 0, 255)


def weighted_centroid(weight: np.ndarray) -> tuple[float, float]:
    vs, us = np.indices(weight.shape)
    total = weight.sum()
    return float((us * weight).sum() / total), float((vs * weight).sum() / total)


def measured_center(target, image: np.ndarray) -> tuple[float, float]:
    """Centre from pixels alone, using a colour weight that is linear in coverage and
    whose region is point-symmetric about the true centre."""
    b, g, r = (image[..., i].astype(float) for i in range(3))
    if isinstance(target, st.Bullseye):  # red rings on a grey background
        weight = (r - g) / (st.SDF_RED[0] - st.SDF_RED[1])
    elif isinstance(target, st.XMat):  # green arms on a grey background
        weight = (g - r) / target.color[1]
    else:  # the whole plate on a blue background: white and black both give b == r
        weight = 1.0 - (b - r) / 255.0
    return weighted_centroid(np.clip(weight, 0.0, None))


def sdf_links(sdf: str) -> dict:
    links = {}
    for link in ET.fromstring(sdf).iter("link"):
        pose = [float(v) for v in link.findtext("pose").split()]
        geometry = link.find("visual/geometry")
        ambient = [float(v) for v in link.findtext("visual/material/ambient").split()]
        box = geometry.find("box")
        links[link.get("name")] = {
            "x": pose[0],
            "y": pose[1],
            "size": [float(v) for v in box.findtext("size").split()] if box is not None else None,
            "radius": float(geometry.findtext("cylinder/radius")) if box is None else None,
            "rgb": st._sdf_rgb(*ambient[:3]),
        }
    return links


class CenterAccuracyTest(unittest.TestCase):
    """No tilt and no blur: the rendered centre is within 0.5 px of the ground truth."""

    CASES = [
        (st.Bullseye(), GREY, 0.012),
        (st.Bullseye(num_rings=5), GREY, 0.02),
        (st.XMat(), GREY, 0.012),
        (st.XMat(shape="circle", arm_width_m=0.3, outline=True), GREY, 0.015),
        (st.NumberedMarker(digit=4), BLUE, 0.0015),
        (st.NumberedMarker(digit=7), BLUE, 0.003),
    ]

    def test_center_within_half_pixel(self) -> None:
        rng = np.random.default_rng(42)
        for target, background, mpp in self.CASES:
            for _ in range(3):
                options = st.RenderOptions(
                    width=800,
                    height=600,
                    mpp=mpp,
                    rotation_deg=float(rng.uniform(0, 360)),
                    offset_px=(float(rng.uniform(-40, 40)), float(rng.uniform(-30, 30))),
                    background_color=background,
                )
                image, truth = st.render(target, options, seed=1)
                u, v = measured_center(target, image)
                error = math.hypot(u - truth["center_px"][0], v - truth["center_px"][1])
                with self.subTest(target=target, rotation=options.rotation_deg):
                    self.assertLess(error, 0.5)

    def test_center_and_corners_follow_offset_and_scale(self) -> None:
        options = st.RenderOptions(width=640, height=480, mpp=0.02, offset_px=(12.25, -7.5))
        _, truth = st.render(st.Bullseye(), options)
        self.assertAlmostEqual(truth["center_px"][0], 319.5 + 12.25)
        self.assertAlmostEqual(truth["center_px"][1], 239.5 - 7.5)
        corners = np.array(truth["corners_px"])
        side = np.linalg.norm(corners[1] - corners[0])
        self.assertAlmostEqual(side, 5.0 / 0.02, places=6)
        self.assertLess(corners[0][0], corners[1][0])  # TL is left of TR
        self.assertLess(corners[0][1], corners[3][1])  # TL is above BL

    def test_tilt_keeps_center_in_place(self) -> None:
        options = st.RenderOptions(mpp=0.01, offset_px=(30.0, 20.0), tilt_deg=(15.0, -15.0))
        _, truth = st.render(st.XMat(), options)
        self.assertAlmostEqual(truth["center_px"][0], 639.5 + 30.0)
        self.assertAlmostEqual(truth["center_px"][1], 359.5 + 20.0)


class DeterminismTest(unittest.TestCase):
    OPTIONS = st.RenderOptions(
        width=320,
        height=240,
        mpp=0.025,
        rotation_deg=33.0,
        tilt_deg=(8.0, -5.0),
        blur_sigma_px=1.2,
        noise_sigma=6.0,
        brightness=0.9,
        background="noise",
    )

    def test_same_seed_gives_identical_images(self) -> None:
        for target in (st.Bullseye(), st.XMat(bucket=True), st.NumberedMarker(digit=3)):
            first, truth_a = st.render(target, self.OPTIONS, seed=7)
            second, truth_b = st.render(target, self.OPTIONS, seed=7)
            with self.subTest(target=target.name):
                np.testing.assert_array_equal(first, second)
                self.assertEqual(truth_a, truth_b)

    def test_different_seed_changes_image(self) -> None:
        first, _ = st.render(st.Bullseye(), self.OPTIONS, seed=7)
        second, _ = st.render(st.Bullseye(), self.OPTIONS, seed=8)
        self.assertFalse(np.array_equal(first, second))

    def test_cli_same_seed_gives_identical_files(self) -> None:
        with TemporaryDirectory() as tmp:
            runs = []
            for name in ("a", "b"):
                out = Path(tmp) / name
                args = ["--n", "4", "--seed", "5", "--out", str(out), "--width", "320", "--height", "240"]
                st.main(args)
                runs.append(out)
            files_a = sorted(p.name for p in runs[0].iterdir())
            self.assertEqual(files_a, sorted(p.name for p in runs[1].iterdir()))
            self.assertEqual(len(files_a), 5)
            for name in files_a:
                if name.endswith(".png"):
                    self.assertEqual((runs[0] / name).read_bytes(), (runs[1] / name).read_bytes(), name)
            manifest, other = (json.loads((run / "ground_truth.json").read_text()) for run in runs)
            self.assertEqual(manifest["samples"], other["samples"])
            self.assertEqual(len(manifest["samples"]), 4)
            for sample in manifest["samples"]:
                self.assertTrue((runs[0] / sample["file"]).exists())
                self.assertEqual(len(sample["center_px"]), 2)
                self.assertEqual(len(sample["corners_px"]), 4)
                self.assertIn("rotation_deg", sample)
                self.assertEqual(sample["digit"] is not None, sample["target"] == "marker")


class OptionsTest(unittest.TestCase):
    def test_tilt_over_15_degrees_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            st.render(st.Bullseye(), st.RenderOptions(tilt_deg=(15.5, 0.0)))
        with self.assertRaises(SystemExit):
            st.main(["--n", "1", "--out", "unused", "--max-tilt", "20"])

    def test_image_background(self) -> None:
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "bg.png"
            ramp = np.tile(np.arange(256, dtype=np.uint8), (100, 1))
            cv2.imwrite(str(path), cv2.merge([ramp, ramp, ramp]))
            options = st.RenderOptions(width=320, height=240, mpp=0.05, background="image", background_image=str(path))
            image, _ = st.render(st.Bullseye(), options, seed=2)
            self.assertEqual(image.shape, (240, 320, 3))
            self.assertGreater(int(image[0, -1, 0]), int(image[0, 0, 0]))  # the ramp survived

    def test_ground_truth_reports_digit_and_rotation(self) -> None:
        _, truth = st.render(st.NumberedMarker(digit=9), st.RenderOptions(mpp=0.002, rotation_deg=-30.0))
        self.assertEqual(truth["digit"], 9)
        self.assertAlmostEqual(truth["rotation_deg"], 330.0)

    def test_bad_target_parameters_are_rejected(self) -> None:
        for make in (lambda: st.NumberedMarker(digit=10), lambda: st.XMat(shape="hexagon")):
            with self.assertRaises(ValueError):
                make()

    def test_x_mat_options_change_the_image(self) -> None:
        options = st.RenderOptions(width=320, height=240, mpp=0.025)
        plain, _ = st.render(st.XMat(), options)
        center = (120, 160)
        bucket, _ = st.render(st.XMat(bucket=True, bucket_color=(255, 0, 0)), options)
        np.testing.assert_array_equal(bucket[center], (0, 0, 255))
        np.testing.assert_array_equal(plain[center], (0, 160, 0))
        outlined, _ = st.render(st.XMat(outline=True, outline_width_m=0.2), options)
        edge = (120, 160 + int(2.45 / 0.025))  # just inside the right edge
        np.testing.assert_array_equal(outlined[edge], (0, 0, 0))
        np.testing.assert_array_equal(plain[edge], st.SDF_WHITE)


class MatchesSdfTest(unittest.TestCase):
    """The synthetic targets are built from the same shapes as the Gazebo SDF models."""

    def test_bullseye_matches_build_red_white_target_sdf(self) -> None:
        for size, num_rings in ((5.0, 8), (3.0, 5)):
            links = sdf_links(build_red_white_target_sdf("t", size=size, num_rings=num_rings))
            target = st.Bullseye(size_m=size, num_rings=num_rings)
            sdf_discs = [links[f"ring_{i}"] for i in range(num_rings)] + [links["center"]]
            with self.subTest(size=size, num_rings=num_rings):
                self.assertEqual(links["base"]["size"][:2], [size, size])
                self.assertEqual(links["base"]["rgb"], st.SDF_WHITE)
                self.assertEqual(len(sdf_discs), len(target.discs()))
                for sdf, (radius, rgb) in zip(sdf_discs, target.discs()):
                    self.assertAlmostEqual(sdf["radius"], radius, delta=5e-5)
                    self.assertEqual(sdf["rgb"], rgb)

    def test_bullseye_default_ring_count_matches_sdf(self) -> None:
        import inspect

        default = inspect.signature(build_red_white_target_sdf).parameters["num_rings"].default
        self.assertEqual(st.Bullseye().num_rings, default)

    def test_marker_matches_build_bw_target_sdf(self) -> None:
        for digit in range(10):
            links = sdf_links(build_bw_target_sdf("t", digit=digit))
            boxes = [
                (round(l["x"], 4), round(l["y"], 4), round(l["size"][0], 4), round(l["size"][1], 4), l["rgb"])
                for name, l in links.items()
                if name != "base"
            ]
            ours = [
                (round(r.cx, 4), round(r.cy, 4), round(r.w, 4), round(r.h, 4), r.rgb)
                for r in st.NumberedMarker(digit=digit).rects()
            ]
            with self.subTest(digit=digit):
                self.assertEqual(links["base"]["size"][:2], [0.61, 0.61])
                self.assertEqual(links["base"]["rgb"], st.SDF_WHITE)
                self.assertEqual(sorted(boxes), sorted(ours))


if __name__ == "__main__":
    unittest.main()
