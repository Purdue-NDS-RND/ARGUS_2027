"""tools/ring_fit_try.py: the near-field ring fit on a photo or a saved frame (CEN-13's bench check). No ROS."""

import re
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools" / "ring_fit_try.py"
sys.path.insert(0, str(ROOT / "tools"))

import synth_targets as synth  # noqa: E402


def run(*args):
    return subprocess.run([sys.executable, str(TOOL), *map(str, args)], capture_output=True, text=True)


def photo(tmp_path, name="view.png", height_m=2.4, offset_px=(150.0, -40.0), tilt_deg=(0.0, 0.0)):
    options = synth.RenderOptions(width=640, height=360, mpp=height_m / 389.16536, altitude_m=height_m, offset_px=offset_px, tilt_deg=tilt_deg, blur_sigma_px=1.0, noise_sigma=4.0, supersample=2)
    image, truth = synth.render(synth.Bullseye(), options, seed=2)
    path = tmp_path / name
    cv2.imwrite(str(path), image)
    return path, truth["center_px"]


def test_a_photo_of_the_target_gives_its_centre_and_a_marked_copy(tmp_path):
    path, truth = photo(tmp_path)
    done = run("--image", path)
    assert done.returncode == 0, done.stderr
    found = re.search(r"centre at \(([\d.]+), ([\d.]+)\) px", done.stdout)
    assert found and abs(float(found.group(1)) - truth[0]) < 1.5 and abs(float(found.group(2)) - truth[1]) < 1.5
    assert "rings" in done.stdout and "2026 detector" in done.stdout  # the box detector's answer beside it, to compare
    marked = cv2.imread(str(tmp_path / "view_ring_fit.png"))
    assert marked is not None and marked.shape == (360, 640, 3)
    assert not np.array_equal(marked, cv2.imread(str(path)))


def test_a_known_centre_is_compared_with_the_fit(tmp_path):
    path, truth = photo(tmp_path)
    near = run("--image", path, "--expect", f"{truth[0]:.1f},{truth[1]:.1f}")
    assert near.returncode == 0 and "from the expected centre: PASS" in near.stdout
    far = run("--image", path, "--expect", f"{truth[0] + 20:.1f},{truth[1]:.1f}")
    assert far.returncode == 1 and "FAIL" in far.stdout


def test_a_tilted_camera_is_levelled_when_its_tilt_and_focal_length_are_given(tmp_path):
    path, truth = photo(tmp_path, height_m=1.5, tilt_deg=(0.0, -8.0))
    levelled = run("--image", path, "--tilt", "0,-8", "--focal-px", "389.16536", "--expect", f"{truth[0]:.1f},{truth[1]:.1f}")
    assert levelled.returncode == 0, levelled.stdout + levelled.stderr
    assert "levelled" in levelled.stdout


def test_the_time_one_fit_takes_is_printed_on_request(tmp_path):
    path, _truth = photo(tmp_path)
    done = run("--image", path, "--time", "5")
    assert done.returncode == 0 and re.search(r"one fit of this 640 x 360 picture takes [\d.]+ ms \(\d+ a second\)", done.stdout)


def test_a_picture_with_no_target_says_so(tmp_path):
    plain = tmp_path / "grass.png"
    cv2.imwrite(str(plain), np.full((360, 640, 3), (60, 140, 60), np.uint8))
    done = run("--image", plain)
    assert done.returncode == 1 and "no rings found" in done.stdout
    assert run("--image", tmp_path / "missing.png").returncode == 2
