#!/usr/bin/env python3
"""Render seeded, top-down synthetic images of the competition targets, with ground truth.

No ROS needed (numpy and OpenCV only). Three targets, drawn to scale in metres:

  bullseye  red-and-white rings on a 5 m white square, as build_red_white_target_sdf
  x         green X on a 5 m white mat (square or circle), optional bucket over the centre
  marker    0.61 m black-and-white plate with a 7-segment digit and underline, as
            build_bw_target_sdf

Each image comes with exact ground truth: the sub-pixel centre, four corners, the digit
and the rotation. Pixel coordinates follow OpenCV: pixel (0, 0) is centred on (0.0, 0.0).
Images are BGR uint8, like cv2.imread. Colour parameters are RGB.

Geometry: the target lies in a plane with X right and Y up (the SDF's x and y). A pinhole
camera looks straight down from altitude_m; its focal length is altitude_m / mpp, so mpp is
exact at the target centre when there is no tilt. Tilt rotates the target plane about its
own centre, so the centre stays where rotation and offset put it. Each pixel averages
supersample x supersample samples, which gives anti-aliased edges whose centroid is exact.

Examples:
  python3 tools/synth_targets.py --n 50 --out /tmp/synth --seed 1
  python3 tools/synth_targets.py --n 12 --out /tmp/synth --sheet sheet.png
  python3 tools/synth_targets.py --n 20 --out /tmp/synth --targets marker --max-tilt 0
  python3 tools/synth_targets.py --n 20 --out /tmp/synth --background-image field.jpg
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Optional, Union

import cv2
import numpy as np

RGB = tuple[int, int, int]

MAX_TILT_DEG = 15.0


def _sdf_rgb(r: float, g: float, b: float) -> RGB:
    """8-bit colour for an SDF material's 0..1 ambient/diffuse values."""
    return (round(r * 255), round(g * 255), round(b * 255))


# Materials used by drone_utils/sdf_objects.py.
SDF_RED = _sdf_rgb(0.85, 0.05, 0.05)
SDF_WHITE = _sdf_rgb(0.95, 0.95, 0.95)
SDF_PLATE_BLACK = _sdf_rgb(0.05, 0.05, 0.05)
SDF_DIGIT_BLACK = _sdf_rgb(0.02, 0.02, 0.02)

# Seven-segment layout, segments A..G, as in sdf_objects._build_digit_sdf.
DIGIT_SEGMENTS = {
    0: (1, 1, 1, 1, 1, 1, 0),
    1: (0, 1, 1, 0, 0, 0, 0),
    2: (1, 1, 0, 1, 1, 0, 1),
    3: (1, 1, 1, 1, 0, 0, 1),
    4: (0, 1, 1, 0, 0, 1, 1),
    5: (1, 0, 1, 1, 0, 1, 1),
    6: (1, 0, 1, 1, 1, 1, 1),
    7: (1, 1, 1, 0, 0, 0, 0),
    8: (1, 1, 1, 1, 1, 1, 1),
    9: (1, 1, 1, 1, 0, 1, 1),
}


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in target metres: centre, size, colour."""

    cx: float
    cy: float
    w: float
    h: float
    rgb: RGB

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return (np.abs(x - self.cx) <= self.w / 2) & (np.abs(y - self.cy) <= self.h / 2)


def _paint(rgb: np.ndarray, mask: np.ndarray, color: RGB) -> None:
    rgb[mask] = color


# --------------------------------------------------------------------------- targets


@dataclass(frozen=True)
class Bullseye:
    """build_red_white_target_sdf: white square base, then num_rings discs, red outermost
    and alternating inward, each one band narrower, topped by a red centre dot.
    The band width is radius / (num_rings + 1)."""

    size_m: float = 5.0
    num_rings: int = 8

    name = "bullseye"
    digit = None

    def __post_init__(self) -> None:
        if self.size_m <= 0 or self.num_rings < 0:
            raise ValueError("bullseye needs size_m > 0 and num_rings >= 0")

    @property
    def half_extent_m(self) -> float:
        return self.size_m / 2

    def discs(self) -> list[tuple[float, RGB]]:
        """(radius_m, rgb) from bottom layer to top, centre dot last."""
        radius = self.size_m / 2
        band = radius / (self.num_rings + 1)
        discs = [
            (radius - i * band, SDF_RED if i % 2 == 0 else SDF_WHITE)
            for i in range(self.num_rings)
        ]
        discs.append((band, SDF_RED))
        return discs

    def corners_m(self) -> np.ndarray:
        return _square_corners(self.size_m / 2)

    def paint(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        half = self.size_m / 2
        inside = (np.abs(x) <= half) & (np.abs(y) <= half)
        rgb = np.empty(x.shape + (3,), np.float32)
        rgb[:] = SDF_WHITE
        r = np.hypot(x, y)
        for radius, color in self.discs():
            _paint(rgb, r <= radius, color)
        return rgb, inside

    def params(self) -> dict:
        return {"size_m": self.size_m, "num_rings": self.num_rings}


@dataclass(frozen=True)
class XMat:
    """Green X on a white mat. The arms run along the diagonals and stop arm_inset_m
    short of the mat corners (square) or rim (circle); they are clipped to the mat."""

    size_m: float = 5.0
    shape: str = "square"
    arm_width_m: float = 0.5
    arm_inset_m: float = 0.25
    color: RGB = (0, 160, 0)
    mat_color: RGB = SDF_WHITE
    outline: bool = False
    outline_width_m: float = 0.05
    outline_color: RGB = (0, 0, 0)
    bucket: bool = False
    bucket_diameter_m: float = 0.2
    bucket_color: RGB = (235, 110, 20)

    name = "x"
    digit = None

    def __post_init__(self) -> None:
        if self.shape not in ("square", "circle"):
            raise ValueError(f"mat shape must be 'square' or 'circle', not {self.shape!r}")
        if self.size_m <= 0 or self.arm_width_m <= 0 or self.arm_half_length_m <= 0:
            raise ValueError("x mat needs positive size, arm width and arm length")

    @property
    def half_extent_m(self) -> float:
        return self.size_m / 2

    @property
    def arm_half_length_m(self) -> float:
        """Centre to arm tip, along the diagonal."""
        half = self.size_m / 2
        reach = half * math.sqrt(2) if self.shape == "square" else half
        return reach - self.arm_inset_m

    def corners_m(self) -> np.ndarray:
        """Square: the mat corners. Circle: where the diagonals meet the rim."""
        half = self.size_m / 2
        if self.shape == "circle":
            half /= math.sqrt(2)
        return _square_corners(half)

    def paint(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        half = self.size_m / 2
        r = np.hypot(x, y)
        if self.shape == "square":
            edge_dist = half - np.maximum(np.abs(x), np.abs(y))
        else:
            edge_dist = half - r
        inside = edge_dist >= 0
        rgb = np.empty(x.shape + (3,), np.float32)
        rgb[:] = self.mat_color
        for dx, dy in ((1.0, 1.0), (1.0, -1.0)):
            dx, dy = dx / math.sqrt(2), dy / math.sqrt(2)
            along = x * dx + y * dy
            across = y * dx - x * dy
            arm = (np.abs(along) <= self.arm_half_length_m) & (np.abs(across) <= self.arm_width_m / 2)
            _paint(rgb, arm, self.color)
        if self.outline:
            _paint(rgb, edge_dist <= self.outline_width_m, self.outline_color)
        if self.bucket:
            _paint(rgb, r <= self.bucket_diameter_m / 2, self.bucket_color)
        return rgb, inside

    def params(self) -> dict:
        return {k: list(v) if isinstance(v, tuple) else v for k, v in dataclasses.asdict(self).items()}


@dataclass(frozen=True)
class NumberedMarker:
    """build_bw_target_sdf: white plate, black top and bottom triangles built from ten
    stepped strips, a 7-segment digit in the right-hand white triangle, and an underline."""

    size_m: float = 0.61
    digit: int = 1

    name = "marker"

    def __post_init__(self) -> None:
        if self.digit not in DIGIT_SEGMENTS:
            raise ValueError(f"digit must be 0-9, not {self.digit!r}")
        if self.size_m <= 0:
            raise ValueError("marker needs size_m > 0")

    @property
    def half_extent_m(self) -> float:
        return self.size_m / 2

    def rects(self) -> list[Rect]:
        """Every box laid over the white plate, bottom layer first, in SDF order."""
        size, half = self.size_m, self.size_m / 2
        num_strips = 10
        strip_h = half / num_strips
        rects = []
        for sign in (1, -1):
            for i in range(num_strips):
                y = (i + 0.5) * strip_h
                progress = 1.0 - y / half
                width = size * (1.0 - progress * 0.95)
                rects.append(Rect(0.0, sign * y, width, strip_h, SDF_PLATE_BLACK))

        cx, cy, s = half * 0.65, -half * 0.05, size * 0.42
        seg_len, seg_w, v_len = s * 0.4, s * 0.12, s * 0.35
        top_y, mid_y, bot_y = cy + s * 0.35, cy, cy - s * 0.35
        left_x, right_x = cx - seg_len * 0.4, cx + seg_len * 0.4
        segments = (
            (cx, top_y, seg_len, seg_w),  # A
            (right_x, (top_y + mid_y) / 2, seg_w, v_len),  # B
            (right_x, (mid_y + bot_y) / 2, seg_w, v_len),  # C
            (cx, bot_y, seg_len, seg_w),  # D
            (left_x, (mid_y + bot_y) / 2, seg_w, v_len),  # E
            (left_x, (top_y + mid_y) / 2, seg_w, v_len),  # F
            (cx, mid_y, seg_len, seg_w),  # G
        )
        for on, (x, y, w, h) in zip(DIGIT_SEGMENTS[self.digit], segments):
            if on:
                rects.append(Rect(x, y, w, h, SDF_DIGIT_BLACK))
        rects.append(Rect(cx, cy - s * 0.75, size * 0.18, size * 0.025, SDF_DIGIT_BLACK))
        return rects

    def corners_m(self) -> np.ndarray:
        return _square_corners(self.size_m / 2)

    def paint(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        half = self.size_m / 2
        inside = (np.abs(x) <= half) & (np.abs(y) <= half)
        rgb = np.empty(x.shape + (3,), np.float32)
        rgb[:] = SDF_WHITE
        for rect in self.rects():
            _paint(rgb, rect.contains(x, y), rect.rgb)
        return rgb, inside

    def params(self) -> dict:
        return {"size_m": self.size_m, "digit": self.digit}


Target = Union[Bullseye, XMat, NumberedMarker]


def _square_corners(half: float) -> np.ndarray:
    """Corners in target metres, in image order at zero rotation: TL, TR, BR, BL."""
    return np.array([[-half, half], [half, half], [half, -half], [-half, -half]], float)


# --------------------------------------------------------------------------- rendering


@dataclass(frozen=True)
class RenderOptions:
    width: int = 1280
    height: int = 720
    mpp: float = 0.01  # metres per pixel at the target centre
    altitude_m: float = 15.0
    rotation_deg: float = 0.0  # counter-clockwise as seen in the image
    offset_px: tuple[float, float] = (0.0, 0.0)  # target centre from image centre; +x right, +y down
    tilt_deg: tuple[float, float] = (0.0, 0.0)  # target plane rotation about the image x and y axes
    blur_sigma_px: float = 0.0
    noise_sigma: float = 0.0  # Gaussian noise, in 8-bit levels
    brightness: float = 1.0  # gain applied after blur
    background: str = "plain"  # plain, noise or image
    background_color: RGB = (95, 120, 85)  # plain colour, or base colour of the noise texture
    background_image: Optional[str] = None
    supersample: int = 4

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.mpp <= 0 or self.altitude_m <= 0:
            raise ValueError("width, height, mpp and altitude_m must be positive")
        if any(abs(t) > MAX_TILT_DEG for t in self.tilt_deg):
            raise ValueError(f"tilt must be within ±{MAX_TILT_DEG}°, got {self.tilt_deg}")
        if self.background not in ("plain", "noise", "image"):
            raise ValueError(f"unknown background {self.background!r}")
        if self.background == "image" and not self.background_image:
            raise ValueError("background 'image' needs background_image")
        if self.supersample < 1 or self.blur_sigma_px < 0 or self.noise_sigma < 0:
            raise ValueError("supersample must be >= 1; blur and noise must be >= 0")


def homography(options: RenderOptions) -> np.ndarray:
    """3x3 matrix taking target-plane metres (X, Y, 1) to image pixels (u, v, 1)."""
    rot = math.radians(options.rotation_deg)
    tx, ty = (math.radians(t) for t in options.tilt_deg)
    rz = np.array([[math.cos(rot), -math.sin(rot), 0], [math.sin(rot), math.cos(rot), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(tx), -math.sin(tx)], [0, math.sin(tx), math.cos(tx)]])
    ry = np.array([[math.cos(ty), 0, math.sin(ty)], [0, 1, 0], [-math.sin(ty), 0, math.cos(ty)]])
    plane = ry @ rx @ rz  # target axes in the world frame (X right, Y up, Z towards camera)

    offset_m = np.array([options.offset_px[0] * options.mpp, -options.offset_px[1] * options.mpp, 0.0])
    world_to_cam = np.diag([1.0, -1.0, -1.0])  # camera x right, y down, z down at the ground
    camera_pos = np.array([0.0, 0.0, options.altitude_m])
    a = np.column_stack(
        [world_to_cam @ plane[:, 0], world_to_cam @ plane[:, 1], world_to_cam @ (offset_m - camera_pos)]
    )
    f = options.altitude_m / options.mpp
    k = np.array(
        [[f, 0, (options.width - 1) / 2], [0, f, (options.height - 1) / 2], [0, 0, 1]]
    )
    return k @ a


def project(h: np.ndarray, points_m: np.ndarray) -> np.ndarray:
    pts = np.column_stack([points_m, np.ones(len(points_m))]) @ h.T
    return pts[:, :2] / pts[:, 2:3]


def _background(options: RenderOptions, rng: np.random.Generator) -> np.ndarray:
    """Float BGR background, shape (height, width, 3)."""
    w, h = options.width, options.height
    base = np.array(options.background_color[::-1], np.float32)
    if options.background == "plain":
        return np.broadcast_to(base, (h, w, 3)).copy()
    if options.background == "noise":
        field_ = np.zeros((h, w), np.float32)
        for octave, cell in enumerate((160, 48, 12, 3)):
            gw, gh = w // cell + 2, h // cell + 2
            grid = rng.standard_normal((gh, gw)).astype(np.float32)
            up = cv2.resize(grid, (gw * cell, gh * cell), interpolation=cv2.INTER_CUBIC)
            field_ += up[:h, :w] * 0.6**octave
        field_ /= field_.std() + 1e-6
        tint = 1.0 + 0.15 * rng.standard_normal(3).astype(np.float32)
        return base + 20.0 * field_[..., None] * tint
    image = cv2.imread(str(options.background_image), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"could not read background image {options.background_image}")
    scale = max(w / image.shape[1], h / image.shape[0])
    if scale > 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    x0 = int(rng.integers(0, image.shape[1] - w + 1))
    y0 = int(rng.integers(0, image.shape[0] - h + 1))
    return image[y0 : y0 + h, x0 : x0 + w].astype(np.float32)


def _composite_target(image: np.ndarray, target: Target, h: np.ndarray, supersample: int) -> None:
    """Average supersample^2 samples per pixel over the target's bounding box, in place."""
    extent = target.half_extent_m
    box = project(h, _square_corners(extent))
    u0 = max(int(math.floor(box[:, 0].min())) - 2, 0)
    v0 = max(int(math.floor(box[:, 1].min())) - 2, 0)
    u1 = min(int(math.ceil(box[:, 0].max())) + 2, image.shape[1] - 1)
    v1 = min(int(math.ceil(box[:, 1].max())) + 2, image.shape[0] - 1)
    if u0 > u1 or v0 > v1:
        return
    uu, vv = np.meshgrid(np.arange(u0, u1 + 1, dtype=np.float64), np.arange(v0, v1 + 1, dtype=np.float64))
    inv = np.linalg.inv(h)
    color_sum = np.zeros(uu.shape + (3,), np.float32)
    coverage = np.zeros(uu.shape, np.float32)
    offsets = (np.arange(supersample) + 0.5) / supersample - 0.5
    for dv in offsets:
        for du in offsets:
            su, sv = uu + du, vv + dv
            w = inv[2, 0] * su + inv[2, 1] * sv + inv[2, 2]
            x = (inv[0, 0] * su + inv[0, 1] * sv + inv[0, 2]) / w
            y = (inv[1, 0] * su + inv[1, 1] * sv + inv[1, 2]) / w
            rgb, inside = target.paint(x, y)
            inside &= w > 0
            color_sum += rgb[..., ::-1] * inside[..., None]
            coverage += inside
    n = supersample * supersample
    region = image[v0 : v1 + 1, u0 : u1 + 1]
    region[:] = color_sum / n + region * (1.0 - coverage / n)[..., None]


def render(target: Target, options: RenderOptions = RenderOptions(), seed: int = 0) -> tuple[np.ndarray, dict]:
    """Render one BGR uint8 image of target and return it with its ground truth.

    The same target, options and seed always give the same image.
    """
    options.validate()
    rng = np.random.default_rng(seed)
    h = homography(options)

    image = _background(options, rng)
    _composite_target(image, target, h, options.supersample)
    if options.blur_sigma_px > 0:
        image = cv2.GaussianBlur(image, (0, 0), options.blur_sigma_px)
    image = image * options.brightness
    if options.noise_sigma > 0:
        image = image + rng.normal(0.0, options.noise_sigma, image.shape).astype(np.float32)
    image = np.clip(np.rint(image), 0, 255).astype(np.uint8)

    center = project(h, np.zeros((1, 2)))[0]
    truth = {
        "target": target.name,
        "center_px": [float(center[0]), float(center[1])],
        "corners_px": project(h, target.corners_m()).tolist(),
        "digit": target.digit,
        "rotation_deg": float(options.rotation_deg % 360.0),
        "tilt_deg": [float(t) for t in options.tilt_deg],
        "mpp": options.mpp,
        "seed": seed,
        "target_params": target.params(),
        "options": {
            k: list(v) if isinstance(v, tuple) else v for k, v in dataclasses.asdict(options).items()
        },
    }
    return image, truth


# --------------------------------------------------------------------------- CLI


def random_sample(rng: np.random.Generator, args: argparse.Namespace) -> tuple[Target, RenderOptions, int]:
    """Draw one target and render settings from the CLI's ranges."""
    kind = args.targets[int(rng.integers(len(args.targets)))]
    if kind == "bullseye":
        target: Target = Bullseye()
    elif kind == "x":
        target = XMat(
            shape=str(rng.choice(["square", "circle"])),
            outline=bool(rng.random() < 0.3),
            bucket=bool(rng.random() < 0.5),
        )
    else:
        target = NumberedMarker(digit=int(rng.integers(10)))

    if args.mpp:
        mpp = float(rng.uniform(args.mpp[0], args.mpp[-1]))
    else:  # target spans 30-80% of the image height
        mpp = target.size_m / (float(rng.uniform(0.3, 0.8)) * args.height)
    span_px = target.size_m * math.sqrt(2) / mpp
    max_dx = max((args.width - span_px) / 2 * 0.9, 0.0)
    max_dy = max((args.height - span_px) / 2 * 0.9, 0.0)

    backgrounds = list(args.backgrounds) + (["image"] if args.background_image else [])
    background = backgrounds[int(rng.integers(len(backgrounds)))]
    image_file = None
    if background == "image":
        image_file = args.background_image[int(rng.integers(len(args.background_image)))]
    base = tuple(int(c) for c in rng.integers(40, 170, 3))

    options = RenderOptions(
        width=args.width,
        height=args.height,
        mpp=mpp,
        rotation_deg=float(rng.uniform(0, 360)),
        offset_px=(float(rng.uniform(-max_dx, max_dx)), float(rng.uniform(-max_dy, max_dy))),
        tilt_deg=(float(rng.uniform(-args.max_tilt, args.max_tilt)), float(rng.uniform(-args.max_tilt, args.max_tilt))),
        blur_sigma_px=float(rng.uniform(0, args.max_blur)),
        noise_sigma=float(rng.uniform(0, args.max_noise)),
        brightness=float(rng.uniform(1 - args.brightness, 1 + args.brightness)),
        background=background,
        background_color=base,
        background_image=image_file,
    )
    return target, options, int(rng.integers(2**31))


def contact_sheet(images: list[np.ndarray], truths: list[dict], cols: int = 4, tile_w: int = 400) -> np.ndarray:
    """Grid of thumbnails with the ground-truth centre and corners drawn on each."""
    tiles = []
    for image, truth in zip(images, truths):
        scale = tile_w / image.shape[1]
        tile = cv2.resize(image, (tile_w, round(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        corners = (np.array(truth["corners_px"]) + 0.5) * scale - 0.5
        cv2.polylines(tile, [np.rint(corners * 16).astype(np.int32)], True, (255, 255, 0), 1, cv2.LINE_AA, 4)
        cx, cy = ((np.array(truth["center_px"]) + 0.5) * scale - 0.5) * 16
        for dx, dy in ((1, 0), (0, 1)):
            cv2.line(tile, (int(cx - 96 * dx), int(cy - 96 * dy)), (int(cx + 96 * dx), int(cy + 96 * dy)),
                     (255, 0, 255), 1, cv2.LINE_AA, 4)
        opts = truth["options"]
        label = f"{truth['target']}" + (f" {truth['digit']}" if truth["digit"] is not None else "")
        if truth["target"] == "x":
            p = truth["target_params"]
            label += f" {p['shape']}" + (" bucket" if p["bucket"] else "") + (" outline" if p["outline"] else "")
        caption = (f"{label} | {opts['background']} | rot {truth['rotation_deg']:.0f} "
                   f"tilt {truth['tilt_deg'][0]:+.0f}/{truth['tilt_deg'][1]:+.0f} blur {opts['blur_sigma_px']:.1f}")
        bar = np.full((22, tile_w, 3), 30, np.uint8)
        cv2.putText(bar, caption, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (235, 235, 235), 1, cv2.LINE_AA)
        tiles.append(np.vstack([tile, bar]))
    blank = np.zeros_like(tiles[0])
    while len(tiles) % cols:
        tiles.append(blank)
    rows = [np.hstack(tiles[i : i + cols]) for i in range(0, len(tiles), cols)]
    return np.vstack(rows)


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n", type=int, required=True, help="number of samples")
    parser.add_argument("--out", required=True, help="output directory for PNGs and ground_truth.json")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--targets", nargs="+", choices=["bullseye", "x", "marker"], default=["bullseye", "x", "marker"])
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--mpp", type=float, nargs="+", metavar="M",
                        help="metres per pixel, or a LO HI range (default: target spans 30-80%% of the height)")
    parser.add_argument("--max-tilt", type=float, default=MAX_TILT_DEG, help="degrees, at most 15")
    parser.add_argument("--max-blur", type=float, default=2.0, help="largest blur sigma, pixels")
    parser.add_argument("--max-noise", type=float, default=8.0, help="largest noise sigma, 8-bit levels")
    parser.add_argument("--brightness", type=float, default=0.3, help="gain drawn from 1 ± this")
    parser.add_argument("--backgrounds", nargs="+", choices=["plain", "noise"], default=["plain", "noise"])
    parser.add_argument("--background-image", action="append", default=[], metavar="FILE",
                        help="add an image background; repeatable")
    parser.add_argument("--sheet", metavar="PNG", help="also write a contact sheet of the samples")
    args = parser.parse_args(argv)
    if not 0 <= args.max_tilt <= MAX_TILT_DEG:
        parser.error(f"--max-tilt must be between 0 and {MAX_TILT_DEG}")
    if args.mpp and len(args.mpp) > 2:
        parser.error("--mpp takes one value or a LO HI range")

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    images, samples = [], []
    for index in range(args.n):
        target, options, seed = random_sample(rng, args)
        image, truth = render(target, options, seed)
        name = f"{index:05d}_{target.name}.png"
        cv2.imwrite(str(out / name), image)
        samples.append({"file": name, **truth})
        if args.sheet:
            images.append(image)
    manifest = {"seed": args.seed, "argv": sys.argv[1:] if argv is None else argv, "samples": samples}
    (out / "ground_truth.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"wrote {args.n} samples and ground_truth.json to {out}")
    if args.sheet:
        cv2.imwrite(args.sheet, contact_sheet(images, samples), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        print(f"wrote {args.sheet}")


if __name__ == "__main__":
    main()
