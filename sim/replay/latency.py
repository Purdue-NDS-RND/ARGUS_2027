#!/usr/bin/env python3
"""Where a frame's age comes from, stage by stage, from a recorded bag (SIM-09). No ROS imports.

  python3 sim/replay/latency.py <bag> [--out stages.json]

A frame is followed from the moment it was rendered or captured (its stamp)
to the moment the mission hears the detection made from it:

  render    stamp -> its camera_info is heard in ROS. In the sim: Gazebo's
            render, its publish and the bridge of a small message. On the
            aircraft: the driver.
  bridge    camera_info heard -> the image itself is heard by a subscriber
            (the recorder's frame tap). In the sim: the bridge and the
            delivery of the image's bytes.
  detector  image heard -> the detector publishes its detection. Waiting in
            the detector's queue behind earlier frames, then its own work.
  publish   detection published -> the detection is heard.

The four add up to the frame's age when the mission can first use it.

Which frame a detection came from is read from the bag: a detection that
carries its frame's stamp (the detection contract, or the 2026 topic with
features.stamp_from_image on) says so itself; a 2026 detection stamped with
the detector's own clock is matched to the detector's annotated copy of
the frame (the tap on /target_cv/annotated carries the frame's header),
which the detector publishes just before it. The detector's stage is also
split into waiting and working from the annotated copies alone: a frame
waits from its arrival until the previous frame's copy is out.

Clocks. Stamps of frames are on the camera's clock: Gazebo's in the sim.
The recorder's times are on the clock the nodes run on. When that is not
the frames' clock (a sim run on wall time), the bag's sim-clock tap
(/replay/sim_clock, or /clock) says what the recorder's time was at each
sim time, and stamps are converted through it.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parents[2]
if str(REPO / "sim") not in sys.path:
    sys.path.insert(0, str(REPO / "sim"))

from replay import topics  # noqa: E402
from replay.bag import open_bag  # noqa: E402
from replay.cdr import CdrError, stamp_ns  # noqa: E402

STAGES = ("render", "bridge", "detector", "publish")
MATCH_WINDOW_NS = 15_000_000  # a 2026 detection belongs to the annotated copy heard within this of its publish stamp: under half a frame at 30 Hz
SAME_CLOCK_NS = 5_000_000_000  # recorder times within this of the stamps they carry are on the stamps' clock
SLOW_SHARE = 0.10  # "the slow frames" are the slowest tenth by total age
HEIGHT_BAND_M = 3.0  # the detector's waiting and working are also given per band of this much height


def _ms(ns: float) -> float:
    return round(ns / 1.0e6, 3)


def spread_ms(values_ns: List[int]) -> Optional[dict]:
    if not values_ns:
        return None
    ordered = sorted(values_ns)
    return {
        "count": len(ordered),
        "median_ms": _ms(ordered[len(ordered) // 2]),
        "p95_ms": _ms(ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]),
        "max_ms": _ms(ordered[-1]),
        "mean_ms": _ms(sum(ordered) / len(ordered)),
    }


class ClockMap:
    """Recorder time for a stamp on the frames' clock, from samples (stamp clock, recorder time). The identity with no samples."""

    def __init__(self, samples: List[Tuple[int, int]]) -> None:
        samples = sorted(set(samples))
        self._sim = [sim for sim, _ in samples]
        self._heard = [heard for _, heard in samples]
        self.identity = not samples or all(abs(heard - sim) < SAME_CLOCK_NS for sim, heard in samples[:: max(1, len(samples) // 50)])

    def to_recorder(self, stamp: int) -> Optional[int]:
        if self.identity:
            return stamp
        index = bisect.bisect_left(self._sim, stamp)
        if index == 0 or index == len(self._sim):
            return None  # before the first sample or after the last: not known
        sim0, sim1, heard0, heard1 = self._sim[index - 1], self._sim[index], self._heard[index - 1], self._heard[index]
        return heard0 + round((stamp - sim0) * (heard1 - heard0) / (sim1 - sim0))


def read_events(bag_path, image_topic: str = "/camera/image", annotated_topic: str = "/target_cv/annotated") -> dict:
    """What the bag says happened to each frame, as plain lists and maps keyed by the frame's stamp."""
    image_tap, annotated_tap = topics.frame_tap_topic(image_topic), topics.frame_tap_topic(annotated_topic)
    contract = {topics.contract_topic(name) for name in topics.DETECTORS}
    events = {"info": {}, "image": {}, "annotated": {}, "contract": {}, "legacy": [], "sim_clock": [], "ros_clock": [], "height": []}
    for message in open_bag(bag_path):
        topic, heard = message.topic, message.log_time_ns
        if topic not in (topics.CAMERA_INFO, image_tap, annotated_tap, topics.LEGACY_DETECTION, topics.SIM_CLOCK_TAP, topics.CLOCK, topics.POSE) and topic not in contract:
            continue
        try:
            decoded = message.decode()
        except CdrError:
            continue
        if topic == topics.POSE:
            events["height"].append((heard, float(decoded.pose.position.z)))  # above home: where in the flight a frame was
        elif topic == topics.CAMERA_INFO:
            events["info"].setdefault(stamp_ns(decoded.header), heard)
        elif topic == image_tap:
            events["image"].setdefault(stamp_ns(decoded), heard)  # a std_msgs/Header: the image's own
        elif topic == annotated_tap:
            events["annotated"].setdefault(stamp_ns(decoded), heard)
        elif topic in contract:
            events["contract"].setdefault(stamp_ns(decoded.header), heard)
        elif topic == topics.LEGACY_DETECTION:
            if not (decoded.point.x == -1.0 and decoded.point.y == -1.0):  # the detector's "nothing found" has no frame
                events["legacy"].append((stamp_ns(decoded.header), heard))
        else:
            clock = int(decoded.clock.sec) * 1_000_000_000 + int(decoded.clock.nanosec)
            events["sim_clock" if topic == topics.SIM_CLOCK_TAP else "ros_clock"].append((clock, heard))
    return events


def breakdown(events: dict) -> dict:
    """The stages of every frame that can be followed to a detection, and what the slow ones have in common."""
    clock = ClockMap(events["sim_clock"] or events["ros_clock"])
    info, image, annotated = events["info"], events["image"], events["annotated"]

    # Which frame each detection came from, and when it was published and heard.
    detections: Dict[int, Tuple[Optional[int], int]] = {}  # frame stamp -> (published, heard)
    how = {"by the detection's own stamp": 0, "by the annotated copy": 0, "not matched": 0}
    annotated_heard = sorted((heard, stamp) for stamp, heard in annotated.items())
    annotated_times = [heard for heard, _ in annotated_heard]
    for stamp, heard in events["legacy"]:
        if stamp in image or stamp in info:  # stamped with its frame's time
            detections.setdefault(stamp, (None, heard))
            how["by the detection's own stamp"] += 1
            continue
        index = bisect.bisect_left(annotated_times, stamp)
        near = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(annotated_times)]
        best = min(near, key=lambda candidate: abs(annotated_times[candidate] - stamp), default=None)
        if best is None or abs(annotated_times[best] - stamp) > MATCH_WINDOW_NS:
            how["not matched"] += 1
            continue
        detections[annotated_heard[best][1]] = (stamp, heard)  # the stamp is the detector's clock at the publish
        how["by the annotated copy"] += 1
    for stamp, heard in events["contract"].items():
        if stamp not in detections:
            detections[stamp] = (None, heard)
            how["by the detection's own stamp"] += 1

    # The detector's stage as waiting and working, from the annotated copies alone (every frame it handled has one).
    waits, works, finished = [], [], []
    handled = sorted((image[stamp], heard) for stamp, heard in annotated.items() if stamp in image)
    for (arrived, done), (_previous_arrived, previous_done) in zip(handled[1:], handled):
        waits.append(max(0, previous_done - arrived))
        works.append(done - max(arrived, previous_done))
        finished.append(done)

    # The same, by the vehicle's height at the time: the target is larger in the image the lower the vehicle is.
    heights = sorted(events.get("height") or [])
    height_times = [heard for heard, _ in heights]

    def height_at(heard: int) -> Optional[float]:
        if not heights:
            return None
        index = min(max(bisect.bisect_left(height_times, heard), 0), len(heights) - 1)
        return heights[index][1]

    bands: Dict[int, dict] = {}
    for done, wait, work in zip(finished, waits, works):
        height = height_at(done)
        if height is not None:
            band = bands.setdefault(int(max(height, 0.0) // HEIGHT_BAND_M), {"wait": [], "work": []})
            band["wait"].append(wait)
            band["work"].append(work)

    frames = []
    for stamp, (published, heard) in sorted(detections.items()):
        rendered = clock.to_recorder(stamp)
        if rendered is None or stamp not in image:
            continue
        seen = image[stamp]
        stages = {
            "render": None if stamp not in info else info[stamp] - rendered,
            "bridge": None if stamp not in info else seen - info[stamp],
            "detector": (published if published is not None else heard) - seen,
            "publish": None if published is None else heard - published,
        }
        frames.append({"stamp_ns": stamp, "total": heard - rendered, **stages})

    totals = [frame["total"] for frame in frames]
    slow_from = sorted(totals)[int((1.0 - SLOW_SHARE) * len(totals))] if totals else 0
    slow = [frame for frame in frames if frame["total"] >= slow_from] if frames else []

    def stage_means(group: List[dict]) -> dict:
        means = {}
        for stage in STAGES:
            values = [frame[stage] for frame in group if frame[stage] is not None]
            means[stage] = None if not values else _ms(sum(values) / len(values))
        return means

    # Frames seen on the image topic while the detector was running that have no annotated copy in the bag: the
    # detector never took them (its queue was full), or the copy did not reach the tap (the topic is best effort).
    without_copy = [stamp for stamp in image if annotated and min(annotated) <= stamp <= max(annotated) and stamp not in annotated]
    result = {
        "frames_followed": len(frames),
        "matched": how,
        "clock": "the recorder is on the frames' clock" if clock.identity else "frame stamps converted through the bag's sim-clock tap",
        "frame_age_when_heard": spread_ms(totals),
        "stages": {stage: spread_ms([frame[stage] for frame in frames if frame[stage] is not None]) for stage in STAGES},
        "detector_waiting": spread_ms(waits),
        "detector_working": spread_ms(works),
        "detector_by_height": [
            {
                "from_m": index * HEIGHT_BAND_M,
                "to_m": (index + 1) * HEIGHT_BAND_M,
                "frames": len(band["work"]),
                "working": spread_ms(band["work"]),
                "waiting": spread_ms(band["wait"]),
            }
            for index, band in sorted(bands.items())
        ],
        "slow_frames": {
            "share": SLOW_SHARE,
            "count": len(slow),
            "from_ms": _ms(slow_from) if frames else None,
            "mean_stage_ms": stage_means(slow),
            "mean_stage_ms_of_all": stage_means(frames),
        },
        "frames_heard": {"camera_info": len(info), "image": len(image), "annotated": len(annotated)},
        "frames_without_an_annotated_copy": len(without_copy),
        "frames_the_tap_did_not_hear": len([stamp for stamp in info if image and min(image) <= stamp <= max(image) and stamp not in image]),
        "largest_stage_in_slow_frames": None,
    }
    slow_means = {stage: value for stage, value in result["slow_frames"]["mean_stage_ms"].items() if value is not None}
    if slow_means:
        result["largest_stage_in_slow_frames"] = max(slow_means, key=slow_means.get)
    return result


def short_lines(result: dict) -> List[str]:
    lines = [f"{result['frames_followed']} frames followed from stamp to detection ({result['clock']})"]
    age = result["frame_age_when_heard"]
    if age:
        lines.append(f"frame age when the detection is heard: median {age['median_ms']:.0f} ms, p95 {age['p95_ms']:.0f} ms, max {age['max_ms']:.0f} ms")
    for stage in STAGES:
        spread = result["stages"][stage]
        if spread:
            lines.append(f"  {stage:<8} median {spread['median_ms']:6.1f} ms, p95 {spread['p95_ms']:6.1f} ms, max {spread['max_ms']:6.1f} ms")
    for name, key in (("waiting", "detector_waiting"), ("working", "detector_working")):
        spread = result[key]
        if spread:
            lines.append(f"  detector {name}: median {spread['median_ms']:.1f} ms, p95 {spread['p95_ms']:.1f} ms, max {spread['max_ms']:.1f} ms over {spread['count']} frames")
    for band in result["detector_by_height"]:
        lines.append(
            f"  at {band['from_m']:g} to {band['to_m']:g} m: working median {band['working']['median_ms']:.1f} ms, p95 {band['working']['p95_ms']:.1f} ms; "
            f"waiting p95 {band['waiting']['p95_ms']:.1f} ms ({band['frames']} frames)"
        )
    slow = result["slow_frames"]
    if slow["count"]:
        parts = ", ".join(f"{stage} {value:.0f}" for stage, value in slow["mean_stage_ms"].items() if value is not None)
        lines.append(f"the slowest {int(100 * slow['share'])}% of frames (from {slow['from_ms']:.0f} ms), mean ms by stage: {parts}; largest: {result['largest_stage_in_slow_frames']}")
    return lines


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Break a recorded flight's frame age down by stage.")
    parser.add_argument("bag", type=Path, help="a bag folder, a .mcap file or a .db3 file")
    parser.add_argument("--image-topic", default="/camera/image")
    parser.add_argument("--annotated-topic", default="/target_cv/annotated")
    parser.add_argument("--out", type=Path, help="write the breakdown here as JSON")
    args = parser.parse_args(argv)
    result = breakdown(read_events(args.bag, args.image_topic, args.annotated_topic))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    for line in short_lines(result):
        print(line)
    return 0 if result["frames_followed"] else 1


if __name__ == "__main__":
    sys.exit(main())
