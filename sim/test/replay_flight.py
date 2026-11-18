"""A generated flight as bag records, for the replay harness's tests (SIM-09). No ROS, and not a test file itself.

A vehicle hovers at 15 m, 2 m west and 1 m south of a bullseye, for a few
seconds. The 2026 detector says its image size, then reports the bullseye
on the detection contract at 30 Hz, each detection heard `latency_ms` after
its frame was taken, with a little pixel noise from a seeded generator.
"""

import random
import sys
from pathlib import Path

SIM = Path(__file__).resolve().parents[1]
REPO = SIM.parent
for path in (SIM, REPO / "src" / "drone_mission_core", REPO / "src" / "argus_camera"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from replay import topics  # noqa: E402
from replay.bag import Record  # noqa: E402
from replay.cdr import Message  # noqa: E402

MS, S = 1_000_000, 1_000_000_000
T0 = 2_000 * S
HEIGHT = 15.0
TARGET = (2.0, 1.0, 0.0)  # east, north, up of the bullseye in the local frame


def stamp(ns):
    return Message(sec=ns // S, nanosec=ns % S)


def header(ns, frame_id="map"):
    return Message(stamp=stamp(ns), frame_id=frame_id)


def pose_record(ns, east=0.0, north=0.0, up=HEIGHT):
    pose = Message(position=Message(x=east, y=north, z=up), orientation=Message(x=0.0, y=0.0, z=0.0, w=1.0))
    return Record(topics.POSE, topics.RECORDED[topics.POSE], ns + 2 * MS, Message(header=header(ns), pose=pose))


def velocity_record(ns):
    twist = Message(linear=Message(x=0.0, y=0.0, z=0.0), angular=Message(x=0.0, y=0.0, z=0.0))
    return Record(topics.VELOCITY, topics.RECORDED[topics.VELOCITY], ns + 2 * MS, Message(header=header(ns), twist=twist))


def range_record(ns, range_m=HEIGHT):
    message = Message(header=header(ns, "rangefinder"), radiation_type=1, field_of_view=0.05, min_range=0.1, max_range=40.0, range=range_m)
    return Record(topics.RANGEFINDER, topics.RECORDED[topics.RANGEFINDER], ns + 2 * MS, message)


def size_record(ns, width=640, height=360):
    message = Message(header=header(ns, "camera"), point=Message(x=float(width), y=float(height), z=0.0))
    return Record(topics.LEGACY_IMAGE_SIZE, topics.RECORDED[topics.LEGACY_IMAGE_SIZE], ns, message)


def detection_record(frame_ns, heard_ns, pixel, class_id="bullseye", detector="target_cv", size_px=40.0):
    head = header(frame_ns, "down_camera_optical")
    hypothesis = Message(
        hypothesis=Message(class_id=class_id, score=0.9),
        pose=Message(pose=Message(position=Message(x=0.0, y=0.0, z=0.0), orientation=Message(x=0.0, y=0.0, z=0.0, w=1.0)), covariance=[0.0] * 36),
    )
    box = Message(center=Message(position=Message(x=pixel[0], y=pixel[1]), theta=0.0), size_x=size_px, size_y=size_px)
    array = Message(header=head, detections=[Message(header=head, results=[hypothesis], bbox=box, id="")])
    topic = topics.contract_topic(detector)
    return Record(topic, topics.RECORDED[topic], heard_ns, array)


def hover_records(seconds=4.0, latency_ms=40, pixel_sigma=0.5, seed=1, see_from_s=0.5, target=TARGET):
    """The flight described at the top of the file, as records in time order."""
    from drone_mission_core.frame_projection import load_camera
    from drone_mission_core.projection_v2 import pixel_of_ground_point

    camera, mount = load_camera("sim_delivery", "sim_iris_downcam")
    true_pixel = pixel_of_ground_point(camera, mount, target, (0.0, 0.0, HEIGHT), (0.0, 0.0, 0.0, 1.0))
    noise = random.Random(seed)
    records = [size_record(T0 + 10 * MS)]
    for index in range(int(seconds * 30) + 1):
        ns = T0 + index * 33 * MS
        records += [pose_record(ns), velocity_record(ns)]
        if index % 2 == 0:
            records.append(range_record(ns))
        if ns - T0 >= int(see_from_s * S):
            pixel = (true_pixel[0] + noise.gauss(0.0, pixel_sigma), true_pixel[1] + noise.gauss(0.0, pixel_sigma))
            records.append(detection_record(ns, ns + latency_ms * MS, pixel))
    return sorted(records, key=lambda record: record.log_time_ns)
