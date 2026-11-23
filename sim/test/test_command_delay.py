"""PLT-05, second test: capture to command and command to response from a bag (sim/replay/command_delay.py). No ROS."""

import json
import math
import random
import sys
from pathlib import Path

import pytest

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay import bag, command_delay, topics  # noqa: E402
from replay.bag import Record  # noqa: E402
from replay.cdr import Message  # noqa: E402

import replay_flight as flight  # noqa: E402

MS, S = flight.MS, flight.S
T0 = flight.T0
VELOCITY_ONLY = 0b0000_1101_1100_0111  # position, acceleration and yaw ignored: a velocity command


def setpoint_record(ns, east=0.0, north=0.0, mask=VELOCITY_ONLY, position=(0.0, 0.0)):
    message = Message(
        header=flight.header(ns), coordinate_frame=1, type_mask=mask, position=Message(x=position[0], y=position[1], z=10.0),
        velocity=Message(x=east, y=north, z=0.0), acceleration_or_force=Message(x=0.0, y=0.0, z=0.0), yaw=0.0, yaw_rate=0.0,
    )  # fmt: skip
    return Record(topics.SETPOINT_LOCAL, topics.RECORDED[topics.SETPOINT_LOCAL], ns, message)


def imu_record(ns, pitch_deg=0.0, roll_deg=0.0):
    pitch, roll = math.radians(pitch_deg), math.radians(roll_deg)
    cp, sp, cr, sr = math.cos(pitch / 2), math.sin(pitch / 2), math.cos(roll / 2), math.sin(roll / 2)
    orientation = Message(x=sr * cp, y=cr * sp, z=-sr * sp, w=cr * cp)
    zero = Message(x=0.0, y=0.0, z=0.0)
    message = Message(
        header=flight.header(ns, "base_link"), orientation=orientation, orientation_covariance=[0.0] * 9, angular_velocity=zero,
        angular_velocity_covariance=[0.0] * 9, linear_acceleration=zero, linear_acceleration_covariance=[0.0] * 9,
    )  # fmt: skip
    return Record(topics.IMU, topics.RECORDED[topics.IMU], ns + 3 * MS, message)


def flight_records(detection_latency_ms=40, response_ms=80, steps_at_s=(2.0, 5.0, 8.0), seconds=10.0, commands=True, seed=2, clock_offset=0, tilt_noise_deg=0.03):
    """Ten seconds: detections at 30 Hz, velocity commands at 20 Hz that step at `steps_at_s`, and a tilt that follows each step `response_ms` later.

    With `clock_offset` the frames are stamped on another clock (the sim's), `clock_offset` behind the recorder's.
    """
    noise = random.Random(seed)
    records = []
    for index in range(int(seconds * 30)):
        frame = T0 + index * 33 * MS
        records.append(flight.detection_record(frame - clock_offset, frame + detection_latency_ms * MS, (320.0, 180.0)))
    if clock_offset:
        for index in range(int(seconds * 10) + 20):
            heard = T0 - S + index * 100 * MS
            records.append(Record(topics.SIM_CLOCK_TAP, "rosgraph_msgs/msg/Clock", heard, Message(clock=flight.stamp(heard - clock_offset))))
    step_times = [T0 + round(at * S) for at in steps_at_s]
    if commands:
        for index in range(int(seconds * 20)):
            ns = T0 + index * 50 * MS
            speed = float(sum(1 for at in step_times if ns >= at) % 2)  # 0, then 1 m/s, then 0, ...
            records.append(setpoint_record(ns, east=speed))
    for index in range(int(seconds * 50)):
        ns = T0 + index * 20 * MS
        pitch = 0.0
        for number, at in enumerate(step_times):
            since = (ns - at) / S - response_ms / 1000.0
            if since >= 0.0:
                target = 5.0 if number % 2 == 0 else 0.0
                start = 0.0 if number % 2 == 0 else 5.0
                pitch = start + (target - start) * min(1.0, since / 0.2)
        records.append(imu_record(ns, pitch_deg=pitch + noise.gauss(0.0, tilt_noise_deg)))
    return sorted(records, key=lambda record: record.log_time_ns)


def write(tmp_path, records):
    return bag.write_bag(tmp_path / "flight", records)


def test_capture_to_command_is_the_frame_to_the_first_setpoint_that_could_use_it(tmp_path):
    result = command_delay.analyse(command_delay.read_chain(write(tmp_path, flight_records(detection_latency_ms=40))))
    chain = result["capture_to_command"]
    # Heard 40 ms after the frame, then up to one 50 ms command period for the next setpoint.
    assert 40.0 <= chain["median_ms"] <= 90.0
    assert chain["p95_ms"] <= 90.0
    assert chain["count"] > 250
    assert result["capture_to_command_passed"] is True


def test_a_slow_chain_fails_the_150_ms_line(tmp_path):
    result = command_delay.analyse(command_delay.read_chain(write(tmp_path, flight_records(detection_latency_ms=130))))
    assert result["capture_to_command"]["p95_ms"] > 150.0
    assert result["capture_to_command_passed"] is False
    assert command_delay.CAPTURE_TO_COMMAND_P95_S == 0.150


def test_frames_stamped_on_the_sims_clock_are_put_on_the_recorders(tmp_path):
    same = command_delay.analyse(command_delay.read_chain(write(tmp_path / "a", flight_records())))
    other = command_delay.analyse(command_delay.read_chain(write(tmp_path / "b", flight_records(clock_offset=1_700_000_000 * S))))
    assert other["capture_to_command"]["median_ms"] == pytest.approx(same["capture_to_command"]["median_ms"], abs=1.0)


def test_detections_nobody_commanded_on_are_not_counted(tmp_path):
    result = command_delay.analyse(command_delay.read_chain(write(tmp_path, flight_records(commands=False))))
    assert result["capture_to_command"] is None
    assert result["capture_to_command_passed"] is None
    assert result["actuation"] is None


@pytest.mark.parametrize("response_ms", [80, 160])
def test_the_actuation_delay_is_the_command_step_to_the_first_tilt(tmp_path, response_ms):
    result = command_delay.analyse(command_delay.read_chain(write(tmp_path, flight_records(response_ms=response_ms))))
    actuation = result["actuation"]
    assert actuation["count"] == 3
    # The tilt is sampled every 20 ms and heard 3 ms later; its start is carried back from where it clears the noise.
    assert actuation["median_ms"] == pytest.approx(response_ms + 3.0, abs=15.0)


def test_a_noisy_hover_does_not_make_the_delay_look_longer(tmp_path):
    quiet = command_delay.analyse(command_delay.read_chain(write(tmp_path / "a", flight_records(tilt_noise_deg=0.03))))
    noisy = command_delay.analyse(command_delay.read_chain(write(tmp_path / "b", flight_records(tilt_noise_deg=0.25))))
    assert noisy["actuation"]["count"] == 3
    assert noisy["actuation"]["median_ms"] == pytest.approx(quiet["actuation"]["median_ms"], abs=25.0)


def test_a_steady_command_has_no_steps_to_time(tmp_path):
    result = command_delay.analyse(command_delay.read_chain(write(tmp_path, flight_records(steps_at_s=()))))
    assert result["actuation"] is None
    assert result["capture_to_command"] is not None


def test_position_commands_are_stepped_on_too():
    commands = [(T0 + index * 50 * MS, "position", 0.0 if index < 40 else 3.0, 0.0) for index in range(80)]
    steps = command_delay.command_steps(commands)
    assert steps == [T0 + 40 * 50 * MS]
    # A drift of centimetres a tick is not a step.
    creeping = [(T0 + index * 50 * MS, "position", 0.02 * index, 0.0) for index in range(80)]
    assert command_delay.command_steps(creeping) == []


def test_the_tool_prints_both_numbers_and_the_verdict(tmp_path, capsys):
    path = write(tmp_path, flight_records())
    out = tmp_path / "delay.json"
    assert command_delay.main([str(path), "--out", str(out)]) == 0
    printed = capsys.readouterr().out
    assert "capture to command" in printed and "p95" in printed and "PASS" in printed
    assert "actuation delay" in printed
    saved = json.loads(out.read_text())
    assert saved["actuation"]["count"] == 3
    slow = write(tmp_path / "slow", flight_records(detection_latency_ms=130))
    assert command_delay.main([str(slow)]) == 1
    assert "FAIL" in capsys.readouterr().out


def test_a_bag_with_nothing_to_measure_is_not_a_pass(tmp_path, capsys):
    path = write(tmp_path, flight_records(commands=False))
    assert command_delay.main([str(path)]) == 1
    assert "not measured" in capsys.readouterr().out


def test_a_step_probes_bag_has_no_detections_and_still_gives_the_actuation_delay(tmp_path, capsys):
    records = [record for record in flight_records() if record.topic != topics.contract_topic("target_cv")]
    assert command_delay.main([str(write(tmp_path, records))]) == 0
    printed = capsys.readouterr().out
    assert "capture to command: not measured" in printed and "actuation delay: median" in printed


def test_detections_left_out_for_want_of_a_setpoint_are_counted_and_said(tmp_path, capsys):
    records = flight_records()
    gap = (T0 + 4 * S, T0 + 6 * S)  # two seconds with no setpoints: about sixty detections wait longer than 200 ms
    records = [record for record in records if not (record.topic == topics.SETPOINT_LOCAL and gap[0] <= record.log_time_ns < gap[1])]
    path = write(tmp_path, records)
    result = command_delay.analyse(command_delay.read_chain(path))
    assert 50 <= result["no_setpoint_within_200_ms"] <= 62
    assert result["capture_to_command"]["count"] + result["no_setpoint_within_200_ms"] <= result["detections"]
    command_delay.main([str(path)])
    assert f"{result['no_setpoint_within_200_ms']} detections were not counted" in capsys.readouterr().out
