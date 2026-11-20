#!/usr/bin/env python3
"""The centring loop, simulated offline (CEN-02): try a gain here before it is flown. No ROS needed.

  python3 tools/cen_loop_sim.py                              # the 2026 tunings and the 2027 retune, side by side
  python3 tools/cen_loop_sim.py --kp 0.9 --height 2.4 --wind strong
  python3 tools/cen_loop_sim.py --sweep-kp 0.3,0.6,1.0,1.5,2.5 --height 6 --out sweep.json

The loop is the executor's: every 50 ms (20 Hz) the controller is given the
estimator's error to the aim point and commands a horizontal velocity. The
2027 controller is the class the missions run
(drone_mission_core/centring_controller.py), not a copy of it; the 2026 one
is the deadband, low-pass filter, gain and speed cap of
red_bullseye_mission_base.py, for comparison.

What stands in for the world, and how roughly:

  - The estimate the controller sees is one executor tick old (the estimator
    is read on the tick before the command takes effect) and carries noise:
    a slowly wandering error of standard deviation `noise_sigma_m` (0.03 m by
    default, about what the estimator showed in the sim at 6 m and below).
  - The vehicle follows a velocity command as a first-order lag of 0.9 s
    with its acceleration limited to 3 m/s2. The lag is fitted to one
    number, the period of the hold error's swing in Gazebo; it is not a
    model of ArduPilot's guided velocity control.
  - Wind is what the flight controller does not take out: a velocity
    disturbance that wanders with a standard deviation of 0.035 m/s
    (moderate) or 0.10 m/s (strong) and a memory of 3 s, scaled so that
    the 2027 controller without its lead holds about as it did in Gazebo
    at 6 m. The mean wind is left out: the flight controller's own
    velocity loop holds against it.
  - The controller is told the vehicle's true velocity, as MAVROS reports
    it, for its lead term.

So the numbers say how a gain behaves (does it ring, how fast does it
settle, how much does it let gusts through), not what the error will be in
Gazebo or in the air. It under-states what the lead term does in gusts:
in Gazebo the lead took a third off the hold error in strong wind, here a
tenth. Gazebo is the check after: gate G1.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src" / "drone_mission_core"))

from drone_mission_core.centring_controller import CentringController, CentringGains  # noqa: E402
from drone_mission_core.tracking_projection import ground_offset_to_velocity  # noqa: E402

TICK_S = 0.05  # the executor's 20 Hz
DELAY_TICKS = 1  # the estimate a command is made from is one tick old
PHYSICS_STEPS = 10  # physics steps in a tick
VEHICLE_LAG_S = 0.9  # fitted: the period of the hold error's swing in the first Gazebo holds (about 7 s at kp 0.6)
VEHICLE_MAX_ACCEL_MPS2 = 3.0
NOISE_MEMORY_S = 0.5  # the estimator's error wanders: it is a filter's output, not white noise
GUST_MEMORY_S = 3.0
GUST_SIGMA_MPS = {"calm": 0.0, "moderate": 0.035, "strong": 0.10}  # scaled to the first Gazebo holds at 6 m
SETTLED_M = 0.10  # "settled": the true error stays within this
HOLD_FRACTION = 0.5  # the hold error is measured over this last part of the run
RING_OVERSHOOT_M = 0.10  # an overshoot past this, or more than two crossings of the target by RING_CROSSING_M, is ringing
RING_CROSSING_M = 0.05


@dataclass
class Loop:
    """A controller in the loop: step(east error, north error, height, time, the vehicle's velocity) -> (east, north) velocity command."""

    name: str
    step: Callable[[float, float, float, float, Tuple[float, float]], Tuple[float, float]]
    controller: object = None


def controller_2027(gains: Optional[CentringGains] = None) -> Loop:
    controller = CentringController(gains)
    kp = controller.gains.kp
    return Loop(f"2027 retune, kp {kp:.2f}", lambda east, north, height, now, velocity: controller.step(east, north, height, now, velocity_en_mps=velocity), controller)


def controller_2026(gain: float, deadband_m: float, alpha: float, max_speed_mps: float, name: str = "2026") -> Loop:
    """The 2026 centring of red_bullseye_mission_base.py: a deadband on each axis, a low-pass filter, a gain and a speed cap."""
    state: List[Optional[float]] = [None, None]

    def step(east: float, north: float, _height: float, _now: float, _velocity: Tuple[float, float]) -> Tuple[float, float]:
        east = 0.0 if abs(east) <= deadband_m else east
        north = 0.0 if abs(north) <= deadband_m else north
        if state[0] is None:
            state[0], state[1] = east, north
        else:
            state[0] += alpha * (east - state[0])
            state[1] += alpha * (north - state[1])
        return ground_offset_to_velocity(east_error_m=state[0], north_error_m=state[1], gain_mps_per_m=gain, max_speed_mps=max_speed_mps)

    return Loop(name, step)


@dataclass
class Result:
    name: str
    height_m: float
    settle_s: Optional[float]  # when the true error came within SETTLED_M for good; None if it never did
    overshoot_m: float  # how far past the target the vehicle went, along the line it came in on
    hold_p95_m: float  # 95th percentile of the true error over the last half of the run
    hold_max_m: float
    max_command_mps: float
    max_command_change_mps2: float
    rings: bool
    errors: List[float] = field(default_factory=list, repr=False)  # the true error at every tick
    commands: List[Tuple[float, float]] = field(default_factory=list, repr=False)

    def row(self) -> dict:
        return {key: (None if value is None else round(value, 4) if isinstance(value, float) else value) for key, value in self.__dict__.items() if key not in ("errors", "commands")}


class _Wander:
    """A value that wanders about zero with a given standard deviation and memory (an Ornstein-Uhlenbeck process), per axis."""

    def __init__(self, sigma: float, memory_s: float, rng: random.Random) -> None:
        self.sigma, self.memory_s, self.rng = sigma, memory_s, rng
        self.value = [rng.gauss(0.0, sigma), rng.gauss(0.0, sigma)] if sigma > 0.0 else [0.0, 0.0]

    def step(self, dt: float) -> Tuple[float, float]:
        if self.sigma > 0.0:
            keep = math.exp(-dt / self.memory_s)
            push = self.sigma * math.sqrt(1.0 - keep * keep)
            self.value = [keep * value + push * self.rng.gauss(0.0, 1.0) for value in self.value]
        return self.value[0], self.value[1]


def simulate(
    loop: Loop,
    *,
    height_m: float,
    initial_error_m: Sequence[float] = (2.0, 0.0),
    seconds: float = 40.0,
    noise_sigma_m: float = 0.03,
    gust_sigma_mps: float = 0.0,
    delay_ticks: int = DELAY_TICKS,
    seed: int = 1,
) -> Result:
    """Fly the loop: the vehicle starts at rest with the target `initial_error_m` (east, north) away."""
    rng = random.Random(seed)
    noise = _Wander(noise_sigma_m, NOISE_MEMORY_S, rng)
    gust = _Wander(gust_sigma_mps, GUST_MEMORY_S, rng)
    position = [-float(initial_error_m[0]), -float(initial_error_m[1])]  # the target is at the origin
    velocity = [0.0, 0.0]
    estimates: List[Optional[Tuple[float, float]]] = []
    errors: List[float] = []
    commands: List[Tuple[float, float]] = []
    start = (position[0], position[1])
    start_length = math.hypot(*start)
    overshoot = 0.0
    crossings, last_side = 0, 1.0
    max_change = 0.0
    previous = (0.0, 0.0)
    pushed = (0.0, 0.0)
    dt = TICK_S / PHYSICS_STEPS
    for tick in range(int(round(seconds / TICK_S))):
        now = tick * TICK_S
        wander = noise.step(TICK_S)
        estimates.append((-position[0] + wander[0], -position[1] + wander[1]))
        seen = estimates[tick - delay_ticks] if tick - delay_ticks >= 0 else None
        # The velocity the controller is told is the vehicle's own report: what it is doing now, wind and all.
        command = (0.0, 0.0) if seen is None else tuple(loop.step(seen[0], seen[1], height_m, now, (velocity[0] + pushed[0], velocity[1] + pushed[1])))
        commands.append(command)
        max_change = max(max_change, math.hypot(command[0] - previous[0], command[1] - previous[1]) / TICK_S)
        previous = command
        for _ in range(PHYSICS_STEPS):
            push = pushed = gust.step(dt)
            for axis in (0, 1):
                accel = (command[axis] - velocity[axis]) / VEHICLE_LAG_S
                accel = max(-VEHICLE_MAX_ACCEL_MPS2, min(VEHICLE_MAX_ACCEL_MPS2, accel))
                velocity[axis] += accel * dt
                position[axis] += (velocity[axis] + push[axis]) * dt
        errors.append(math.hypot(*position))
        if start_length > 0.0:
            along = -(position[0] * start[0] + position[1] * start[1]) / start_length  # positive once past the target
            overshoot = max(overshoot, along)
            side = 1.0 if along < 0.0 else -1.0
            if side != last_side and abs(along) > RING_CROSSING_M:
                crossings += 1
                last_side = side
    settle_s = None
    for index in range(len(errors) - 1, -1, -1):
        if errors[index] > SETTLED_M:
            settle_s = None if index == len(errors) - 1 else (index + 1) * TICK_S
            break
    else:
        settle_s = 0.0
    held = sorted(errors[int(len(errors) * (1.0 - HOLD_FRACTION)) :])
    return Result(
        name=loop.name,
        height_m=height_m,
        settle_s=settle_s,
        overshoot_m=overshoot,
        hold_p95_m=held[min(len(held) - 1, math.ceil(0.95 * len(held)) - 1)],
        hold_max_m=held[-1],
        max_command_mps=max(math.hypot(*command) for command in commands),
        max_command_change_mps2=max_change,
        rings=bool(gust_sigma_mps == 0.0 and (overshoot > RING_OVERSHOOT_M or crossings > 2)),
        errors=errors,
        commands=commands,
    )


def sweep_kp(values: Sequence[float], height_m: float = 6.0, seconds: float = 40.0) -> List[dict]:
    """One row per gain: a step of 0.8 m with no noise or wind, to see the loop's own behaviour."""
    rows = []
    for kp in values:
        result = simulate(controller_2027(CentringGains(kp=float(kp))), height_m=height_m, initial_error_m=(0.8, 0.0), seconds=seconds, noise_sigma_m=0.0, gust_sigma_mps=0.0)
        rows.append({"kp": float(kp), **{key: value for key, value in result.row().items() if key not in ("name", "height_m")}})
    return rows


def _line(result: Result, wind: str) -> str:
    settle = "never" if result.settle_s is None else f"{result.settle_s:5.1f} s"
    return (
        f"{result.name:<22} {result.height_m:>5.1f} m  {wind:<8}  settles {settle}  overshoot {result.overshoot_m:.2f} m  "
        f"hold p95 {result.hold_p95_m:.2f} m  max command {result.max_command_mps:.2f} m/s{'  rings' if result.rings else ''}"
    )


def comparison(seconds: float, seed: int) -> List[Tuple[Result, str]]:
    """The 2026 tunings and the retune from 2 m away, at three heights, calm and in the two winds."""
    rows = []
    for height in (15.0, 6.0, 2.4):
        for wind in ("calm", "moderate", "strong"):
            for make in (
                lambda: controller_2026(0.25, 0.15, 0.25, 2.0, "2026 demo"),
                lambda: controller_2026(0.15, 0.4, 0.12, 0.45, "2026 live"),
                controller_2027,
            ):
                rows.append((simulate(make(), height_m=height, initial_error_m=(2.0, 0.0), seconds=seconds, gust_sigma_mps=GUST_SIGMA_MPS[wind], seed=seed), wind))
    return rows


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Simulate the centring loop offline.")
    parser.add_argument("--kp", type=float, help="fly the 2027 controller with this gain instead of the comparison")
    parser.add_argument("--sweep-kp", help="comma-separated gains: a step response for each, with no noise or wind")
    parser.add_argument("--height", type=float, default=6.0, help="height above the ground in metres (default 6)")
    parser.add_argument("--wind", choices=sorted(GUST_SIGMA_MPS), default="calm")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", type=Path, help="also write the rows as JSON")
    args = parser.parse_args(argv)
    report: dict = {}
    try:
        if args.sweep_kp:
            rows = sweep_kp([float(value) for value in args.sweep_kp.split(",")], args.height, args.seconds)
            for row in rows:
                settle = "never" if row["settle_s"] is None else f"{row['settle_s']:5.1f} s"
                print(f"kp {row['kp']:.2f}  {args.height:.1f} m  settles {settle}  overshoot {row['overshoot_m']:.2f} m{'  rings' if row['rings'] else ''}")
            report["sweep"] = rows
        elif args.kp is not None:
            gains = CentringGains.from_config({"centring_kp": args.kp})
            result = simulate(controller_2027(gains), height_m=args.height, seconds=args.seconds, gust_sigma_mps=GUST_SIGMA_MPS[args.wind], seed=args.seed)
            print(_line(result, args.wind))
            report["runs"] = [{**result.row(), "wind": args.wind}]
        else:
            rows = comparison(args.seconds, args.seed)
            for result, wind in rows:
                print(_line(result, wind))
            report["runs"] = [{**result.row(), "wind": wind} for result, wind in rows]
    except ValueError as error:
        print(f"cen_loop_sim: {error}", file=sys.stderr)
        return 2
    if args.out:
        args.out.write_text(json.dumps(report, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
