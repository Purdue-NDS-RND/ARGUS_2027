"""The companion computer's health, and what of it is worth a line on the ground station (PLT-11).

What is watched:

  rates         a topic's messages a second against what is expected of it
                (the camera, the pose, the detector's output): under
                MIN_RATE_SHARE of it, or silent, is a finding;
  temperatures  the kernel's thermal zones (/sys/class/thermal): any zone at
                or over the limit;
  throttling    a CPU whose allowed top frequency (scaling_max_freq) has been
                brought under what it can do (cpuinfo_max_freq): the kernel
                is holding it back, which is what heat or a power limit does;
  CPU load      the share of the last interval the CPUs were busy (/proc/stat);
  GPU load      the Jetson's own figure where the kernel gives one (a file
                holding 0 to 1000); "not known" elsewhere, never a guess;
  dropped frames  for a node that says how many frames it had and how many it
                dropped (the localisation node's /loc/status): the share
                dropped over the window.

A condition is a finding only once it has lasted HOLD_S, and is over only
once it has been gone for HOLD_S: a single slow second is not news.

`Reporter` turns findings into status texts for QGroundControl: one short
line when a finding appears, again every REPEAT_S while it lasts, and
`ARGUS H:OK` when the last one has gone. At most one line every MIN_GAP_S,
and each within the 50 characters a STATUSTEXT carries. The codes:

  ARGUS H:RATE <name> <got>/<expected>Hz      ARGUS H:SILENT <name>
  ARGUS H:TEMP <zone> <degrees>C              ARGUS H:THROTTLE cpu
  ARGUS H:CPU <percent>%                      ARGUS H:GPU <percent>%
  ARGUS H:DROP <name> <percent>%              ARGUS H:OK

No ROS imports; the standard library only.
"""

from __future__ import annotations

import glob
import os
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

FLAG = "health_monitor"
WINDOW_S = 5.0  # rates and drops are measured over this long
MIN_RATE_SHARE = 0.8  # of the expected rate
SILENT_S = 3.0  # a topic not heard for this long is silent (and one never heard, this long after the monitor started)
HOLD_S = 5.0  # a condition is a finding once it has lasted this long, and over once it has been gone this long
TEMP_LIMIT_C = 85.0
CPU_LIMIT_PERCENT = 95.0
GPU_LIMIT_PERCENT = 98.0
DROP_LIMIT_SHARE = 0.10
REPEAT_S = 30.0  # a finding that lasts is said again this often
MIN_GAP_S = 2.0  # between two status texts
STATUSTEXT_MAX = 50  # the characters a MAVLink STATUSTEXT carries
PREFIX = "ARGUS H:"
WARNING, NOTICE = 4, 5  # MAVLink severities: MAV_SEVERITY_WARNING, MAV_SEVERITY_NOTICE
GPU_LOAD_PATHS = ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load", "/sys/devices/platform/bus@0/17000000.gpu/load", "/sys/devices/17000000.ga10b/load")


@dataclass(frozen=True)
class Finding:
    code: str  # RATE, SILENT, TEMP, THROTTLE, CPU, GPU, DROP
    subject: str  # what it is about: a topic's name, a thermal zone; "" for the computer as a whole
    text: str  # the status text, whole

    @property
    def key(self) -> Tuple[str, str]:
        return self.code, self.subject


def text_of(code: str, rest: str = "") -> str:
    """A status text of at most STATUSTEXT_MAX characters: the prefix, the code, and as much of the rest as fits."""
    return (f"{PREFIX}{code}" + (f" {rest}" if rest else ""))[:STATUSTEXT_MAX]


class Rate:
    """One topic's rate over the window, from the times its messages arrived."""

    def __init__(self, name: str, expected_hz: float) -> None:
        if not expected_hz > 0.0:
            raise ValueError(f"the expected rate of {name} must be over 0, got {expected_hz!r}")
        self.name, self.expected_hz = str(name), float(expected_hz)
        self._times: Deque[float] = deque()

    def add(self, now_s: float) -> None:
        self._times.append(float(now_s))
        self._trim(now_s)

    def _trim(self, now_s: float) -> None:
        while self._times and now_s - self._times[0] > WINDOW_S:
            self._times.popleft()

    def last_s(self) -> Optional[float]:
        return self._times[-1] if self._times else None

    def hz(self, now_s: float) -> float:
        """Messages a second over the window (over as much of it as has passed since the first one in it)."""
        self._trim(now_s)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(self._times[-1] - self._times[0], 1.0e-6)


# --- reading the computer ------------------------------------------------------------------------------------


def read_temperatures(root: Any = "/sys/class/thermal") -> List[Tuple[str, float]]:
    """(zone's name, degrees Celsius) of every thermal zone that can be read. A zone that reads nonsense is left out."""
    found = []
    for zone in sorted(glob.glob(os.path.join(str(root), "thermal_zone*"))):
        try:
            celsius = int(Path(zone, "temp").read_text().strip()) / 1000.0
            name = Path(zone, "type").read_text().strip() or os.path.basename(zone)
        except (OSError, ValueError):
            continue
        if -40.0 <= celsius <= 150.0:  # (a zone with no sensor behind it reads -274 or 0 on some boards: 0 is kept, a nonsense is not)
            found.append((name, celsius))
    return found


def read_throttled(root: Any = "/sys/devices/system/cpu") -> Optional[bool]:
    """Whether any CPU's allowed top frequency is under what it can do. None when the kernel does not say."""
    said = None
    for policy in sorted(glob.glob(os.path.join(str(root), "cpu[0-9]*", "cpufreq"))):
        try:
            allowed, able = int(Path(policy, "scaling_max_freq").read_text()), int(Path(policy, "cpuinfo_max_freq").read_text())
        except (OSError, ValueError):
            continue
        said = bool(said) or allowed < able
    return said


def cpu_times(stat_text: str) -> Optional[Tuple[int, int]]:
    """(busy, total) jiffies of all CPUs together, from /proc/stat's first line."""
    for line in stat_text.splitlines():
        parts = line.split()
        if parts and parts[0] == "cpu":
            try:
                values = [int(value) for value in parts[1:9]]
            except ValueError:
                return None
            if len(values) < 4:
                return None
            idle = values[3] + (values[4] if len(values) > 4 else 0)  # idle and waiting on the disk
            return sum(values) - idle, sum(values)
    return None


def read_gpu_percent(paths: Sequence[str] = GPU_LOAD_PATHS) -> Optional[float]:
    """The GPU's load in percent where the kernel gives it as 0 to 1000 in a file; None where it does not."""
    for path in paths:
        try:
            value = int(Path(path).read_text().strip())
        except (OSError, ValueError):
            continue
        if 0 <= value <= 1000:
            return value / 10.0
    return None


@dataclass
class Reading:
    """What was read of the computer at one moment. None: not known."""

    temperatures: List[Tuple[str, float]] = field(default_factory=list)
    throttled: Optional[bool] = None
    cpu_percent: Optional[float] = None
    gpu_percent: Optional[float] = None


class Computer:
    """Reads the computer; the CPU's load is the busy share since the last reading."""

    def __init__(self, thermal_root: Any = "/sys/class/thermal", cpu_root: Any = "/sys/devices/system/cpu", stat_path: Any = "/proc/stat", gpu_paths: Sequence[str] = GPU_LOAD_PATHS) -> None:
        self._thermal_root, self._cpu_root, self._stat_path, self._gpu_paths = thermal_root, cpu_root, stat_path, tuple(gpu_paths)
        self._last: Optional[Tuple[int, int]] = None

    def read(self) -> Reading:
        cpu = None
        try:
            now = cpu_times(Path(self._stat_path).read_text())
        except OSError:
            now = None
        if now is not None and self._last is not None and now[1] > self._last[1]:
            cpu = 100.0 * (now[0] - self._last[0]) / (now[1] - self._last[1])
        self._last = now if now is not None else self._last
        return Reading(read_temperatures(self._thermal_root), read_throttled(self._cpu_root), cpu, read_gpu_percent(self._gpu_paths))


# --- what is wrong -------------------------------------------------------------------------------------------


@dataclass
class Limits:
    temp_c: float = TEMP_LIMIT_C
    cpu_percent: float = CPU_LIMIT_PERCENT
    gpu_percent: float = GPU_LIMIT_PERCENT
    drop_share: float = DROP_LIMIT_SHARE
    min_rate_share: float = MIN_RATE_SHARE


class Health:
    """The rates, the drops and the computer's readings, and what of them has been wrong for HOLD_S."""

    def __init__(self, rates: Sequence[Rate] = (), limits: Optional[Limits] = None, started_s: float = 0.0) -> None:
        self.rates: Dict[str, Rate] = {rate.name: rate for rate in rates}
        self.limits = limits or Limits()
        self.started_s = float(started_s)
        self.reading = Reading()
        self._drops: Dict[str, Deque[Tuple[float, int, int]]] = {}
        self._since: Dict[Tuple[str, str], float] = {}  # a condition, and since when it holds
        self._gone: Dict[Tuple[str, str], float] = {}  # a finding, and since when its condition is gone
        self._active: Dict[Tuple[str, str], Finding] = {}

    def heard(self, name: str, now_s: float) -> None:
        if name in self.rates:
            self.rates[name].add(now_s)

    def counted(self, name: str, frames: Any, dropped: Any, now_s: float) -> None:
        """A node's own count so far of frames it had and frames it dropped."""
        try:
            frames, dropped = int(frames), int(dropped)
        except (TypeError, ValueError):
            return
        history = self._drops.setdefault(str(name), deque())
        if history and (frames < history[-1][1] or dropped < history[-1][2]):
            history.clear()  # the node started again: its counts begin anew
        history.append((float(now_s), frames, dropped))
        while history and now_s - history[0][0] > WINDOW_S:
            history.popleft()

    def read(self, reading: Reading) -> None:
        self.reading = reading

    def _conditions(self, now_s: float) -> Dict[Tuple[str, str], Finding]:
        wrong: Dict[Tuple[str, str], Finding] = {}

        def note(code: str, subject: str, rest: str) -> None:
            wrong[(code, subject)] = Finding(code, subject, text_of(code, rest))

        for rate in self.rates.values():
            last = rate.last_s()
            if (last is None and now_s - self.started_s > SILENT_S) or (last is not None and now_s - last > SILENT_S):
                note("SILENT", rate.name, rate.name)
            elif last is not None and now_s - self.started_s > WINDOW_S and rate.hz(now_s) < self.limits.min_rate_share * rate.expected_hz:
                note("RATE", rate.name, f"{rate.name} {rate.hz(now_s):.0f}/{rate.expected_hz:.0f}Hz")
        hot = [(name, celsius) for name, celsius in self.reading.temperatures if celsius >= self.limits.temp_c]
        if hot:
            name, celsius = max(hot, key=lambda zone: zone[1])
            note("TEMP", "", f"{name} {celsius:.0f}C")  # one finding for the computer, named by its hottest zone
        if self.reading.throttled:
            note("THROTTLE", "", "cpu")
        if self.reading.cpu_percent is not None and self.reading.cpu_percent >= self.limits.cpu_percent:
            note("CPU", "", f"{self.reading.cpu_percent:.0f}%")
        if self.reading.gpu_percent is not None and self.reading.gpu_percent >= self.limits.gpu_percent:
            note("GPU", "", f"{self.reading.gpu_percent:.0f}%")
        for name, history in self._drops.items():
            if len(history) >= 2:
                frames, dropped = history[-1][1] - history[0][1], history[-1][2] - history[0][2]
                offered = frames + dropped
                if offered > 0 and dropped / offered >= self.limits.drop_share:
                    note("DROP", name, f"{name} {100.0 * dropped / offered:.0f}%")
        return wrong

    def findings(self, now_s: float) -> List[Finding]:
        """What has been wrong for HOLD_S, and is not yet HOLD_S gone. Call it every second or so."""
        wrong = self._conditions(now_s)
        for key in list(self._since):
            if key not in wrong:
                del self._since[key]
        for key, finding in wrong.items():
            self._since.setdefault(key, now_s)
            self._gone.pop(key, None)
            if now_s - self._since[key] >= HOLD_S or key in self._active:
                self._active[key] = finding  # (its text is kept up to date while it lasts)
        for key in list(self._active):
            if key not in wrong:
                self._gone.setdefault(key, now_s)
                if now_s - self._gone[key] >= HOLD_S:
                    del self._active[key]
                    del self._gone[key]
        return [self._active[key] for key in sorted(self._active)]

    def summary(self, now_s: float) -> Dict[str, Any]:
        """Everything as it stands, for the node's own status topic."""
        return {
            "rates": {rate.name: {"hz": round(rate.hz(now_s), 1), "expected_hz": rate.expected_hz} for rate in self.rates.values()},
            "temperatures_c": {name: round(celsius, 1) for name, celsius in self.reading.temperatures},
            "throttled": self.reading.throttled,
            "cpu_percent": None if self.reading.cpu_percent is None else round(self.reading.cpu_percent, 1),
            "gpu_percent": None if self.reading.gpu_percent is None else round(self.reading.gpu_percent, 1),
            "findings": [finding.text for finding in self._active.values()],
        }


class Reporter:
    """Which status text goes to the ground station now, if any."""

    def __init__(self) -> None:
        self._said: Dict[Tuple[str, str], float] = {}  # a finding, and when it was last said
        self._last_s: Optional[float] = None
        self._owes_ok = False
        self.sent = 0

    def next(self, findings: Sequence[Finding], now_s: float) -> Optional[Tuple[int, str]]:
        """(severity, text) to send now, or None. One at a time: call it again next second for the next."""
        if self._last_s is not None and now_s - self._last_s < MIN_GAP_S:
            return None
        current = {finding.key: finding for finding in findings}
        for key in list(self._said):
            if key not in current:
                del self._said[key]
        due = [finding for key, finding in current.items() if key not in self._said or now_s - self._said[key] >= REPEAT_S]
        if due:
            finding = min(due, key=lambda item: (self._said.get(item.key, -1.0), item.key))  # one never said first, then the one said longest ago
            self._said[finding.key], self._last_s, self._owes_ok = now_s, now_s, True
            self.sent += 1
            return WARNING, finding.text
        if not current and self._owes_ok:
            self._owes_ok, self._last_s = False, now_s
            self.sent += 1
            return NOTICE, text_of("OK")
        return None


def parse_rates(items: Sequence[str]) -> List[Tuple[str, str, str, float]]:
    """The node's `rates` parameter, each "name:topic:type:hz" (the type as pkg/msg/Name), as (name, topic, type, hz). A wrong one is an error, said whole."""
    parsed = []
    for item in items:
        parts = str(item).split(":")
        try:
            if len(parts) != 4 or not parts[0] or not parts[1] or parts[2].count("/") != 2:
                raise ValueError
            hz = float(parts[3])
            if not hz > 0.0:
                raise ValueError
        except ValueError:
            raise ValueError(f"'{item}' is not name:topic:type:hz with a type like sensor_msgs/msg/Image and a rate over 0") from None
        parsed.append((parts[0], parts[1], parts[2], hz))
    if len({name for name, _topic, _type, _hz in parsed}) != len(parsed):
        raise ValueError("two rates have one name")
    return parsed


def parse_counts(items: Sequence[str]) -> List[Tuple[str, str]]:
    """The node's `frame_counts` parameter, each "name:topic": a std_msgs/String topic whose JSON says `frames` and `dropped` so far."""
    parsed = []
    for item in items:
        name, colon, topic = str(item).partition(":")
        if not colon or not name or not topic or ":" in topic:
            raise ValueError(f"'{item}' is not name:topic")
        parsed.append((name, topic))
    return parsed
