"""The preflight go/no-go (PLT-10): eleven checks over one snapshot of the aircraft. No ROS imports.

    GO      every check passed
    NO-GO   one did not, or could not be made: the start is blocked

A check that has nothing to read is NO-GO: a preflight that could not look
is not a preflight that found nothing wrong. NO-GO can be overridden by
hand, with a reason, and the override is written to a log.

The checks, in the spec's order:

  params        the flight controller's parameters are the baseline's: the
                values read from it (or a dump saved today) hold to
                platform/fcu/argus_2027.param and its rules; the file's hash
                is printed and logged
  fence         the fence is enabled and healthy (SYS_STATUS)
  gps           a 3D fix or better and HDOP within the limit
  ekf           the attitude and position estimate is healthy, and the
                flight controller's own pre-arm checks pass (SYS_STATUS)
  rangefinder   a reading of this moment, inside the sensor's range
  cameras       each camera the profile needs is publishing at its mode's rate
  engine        the detector's engine file is there and is the one the
                manifest names (by sha256); a profile with no engine passes
  timesync      MAVROS's time sync with the flight controller is alive and
                steady; the companion's clock is on the ground laptop's
  disk          room for the flight's logs
  battery       enough voltage a cell to start
  profile       the flag profile reads, fits the sequence, and is the one asked for

`run_checks` is the rule. The node that fills the `Snapshot`
(preflight_node.py) and the `go` command (PLT-07) are the callers.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

GO, NO_GO = "GO", "NO-GO"
CHECKS = ("params", "fence", "gps", "ekf", "rangefinder", "cameras", "engine", "timesync", "disk", "battery", "profile")
# For each check: what GO means, in a person's words with the limit's own name in braces (filled from Limits), and
# what to do about a NO-GO. The field documents (tools/ops_documents.py: the checklist, the SOP) are written from
# these and from CHECKS, so that the paper and the command cannot say different things.
GUIDE = {
    "params": ("The flight controller's parameters are the 2027 baseline's (`platform/fcu/argus_2027.param`), read from it or from a dump under {max_param_dump_age_h:g} h old.",
               "Load the baseline and upload the fence (`platform/fcu/README.md`), save a new dump, run the preflight again."),  # fmt: skip
    "fence": ("The fence is enabled and healthy, and its polygon is stored.", "Upload the site's fence (`sim/fence_upload.py --site field/<name>/site.yaml`). Never fly without it."),
    "gps": ("A 3D fix or better, HDOP {max_hdop:g} or less, {min_satellites} satellites or more, in a reading under {max_reading_age_s:g} s old (as every reading here must be).", "Wait in the open, away from the tents and vehicles. An override here costs the missions that fly on GPS."),
    "ekf": ("The position estimate is healthy and the flight controller's own pre-arm checks pass.", "Read the ground station's pre-arm message; it names what is wrong. Do not override."),
    "rangefinder": ("The downward rangefinder gives a reading and the flight controller says it has data.", "Check its cable and its window. The drop and the delivery need it; a waypoint flight can be overridden."),
    "cameras": ("Each camera named is publishing at {min_camera_rate_share:.0%} of its mode's rate or more.", "Check the ribbon cable and the camera service (`systemctl status argus-camera`), or the grabber the flight starts."),
    "engine": ("When the flight names an engine (`--engine`), the file is the one in the model registry (its sha256). With none named, nothing is compared and the line is GO.",
               "Copy the registered engine again; an engine built on another machine is not used. A flight that uses a trained model must name its engine."),  # fmt: skip
    "timesync": ("The companion and the flight controller agree on the time (an estimate under {max_timesync_age_s:g} s old; nine in ten of its steps {max_timesync_jitter_ms:g} ms or less and none over "
                 "{max_timesync_step_ms:g} ms), and the companion follows the ground laptop's clock when one is named (`--clock-source`).",
                 "Check the laptop's chrony and the network (`platform/jetson/clock_sync.py`). Stamped data on a clock that jumps is not flown on."),  # fmt: skip
    "disk": ("{min_free_gb:g} GB or more free where the logs go.", "Move old attempts off the disk (they are bundled: `attempts/`), not delete them on the field."),
    "battery": ("{min_cell_v:g} V a cell or more, resting, on a {cells}-cell pack.", "Change the pack. Never override: a pack that starts low does not finish a slot."),
    "profile": ("The flag profile fits the sequence: every flag declared, every dependency met.", "Fix the profile or pick the sequence it was written for; `tools/go.py --dry-run` shows the same refusal."),
}


def guide(limits: "Limits" = None) -> Dict[str, tuple]:
    """GUIDE with the limits written in: {check: (what GO means, what to do about a NO-GO)}, in CHECKS' order."""
    values = dataclasses.asdict(limits or Limits())
    return {name: (GUIDE[name][0].format(**values), GUIDE[name][1]) for name in CHECKS}


# MAV_SYS_STATUS_SENSOR bits (MAVLink common): what SYS_STATUS says is present, enabled and healthy.
SENSOR_AHRS = 0x200000
SENSOR_GEOFENCE = 0x100000
SENSOR_PREARM_CHECK = 0x10000000
SENSOR_LASER_POSITION = 0x100  # the downward rangefinder: healthy only while it has data
MIN_FENCE_POINTS = 3  # an inclusion polygon has three corners at least
GPS_FIX_3D = 3  # GPS_RAW_INT.fix_type: 3 is a 3D fix, 4 to 6 are better ones


@dataclass(frozen=True)
class Limits:
    max_hdop: float = 1.4  # ArduPilot's own GPS_HDOP_GOOD is 140
    min_satellites: int = 8
    max_reading_age_s: float = 2.0  # how old any reading may be to count as "now"
    min_camera_rate_share: float = 0.8  # of the mode's frame rate
    min_free_gb: float = 20.0  # on the log disk: a survey's frames are about 40 MB a second
    cells: int = 6
    min_cell_v: float = 3.9  # resting, before the flight: a pack that starts lower will not finish a slot
    max_timesync_age_s: float = 2.0
    max_timesync_jitter_ms: float = 20.0  # the 90th percentile of the offset estimate's steps
    max_timesync_step_ms: float = 100.0  # and the largest one: a hiccup of the link is tens of ms, a clock that jumped is more
    max_param_dump_age_h: float = 24.0


@dataclass
class Snapshot:
    """What the preflight read. None, or an empty value, means it could not be read."""

    now_s: float = 0.0
    # params: where the values came from, what does not hold, and the baseline file's hash
    params_source: Optional[str] = None  # "flight controller", or a dump's path
    params_problems: Optional[List[str]] = None  # None: not compared
    params_age_h: Optional[float] = None  # a dump's age; None when read live
    baseline_sha256: Optional[str] = None
    # SYS_STATUS
    sensors_present: Optional[int] = None
    sensors_enabled: Optional[int] = None
    sensors_health: Optional[int] = None
    sys_status_age_s: Optional[float] = None
    fence_points: Optional[int] = None  # FENCE_TOTAL, when the parameters were read: how many fence points are stored
    # GPS_RAW_INT
    gps_fix_type: Optional[int] = None
    gps_hdop: Optional[float] = None
    gps_satellites: Optional[int] = None
    gps_age_s: Optional[float] = None
    # rangefinder
    range_m: Optional[float] = None
    range_min_m: Optional[float] = None
    range_max_m: Optional[float] = None
    range_age_s: Optional[float] = None
    # cameras: {topic: (measured Hz, the mode's Hz)}; None: the profile was not asked which it needs
    camera_rates: Optional[Dict[str, Tuple[float, float]]] = None
    # engine: None: the profile uses none. Else the file's path, the hash found (None: no file) and the manifest's.
    engine: Optional[Dict[str, Any]] = None
    # time
    timesync_age_s: Optional[float] = None
    timesync_jitter_ms: Optional[float] = None  # the 90th percentile of the offset estimate's steps over the listening window
    timesync_largest_step_ms: Optional[float] = None
    clock: Optional[Tuple[bool, str]] = None  # chrony's verdict (clock_sync.py); None: not asked (the sim, or no laptop)
    clock_wanted: bool = False
    # disk and battery
    disk_free_gb: Optional[float] = None
    log_dir: Optional[str] = None
    battery_v: Optional[float] = None
    battery_age_s: Optional[float] = None
    # profile: (reads and fits, the flags it turns on or the problem); None: not checked
    profile: Optional[Tuple[bool, str]] = None
    profile_name: Optional[str] = None


@dataclass(frozen=True)
class Check:
    name: str
    go: bool
    detail: str


def sha256_of(path) -> Optional[str]:
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return None


def _fresh(age_s: Optional[float], limits: Limits) -> bool:
    return age_s is not None and math.isfinite(age_s) and -0.5 <= age_s <= limits.max_reading_age_s


def _params(s: Snapshot, limits: Limits) -> Check:
    if s.params_problems is None or s.baseline_sha256 is None:
        return Check("params", False, "the flight controller's parameters were not compared with the baseline (no link to read them and no dump given)")
    short = s.baseline_sha256[:12]
    if s.params_age_h is not None and not 0.0 <= s.params_age_h <= limits.max_param_dump_age_h:
        return Check("params", False, f"the parameter dump {s.params_source} is {s.params_age_h:.0f} h old (limit {limits.max_param_dump_age_h:g} h): save today's")
    if s.params_problems:
        more = "" if len(s.params_problems) == 1 else f" (and {len(s.params_problems) - 1} more)"
        return Check("params", False, f"not the baseline {short}: {s.params_problems[0]}{more}")
    return Check("params", True, f"{s.params_source} holds to the baseline, sha256 {short}")


def _sensor(s: Snapshot, bit: int) -> Optional[bool]:
    if s.sensors_present is None or s.sensors_enabled is None or s.sensors_health is None:
        return None
    return bool(s.sensors_present & bit) and bool(s.sensors_enabled & bit) and bool(s.sensors_health & bit)


def _fence(s: Snapshot, limits: Limits) -> Check:
    state = _sensor(s, SENSOR_GEOFENCE)
    if state is None or not _fresh(s.sys_status_age_s, limits):
        return Check("fence", False, "no system status of this moment from the flight controller")
    if not state:
        return Check("fence", False, "the flight controller does not report an enabled, healthy fence: load the baseline and upload the site's fence")
    if s.fence_points is not None and s.fence_points < MIN_FENCE_POINTS:
        return Check("fence", False, f"the fence is enabled but {s.fence_points} fence points are stored: upload the site's fence")
    points = "" if s.fence_points is None else f", {s.fence_points} points stored"
    return Check("fence", True, f"enabled and healthy{points}")


def _gps(s: Snapshot, limits: Limits) -> Check:
    if s.gps_fix_type is None or not _fresh(s.gps_age_s, limits):
        return Check("gps", False, "no GPS status of this moment")
    if s.gps_fix_type < GPS_FIX_3D:
        return Check("gps", False, f"fix type {s.gps_fix_type}: no 3D fix")
    if s.gps_hdop is None or not math.isfinite(s.gps_hdop) or s.gps_hdop > limits.max_hdop:
        return Check("gps", False, f"HDOP {s.gps_hdop if s.gps_hdop is not None else 'unknown'} (limit {limits.max_hdop:g})")
    if s.gps_satellites is None or s.gps_satellites < limits.min_satellites:
        return Check("gps", False, f"{s.gps_satellites if s.gps_satellites is not None else 'unknown'} satellites (at least {limits.min_satellites})")
    return Check("gps", True, f"fix type {s.gps_fix_type}, HDOP {s.gps_hdop:.2f}, {s.gps_satellites} satellites")


def _ekf(s: Snapshot, limits: Limits) -> Check:
    ahrs, prearm = _sensor(s, SENSOR_AHRS), _sensor(s, SENSOR_PREARM_CHECK)
    if ahrs is None or not _fresh(s.sys_status_age_s, limits):
        return Check("ekf", False, "no system status of this moment from the flight controller")
    if not ahrs:
        return Check("ekf", False, "the flight controller's attitude and position estimate is not healthy")
    if not prearm:
        return Check("ekf", False, "the flight controller's own pre-arm checks do not pass (its messages say which)")
    return Check("ekf", True, "estimate healthy, pre-arm checks pass")


def _rangefinder(s: Snapshot, limits: Limits) -> Check:
    if s.range_m is None or not _fresh(s.range_age_s, limits):
        return Check("rangefinder", False, "no rangefinder reading of this moment")
    # A reading alone is not a sensor: the flight controller goes on sending a range (of zero) for a rangefinder
    # that is configured and gives no data. Its own word on the sensor's health is asked too.
    sensor = _sensor(s, SENSOR_LASER_POSITION)
    if sensor is None or not _fresh(s.sys_status_age_s, limits):
        return Check("rangefinder", False, "no system status of this moment says whether the rangefinder has data")
    if not sensor:
        return Check("rangefinder", False, "the flight controller says the downward rangefinder is not fitted or has no data")
    low = s.range_min_m if s.range_min_m is not None else 0.0
    high = s.range_max_m if s.range_max_m is not None else math.inf
    if not math.isfinite(s.range_m) or s.range_m < 0.0 or s.range_m > high:
        return Check("rangefinder", False, f"reading {s.range_m} m is outside the sensor's range ({low:g} to {high:g} m)")
    # On the ground the reading is under the sensor's minimum: that is the sensor seeing the ground, not a fault.
    return Check("rangefinder", True, f"{s.range_m:.2f} m")


def _cameras(s: Snapshot, limits: Limits) -> Check:
    if s.camera_rates is None:
        return Check("cameras", False, "no camera was named: say which the flight needs (--camera TOPIC=HZ), or that it needs none (--no-camera)")
    if not s.camera_rates:
        return Check("cameras", True, "the flight needs no camera (said so)")
    slow = [f"{topic} at {hz:.1f} Hz (mode {wanted:g})" for topic, (hz, wanted) in sorted(s.camera_rates.items()) if not hz >= limits.min_camera_rate_share * wanted]
    if slow:
        return Check("cameras", False, "; ".join(slow))
    return Check("cameras", True, ", ".join(f"{topic} {hz:.1f} Hz" for topic, (hz, _wanted) in sorted(s.camera_rates.items())))


def _engine(s: Snapshot, _limits: Limits) -> Check:
    if s.engine is None:
        return Check("engine", True, "no engine was named (--engine): none is checked")
    path, found, wanted = s.engine.get("path"), s.engine.get("sha256"), s.engine.get("manifest_sha256")
    if found is None:
        return Check("engine", False, f"no engine file at {path}")
    if not wanted:
        return Check("engine", False, f"{path} is not in a manifest: nothing says it is the engine that was evaluated")
    if found != wanted:
        return Check("engine", False, f"{path} is {found[:12]}, the manifest has {str(wanted)[:12]}")
    return Check("engine", True, f"{path}, sha256 {found[:12]}")


def _timesync(s: Snapshot, limits: Limits) -> Check:
    if s.timesync_age_s is None or not 0.0 <= s.timesync_age_s <= limits.max_timesync_age_s:
        return Check("timesync", False, "no time-sync status of this moment from MAVROS")
    if s.timesync_jitter_ms is None:
        return Check("timesync", False, "too few time-sync exchanges were heard to say whether the sync is steady")
    largest = "" if s.timesync_largest_step_ms is None else f", largest step {s.timesync_largest_step_ms:.1f} ms"
    if s.timesync_jitter_ms > limits.max_timesync_jitter_ms:
        return Check("timesync", False, f"the time sync with the flight controller wanders by {s.timesync_jitter_ms:.1f} ms at the 90th percentile (limit {limits.max_timesync_jitter_ms:g}){largest}")
    if s.clock_wanted and s.clock is None:
        return Check("timesync", False, "the companion's clock was not compared with the ground laptop's")
    if s.clock is not None and not s.clock[0]:
        return Check("timesync", False, f"the companion's clock: {s.clock[1]}")
    if s.timesync_largest_step_ms is not None and s.timesync_largest_step_ms > limits.max_timesync_step_ms:
        return Check("timesync", False, f"the time sync with the flight controller stepped by {s.timesync_largest_step_ms:.1f} ms while it was listened to (limit {limits.max_timesync_step_ms:g})")
    clock = f"; clock: {s.clock[1]}" if s.clock is not None else "; the companion's clock was not compared with a ground laptop's (no --clock-source)"
    return Check("timesync", True, f"flight controller sync steady to {s.timesync_jitter_ms:.1f} ms{largest}{clock}")


def _disk(s: Snapshot, limits: Limits) -> Check:
    if s.disk_free_gb is None:
        return Check("disk", False, f"the log folder {s.log_dir or '(none given)'} could not be read")
    if s.disk_free_gb < limits.min_free_gb:
        return Check("disk", False, f"{s.disk_free_gb:.1f} GB free in {s.log_dir} (at least {limits.min_free_gb:g})")
    return Check("disk", True, f"{s.disk_free_gb:.0f} GB free in {s.log_dir}")


def _battery(s: Snapshot, limits: Limits) -> Check:
    if s.battery_v is None or not _fresh(s.battery_age_s, limits):
        return Check("battery", False, "no battery reading of this moment")
    cell = s.battery_v / limits.cells
    if not math.isfinite(cell) or cell < limits.min_cell_v:
        return Check("battery", False, f"{s.battery_v:.1f} V is {cell:.2f} V a cell on {limits.cells} cells (at least {limits.min_cell_v:g})")
    return Check("battery", True, f"{s.battery_v:.1f} V, {cell:.2f} V a cell")


def _profile(s: Snapshot, _limits: Limits) -> Check:
    if s.profile is None:
        return Check("profile", False, "no flag profile was checked")
    ok, text = s.profile
    if not ok:
        return Check("profile", False, f"{s.profile_name or 'the profile'}: {text}")
    return Check("profile", True, f"{s.profile_name or 'profile'}: {text or 'every flag off (the 2026 code)'}")


_RULES = {"params": _params, "fence": _fence, "gps": _gps, "ekf": _ekf, "rangefinder": _rangefinder, "cameras": _cameras, "engine": _engine,
          "timesync": _timesync, "disk": _disk, "battery": _battery, "profile": _profile}  # fmt: skip


def run_checks(snapshot: Snapshot, limits: Limits = Limits()) -> List[Check]:
    """Every check, in the spec's order."""
    return [_RULES[name](snapshot, limits) for name in CHECKS]


@dataclass(frozen=True)
class Verdict:
    go: bool  # the start may go ahead
    clean: bool  # every check passed (False with go True: overridden)
    line: str
    failed: Tuple[str, ...] = field(default_factory=tuple)


def verdict(checks: List[Check], override_reason: Optional[str] = None, who: str = "") -> Verdict:
    failed = tuple(check.name for check in checks if not check.go)
    if not failed:
        return Verdict(True, True, GO)
    reason = (override_reason or "").strip()
    if not reason:
        return Verdict(False, False, f"{NO_GO}: {', '.join(failed)}", failed)
    return Verdict(True, False, f"{NO_GO} OVERRIDDEN by {who or 'hand'}: {', '.join(failed)} (\"{reason}\")", failed)


def report_lines(checks: List[Check], result: Verdict) -> List[str]:
    width = max(len(check.name) for check in checks)
    lines = [f"[{GO if check.go else NO_GO:>5}] {check.name:<{width}}  {check.detail}" for check in checks]
    return lines + [result.line]


def log_record(checks: List[Check], result: Verdict, stamp: str, override_reason: Optional[str] = None, who: str = "", extra: Optional[dict] = None) -> dict:
    """One preflight as a JSON-ready record: what was found, what was decided, and who overrode what."""
    record = {
        "stamp": stamp,
        "verdict": GO if result.clean else (f"{NO_GO} OVERRIDDEN" if result.go else NO_GO),
        "failed": list(result.failed),
        "checks": [{"name": check.name, "go": check.go, "detail": check.detail} for check in checks],
    }
    if result.go and not result.clean:
        record["override"] = {"by": who or "hand", "reason": (override_reason or "").strip()}
    record.update(extra or {})
    return record


def append_log(path, record: dict) -> None:
    """Add a record to the preflight log (JSON lines). Raises OSError if it cannot be written: an override that is not logged is not allowed."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, sort_keys=True) + "\n")


# --- what the node needs that is not a rule: files, and the flight controller's parameters -------------------------


def read_param_file(path) -> Dict[str, float]:
    """A parameter file as {name: value}: "NAME,VALUE", "NAME VALUE", or QGroundControl's five columns. `#` starts a comment."""
    import re

    params: Dict[str, float] = {}
    for number, raw in enumerate(Path(path).read_text().splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = [part for part in re.split(r"[,\s]+", line) if part]
        if len(parts) == 5 and parts[0].isdigit() and parts[1].isdigit():
            parts = parts[2:4]
        try:
            params[parts[0]] = float(parts[1])
        except (IndexError, ValueError):
            raise ValueError(f"{path}, line {number}: expected NAME,VALUE, got {raw.strip()!r}") from None
    return params


def compare_params(have: Dict[str, float], baseline: Dict[str, float]) -> List[str]:
    """Where the flight controller's values are not the baseline's, a line a parameter. Empty: they hold."""
    problems = []
    if not baseline:
        return ["the baseline has no parameters in it: there is nothing to hold the flight controller to"]
    for name, wanted in baseline.items():
        if name not in have or have[name] is None:
            problems.append(f"{name} was not read; the baseline has {wanted:g}")
        elif abs(have[name] - wanted) > 1.0e-3:
            problems.append(f"{name} is {have[name]:g}, the baseline has {wanted:g}")
    return problems


def could_not_run(problem: str) -> List[Check]:
    """The checks of a preflight that could not look (a file that is not there, a link that did not answer): every one NO-GO, for that reason."""
    return [Check(name, False, f"the preflight could not run: {problem}") for name in CHECKS]


def check_profile(profile_path, sequence_path) -> Tuple[bool, str]:
    """Whether a flag profile reads and fits the sequence to be flown: (True, its flags) or (False, why not)."""
    import yaml
    from drone_mission_core.features import FeatureFlagError, load_profile, resolve_features

    try:
        enabled = load_profile(Path(profile_path))
        data = yaml.safe_load(Path(sequence_path).read_text(encoding="utf-8")) or {}
        declared = dict((data.get("mission_sequence", data) or {}).get("features") or {})
        flown = resolve_features(declared, enabled)
    except (FeatureFlagError, OSError, ValueError, yaml.YAMLError) as problem:
        return False, str(problem)
    # What will be on in flight: the profile's flags and any the sequence file itself has true.
    return True, ",".join(flag for flag, value in flown.items() if value is True)


def timesync_steps_ms(estimated_offsets_ns: List[int]) -> Tuple[Optional[float], Optional[float]]:
    """How much MAVROS's estimate of the flight controller's clock offset moves from one exchange to the next, in ms:
    (the 90th percentile of the steps, the largest step). (None, None) with fewer than two estimates.

    The check is on the 90th percentile: one late exchange in a listening window is a hiccup of the link, a
    tenth of them is a time base that wanders. The largest is shown beside it.
    """
    steps = sorted(abs(b - a) / 1.0e6 for a, b in zip(estimated_offsets_ns, estimated_offsets_ns[1:]))
    if not steps:
        return None, None
    return steps[min(len(steps) - 1, math.ceil(0.9 * len(steps)) - 1)], steps[-1]
