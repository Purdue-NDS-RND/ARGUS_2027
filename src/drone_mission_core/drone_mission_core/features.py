"""Mission feature flags read from the sequence YAML ``features:`` block."""

from __future__ import annotations

import dataclasses
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


def get_feature_flag(features: Optional[Mapping[str, Any]], name: str) -> Any:
    """Return the value of flag ``name``, or False if it or the block is missing.

    Missions pass ``self.spec.features``. A missing key means the feature is off,
    so a sequence file does not need to list every flag.
    """
    if not features:
        return False
    return features.get(name, False)


# --- 2027 plumbing: launch-line overrides, flag dependencies, profiles -------
#
# Nothing below runs differently unless a flag is turned on. With no
# overrides and every flag false, a sequence's flags come out exactly as they
# went in, which is the June 2026 behaviour.

# Flags that only make sense when another flag is on. The executor refuses to
# start with a flag on while one it requires is off. A feature that adds a
# flag depending on another adds its line here.
FLAG_REQUIRES: Dict[str, Tuple[str, ...]] = {
    # The drop's release gates (CEN-04) read the target estimator (FND-06), through the centring on it (CEN-01),
    # and their floor and band are judged on the height estimator's height (CEN-03): the bare rangefinder
    # height has no rule for a sensor that repeats one reading.
    "drop_release_gates": ("use_target_estimator", "centre_on_estimator", "use_height_estimator"),
    # The delivery's soft set-down (CEN-05) comes down on the estimator's track through the centring, and its
    # contact rule reads the rangefinder the height estimator keeps.
    "delivery_soft_set_down": ("use_target_estimator", "centre_on_estimator", "use_height_estimator"),
    # The survey mission (LOC-3) keeps its height with the guard (LOC-1), which watches the height estimator's height.
    "survey_search": ("use_height_estimator",),
    # The re-visit of uncertain markers (LOC-4) is a part of the survey mission.
    "survey_revisit": ("survey_search",),
    # Smart re-acquisition (CEN-06) climbs by the lost track's uncertainty, over where the centring last had it.
    "smart_reacquire": ("use_target_estimator", "centre_on_estimator"),
    # Centring on the estimator (CEN-01) steers on its track of the target.
    "centre_on_estimator": ("use_target_estimator",),
    # The retuned centring controller (CEN-02) is fed the estimator's error.
    "centring_retune": ("centre_on_estimator",),
    "wind_compensation": ("centring_retune",),
    # The near-field ring fit (CEN-13) is a second source of the bullseye for the estimator.
    "use_ring_fit": ("use_target_estimator",),
    # The recovery mission (REC-2) flies on the estimator's track of the bucket and on the height estimator's height.
    "package_recovery": ("use_target_estimator", "use_height_estimator"),
    # Its swing shaping (REC-4) and its GPS-untrusted mode (REC-7) are the recovery mission's own.
    "swing_shaping": ("package_recovery",),
    # The GPS health monitor and the source-set switch (REC-6) are the recovery mission's.
    "ekf_source_switching": ("package_recovery",),
    "recovery_gps_untrusted": ("package_recovery",),
    "marker_homing": ("package_recovery",),
    "keyframe_homing": ("package_recovery",),
    # The height estimator (CEN-03) corrects each rangefinder reading with the
    # tilt the vehicle had at that moment, from the pose history (FND-04).
    "use_height_estimator": ("use_frame_time_pose",),
    # The target estimator (FND-06) projects with the pose at each frame's
    # capture time (FND-04, FND-05).
    "use_target_estimator": ("use_frame_time_pose",),
}

_FLAG_NAME = re.compile(r"[a-z][a-z0-9_]*")
_TRUE = {"true", "on", "1"}
_FALSE = {"false", "off", "0"}


class FeatureFlagError(ValueError):
    """Flags that cannot be flown as asked. The executor does not start."""


def parse_feature_overrides(text: Optional[str]) -> Dict[str, bool]:
    """Read the `feature_overrides` launch argument.

    A comma-separated list of flag names turns each on: ``"a,b"``. A name may
    carry a value, ``"a=true,b=false"`` (also on/off, 1/0). An empty string
    is no overrides.
    """
    overrides: Dict[str, bool] = {}
    for item in (text or "").split(","):
        item = item.strip()
        if not item:
            continue
        name, equals, raw = item.partition("=")
        name, raw = name.strip(), raw.strip().lower()
        if not _FLAG_NAME.fullmatch(name):
            raise FeatureFlagError(f"feature_overrides: '{item}' is not a flag name (lower case, digits and underscores)")
        if not equals:
            value = True
        elif raw in _TRUE:
            value = True
        elif raw in _FALSE:
            value = False
        else:
            raise FeatureFlagError(f"feature_overrides: '{item}' must be {name}=true or {name}=false")
        if overrides.get(name, value) != value:
            raise FeatureFlagError(f"feature_overrides: {name} is given twice with different values")
        overrides[name] = value
    return overrides


def missing_requirements(features: Optional[Mapping[str, Any]]) -> List[str]:
    """One line per flag that is on while a flag it requires is off.

    Anything other than False counts as on for the flag that has the
    requirement, and only True counts as on for the flag it requires: a value
    such as 1 or "true" can never slip a dependency past this check.
    """
    problems = []
    for flag, needs in FLAG_REQUIRES.items():
        if get_feature_flag(features, flag) is not False:
            for need in needs:
                if get_feature_flag(features, need) is not True:
                    problems.append(f"{flag} requires {need}")
    return problems


def resolve_features(declared: Optional[Mapping[str, Any]], overrides: Mapping[str, bool]) -> Dict[str, Any]:
    """The flags to fly: the sequence's own, with the overrides applied and the dependencies checked.

    An override must name a flag the sequence declares in its ``features:``
    block. A typo would otherwise fly the 2026 code while the operator
    believes a flag is on.
    """
    resolved = dict(declared or {})
    unknown = sorted(name for name in overrides if name not in resolved)
    if unknown:
        raise FeatureFlagError(
            f"feature_overrides names flag(s) this sequence does not declare: {', '.join(unknown)} "
            f"(it declares: {', '.join(sorted(resolved)) or 'none'})"
        )
    resolved.update(overrides)
    # A flag is true or false. A mission reads `if get_feature_flag(...)`, so
    # 1, "true" or even "false" (a non-empty string) would all read as on.
    not_bool = sorted(f"{name}={value!r}" for name, value in resolved.items() if not isinstance(value, bool))
    if not_bool:
        raise FeatureFlagError(f"feature flags must be true or false, got: {', '.join(not_bool)}")
    problems = missing_requirements(resolved)
    if problems:
        raise FeatureFlagError("feature flags with a missing dependency: " + "; ".join(problems))
    return resolved


def apply_feature_overrides(specs: Iterable[Any], overrides_text: Optional[str]) -> list:
    """Mission specs with the launch line's overrides applied to each one's flags.

    Called by the executor after it loads the sequence. With no overrides the
    specs come back equal to what was loaded.
    """
    overrides = parse_feature_overrides(overrides_text)
    return [dataclasses.replace(spec, features=resolve_features(spec.features, overrides)) for spec in specs]


def load_profile(path) -> Dict[str, bool]:
    """A flag profile (``profiles/*.yaml``): the flags it enables, each mapped to True.

    A profile is an enable-list, kept outside ``src/*/config`` so that turning
    flags on for a flight never edits a tuned file::

        description: what this profile is for
        enable:
          - use_frame_time_pose
          - use_target_estimator
    """
    import yaml

    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise FeatureFlagError(f"{path}: {error}") from error
    if not isinstance(data, dict):
        raise FeatureFlagError(f"{path}: a profile is a mapping with an 'enable' list")
    # `nodes` (node flags) and `start` (what the bringup launch starts) are the bringup's (PLT-07,
    # drone_mission_core/bringup.py), which reads and checks them; here they are only allowed.
    unknown = sorted(str(key) for key in set(data) - {"description", "enable", "nodes", "start"})
    if unknown:
        raise FeatureFlagError(f"{path}: unknown key(s) {', '.join(unknown)} (a profile has description, enable, nodes and start)")
    names = data.get("enable") or []
    if not isinstance(names, list):
        raise FeatureFlagError(f"{path}: 'enable' must be a list of flag names")
    enabled: Dict[str, bool] = {}
    for name in names:
        if not isinstance(name, str) or not _FLAG_NAME.fullmatch(name):
            raise FeatureFlagError(f"{path}: '{name}' is not a flag name (lower case, digits and underscores)")
        if name in enabled:
            raise FeatureFlagError(f"{path}: {name} is listed twice")
        enabled[name] = True
    problems = missing_requirements(enabled)
    if problems:
        raise FeatureFlagError(f"{path}: " + "; ".join(problems))
    return enabled
