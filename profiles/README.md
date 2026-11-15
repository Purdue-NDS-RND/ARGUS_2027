# Flag profiles

A profile is a named list of feature flags to turn on for a flight. Profiles live here, outside `src/*/config`, so choosing what to fly never edits a tuned file and the shipped YAML keeps every flag `false`.

```yaml
description: what this profile is for
enable:
  - use_target_estimator
  - drop_release_gates
```

Use one through the `feature_overrides` argument every mission launch file takes, in two steps so that a failure stops you before the launch:

```bash
python3 tools/flag_profile.py profiles/comp_2026.yaml           # prints the flags, comma-separated (nothing for comp_2026)

flags=$(python3 tools/flag_profile.py profiles/<name>.yaml --sequence package_drop_live.yaml) || exit
ros2 launch drone_mission_demo package_drop_live.launch.py feature_overrides:="$flags"
```

`feature_overrides` also takes names directly: `feature_overrides:=use_frame_time_pose`, or `name=false` to turn one off.

If anything goes wrong in `flag_profile.py` (a bad profile, a bad option, a sequence it cannot read), it says why on stderr, exits with code 2 and prints `PROFILE_ERROR` in place of the flags. The executor refuses that word. Keep the `|| exit` all the same: if the tool does not run at all (a wrong path), the shell gives the launch an empty string, and an empty `feature_overrides` flies with every flag off. When flags are on, the executor's first log line says `Feature overrides: ...`; no such line means none are.

Profiles and `feature_overrides` cover mission flags, the ones in a sequence's `features:` block. Node flags (`features.<flag>` parameters of `target_cv`, the grabbers and other nodes) are not reached by them yet; the bringup launch of PLT-07 turns those on.

Rules the code enforces (`drone_mission_core/features.py`):

- **A flag must be declared.** An override has to name a flag in the `features:` block of the sequence being flown. A typo stops the executor from starting; it does not silently fly the 2026 code.
- **Dependencies.** `FLAG_REQUIRES` lists flags that need another flag. A profile, or a launch line, that turns one on without the other is refused.
- **True or false only.** A flag set to anything else in a sequence file (`1`, `"true"`) stops the executor from starting.
- **Nothing on by default.** `comp_2026.yaml` enables nothing and is the fallback. With no `feature_overrides`, the mission's flags are exactly what its sequence file says.

The bringup launch that takes a profile by name comes with PLT-07.
