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

`feature_overrides` covers mission flags, the ones in a sequence's `features:` block. Node flags (`features.<flag>` parameters of `target_cv` and other nodes) are turned on by the bringup launch, from the profile's `nodes:` block (below).

Rules the code enforces (`drone_mission_core/features.py`):

- **A flag must be declared.** An override has to name a flag in the `features:` block of the sequence being flown. A typo stops the executor from starting; it does not silently fly the 2026 code.
- **Dependencies.** `FLAG_REQUIRES` lists flags that need another flag. A profile, or a launch line, that turns one on without the other is refused.
- **True or false only.** A flag set to anything else in a sequence file (`1`, `"true"`) stops the executor from starting.
- **Nothing on by default.** `comp_2026.yaml` enables nothing and is the fallback. With no `feature_overrides`, the mission's flags are exactly what its sequence file says.

## GO: a profile, a sequence and a params file to a flight

`tools/go.py` is the one command that starts a flight, in the sim and on the aircraft:

```bash
python3 tools/go.py --profile comp_2026 --sequence package_delivery_live.yaml --params live_target_mission.yaml \
    --start-camera mipi --detector target_cv --camera /camera/image=30 \
    --param-dump ~/today.param --log-dir /mnt/argus-logs --clock-source <ground laptop>
```

It does these things and stops at the first that fails:

1. Checks the profile against the files it will be flown with: every mission flag declared in the sequence, every node flag declared (false) in the params file, every dependency met. A flight that cannot be planned is `NO-GO` with exit code 2.
2. Starts the 2026 grabber, if the flight uses one (`--start-camera`), and gives it five seconds (`--camera-wait-s`): the preflight counts the camera's frames, so the camera has to be up before it. A grabber commands nothing, and is stopped again on NO-GO.
3. Runs the preflight (`ros2 run drone_utils preflight`, `src/drone_utils/README.md`) with the same profile and sequence. On NO-GO it prints `NO-GO`, exits with code 1 and starts no flight node. `--override "<reason>"` goes ahead; the preflight logs who and why.
4. Prints `GO` and runs the launch, waiting on it (a Ctrl-C stops the nodes as under `ros2 launch`; a SIGTERM to `go` is passed on): `ros2 launch drone_mission_demo argus_bringup.launch.py` with the sequence, the params file, the profile's mission flags (`feature_overrides`) and node flags (`node_flags`).

MAVROS must be up already: on the aircraft it is a boot service (`platform/jetson`), in the sim the sim starts it. Without it the preflight reads nothing and says NO-GO. The launch itself takes flags, not a profile: start flights with `go`, which is where the profile is checked.

`--dry-run` prints both commands and starts nothing. `--sim` puts every node, the preflight's included, on the sim's clock.

A profile for the bringup may carry two more blocks:

```yaml
description: the delivery with the target estimator
enable: [use_frame_time_pose, use_target_estimator]
nodes:                          # node flags: features.<flag> set true for that node, over the params file
  target_cv: [stamp_from_image, detection_contract]
start:                          # what the launch starts besides the take-off service and the executor
  camera: none                  # none, mipi or usb: the 2026 grabber
  detector: target_cv           # none or target_cv
```

- A node flag must be declared, `false`, under that node's `ros__parameters: features:` in the params file. The launch passes `features.<flag>: true` to the node; no file is edited.
- A node that exists only for its flag (`ring_fit`, `recovery_cv`, `loc_node`) is started when the profile turns that flag on, and not otherwise.
- Flags for a node the flight does not start are refused.
- A flight that starts `target_cv` with `use_frame_time_pose` or `time_health_guard` on needs `target_cv: [stamp_from_image]`: those mission flags read a detection's stamp as the frame's capture time.
- A sequence with a drop or a delivery needs the detector started (`--detector target_cv` or `start: detector:`).
- A sequence or params file given by name is the installed workspace's, and must be the same as the checked-out tree's: after editing one, rebuild (`colcon build`) or give the file's path.
- `tools/flag_profile.py` refuses a profile with a `nodes:` block: `feature_overrides` cannot carry node flags.
- `--start-camera` and `--detector` on the command line take the place of `start:`. What a flight needs started depends on its mission too, and `comp_2026.yaml` names neither: a 2026 delivery on the aircraft is `--start-camera mipi --detector target_cv`, a waypoint flight neither.
- The 2027 camera driver (`argus_camera`) is a boot service (`platform/jetson`), not something the launch starts.

`comp_2026.yaml` through `go` is the fallback: the bringup launch starts the same nodes with the same params file as the 2026 launch files, with every flag false. The 2026 launch files are unchanged and still work.

`delivery_estimator.yaml` is a worked example with node flags, flown in the sim (`sim/scenarios/delivery/go_estimator.yaml`). Which flags the competition flies is the lead's to decide; a profile for each mission is written then.
