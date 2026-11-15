# SETUP-1: argus_interfaces package skeleton

Flag: none. The package only defines message and service types; nothing publishes or subscribes to them yet · Status: DONE · Commits: `git log --oneline --grep '^SETUP-1:' platform/build-2027`

## What it solves

The 2027 features need message types that standard ROS packages do not have (a tracked marker, a request for crops of a 4K frame, a stage timing), and custom messages need their own `ament_cmake` package. This creates that package with the four definitions the build plan lists, so the localisation and platform lanes have something to build against. It is groundwork, not a fix: nothing uses the types yet.

## Architecture

New package `src/argus_interfaces` (`ament_cmake`, `rosidl_default_generators`):

- `msg/MarkerTrack.msg`: header, `track_id`, `state` (TENTATIVE 0, CONFIRMED 1, COASTING 2, LOST 3), `position_enu`, `covariance_en` (4 values), `digit_log_prob` (10 values), `digit`, `sightings`, `outside`.
- `msg/MarkerTrackArray.msg`: header and `MarkerTrack[] tracks`.
- `msg/StageTiming.msg`: header, `stage`, `latency_ms`.
- `srv/GetCrops.srv`: request `stamp` and `rois`; response `crops`, `ok`, `message`.

Depends on `std_msgs`, `geometry_msgs`, `sensor_msgs` and `builtin_interfaces`, all already in the sim image. No other package depends on it yet.

## Flow

Before: `colcon build` built six Python packages; there was nowhere to put a custom message.

After: it builds seven. `ros2 interface show argus_interfaces/msg/MarkerTrack` prints the definition, and `from argus_interfaces.msg import MarkerTrack` works in a node. The sim start-up builds the new package with the rest (the whole build took 3 s once built, about 5 s for this package the first time).

## Code

- `src/argus_interfaces/msg/MarkerTrack.msg`: the fields in the spec's order, each with a comment saying what it holds and in which unit.
- `src/argus_interfaces/CMakeLists.txt`: `rosidl_generate_interfaces` over the four files with their four dependencies.
- `src/argus_interfaces/test/test_interface_files.py`: parses the files as text and compares them field by field with the spec; checks the names and types are ones rosidl accepts, that CMake generates every file and names every package a field uses, and that `package.xml` is an interface package.

## Evidence

```
$ python3 -m pytest -q -p no:cacheprovider src/argus_interfaces/test/test_interface_files.py
12 passed in 0.01s
$ CHECK_STRICT=1 PR_TITLE=local PR_BODY=local scripts/check.sh | tail -1
OK
```

One field broken on purpose (`int8 digit` to `uint8 digit`): `test_marker_track` and `test_digit_can_say_unknown_and_the_state_constants_fit_their_field` failed; restored. The files were written before the test, so this deliberate break is the evidence that the test bites.

The build test, in the ROS 2 Humble sim image (`sim/container/build_ws.sh`, output in `sim/.state/setup1_build.log`):

```
Summary: 7 packages finished [3.03s]
$ ros2 interface list | grep argus_interfaces
    argus_interfaces/msg/MarkerTrack
    argus_interfaces/msg/MarkerTrackArray
    argus_interfaces/msg/StageTiming
    argus_interfaces/srv/GetCrops
$ ros2 interface show argus_interfaces/msg/StageTiming
std_msgs/Header header
string stage
float32 latency_ms
```

An earlier build of the package on its own in the same image (terminal output, not kept in a log) also imported the generated Python types: `MarkerTrack.CONFIRMED` was 1, `covariance_en` had 4 values, `digit_log_prob` had 10, and `GetCrops.Request().rois` was a list.

The sim still starts and flies with the seventh package in the workspace: `sim/run_scenarios.sh smoke` gave `OK delivery/baseline seed 1 off: 16.15 points, 0.08 m from the target`.

## Spec check

spec-auditor: COMPLETE in one round, every row DONE. It noted that the Python import result quoted above is not in the saved log (reworded to say so), and that the files were written before the test, with the deliberate break as the evidence.

flight-guardian did not run: the package defines types only; no mission, executor, target CV, node or `config/` YAML changed.

## Not done

- Nothing publishes or uses these types yet: LOC-5 (`GetCrops`), LOC-7 (`MarkerTrack`, `MarkerTrackArray`) and PLT-05 (`StageTiming`) do, and may add fields.
- `digit` has no default in the file, so a fresh message has 0, not −1; the publisher sets −1 for unknown (Decisions).
- The maintainer and licence lines in `package.xml` copy the other packages'; the licence is still unset repo-wide.
- The build test runs in the sim image, not in CI: CI has no ROS, so there the text parse test is the proof.
