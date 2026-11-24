# GO launcher (PLT-07): the real command starting sim flights

2026-11-23, build machine, ArduCopter SITL 4.6.3, ROS 2 Humble in the sim's container. In these scenarios the sim's runner starts `tools/go.py` in place of a launch file: the command checks the profile, runs the preflight (`ros2 run drone_utils preflight`) and, on GO, runs `argus_bringup.launch.py`. The run's own sequence and params files keep every flag false; flags reach the flight only through the committed profile. `--variants on` puts the runs on the sim's clock.

The three results below are of the final code: each scenario was flown once more after the last change to `tools/go.py` and the runner, and the two that launch once more after the last change to the launch file (its `use_sim_time` argument put in the shape `sim/test/test_time_base.py` holds every executor launch to).

| Scenario | Profile | Result |
| --- | --- | --- |
| `delivery/go` | `comp_2026` | OK: `GO`, four nodes started with no flag, 16.21 points at 0.03 m, the flags-off baseline's score |
| `delivery/go_estimator` | `delivery_estimator` | OK: `GO`, the executor logs `Feature overrides: use_frame_time_pose,use_target_estimator`, 1766 of 1767 detections carry their frame's stamp; the time, rate, projection and estimator checks pass; 16.20 points at 0.04 m |
| `delivery/go_nogo` | `comp_2026`, SITL's own parameters and battery | OK: `NO-GO: params, battery`, exit code 1, no node started, the vehicle watched for 10 s and never armed (`NOT_STARTED`) |

```
$ sim/run_scenarios.sh delivery/go --variants on
   OK          delivery/go seed 1 on: 16.21 points, 0.03 m from the target
$ sim/run_scenarios.sh delivery/go_estimator --variants on
   OK          delivery/go_estimator seed 1 on: 16.20 points, 0.04 m from the target
$ sim/run_scenarios.sh delivery/go_nogo --variants on
   OK          delivery/go_nogo seed 1 on: 0.00 points, nothing released
```

The command's own lines in the GO flight (`launch.log` of the run; paths shortened):

```
profile: profiles/comp_2026.yaml; mission flags: (none: the 2026 code); node flags: (none)
nodes: simple_takeoff_service, target_cv, mission_executor, session_logger
sequence: sim/.state/suites/delivery-go-20261124T002557Z/runs/delivery-go-s1-on/sequence.yaml
params: sim/.state/suites/delivery-go-20261124T002557Z/runs/delivery-go-s1-on/params.yaml
[   GO] params       flight controller holds to the baseline, sha256 be80db09cb93
[   GO] fence        enabled and healthy, 6 points stored
[   GO] gps          fix type 6, HDOP 1.21, 10 satellites
[   GO] ekf          estimate healthy, pre-arm checks pass
[   GO] rangefinder  0.00 m
[   GO] cameras      /camera/image 26.5 Hz
[   GO] engine       no engine was named (--engine): none is checked
[   GO] timesync     flight controller sync steady to 0.0 ms, largest step 0.0 ms; the companion's clock was not compared with a ground laptop's (no --clock-source)
[   GO] disk         11 GB free in sim/.state/suites/delivery-go-20261124T002557Z/runs/delivery-go-s1-on
[   GO] battery      25.0 V, 4.17 V a cell
[   GO] profile      comp_2026: every flag off (the 2026 code)
GO
GO
```

and in the NO-GO flight:

```
profile: profiles/comp_2026.yaml; mission flags: (none: the 2026 code); node flags: (none)
nodes: simple_takeoff_service, target_cv, mission_executor, session_logger
sequence: sim/.state/suites/delivery-go_nogo-20261124T001107Z/runs/delivery-go_nogo-s1-on/sequence.yaml
params: sim/.state/suites/delivery-go_nogo-20261124T001107Z/runs/delivery-go_nogo-s1-on/params.yaml
[NO-GO] params       not the baseline be80db09cb93: BATT_LOW_VOLT is 10.5, the baseline has 21 (and 8 more)
[   GO] fence        enabled and healthy, 6 points stored
[   GO] gps          fix type 6, HDOP 1.21, 10 satellites
[   GO] ekf          estimate healthy, pre-arm checks pass
[   GO] rangefinder  0.00 m
[   GO] cameras      /camera/image 24.0 Hz
[   GO] engine       no engine was named (--engine): none is checked
[   GO] timesync     flight controller sync steady to 0.0 ms, largest step 0.0 ms; the companion's clock was not compared with a ground laptop's (no --clock-source)
[   GO] disk         11 GB free in sim/.state/suites/delivery-go_nogo-20261124T001107Z/runs/delivery-go_nogo-s1-on
[NO-GO] battery      12.6 V is 2.10 V a cell on 6 cells (at least 3.9)
[   GO] profile      comp_2026: every flag off (the 2026 code)
NO-GO: params, battery
[ros2run]: Process exited with failure 1
NO-GO
```

## What went wrong on the way, and what it changed

- **An empty launch argument.** The first GO flight ended at `malformed launch argument 'feature_overrides:=', expected format '<name>:=<value>'`: `ros2 launch` refuses a name with nothing after `:=`. With no flags `go` now leaves the two arguments out and the launch file's defaults (no flags) stand. The runner marked that flight FAIL at once (the command ended, exit code 1, the vehicle never armed) instead of waiting out the arming timeout.
- **Node flags that did not arrive.** The first flags-on flight was INVALID: `only 27 of 1874 detections carry the stamp of a frame`, and the estimator had no track. The launch gave each node its flags as a parameter dictionary; launch_ros writes that under the wildcard `/**`, and on a node's command line a wildcard entry loses to the params file's entry for the node by name whenever a global parameter rule comes first, as `use_sim_time` does in the sim. On the aircraft, with no such rule, the same launch would have turned the flags on: the sim and the aircraft would have differed. Reproduced outside the sim with a ten-line node on Humble and on the build machine's Jazzy:

  ```
  -p use_sim_time:=True --params-file a.yaml --params-file wildcard.yaml   ->  False   (a.yaml: the flag false under the node's name)
  -p use_sim_time:=True --params-file a.yaml --params-file named.yaml      ->  True
                        --params-file a.yaml --params-file wildcard.yaml   ->  True
  ```

  The launch now writes a node's flags to a small params file keyed by the node's name and gives it after the flight's params file (`bringup.feature_parameters`). The flags-on flight above is of that code.

- **A grabber that would have outlived its NO-GO.** The spec audit asked whether stopping the `ros2 run` wrapper stops the node. On Humble, in the sim's image, with a stand-in node (`ros2 run tf2_ros static_transform_publisher ...`):

  ```
  after SIGTERM to the wrapper alone: 1 left   (the node itself)
  after go._stop: 0 left, wrapper exit 0, took 0.2 s
  ```

  `go` starts a grabber in a process group of its own and ends the group: an interrupt, then SIGTERM, then SIGKILL, until nothing of it is left.

## Not shown here

- The aircraft. `go` has only run in the sim's container.
- A 2026 grabber started by `go` before the preflight (`--start-camera mipi|usb`): the sim's camera is Gazebo's and is up before the command. Unit-tested with stand-ins (`tools/test/test_go.py`).
- An override, a SIGTERM to `go`, and the installed-against-source file check: unit-tested.
- The time sync, which reads 0.0 ms on the sim's clock (see `2026-11-23-preflight.md`).
