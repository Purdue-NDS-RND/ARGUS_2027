# The health monitor in the sim (PLT-11), 2026-11-24

## Commands

```
sim/run_scenarios.sh delivery/health_monitor --variants on
docker run --rm -v "$PWD:$PWD" -w "$PWD" argus-sim:humble python3 sim/statustext_from_tlog.py sim/.state/logs/latest/mavlink.tlog --prefix "ARGUS H"
```

The 2026 delivery (no mission flag on) with the health monitor node started and its flag on, seed 1, calm. The scenario tells the monitor to watch the camera and the pose at 30 a second each, and a topic nobody publishes (`ghost`, 5 a second). "The computer" in the sim is this laptop, read from inside the sim's container. Three flights were flown as the node changed; the last is on the code as committed and is the one below.

## The flight

`OK`, 16.21 points, 0.03 m from the target: the delivery itself is the 2026 one.

The monitor's own lines (the launch log):

```
Health monitor on (features.health_monitor): camera at 30 Hz, pose at 30 Hz, ghost at 5 Hz; the computer every 1 s; findings to the ground station as STATUSTEXT
Health: ARGUS H:ON 3 rate(s) watched
Health: camera 26.5/30 Hz, pose 0/30 Hz, ghost 0/5 Hz; hottest zone 91 C; CPU not known; GPU not known; throttled no; 0 finding(s)
Health: ARGUS H:TEMP TCPU 92C
Health: ARGUS H:SILENT ghost
Health: ARGUS H:RATE pose 4/30Hz
Health: camera 27.1/30 Hz, pose 3.9/30 Hz, ghost 0/5 Hz; hottest zone 68 C; CPU 48.1%; GPU not known; throttled no; 2 finding(s)
Health: ARGUS H:SILENT ghost
Health: ARGUS H:RATE pose 4/30Hz
```

What reached the ground station's side of the link (the flight's telemetry log, as a ground station records it):

```
14 status text(s) starting with 'ARGUS H' in the telemetry log
(1, 191, 5, 'ARGUS H:ON 3 rate(s) watched')
(1, 191, 4, 'ARGUS H:TEMP TCPU 92C')
(1, 191, 4, 'ARGUS H:SILENT ghost')
(1, 191, 4, 'ARGUS H:RATE pose 4/30Hz')
(1, 191, 4, 'ARGUS H:SILENT ghost')
```

(system 1, component 191: the companion computer; severity 5 is a notice, 4 a warning.)

## What it shows

- The monitor says it has started (`ARGUS H:ON`), so a ground station that has had no line has no monitor. The silent topic was found and said to the ground station, eight seconds after the monitor started (three silent, five held), and again every thirty seconds.
- The other lines were not planted. The laptop's CPU was at 93 °C when the flight began (the limit is 85): true. The pose came at 4 a second, where 30 is expected: true of a flight with the 2026 start-up, in which nothing asks the flight controller for the 30 a second that the stream keeper (PLT-06) holds in a 2027 start-up. The camera's rate was 23 to 28 of 30 across the three flights and twice fell under the four fifths that is the line (20 and 21 of 30). Throttling read "no" throughout: this laptop's CPUs were allowed their full frequency.
- The CPU's load is known from the second reading on. The GPU's is "not known" here: this laptop has no file that says it (the Jetson has).
- `ARGUS H:OK` was not seen: something was wrong throughout (the ghost topic, by design).

## The shipped params file in ROS's own parser

```
docker run --rm -v "$PWD:$PWD" -w "$PWD" -e ROS_LOG_DIR=/tmp/roslog argus-sim:humble bash -lc "source /opt/ros/humble/setup.bash && python3 <a node made with automatically_declare_parameters_from_overrides, --params-file src/drone_mission_demo/config/params/sim_target_mission.yaml>"
health_monitor 7 {'features.health_monitor': False, 'rates': ['camera:camera/image:sensor_msgs/msg/Image:30'], 'temp_limit_c': 85.0}
target_cv 8 {'image_topic': 'camera/image'}
mission_executor 2 {'loop_rate_hz': 20.0}
ring_fit 6 {'image_topic': 'camera/image'}
```

The sim flies a rewritten copy of the params file, so a flight does not show that the file as shipped parses. This does: every node of the delivery launch reads its own block from it, the new one included.

## Not shown

What the monitor's own subscription costs the camera's stream (it takes every frame, undecoded): the camera's rate was not measured in the same flight with the monitor off. A flight in which nothing is wrong and the monitor stays silent after `ON`, and one that ends a finding with `ARGUS H:OK` (both are unit tests; the scenario's silent topic never comes back). The Jetson's own temperatures, throttling and GPU load (the files are read the same way; none of it has run there); QGroundControl showing the lines (the telemetry log holds them; nobody looked at a screen); dropped frames in a flight (unit tests only: no survey was flown with the monitor); one seed.
