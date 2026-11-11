# GPS missions against the club sim

Scripts for flying last year's non-vision missions (square, waypoint
navigation, circuit time trial) against the club simulator
(`~/Drone_Sim_Docker`, `make run`) with MAVROS in an `argus-sim:humble`
container. Each script's header comment explains its usage.

- `start_mavros.sh`: build our packages and run MAVROS (container `argus-gps`)
- `set_home.sh`: move SITL home to the competition site, `--restore` puts it back
- `run_mission.sh`: pre-flight checks, fly one sequence, report pass/fail
- `fly_all.sh`: fly all three in a row
- `flight_monitor.py`, `flight_log.py`: watch and judge a flight
  (tests: `sim/test/test_gps_flight_log.py`)

## Flying

Start the club sim and QGroundControl yourself, then:

    sim/gps/fly_all.sh --restore-home       # asks for the QGC check first

or one at a time:

    sim/gps/start_mavros.sh                  # its own terminal
    sim/gps/set_home.sh                      # saves the current home first
    sim/gps/run_mission.sh --check
    sim/gps/run_mission.sh circuit_time_trial_live
    sim/gps/set_home.sh --restore

`run_mission.sh` (when it flies) and `fly_all.sh` hold a
`systemd-inhibit --what=sleep:idle:handle-lid-switch` lock until they exit, so
idle, the sleep key and closing the lid do not suspend the laptop mid-flight.
No sudo and no system settings change; if the lock can't be taken they warn
and fly anyway. The lid lock matters: logind ignores sleep locks when the lid
closes, and a lid close is what cut off the first time-trial attempt.

`flight_monitor.py` times flights on the monotonic clock (`FlightWatch` in
`flight_log.py`), so a suspend no longer fires its timeouts, and the reported
flight time leaves out suspended time.

## Status (2026-11-11)

All three missions flown in the sim and passed: `circuit_time_trial_live` on
its own, then `fly_all.sh --yes` (square, waypoint navigation, time trial).
Results: `docs/results/2026-11-11-gps-missions.md`. SITL home was restored
to CMAC afterwards.

SITL GPS is ideal (no noise), so the waypoint distances are far tighter than
real GPS would give.
