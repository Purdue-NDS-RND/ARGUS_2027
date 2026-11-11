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

## Status (parked, 2026-11-11)

Work in progress, not ready for review.

- Flights 1 and 2 passed.
- Flight 3 (`circuit_time_trial_live`) was cut off by a laptop suspend and
  has not been re-flown.
- `fly_all.sh` is untested.
- `flight_monitor.py` needs the suspend-safe clock fix: it times flights with
  the wall clock (`time.time()`), which jumps across a suspend.
- SITL home was restored to the CMAC defaults on 2026-11-11.
