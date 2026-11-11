#!/usr/bin/env bash
# Fly last year's three non-vision missions in a row against the club sim.
#
#   sim/gps/fly_all.sh                  square, waypoint navigation, time trial
#   sim/gps/fly_all.sh waypoint_navigation_live    just the ones you name
#
# Options:
#   --yes            don't ask for the QGroundControl check before arming
#   --restore-home   put the club sim's SITL home back when done
#   --stop-mavros    stop MAVROS when done (if this script started it)
#
# Start these yourself first, each in its own terminal:
#   cd ~/Drone_Sim_Docker && make run     (wait for the Iris in Gazebo)
#   QGroundControl
#
# The script then starts MAVROS in the background if it isn't running (log in
# sim/.state/gps/mavros.log), moves SITL home to the competition site if it
# isn't there yet, runs the pre-flight checks, asks you to confirm QGC shows
# a GPS fix and EKF OK, and flies each sequence with sim/gps/run_mission.sh.
# It stops at the first mission that fails.
set -Eeuo pipefail

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
DEFAULT_SEQUENCES=(square_only waypoint_navigation_live circuit_time_trial_live)

ask=1 restore=0 stop_mavros=0 sequences=()
for arg in "$@"; do
    case $arg in
        --yes|-y) ask=0 ;;
        --restore-home) restore=1 ;;
        --stop-mavros) stop_mavros=1 ;;
        --help|-h) sed -n '2,21p' "$0"; exit 0 ;;
        -*) echo "ERROR: unknown option: $arg" >&2; exit 2 ;;
        *) sequences+=("${arg%.yaml}") ;;
    esac
done
((${#sequences[@]})) || sequences=("${DEFAULT_SEQUENCES[@]}")

# Hold off suspend (idle, sleep keys and the lid) until every flight is done;
# run_mission.sh sees the lock and does not take a second one.
inhibit_sleep "$0" "$@"

if ! is_running "$CLUB_CONTAINER"; then
    echo "ERROR: the club sim is not running. In another terminal: cd ~/Drone_Sim_Docker && make run" >&2
    exit 1
fi

started_mavros=0
if ! is_running "$GPS_CONTAINER"; then
    echo "== Starting MAVROS in the background (log: sim/.state/gps/mavros.log)"
    mkdir -p "$GPS_STATE"
    nohup "$GPS_DIR/start_mavros.sh" > "$GPS_STATE/mavros.log" 2>&1 < /dev/null &
    started_mavros=1
fi
echo "== Waiting for MAVROS to connect to SITL..."
connected=0
for _ in $(seq 1 90); do
    if is_running "$GPS_CONTAINER" \
        && in_gps timeout 5 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q 'connected: true'; then
        connected=1
        break
    fi
    sleep 2
done
if ((!connected)); then
    echo "ERROR: MAVROS did not connect within 3 minutes. See sim/.state/gps/mavros.log" >&2
    exit 1
fi

lat=$("$GPS_DIR/set_home.sh" --show | awk '/SIM_OPOS_LAT/{print $3}')
if awk -v lat="$lat" 'BEGIN{exit !(lat > 35.04 && lat < 35.06)}'; then
    echo "== SITL home is already at the competition site (SIM_OPOS_LAT $lat)"
else
    echo "== Moving SITL home to the competition site (SIM_OPOS_LAT is $lat)"
    "$GPS_DIR/set_home.sh"
    echo "== Letting the EKF settle after the reboot (30 s)"
    sleep 30
fi

"$GPS_DIR/run_mission.sh" --check

if ((ask)); then
    echo
    read -r -p "Does QGroundControl show a GPS fix and EKF OK, with no pre-arm warnings? Launching arms the vehicle. [y/N] " answer
    if [[ ! $answer =~ ^[Yy] ]]; then
        echo "Not flying."
        exit 1
    fi
fi

results=()
status=0
for sequence in "${sequences[@]}"; do
    echo
    echo "################ $sequence ################"
    if "$GPS_DIR/run_mission.sh" "$sequence"; then
        results+=("PASS  $sequence")
    else
        results+=("FAIL  $sequence")
        status=1
        echo "== $sequence failed; not flying the rest. Its logs are in sim/.state/gps/runs/." >&2
        break
    fi
    sleep 5
done

if ((restore)); then
    echo
    "$GPS_DIR/set_home.sh" --restore
fi
if ((stop_mavros && started_mavros)); then
    "$GPS_DIR/start_mavros.sh" --down
fi

echo
echo "================ Summary ================"
printf '%s\n' "${results[@]}"
exit "$status"
