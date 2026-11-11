# Host-side helpers shared by the sim/gps scripts.
# shellcheck shell=bash

GPS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ARGUS_REPO=$(cd -- "$GPS_DIR/../.." && pwd)
GPS_STATE="$ARGUS_REPO/sim/.state/gps"
GPS_CONTAINER=argus-gps
CLUB_CONTAINER=ardupilot-gz-sim
export ARGUS_REPO
export ARGUS_ROS_DOMAIN_ID=${ARGUS_ROS_DOMAIN_ID:-42}

# What a flight holds locked. handle-lid-switch matters: logind ignores sleep
# locks when the lid closes (LidSwitchIgnoreInhibited=yes), and a lid close is
# what cut off flight 3 of the first session.
INHIBIT_WHAT=sleep:idle:handle-lid-switch

# Re-run the calling script under a systemd-inhibit lock, so the laptop does
# not suspend mid-flight. Changes no system settings and needs no sudo; the
# lock goes away when the script exits. Call it as: inhibit_sleep "$0" "$@"
# It returns only if the script is already locked (ARGUS_SLEEP_INHIBITED, so
# fly_all.sh -> run_mission.sh takes one lock) or no lock could be taken.
inhibit_sleep() {
    [[ -n ${ARGUS_SLEEP_INHIBITED:-} ]] && return 0
    if command -v systemd-inhibit >/dev/null 2>&1 \
        && systemd-inhibit --what="$INHIBIT_WHAT" --mode=block true >/dev/null 2>&1; then
        export ARGUS_SLEEP_INHIBITED=1
        exec systemd-inhibit --what="$INHIBIT_WHAT" --mode=block \
            --who="ARGUS sim/gps" --why="Flying a mission in the sim" \
            "$BASH" "$@"
    fi
    echo "WARNING: could not take a systemd-inhibit lock ($INHIBIT_WHAT): the laptop may suspend mid-flight. Keep the lid open." >&2
}

is_running() {
    [[ $(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null) == true ]]
}

require_gps_container() {
    if ! is_running "$GPS_CONTAINER"; then
        echo "ERROR: MAVROS is not running. Start it first in another terminal: sim/gps/start_mavros.sh" >&2
        exit 1
    fi
}

# Run a command inside the argus-gps container with ROS and our build sourced.
in_gps() {
    docker exec -i -w "$ARGUS_REPO" "$GPS_CONTAINER" \
        bash -c 'source "$ARGUS_REPO/sim/gps/env.sh" && "$@"' argus-gps "$@"
}
