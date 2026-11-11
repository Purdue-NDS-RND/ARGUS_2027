# Host-side helpers shared by the sim/gps scripts.
# shellcheck shell=bash

GPS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ARGUS_REPO=$(cd -- "$GPS_DIR/../.." && pwd)
GPS_STATE="$ARGUS_REPO/sim/.state/gps"
GPS_CONTAINER=argus-gps
CLUB_CONTAINER=ardupilot-gz-sim
export ARGUS_REPO
export ARGUS_ROS_DOMAIN_ID=${ARGUS_ROS_DOMAIN_ID:-42}

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
