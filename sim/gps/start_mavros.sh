#!/usr/bin/env bash
# Start MAVROS for the GPS missions, against the club simulator's SITL.
#
#   sim/gps/start_mavros.sh          build our three packages, then run MAVROS
#                                    in the foreground; Ctrl+C stops it
#   sim/gps/start_mavros.sh --down   remove the container if it was left behind
#
# The container comes from argus-sim:humble (sim/Dockerfile) with its
# entrypoint replaced, so it starts no Gazebo and no SITL: those come from the
# club sim (cd ~/Drone_Sim_Docker && make run).
#
# MAVROS connects straight to SITL's SERIAL1 (TCP 5762), not through MAVProxy.
# QGroundControl keeps UDP 14550. On MAVProxy's outputs (14550, 14551) every
# client shares SITL's one channel and its stream rates: MAVProxy requests
# 4 Hz on all streams every 15 s and QGC sets its own, so set_stream_rate.sh's
# 20 Hz would not hold. SERIAL1 has its own rates (SR1_*).
# FCU_URL=udp://:14551@ still works if you want the old way.
#
# The repo is mounted read-only. Only sim/.state/gps/ (build, ROS logs, run
# logs, the saved SITL home) is writable.
set -Eeuo pipefail

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
IMAGE=${ARGUS_SIM_IMAGE:-argus-sim:humble}
export FCU_URL=${FCU_URL:-tcp://127.0.0.1:5762}
PACKAGES=(drone_utils drone_mission_core drone_mission_demo)

case "${1:-}" in
    "") ;;
    --down) docker rm -f "$GPS_CONTAINER" >/dev/null 2>&1 || true; exit 0 ;;
    --help|-h) sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "ERROR: unknown option: $1" >&2; exit 2 ;;
esac

if ! docker info >/dev/null 2>&1; then
    echo "ERROR: this user cannot reach the Docker daemon." >&2
    exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "ERROR: image $IMAGE is missing. Build it once: sim/run_sim.sh --build" >&2
    exit 1
fi
if is_running "$GPS_CONTAINER"; then
    echo "ERROR: $GPS_CONTAINER is already running. Stop it with Ctrl+C in its terminal, or: sim/gps/start_mavros.sh --down" >&2
    exit 1
fi
if is_running argus-sim; then
    echo "ERROR: the vision sim (argus-sim) is running and has its own SITL and MAVROS. Stop it first: sim/run_sim.sh --down" >&2
    exit 1
fi
if ! is_running "$CLUB_CONTAINER"; then
    echo "WARNING: the club sim ($CLUB_CONTAINER) is not running yet. MAVROS will connect once it is: cd ~/Drone_Sim_Docker && make run" >&2
fi
docker rm -f "$GPS_CONTAINER" >/dev/null 2>&1 || true
mkdir -p "$GPS_STATE"

echo "MAVROS: fcu_url=$FCU_URL, ROS_DOMAIN_ID=$ARGUS_ROS_DOMAIN_ID"
exec docker run --rm --init -i --name "$GPS_CONTAINER" \
    --network host \
    --user "$(id -u):$(id -g)" \
    --entrypoint bash \
    -e ARGUS_REPO -e ARGUS_ROS_DOMAIN_ID -e FCU_URL \
    -v "$ARGUS_REPO:$ARGUS_REPO:ro" \
    -v "$GPS_STATE:$GPS_STATE" \
    -w "$ARGUS_REPO" \
    "$IMAGE" -c '
set -Eeuo pipefail
source "$ARGUS_REPO/sim/gps/env.sh"
echo "== colcon build: $*"
# Scan only these package folders: drone_mission_demo lists drone_target_cv
# as an exec_depend, and colcon insists on building any dependency it finds
# in the workspace. Left outside, it is treated as external; no GPS mission
# imports it.
paths=()
for pkg in "$@"; do paths+=("$ARGUS_REPO/src/$pkg"); done
colcon --log-base "$ARGUS_GPS_WS/log" build \
    --base-paths "${paths[@]}" \
    --build-base "$ARGUS_GPS_WS/build" \
    --install-base "$ARGUS_GPS_WS/install" \
    --event-handlers console_cohesion- summary+
source "$ARGUS_REPO/sim/gps/env.sh"   # now picks up the fresh install
echo "== MAVROS"
# apm.launch with our plugin list (apm.launch hard-codes its own).
exec ros2 launch "$ARGUS_REPO/sim/gps/mavros.launch.xml" \
    fcu_url:="$FCU_URL" \
    pluginlists_yaml:="$ARGUS_REPO/sim/gps/mavros_pluginlists.yaml"
' argus-gps "${PACKAGES[@]}"
