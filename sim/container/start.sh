#!/usr/bin/env bash
# Container entrypoint (sim/compose.yaml). Starts, in order:
#   1. colcon build of our packages (SKIP_BUILD=1 to skip)
#   2. Gazebo Harmonic with sim/worlds/argus_runway.sdf
#   3. the red delivery target (sim/spawn_target.py; SPAWN_TARGET=0 to skip)
#   4. ros_gz_bridge: /camera/image and /camera/camera_info
#   5. ArduCopter SITL + MAVProxy: QGC on UDP 14550, MAVROS on UDP 14551
#   6. MAVROS (mavros_boot.sh sim)
# then checks that MAVROS connects and that SITL home is the world origin.
# If any of them stops, everything stops. Logs: sim/.state/logs/<run id>/.
set -Eeuo pipefail

# The whole script is one block, so bash reads it all before running it and
# editing this file while the sim runs cannot break the running copy.
{

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"

WORLD_FILE="$ARGUS_SIM/worlds/argus_runway.sdf"
GAZEBO_STARTUP_TIMEOUT=${GAZEBO_STARTUP_TIMEOUT:-90}
RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)
LOG_DIR="$ARGUS_STATE/logs/$RUN_ID"
SITL_DIR="$ARGUS_STATE/sitl"
mkdir -p "$LOG_DIR" "$SITL_DIR"
ln -sfn "$RUN_ID" "$ARGUS_STATE/logs/latest"

pids=()
names=()

stop_group() {
    local pid=$1
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.25
        done
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    echo "Stopping MAVROS, SITL, the bridge and Gazebo..."
    local i
    for ((i = ${#pids[@]} - 1; i >= 0; i--)); do
        stop_group "${pids[i]}"
    done
    exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# start <name> <command...>: run in its own process group, log to $LOG_DIR/<name>.log
start() {
    local name=$1
    shift
    setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
    pids+=("$!")
    names+=("$name")
    echo "  started $name (log: sim/.state/logs/$RUN_ID/$name.log)"
}

check_alive() {
    local i
    for i in "${!pids[@]}"; do
        if ! kill -0 "${pids[i]}" 2>/dev/null; then
            echo "ERROR: ${names[i]} stopped. Last lines of sim/.state/logs/$RUN_ID/${names[i]}.log:" >&2
            tail -n 25 "$LOG_DIR/${names[i]}.log" >&2 || true
            exit 1
        fi
    done
}

echo "== ARGUS sim, run $RUN_ID"

# 1. Our code
if [[ ${SKIP_BUILD:-0} == 1 ]]; then
    echo "== Skipping colcon build (SKIP_BUILD=1)"
else
    echo "== Building our packages (log: sim/.state/logs/$RUN_ID/colcon.log)"
    if ! "$ARGUS_SIM/container/build_ws.sh" >"$LOG_DIR/colcon.log" 2>&1; then
        tail -n 40 "$LOG_DIR/colcon.log" >&2
        echo "ERROR: colcon build failed. Full log: sim/.state/logs/$RUN_ID/colcon.log" >&2
        exit 1
    fi
    tail -n 3 "$LOG_DIR/colcon.log"
    set +u
    # shellcheck disable=SC1091
    source "$ARGUS_WS/install/setup.bash"
    set -u
fi

# 2. Gazebo
echo "== Starting Gazebo: $(basename "$WORLD_FILE")"
gz_args=(-v 3 -r)
if [[ ${HEADLESS:-0} == 1 ]]; then
    gz_args+=(-s --headless-rendering)
fi
start gazebo gz sim "${gz_args[@]}" "$WORLD_FILE"

echo "   waiting up to ${GAZEBO_STARTUP_TIMEOUT}s for the world clock..."
deadline=$((SECONDS + GAZEBO_STARTUP_TIMEOUT))
topics=
while ((SECONDS < deadline)); do
    check_alive
    topics=$(timeout 4 gz topic -l 2>/dev/null || true)
    grep -q '^/world/.*/clock$' <<<"$topics" && break
    sleep 1
done
if ! grep -q '^/world/.*/clock$' <<<"$topics"; then
    echo "ERROR: Gazebo did not publish a clock within ${GAZEBO_STARTUP_TIMEOUT}s." >&2
    tail -n 25 "$LOG_DIR/gazebo.log" >&2
    exit 1
fi
echo "   Gazebo is up."

# 3. Target
if [[ ${SPAWN_TARGET:-1} == 1 ]]; then
    echo "== Spawning the delivery target"
    python3 "$ARGUS_SIM/spawn_target.py" --world-file "$WORLD_FILE" 2>&1 | tee "$LOG_DIR/spawn_target.log"
    [[ ${PIPESTATUS[0]} == 0 ]] || exit 1
else
    echo "== Not spawning the target (SPAWN_TARGET=0)"
fi

# 4. Camera bridge
echo "== Starting the camera bridge"
start bridge ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p "config_file:=$ARGUS_SIM/config/bridge.yaml"

# 5. SITL. Home goes at the world origin so GPS and Gazebo positions agree.
location=$(python3 - "$WORLD_FILE" <<'EOF'
import sys
sys.path.insert(0, sys.argv[1].rsplit("/worlds/", 1)[0])
from target_placement import read_world, sitl_location_arg
print(sitl_location_arg(read_world(open(sys.argv[1]).read())[1]))
EOF
)
sitl_args=(
    -v ArduCopter
    -f gazebo-iris
    --model JSON
    --no-rebuild
    --no-extra-ports
    --use-dir "$SITL_DIR"
    --custom-location "$location"
    --out udp:127.0.0.1:14550
    --out udp:127.0.0.1:14551
    --mavproxy-args "--non-interactive --logfile=$LOG_DIR/mavlink.tlog"
)
case "${WIPE_PARAMS:-0}" in
    1|true|yes|on) sitl_args+=(--wipe-eeprom) ;;
esac
echo "== Starting ArduCopter SITL (home $location; QGC on 14550, MAVROS on 14551)"
export SITL_RITW_TERMINAL=/usr/local/bin/sitl-process-wrapper.sh
export SITL_PROCESS_LOG="$LOG_DIR/arducopter.log"
# sim_vehicle.py and MAVProxy live in the base image's user site.
cd /opt/ardupilot
start sitl env -u PYTHONNOUSERSITE sim_vehicle.py "${sitl_args[@]}"
cd "$ARGUS_REPO"

# 6. MAVROS
echo "== Starting MAVROS"
start mavros "$ARGUS_REPO/mavros_boot.sh" sim

# Wait for MAVROS to see the autopilot.
echo "   waiting up to 120s for /mavros/state connected..."
deadline=$((SECONDS + 120))
connected=0
while ((SECONDS < deadline)); do
    check_alive
    if timeout 5 ros2 topic echo --once --field connected /mavros/state 2>/dev/null | grep -qi true; then
        connected=1
        break
    fi
    sleep 2
done
if ((connected == 0)); then
    echo "ERROR: MAVROS did not connect to SITL on UDP 14551 within 120s." >&2
    tail -n 25 "$LOG_DIR/mavros.log" >&2
    exit 1
fi
echo "   MAVROS connected."

# Home: ArduPilot sets it once the EKF has GPS. Check it is the world origin,
# where spawn_target.py assumed it would be.
echo "   waiting up to 300s for home (EKF using GPS)..."
home=$(timeout 300 ros2 topic echo --once --field geo /mavros/home_position/home 2>/dev/null || true)
check_alive
python3 - "$WORLD_FILE" "$home" <<'EOF' | tee "$LOG_DIR/home_check.log"
import re, sys
sys.path.insert(0, sys.argv[1].rsplit("/worlds/", 1)[0])
from target_placement import offset_ne_m, read_world
_, origin = read_world(open(sys.argv[1]).read())
# `--field geo` prints a Python repr: GeoPoint(latitude=..., longitude=...)
lat = re.search(r"latitude[=:]\s*(-?[\d.]+)", sys.argv[2])
lon = re.search(r"longitude[=:]\s*(-?[\d.]+)", sys.argv[2])
if not (lat and lon):
    print("   WARNING: no /mavros/home_position/home within 300s; target position not checked against home")
    sys.exit(0)
n, e = offset_ne_m(origin.latitude_deg, origin.longitude_deg, float(lat.group(1)), float(lon.group(1)))
d = (n * n + e * e) ** 0.5
tag = "OK" if d <= 0.5 else "WARNING"
print(f"   {tag}: SITL home {float(lat.group(1)):.7f}, {float(lon.group(1)):.7f} is {d:.2f} m from the world origin")
EOF
echo "   home set after ${SECONDS}s."

# Pre-arm checks. Right after start-up SITL fails them for a few minutes
# ("Arm: Gyros inconsistent"), longer than simple_takeoff_service's arm
# retries, so only report READY once ArduPilot's SYS_STATUS pre-arm health
# bit (MAV_SYS_STATUS_PREARM_CHECK, 0x10000000) is set.
echo "   waiting up to 600s for ArduPilot pre-arm checks to pass..."
deadline=$((SECONDS + 600))
prearm=0
while ((SECONDS < deadline)); do
    check_alive
    health=$(timeout 5 ros2 topic echo --once --field sensors_health /mavros/sys_status 2>/dev/null | head -n 1 || true)
    if [[ $health =~ ^[0-9]+$ ]] && (( health & 0x10000000 )); then
        prearm=1
        break
    fi
    sleep 2
done
if ((prearm)); then
    echo "   pre-arm checks pass after ${SECONDS}s."
else
    echo "   WARNING: pre-arm checks still failing after 600s. Last FCU messages:"
    grep -E "PreArm|Arm:" "$LOG_DIR/mavros.log" | tail -n 3 || true
fi

cat <<EOF

== READY. In another terminal:
     sim/run_sim.sh --shell
     ros2 launch drone_mission_demo package_delivery_demo.launch.py
   QGroundControl on the host connects on UDP 14550.
   Ctrl+C here stops everything.

EOF

while true; do
    check_alive
    sleep 1
done
exit
}
