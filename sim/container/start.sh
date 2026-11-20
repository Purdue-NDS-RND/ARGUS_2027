#!/usr/bin/env bash
# Container entrypoint (sim/compose.yaml). Starts, in order:
#   1. colcon build of our packages (SKIP_BUILD=1 to skip)
#   2. Gazebo Harmonic with sim/worlds/argus_runway.sdf
#   3. the red delivery target (sim/spawn_target.py; SPAWN_TARGET=0 to skip)
#   4. ros_gz_bridge: /camera/image and /camera/camera_info
#   5. ArduCopter SITL + MAVProxy: QGC on UDP 14550, MAVROS on UDP 14551
#   6. MAVROS (mavros_boot.sh sim)
# then checks that MAVROS connects and that SITL home is the world origin.
#
# With a site (ARGUS_SITE_FILE, set by `SITE=<name> sim/run_sim.sh`) the world
# is the base world moved to that site (sim/site_file.py), SITL's stored
# parameters are wiped, the 2026 delivery target is not spawned, MAVProxy gets
# a tools port (UDP 14552) and the site's geofence is uploaded before READY.
# With no site, none of that happens and the sim starts as it always has.
#
# With a run folder (ARGUS_RUN_DIR, set by `RUN_DIR=<folder> sim/run_sim.sh`;
# built by sim/scenario/build.py) the site is the scenario's, and the world,
# the bridge file and the SITL parameters are the ones in that folder.
#
# On sim time (SIM_TIME=1, or a run folder whose scenario has sim_time: true)
# Gazebo's clock is bridged to /clock (sim/clock_bridge.py, 250 messages per
# sim second), and the camera bridge and MAVROS follow it
# (use_sim_time), with MAVROS started from config/mavros/argus_mavros.launch.py
# on SITL's own second serial port (TCP 5762, not through MAVProxy), with its
# TIMESYNC on and the stream keeper holding the message rates. Mission launches then take
# use_sim_time:=true. Without it nothing here changes: ROS runs on wall time,
# as the 2026 sim did.
# If any of them stops, everything stops. Logs: sim/.state/logs/<run id>/.
set -Eeuo pipefail

# The whole script is one block, so bash reads it all before running it and
# editing this file while the sim runs cannot break the running copy.
{

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
# shellcheck disable=SC1091
source "$ARGUS_SIM/container/sitl_storage.sh"

WORLD_FILE="$ARGUS_SIM/worlds/argus_runway.sdf"
SITE_FILE=${ARGUS_SITE_FILE:-}
RUN_DIR=${ARGUS_RUN_DIR:-}
BRIDGE_FILE="$ARGUS_SIM/config/bridge.yaml"
TOOLS_PORT=14552
FAULTS_PORT=14553  # a second MAVLink output, for sim/inject.py, on runs whose scenario has gps_events (SIM-04)
MAVROS_PORT=5762  # SITL's SERIAL1, used by MAVROS on sim time
SIM_TIME=${SIM_TIME:-0}
if [[ -n $RUN_DIR ]]; then
    rm -f "$RUN_DIR/READY"
    # A scenario run is on sim time only if its run folder says so.
    SIM_TIME=$(python3 -c 'import json, sys; print(1 if json.load(open(sys.argv[1])).get("sim_time") is True else 0)' "$RUN_DIR/scenario.json")
    SITE_FILE=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["site_file"])' "$RUN_DIR/scenario.json")
    BRIDGE_FILE="$RUN_DIR/bridge.yaml"
    # A scenario that carries large camera frames asks for a Fast DDS profile (dds_profile; SIM-05). Its file is
    # named in a marker that env.sh reads, so every process of this container, the runner's tools among them,
    # uses the same transports. Without the key nothing is set and Fast DDS runs on its built-in transports.
    DDS_PROFILE=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get("dds_profile") or "")' "$RUN_DIR/scenario.json")
    rm -f "$ARGUS_DDS_PROFILE_FILE"
    if [[ -n $DDS_PROFILE ]]; then
        if [[ ! -f $ARGUS_SIM/config/fastdds_$DDS_PROFILE.xml ]]; then
            echo "ERROR: the run asks for the DDS profile '$DDS_PROFILE', and sim/config/fastdds_$DDS_PROFILE.xml does not exist." >&2
            exit 1
        fi
        echo "$ARGUS_SIM/config/fastdds_$DDS_PROFILE.xml" >"$ARGUS_DDS_PROFILE_FILE"
        export FASTRTPS_DEFAULT_PROFILES_FILE="$ARGUS_SIM/config/fastdds_$DDS_PROFILE.xml"
        echo "== Fast DDS profile for this run: $FASTRTPS_DEFAULT_PROFILES_FILE"
    fi
    # A camera profile too heavy to render beside a window (still4k) is flown headless or not at all.
    if [[ ${HEADLESS:-0} != 1 ]] && python3 -c 'import json, sys; sys.exit(0 if (json.load(open(sys.argv[1])).get("vehicle") or {}).get("headless_only") is True else 1)' "$RUN_DIR/scenario.json"; then
        echo "ERROR: this run's camera profile is headless only. Start it with HEADLESS=1 (sim/run_scenarios.sh does)." >&2
        exit 1
    fi
fi
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

# The site's world: the base world with its origin and runway moved.
if [[ -n $SITE_FILE ]]; then
    echo "== Site: $SITE_FILE"
    python3 "$ARGUS_SIM/site_file.py" check "$SITE_FILE" || exit 1
    if [[ -n $RUN_DIR ]]; then
        echo "== Scenario run folder: $RUN_DIR"
        WORLD_FILE="$RUN_DIR/world.sdf"
    else
        site_world="$ARGUS_STATE/worlds/$(basename "${SITE_FILE%.*}").sdf"
        python3 "$ARGUS_SIM/site_file.py" world "$SITE_FILE" --base "$WORLD_FILE" --out "$site_world" || exit 1
        WORLD_FILE=$site_world
    fi
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
if [[ -n $SITE_FILE ]]; then
    echo "== Not spawning the 2026 delivery target (its coordinates belong to the base world, not to a site)"
elif [[ ${SPAWN_TARGET:-1} == 1 ]]; then
    echo "== Spawning the delivery target"
    python3 "$ARGUS_SIM/spawn_target.py" --world-file "$WORLD_FILE" 2>&1 | tee "$LOG_DIR/spawn_target.log"
    [[ ${PIPESTATUS[0]} == 0 ]] || exit 1
else
    echo "== Not spawning the target (SPAWN_TARGET=0)"
fi

# 4. Camera bridge
bridge_args=(-p "config_file:=$BRIDGE_FILE")
if [[ $SIM_TIME == 1 ]]; then
    echo "== Sim time: bridging Gazebo's clock to /clock"
    start clock python3 "$ARGUS_SIM/clock_bridge.py"
    bridge_args+=(-p use_sim_time:=true)
fi
echo "== Starting the camera bridge"
start bridge ros2 run ros_gz_bridge parameter_bridge --ros-args "${bridge_args[@]}"

# 5. SITL. Home goes at the world origin so GPS and Gazebo positions agree.
location=$(python3 - "$WORLD_FILE" <<'EOF'
import os, sys
sys.path.insert(0, os.environ["ARGUS_SIM"])  # the world may be a generated one outside sim/worlds
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
# A site run, and the first plain run after one, start from wiped storage
# (sitl_storage.sh says why).
wipe=$(sitl_wipe_needed "$SITL_DIR" "$SITE_FILE" "${WIPE_PARAMS:-0}")
if ((wipe)); then
    sitl_args+=(--wipe-eeprom)
fi
if [[ -n $SITE_FILE ]]; then
    sitl_mark_storage "$SITL_DIR" "$SITE_FILE"
    # Flat ground at the site's elevation instead of SITL's terrain database,
    # so the SITL rangefinder reads 0 on the ground whatever the real terrain is.
    site_parm="$SITL_DIR/site.parm"
    printf 'SIM_TERRAIN 0\n' >"$site_parm"
    if [[ -n $RUN_DIR ]]; then
        site_parm="$RUN_DIR/sitl.parm"  # the scenario's own, which also has SIM_TERRAIN 0
    fi
    sitl_args+=(--add-param-file "$site_parm" --out "udp:127.0.0.1:$TOOLS_PORT")
    # The fault injector gets a port of its own: a driver or the payload bridge is already listening on the tools port.
    if [[ -n $RUN_DIR ]] && python3 -c 'import json, sys; sys.exit(0 if json.load(open(sys.argv[1])).get("gps_events") else 1)' "$RUN_DIR/scenario.json"; then
        sitl_args+=(--out "udp:127.0.0.1:$FAULTS_PORT")
    fi
fi
echo "== Starting ArduCopter SITL (home $location; QGC on 14550, MAVROS on 14551)"
export SITL_RITW_TERMINAL=/usr/local/bin/sitl-process-wrapper.sh
export SITL_PROCESS_LOG="$LOG_DIR/arducopter.log"
# sim_vehicle.py and MAVProxy live in the base image's user site.
cd /opt/ardupilot
start sitl env -u PYTHONNOUSERSITE sim_vehicle.py "${sitl_args[@]}"
cd "$ARGUS_REPO"

# 6. MAVROS
if [[ $SIM_TIME == 1 ]]; then
    # MAVROS goes straight to SITL's second serial port, as it is wired to the
    # flight controller on the aircraft. Through MAVProxy every message is
    # relayed by a Python process, and MAVProxy's own stream requests replace
    # ours (poses came at 4 Hz). Nothing is sent on this port until it is
    # asked for, which is the stream keeper's job.
    echo "   waiting up to 120s for SITL's second serial port (TCP $MAVROS_PORT)..."
    deadline=$((SECONDS + 120))
    until grep -q "SERIAL1 on TCP port $MAVROS_PORT" "$LOG_DIR/arducopter.log" 2>/dev/null; do
        check_alive
        if ((SECONDS >= deadline)); then
            echo "ERROR: SITL did not open TCP $MAVROS_PORT within 120s." >&2
            exit 1
        fi
        sleep 1
    done
    # The sim's own plugin list: the flight's, plus optical flow, vision pose
    # and speed, and distance sensor (sim/config/mavros_sim.yaml, SIM-05).
    echo "== Starting MAVROS on sim time (sim/config/mavros_sim.yaml) with the stream keeper"
    start mavros ros2 launch "$ARGUS_REPO/config/mavros/argus_mavros.launch.py" \
        "fcu_url:=tcp://127.0.0.1:$MAVROS_PORT" "config:=$ARGUS_SIM/config/mavros_sim.yaml" use_sim_time:=true stream_keeper:=true
else
    echo "== Starting MAVROS"
    start mavros "$ARGUS_REPO/mavros_boot.sh" sim
fi

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
if [[ -z $SITE_FILE ]]; then
    sitl_mark_storage "$SITL_DIR" ""  # SITL is up, so any wipe has happened
fi

# Home: ArduPilot sets it once the EKF has GPS. Check it is the world origin,
# where spawn_target.py assumed it would be.
echo "   waiting up to 300s for home (EKF using GPS)..."
home=$(timeout 300 ros2 topic echo --once --field geo /mavros/home_position/home 2>/dev/null || true)
check_alive
python3 - "$WORLD_FILE" "$home" <<'EOF' | tee "$LOG_DIR/home_check.log"
import os, re, sys
sys.path.insert(0, os.environ["ARGUS_SIM"])
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

# The site's geofence, uploaded and switched on before the pre-arm wait, so
# READY also means ArduPilot accepts the fence with the vehicle inside it.
if [[ -n $SITE_FILE ]]; then
    echo "== Uploading the geofence (MAVLink tools port, UDP $TOOLS_PORT)"
    env -u PYTHONNOUSERSITE python3 "$ARGUS_SIM/fence_upload.py" --site "$SITE_FILE" --allow-unverified \
        --url "udpin:127.0.0.1:$TOOLS_PORT" 2>&1 | tee "$LOG_DIR/fence_upload.log"
    if [[ ${PIPESTATUS[0]} != 0 ]]; then
        echo "ERROR: the geofence was not uploaded. Log: sim/.state/logs/$RUN_ID/fence_upload.log" >&2
        exit 1
    fi
fi

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

# On sim time, also wait until MAVROS stamps poses with the flight
# controller's time: it only applies its time-sync offset once the
# synchroniser has converged, which in the sim takes about two minutes from
# MAVROS's start (docs/results/2026-11-17-time-base.md).
if [[ $SIM_TIME == 1 ]]; then
    echo "   waiting up to 300s for MAVROS's time sync to be applied..."
    check_alive
    if python3 "$ARGUS_SIM/time_check.py" --wait-synced 300 2>&1 | tee "$LOG_DIR/time_sync_wait.log" | sed 's/^/   /' && [[ ${PIPESTATUS[0]} == 0 ]]; then
        echo "   time sync applied after ${SECONDS}s."
    else
        echo "   WARNING: MAVROS's pose stamps are still arrival times, not synchronised flight-controller time."
    fi
fi

if [[ -n $RUN_DIR ]]; then
    date -u +%Y-%m-%dT%H:%M:%SZ >"$RUN_DIR/READY"  # sim/run_scenarios.sh waits for this file
fi
cat <<EOF

== READY. In another terminal:
     sim/run_sim.sh --shell
     ros2 launch drone_mission_demo package_delivery_demo.launch.py$([[ $SIM_TIME == 1 ]] && echo ' use_sim_time:=true')
   QGroundControl on the host connects on UDP 14550.
   Ctrl+C here stops everything.

EOF

while true; do
    check_alive
    sleep 1
done
exit
}
