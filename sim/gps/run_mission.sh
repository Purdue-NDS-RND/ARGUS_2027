#!/usr/bin/env bash
# Fly one mission sequence against the club sim, then say whether it passed.
#
#   sim/gps/run_mission.sh --check       pre-flight checks only (no arming)
#   sim/gps/run_mission.sh <sequence>    checks, then fly it, e.g. square_only,
#                                        waypoint_navigation_live,
#                                        circuit_time_trial_live
#   sim/gps/run_mission.sh --stop        stop a mission launch left running
#
# <sequence> is a file name in src/drone_mission_demo/config/sequences/
# without .yaml. If drone_mission_demo has a launch file of the same name, that
# launch runs with its defaults; otherwise multi_mission_demo.launch.py runs
# with sequence:=config/sequences/<sequence>.yaml.
#
# Launching arms the vehicle and takes off: run --check first and look at
# QGroundControl (GPS fix, EKF OK) before you fly.
#
# The run waits until the vehicle has armed, flown, landed and disarmed, then
# stops the launch and prints the result: missions completed, flight time,
# Gazebo's real-time factor, and for GPS missions the closest distance to each
# waypoint. Logs stay in sim/.state/gps/runs/ (not committed).
set -Eeuo pipefail

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
LAUNCH_PATTERN='ros2 launch drone_mission_demo'
TOPICS=(/mavros/local_position/pose /mavros/imu/data /mavros/extended_state /mavros/global_position/global)

stop_launch() {
    docker exec "$GPS_CONTAINER" pkill -INT -f "$LAUNCH_PATTERN" 2>/dev/null || return 0
    for _ in $(seq 1 30); do
        docker exec "$GPS_CONTAINER" pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1 || return 0
        sleep 0.5
    done
    docker exec "$GPS_CONTAINER" pkill -KILL -f "$LAUNCH_PATTERN" 2>/dev/null || true
}

preflight() {
    in_gps bash -s "${TOPICS[@]}" <<'EOS'
set -Eeuo pipefail
ok=1
echo "== ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
state=$(timeout 10 ros2 topic echo --once /mavros/state 2>/dev/null || true)
if grep -q 'connected: true' <<<"$state"; then
    echo "== /mavros/state: connected, $(grep -E '^(armed|mode):' <<<"$state" | tr '\n' ' ')"
else
    echo "FAIL: /mavros/state is not connected"; exit 1
fi
echo "== set_stream_rate.sh"
./set_stream_rate.sh >/dev/null && echo "   requested 20 Hz streams and EXTENDED_SYS_STATE at 2 Hz"
sleep 3
for topic in "$@"; do
    # Each ros2 CLI call starts Python and DDS discovery afresh, which can take
    # several seconds on a loaded laptop: allow 15 s, and one retry.
    rate=
    for _ in 1 2; do
        rate=$(timeout 15 ros2 topic hz "$topic" 2>/dev/null | awk '/average rate/{r=$3} END{print r}' || true)
        [[ -n $rate ]] && break
    done
    if [[ -n $rate ]]; then
        printf '== %-36s %6.1f Hz\n' "$topic" "$rate"
    else
        printf 'FAIL: %-34s not publishing\n' "$topic"; ok=0
    fi
done
fix=$(timeout 8 ros2 topic echo --once /mavros/gpsstatus/gps1/raw 2>/dev/null || true)
if [[ -n $fix ]]; then
    echo "== GPS: fix_type $(awk '/^fix_type:/{print $2}' <<<"$fix") (3 = 3D fix), $(awk '/^satellites_visible:/{print $2}' <<<"$fix") satellites"
fi
pos=$(timeout 8 ros2 topic echo --once /mavros/global_position/global 2>/dev/null || true)
echo "== position: $(grep -E '^(latitude|longitude):' <<<"$pos" | tr '\n' ' ')"
((ok)) || exit 1
EOS
}

action=fly
sequence=
case "${1:-}" in
    --check) action=check ;;
    --stop) action=stop ;;
    --help|-h) sed -n '2,23p' "$0"; exit 0 ;;
    "") sed -n '2,23p' "$0" >&2; exit 2 ;;
    -*) echo "ERROR: unknown option: $1" >&2; exit 2 ;;
    *) sequence=${1%.yaml} ;;
esac
require_gps_container

case $action in
    stop) stop_launch; exit 0 ;;
    check) preflight; echo "Pre-flight checks passed."; exit 0 ;;
esac

seq_file="src/drone_mission_demo/config/sequences/$sequence.yaml"
if [[ ! -f "$ARGUS_REPO/$seq_file" ]]; then
    echo "ERROR: no sequence $seq_file" >&2
    exit 2
fi
if docker exec "$GPS_CONTAINER" pgrep -f "$LAUNCH_PATTERN" >/dev/null 2>&1; then
    echo "ERROR: a mission launch is already running; stop it first: sim/gps/run_mission.sh --stop" >&2
    exit 1
fi
if in_gps timeout 10 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q 'armed: true'; then
    echo "ERROR: the vehicle is already armed" >&2
    exit 1
fi

preflight

if [[ -f "$ARGUS_REPO/src/drone_mission_demo/launch/$sequence.launch.py" ]]; then
    launch=(ros2 launch drone_mission_demo "$sequence.launch.py")
else
    launch=(ros2 launch drone_mission_demo multi_mission_demo.launch.py "sequence:=config/sequences/$sequence.yaml")
fi

run_dir="$GPS_STATE/runs/$sequence-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$run_dir"
in_gps python3 -c 'import sys, yaml
seq = yaml.safe_load(open(sys.argv[1]))["mission_sequence"]
print("\n".join(m["name"] for m in seq["missions"]))' "$seq_file" > "$run_dir/expected.txt"
echo "${launch[*]}" > "$run_dir/command.txt"

# Gazebo's real-time factor, from the club sim, every 5 s while we fly.
rtf_pid=
if is_running "$CLUB_CONTAINER"; then
    stats=$(docker exec "$CLUB_CONTAINER" gz topic -l 2>/dev/null | grep -m1 '^/world/.*/stats$' || true)
    if [[ -n $stats ]]; then
        (
            while :; do
                v=$(timeout 10 docker exec "$CLUB_CONTAINER" gz topic -e -t "$stats" -n 1 2>/dev/null \
                    | awk '/real_time_factor/{print $2; exit}' || true)
                [[ -n $v ]] && echo "$(date +%s.%N) $v" >> "$run_dir/rtf.log"
                sleep 5
            done
        ) &
        rtf_pid=$!
    fi
fi

cleanup() {
    trap - EXIT INT TERM
    [[ -n $rtf_pid ]] && kill "$rtf_pid" 2>/dev/null || true
    [[ -n ${tail_pid:-} ]] && kill "$tail_pid" 2>/dev/null || true
    pkill -P $$ -x tail 2>/dev/null || true
    docker exec "$GPS_CONTAINER" pkill -INT -f sim/gps/flight_monitor.py 2>/dev/null || true
    stop_launch
}
trap cleanup EXIT
trap 'echo; echo "Interrupted: stopping the mission launch (the vehicle keeps its current mode)."; exit 130' INT TERM

echo
echo "== Launching: ${launch[*]}"
echo "   log: $run_dir/launch.log"
docker exec -d -w "$ARGUS_REPO" "$GPS_CONTAINER" bash -c \
    'source "$ARGUS_REPO/sim/gps/env.sh" && exec "$@" > "$0" 2>&1' \
    "$run_dir/launch.log" "${launch[@]}"

# Show the mission's own log lines live while the monitor waits for the end.
tail -n +1 -F "$run_dir/launch.log" 2>/dev/null \
    | grep --line-buffered -E 'mission_executor|simple_takeoff_service|ERROR|WARN' &
tail_pid=$!
set +e
in_gps python3 sim/gps/flight_monitor.py > "$run_dir/state.log"
monitor=$?
set -e
sleep 2
cleanup

echo
case $monitor in
    0) ;;
    2) echo "Monitor timed out before the vehicle landed and disarmed." ;;
    3) echo "The vehicle never armed." ;;
    *) echo "Monitor exited with $monitor." ;;
esac
echo "== $sequence: $run_dir"
python3 "$ARGUS_REPO/sim/gps/flight_log.py" "$run_dir"
