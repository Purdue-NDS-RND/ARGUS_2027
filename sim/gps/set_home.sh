#!/usr/bin/env bash
# Move the club sim's SITL home to the competition site, or put it back.
#
#   sim/gps/set_home.sh            save the current SIM_OPOS_* values, set the
#                                  competition home, reboot SITL
#   sim/gps/set_home.sh --restore  set the saved values back, reboot SITL
#   sim/gps/set_home.sh --show     print the current values
#
# The club sim passes no --home to SITL, so SITL starts at SIM_OPOS_LAT/LNG/
# ALT/HDG. Those are stored in the club sim's eeprom.bin and survive a
# restart: run --restore when you are done, or the next plain `make run` also
# starts at the competition site. Gazebo keeps running through the reboot.
#
# Needs MAVROS running (sim/gps/start_mavros.sh) and the vehicle disarmed.
# Parameters are read and written on SITL's SERIAL2 (TCP 5763), see sitl_params.py.
# Override the target with HOME_LAT, HOME_LNG, HOME_ALT (metres AMSL).
set -Eeuo pipefail

# shellcheck disable=SC1091
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
SAVED="$GPS_STATE/home-saved.txt"

# Centre of src/drone_mission_demo/config/waypoints/competition_waypoints.yaml.
# HOME_ALT must match the SRTM terrain height there, not just be close: SITL's
# simulated rangefinder (gazebo-iris.parm) measures down to that terrain, and
# with WPNAV_RFND_USE=1 a GUIDED takeoff lower than the rangefinder reading is
# refused (MAV_RESULT_FAILED). At 850 m it read 18.5 m on the ground and the
# 10 m takeoff was rejected. CMAC's default 584 m matches its terrain.
HOME_LAT=${HOME_LAT:-35.0497}
HOME_LNG=${HOME_LNG:--118.1509}
HOME_ALT=${HOME_ALT:-831.7}

action=set
case "${1:-}" in
    "") ;;
    --restore) action=restore ;;
    --show) action=show ;;
    --help|-h) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "ERROR: unknown option: $1" >&2; exit 2 ;;
esac
require_gps_container

in_gps bash -s "$action" "$SAVED" "$HOME_LAT" "$HOME_LNG" "$HOME_ALT" <<'EOS'
set -Eeuo pipefail
action=$1 saved=$2 lat=$3 lng=$4 alt=$5
NAMES=(SIM_OPOS_LAT SIM_OPOS_LNG SIM_OPOS_ALT SIM_OPOS_HDG)

# Parameters go over SITL's spare SERIAL2 port with pymavlink (sitl_params.py),
# not through MAVROS: on MAVROS's link the parameter download never finishes.
sitl() { env -u PYTHONNOUSERSITE python3 "$ARGUS_REPO/sim/gps/sitl_params.py" "$@"; }
get() {  # print one SITL parameter
    sitl get "$1" | awk '{print $2}'
}
set_param() {
    local out
    out=$(sitl set "$1=$2") || { echo "ERROR: setting $1=$2 failed" >&2; return 1; }
    echo "  $out" | sed 's/ / = /2'
}
show() {
    for n in "${NAMES[@]}"; do echo "  $n = $(get "$n")"; done
}
wait_connected() {
    for _ in $(seq 1 90); do
        if timeout 5 ros2 topic echo --once /mavros/state 2>/dev/null | grep -q 'connected: true'; then
            return 0
        fi
        sleep 1
    done
    echo "ERROR: MAVROS did not reconnect within 90 s" >&2
    return 1
}
reboot_sitl() {
    if timeout 5 ros2 topic echo --once /mavros/state | grep -q 'armed: true'; then
        echo "ERROR: the vehicle is armed; land and disarm first" >&2
        exit 1
    fi
    echo "Rebooting SITL (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)..."
    sitl reboot
    sleep 8
    wait_connected
    # A reboot resets SITL's stream rates on our link to 0: ask again.
    ./set_stream_rate.sh >/dev/null
    sleep 3
    echo "Reconnected. SITL position now:"
    for _ in $(seq 1 20); do
        fix=$(timeout 5 ros2 topic echo --once /mavros/global_position/global 2>/dev/null || true)
        if [[ -n $fix ]]; then
            grep -E '^(latitude|longitude|altitude):' <<<"$fix" | sed 's/^/  /'
            return 0
        fi
        sleep 2
    done
    echo "  (no /mavros/global_position/global yet; the EKF may still be initialising)"
}
check_rangefinder() {  # on the ground it should read about 0.2 m
    local r
    r=$(timeout 10 ros2 topic echo --once /mavros/rangefinder/rangefinder 2>/dev/null | awk '/^range:/{print $2}')
    if [[ -z $r ]]; then
        echo "  rangefinder: no reading yet"
    elif awk -v r="$r" 'BEGIN{exit !(r > 1.0)}'; then
        echo "WARNING: the rangefinder reads $r m on the ground: SIM_OPOS_ALT is that far above the terrain."
        echo "         GUIDED takeoffs below that height will be refused. Lower HOME_ALT by about $r m and rerun."
    else
        echo "  rangefinder: $r m (on the ground, OK)"
    fi
}

wait_connected
case $action in
    show)
        show
        ;;
    set)
        if [[ -f $saved ]]; then
            echo "Keeping the values saved earlier in $saved (run --restore to use them):"
            cat "$saved"
        else
            echo "Current values (saving them to $saved):"
            : > "$saved.tmp"
            for n in "${NAMES[@]}"; do
                v=$(get "$n")
                echo "$n $v" >> "$saved.tmp"
                echo "  $n = $v"
            done
            mv "$saved.tmp" "$saved"
        fi
        echo "Setting the competition home:"
        set_param SIM_OPOS_LAT "$lat"
        set_param SIM_OPOS_LNG "$lng"
        set_param SIM_OPOS_ALT "$alt"
        reboot_sitl
        echo "Stored values after the reboot:"
        show
        sleep 5
        check_rangefinder
        ;;
    restore)
        if [[ ! -f $saved ]]; then
            echo "ERROR: nothing saved in $saved" >&2
            exit 1
        fi
        echo "Restoring the saved values:"
        while read -r n v; do set_param "$n" "$v"; done < "$saved"
        reboot_sitl
        echo "Stored values after the reboot:"
        show
        mv "$saved" "$saved.restored-$(date -u +%Y%m%dT%H%M%SZ)"
        ;;
esac
EOS
