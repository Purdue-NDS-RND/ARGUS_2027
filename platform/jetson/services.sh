#!/usr/bin/env bash
# Boot services and the shared clock (PLT-02). Run it after setup.sh.
#
#   sudo platform/jetson/services.sh --check
#   sudo platform/jetson/services.sh --apply --ground-ip 192.168.10.2      on the Jetson
#   sudo platform/jetson/services.sh --apply --role ground --allow 192.168.10.0/24
#                                                                          on the ground laptop
#   sudo platform/jetson/services.sh --apply --only chrony --ground-ip <address>
#
# On the Jetson: systemd units that start MAVROS and the camera driver at boot
# as /etc/argus/boot.env says, and chrony following the ground laptop's clock.
# On the ground laptop: chrony serving its clock to that network, with or
# without internet. Missions are never started at boot: only the GO launcher
# starts them.
#
# Exit code: 0 when every step asked for is ok, 1 when one is not, 2 for a wrong call.

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# shellcheck source=platform/jetson/lib.sh
source "$HERE/lib.sh"

usage() {
    sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
}

BOOT_ENV=/etc/argus/boot.env
BOOT_KEYS=(ARGUS_BOOT_MAVROS ARGUS_BOOT_CAMERA ARGUS_BOOT_CAMERA_LEGACY_TOPICS ARGUS_WS)
GROUND_IP=""
ALLOW=""

# ---------------------------------------------------------------- boot_env

boot_env_text() {
    cat <<EOF
# What starts when the Jetson boots (platform/jetson/boot/argus-boot.sh reads this).
# Missions never start at boot: only the GO launcher starts them.
# After a change: sudo systemctl restart argus-mavros argus-camera

# apm: MAVROS's own apm.launch, as in 2026. argus: config/mavros/argus_mavros.launch.py
# with the stream keeper. off: do not start MAVROS.
ARGUS_BOOT_MAVROS=apm

# off: no camera driver at boot; the 2026 launch files start their own grabber and two
# readers cannot share the camera. Or a sensor mode of argus_camera, e.g. imx519_720p60.
ARGUS_BOOT_CAMERA=off
# true: the camera driver also serves the 2026 topic names.
ARGUS_BOOT_CAMERA_LEGACY_TOPICS=false

# The workspace whose install/setup.bash the services source.
ARGUS_WS=$REPO
EOF
}

check_boot_env() {
    local key
    if [[ ! -f "$SYSROOT$BOOT_ENV" ]]; then
        say "$BOOT_ENV does not exist"
        return 1
    fi
    for key in "${BOOT_KEYS[@]}"; do
        if ! grep -q "^$key=" "$SYSROOT$BOOT_ENV"; then
            say "$BOOT_ENV has no $key= line: add it (this script does not rewrite a file that exists)"
            return 1
        fi
    done
    say "$BOOT_ENV: MAVROS $(sed -n 's/^ARGUS_BOOT_MAVROS=//p' "$SYSROOT$BOOT_ENV" | tail -n 1), camera $(sed -n 's/^ARGUS_BOOT_CAMERA=//p' "$SYSROOT$BOOT_ENV" | tail -n 1)"
}

apply_boot_env() {
    [[ -f "$SYSROOT$BOOT_ENV" ]] && return 1
    mkdir -p "$SYSROOT/etc/argus"
    boot_env_text >"$SYSROOT$BOOT_ENV"
}

# ------------------------------------------- mavros_service, camera_service

unit_text() {
    sed -e "s|@USER@|$(flight_user)|g" -e "s|@REPO@|$REPO|g" "$HERE/systemd/$1"
}

check_unit() {
    if ! unit_text "$1" | cmp -s - "$SYSROOT/etc/systemd/system/$1"; then
        say "$1 is not installed, or is not the one in this repo"
        return 1
    fi
    if [[ $(sys systemctl is-enabled "$1" 2>/dev/null) != enabled ]]; then
        say "$1 is installed but not enabled"
        return 1
    fi
    say "$1 starts at boot as user $(flight_user)"
}

apply_unit() {
    mkdir -p "$SYSROOT/etc/systemd/system"
    unit_text "$1" >"$SYSROOT/etc/systemd/system/$1"
    sys systemctl daemon-reload
    sys systemctl enable "$1"
}

check_mavros_service() { check_unit argus-mavros.service; }
apply_mavros_service() { apply_unit argus-mavros.service; }
check_camera_service() { check_unit argus-camera.service; }
apply_camera_service() { apply_unit argus-camera.service; }

# ------------------------------------------------------------------ chrony

CHRONY_JETSON=/etc/chrony/conf.d/argus.conf
CHRONY_GROUND=/etc/chrony/conf.d/argus-ground.conf

chrony_jetson_text() {
    cat <<EOF
# ARGUS: the Jetson follows the ground laptop's clock, so the two machines' logs line up
# (platform/jetson/services.sh).
server $1 iburst minpoll 1 maxpoll 4 prefer
# The Jetson boots with an old time (no clock battery). chrony may step the clock in its
# first three updates only; after that it slews, so time never jumps under a mission.
makestep 1.0 3
EOF
}

chrony_ground_text() {
    cat <<EOF
# ARGUS: the ground laptop serves its clock to the Jetson's network, with or without
# internet (platform/jetson/services.sh --role ground).
allow $1
local stratum 8
EOF
}

# The address to follow: the one given, else the one already in the file.
ground_ip() {
    if [[ -n $GROUND_IP ]]; then
        echo "$GROUND_IP"
    else
        sed -n 's/^server \([0-9.]*\) .*/\1/p' "$SYSROOT$CHRONY_JETSON" 2>/dev/null | head -n 1
    fi
}

chrony_installed() {
    [[ $(sys dpkg-query -W -f '${Status}' chrony 2>/dev/null) == "install ok installed" ]] && return 0
    say "chrony is not installed: sudo apt-get install chrony (on the Jetson, setup.sh installs it)"
    return 1
}

# chrony_ready: installed and enabled, or says which it is not.
chrony_ready() {
    chrony_installed || return 1
    if [[ $(sys systemctl is-enabled chrony 2>/dev/null) != enabled ]]; then
        say "chrony is not enabled"
        return 1
    fi
}

check_chrony() {
    local address
    if [[ $ROLE == ground ]]; then
        if [[ -z $ALLOW ]] && [[ -f "$SYSROOT$CHRONY_GROUND" ]]; then
            ALLOW=$(sed -n 's/^allow //p' "$SYSROOT$CHRONY_GROUND" | head -n 1)
        fi
        if [[ -z $ALLOW ]] || ! chrony_ground_text "$ALLOW" | cmp -s - "$SYSROOT$CHRONY_GROUND"; then
            say "$CHRONY_GROUND is missing or differs: say which network to serve with --allow <network>/<bits>"
            return 1
        fi
        chrony_ready || return 1
        say "chrony serves this machine's clock to $ALLOW"
        return 0
    fi
    address=$(ground_ip)
    if [[ -z $address ]]; then
        say "chrony does not follow the ground laptop yet: run again with --ground-ip <the laptop's address>"
        return 1
    fi
    if ! chrony_jetson_text "$address" | cmp -s - "$SYSROOT$CHRONY_JETSON"; then
        say "$CHRONY_JETSON is missing or does not follow $address"
        return 1
    fi
    chrony_ready || return 1
    say "chrony follows the ground laptop at $address (check it with platform/jetson/clock_sync.py --source $address)"
}

apply_chrony() {
    local address
    chrony_installed || return 1
    mkdir -p "$SYSROOT/etc/chrony/conf.d"
    if [[ $ROLE == ground ]]; then
        [[ -n $ALLOW ]] || return 1
        chrony_ground_text "$ALLOW" >"$SYSROOT$CHRONY_GROUND"
    else
        address=$(ground_ip)
        [[ -n $address ]] || return 1
        chrony_jetson_text "$address" >"$SYSROOT$CHRONY_JETSON"
    fi
    sys systemctl enable chrony
    sys systemctl restart chrony
}

# -------------------------------------------------------------------- main

valid_ipv4() {
    local octet
    [[ $1 =~ ^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})$ ]] || return 1
    for octet in "${BASH_REMATCH[@]:1}"; do
        ((10#$octet <= 255)) || return 1
    done
}

mode=""
only=""
ROLE=jetson
while (($#)); do
    case $1 in
        --check | --apply) mode=${1#--} ;;
        --only)
            only=${2:-}
            shift
            ;;
        --role)
            ROLE=${2:-}
            shift
            ;;
        --ground-ip)
            GROUND_IP=${2:-}
            valid_ipv4 "$GROUND_IP" || {
                echo "services.sh: --ground-ip takes an IPv4 address, not '$GROUND_IP'" >&2
                exit 2
            }
            shift
            ;;
        --allow)
            ALLOW=${2:-}
            { [[ $ALLOW =~ ^([0-9.]+)/([0-9]{1,2})$ ]] && ((10#${BASH_REMATCH[2]} <= 32)) && valid_ipv4 "${ALLOW%/*}"; } || {
                echo "services.sh: --allow takes a network such as 192.168.10.0/24, not '$ALLOW'" >&2
                exit 2
            }
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "services.sh: unknown argument '$1'" >&2
            usage
            exit 2
            ;;
    esac
    shift
done

if [[ -z $mode ]]; then
    echo "services.sh: say --check or --apply" >&2
    usage
    exit 2
fi
case $ROLE in
    jetson) STEPS=(boot_env mavros_service camera_service chrony) ;;
    ground) STEPS=(chrony) ;;
    *)
        echo "services.sh: --role is jetson or ground, not '$ROLE'" >&2
        exit 2
        ;;
esac

select_steps services.sh "$only" "${STEPS[@]}"
guard_run services.sh "$mode"
run_steps "$mode" "${selected[@]}"
