#!/usr/bin/env bash
# What the boot services run (PLT-02): MAVROS and the camera driver, and nothing else.
# Missions are never started here; they start only through the GO launcher.
#
#   argus-boot.sh mavros            start MAVROS as /etc/argus/boot.env says
#   argus-boot.sh camera            start the camera driver as it says
#   argus-boot.sh --print <which>   print the command instead of starting it
#
# Settings (systemd reads them from /etc/argus/boot.env):
#   ARGUS_BOOT_MAVROS   apm    MAVROS's own apm.launch, as in 2026 (the default)
#                       argus  config/mavros/argus_mavros.launch.py with the stream keeper
#                       off    do not start MAVROS
#   ARGUS_BOOT_CAMERA   off    no camera driver at boot: the 2026 launch files start their
#                              own grabber, and two readers cannot share the camera (the default)
#                       <mode> a sensor mode of argus_camera, e.g. imx519_720p60
#   ARGUS_BOOT_CAMERA_LEGACY_TOPICS  true|false  also serve the 2026 topic names (default false)
#   ARGUS_WS            the workspace whose install/setup.bash is sourced (default: this repo)
#
# Exit 0 when the service is off, 2 for a setting this script does not know
# (the units tell systemd not to restart on 2).

set -uo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
DEVICE=${ARGUS_PIXHAWK_DEVICE:-/dev/argus/pixhawk}

print_only=0
if [[ ${1:-} == --print ]]; then
    print_only=1
    shift
fi

refuse() {
    echo "argus-boot.sh: $*" >&2
    exit 2
}

command=()
wait_for=""
case ${1:-} in
    mavros)
        case ${ARGUS_BOOT_MAVROS:-apm} in
            apm) command=(ros2 launch mavros apm.launch "fcu_url:=$DEVICE:115200") ;;
            argus) command=(ros2 launch "$REPO/config/mavros/argus_mavros.launch.py" "fcu_url:=$DEVICE:115200" stream_keeper:=true) ;;
            off) ;;
            *) refuse "ARGUS_BOOT_MAVROS must be apm, argus or off, not '${ARGUS_BOOT_MAVROS}'" ;;
        esac
        wait_for=$DEVICE
        ;;
    camera)
        mode=${ARGUS_BOOT_CAMERA:-off}
        legacy=${ARGUS_BOOT_CAMERA_LEGACY_TOPICS:-false}
        [[ $mode =~ ^[a-z0-9_]+$ ]] || refuse "ARGUS_BOOT_CAMERA must be off or a sensor mode's name, not '$mode'"
        [[ $legacy == true || $legacy == false ]] || refuse "ARGUS_BOOT_CAMERA_LEGACY_TOPICS must be true or false, not '$legacy'"
        [[ $mode == off ]] || command=(ros2 launch argus_camera argus_camera.launch.py "mode:=$mode" "legacy_topics:=$legacy")
        ;;
    *) refuse "say mavros or camera" ;;
esac

if ((${#command[@]} == 0)); then
    ((print_only)) && echo off
    exit 0
fi
if ((print_only)); then
    echo "${command[*]}"
    exit 0
fi

# The flight controller may be plugged in or powered after the Jetson boots.
waited=0
while [[ -n $wait_for && ! -e $wait_for ]]; do
    ((waited % 30 == 0)) && echo "argus-boot.sh: waiting for $wait_for"
    sleep "${ARGUS_BOOT_POLL_S:-1}"
    waited=$((waited + 1))
done

set +u
# shellcheck disable=SC1090,SC1091
source "${ARGUS_ROS_SETUP:-/opt/ros/humble/setup.bash}" || exit 1
# shellcheck disable=SC1090,SC1091
source "${ARGUS_WS:-$REPO}/install/setup.bash" || exit 1
set -u
exec "${command[@]}"
