#!/usr/bin/env bash
# Bring a fresh JetPack 6.2 Jetson Orin Nano Super to flight state (PLT-01).
#
#   sudo platform/jetson/setup.sh --check                 the checklist; changes nothing
#   sudo platform/jetson/setup.sh --apply                 do every step that is not done, then the checklist
#   sudo platform/jetson/setup.sh --apply --only fan,udev just those steps
#   sudo platform/jetson/setup.sh --apply --log-device /dev/nvme0n1p1
#                                                         format that blank partition for the logs
#   platform/jetson/setup.sh --list                       the steps, in order
#
# Every step has a check and an apply. --apply runs a step's apply only when
# its check fails, so the script can be run again at any time. Exit code: 0
# when every step asked for is ok, 1 when one is not, 2 for a wrong call.
#
# Values that can differ between boards are in setup.conf beside this file.
# The tests (platform/jetson/test/test_setup.py) run this script against a
# fake Jetson: ARGUS_SYSROOT is put in front of every file path, and
# ARGUS_SETUP_BIN is the only place system commands are then taken from.

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
# shellcheck source=platform/jetson/setup.conf
source "$HERE/setup.conf"

# shellcheck source=platform/jetson/lib.sh
source "$HERE/lib.sh"

STEPS=(platform power_mode jetson_clocks fan log_disk ros onnxruntime udev)
# When this is not the machine the script is for, nothing is applied.
FIRST_STEP_GATES=1

usage() {
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
}

# ---------------------------------------------------------------- platform

check_platform() {
    local release model minor
    release=$(head -n 1 "$SYSROOT/etc/nv_tegra_release" 2>/dev/null)
    model=$(tr -d '\0' <"$SYSROOT/proc/device-tree/model" 2>/dev/null)
    minor=$(sed -n "s/.*$L4T_RELEASE (release), REVISION: $L4T_REVISION_MAJOR\.\([0-9]\{1,\}\).*/\1/p" <<<"$release")
    if [[ -z $minor ]] || ((minor < L4T_REVISION_MIN_MINOR)); then
        say "this is not JetPack 6.2 (Jetson Linux $L4T_RELEASE.$L4T_REVISION_MAJOR.$L4T_REVISION_MIN_MINOR or a later $L4T_RELEASE.$L4T_REVISION_MAJOR): ${release:-no /etc/nv_tegra_release}"
        return 1
    fi
    if [[ $model != *"$BOARD_MODEL"* ]]; then
        say "this is not a Jetson $BOARD_MODEL: ${model:-no device-tree model}"
        return 1
    fi
    release=${release#\# }
    say "$model, ${release%%, GCID*}"
}

apply_platform() {
    # Nothing here can be changed by a script: flash JetPack 6.2 with SDK Manager.
    return 1
}

# -------------------------------------------------------------- power_mode

check_power_mode() {
    local mode
    mode=$(sys nvpmodel -q 2>/dev/null | sed -n 's/^NV Power Mode: //p')
    if [[ $mode == "$POWER_MODE" ]]; then
        say "$mode"
        return 0
    fi
    say "power mode is ${mode:-unknown}, wanted $POWER_MODE"
    return 1
}

apply_power_mode() {
    local id
    id=$(sed -n "s/^< POWER_MODEL ID=\([0-9]*\) NAME=$POWER_MODE >.*/\1/p" "$SYSROOT/etc/nvpmodel.conf" 2>/dev/null | head -n 1)
    if [[ -z $id ]]; then
        say "/etc/nvpmodel.conf has no $POWER_MODE mode: this board or its JetPack is not the Super configuration"
        return 1
    fi
    # nvpmodel asks before a mode that needs a reboot; answer no, and say so below.
    echo no | sys nvpmodel -m "$id" >/dev/null 2>&1
    check_power_mode && return 0
    say "$POWER_MODE (mode $id) was asked for but is not active: reboot, then run this script again"
    return 1
}

# ----------------------------------------------------------- jetson_clocks

CLOCKS_UNIT=argus-jetson-clocks.service

clocks_unit_text() {
    cat <<'EOF'
[Unit]
Description=ARGUS: hold the CPU, GPU and memory clocks at their maximum (jetson_clocks)
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
}

check_jetson_clocks() {
    local unit="$SYSROOT/etc/systemd/system/$CLOCKS_UNIT"
    if ! clocks_unit_text | cmp -s - "$unit"; then
        say "$CLOCKS_UNIT is not installed"
        return 1
    fi
    if [[ $(sys systemctl is-enabled "$CLOCKS_UNIT" 2>/dev/null) != enabled ]]; then
        say "$CLOCKS_UNIT is installed but not enabled"
        return 1
    fi
    say "$CLOCKS_UNIT runs jetson_clocks at every boot"
}

apply_jetson_clocks() {
    mkdir -p "$SYSROOT/etc/systemd/system"
    clocks_unit_text >"$SYSROOT/etc/systemd/system/$CLOCKS_UNIT"
    sys systemctl daemon-reload
    sys systemctl enable --now "$CLOCKS_UNIT"
}

# --------------------------------------------------------------------- fan

check_fan() {
    local profile
    profile=$(sed -n 's/^[[:space:]]*FAN_DEFAULT_PROFILE[[:space:]]\{1,\}\([a-z_]*\).*/\1/p' "$SYSROOT/etc/nvfancontrol.conf" 2>/dev/null | head -n 1)
    if [[ $profile == "$FAN_PROFILE" ]]; then
        say "nvfancontrol profile $profile"
        return 0
    fi
    say "nvfancontrol profile is ${profile:-unknown}, wanted $FAN_PROFILE"
    return 1
}

apply_fan() {
    [[ -f "$SYSROOT/etc/nvfancontrol.conf" ]] || return 1
    sed -i "s/^\([[:space:]]*FAN_DEFAULT_PROFILE[[:space:]]\{1,\}\)[a-z_]*/\1$FAN_PROFILE/" "$SYSROOT/etc/nvfancontrol.conf"
    # nvfancontrol reads the profile only when it has no saved status.
    sys systemctl stop nvfancontrol
    rm -f "$SYSROOT/var/lib/nvfancontrol/status"
    sys systemctl start nvfancontrol
}

# ---------------------------------------------------------------- log_disk

LOG_DEVICE=""
FSTAB_LINE="LABEL=$LOG_LABEL $LOG_MOUNT ext4 defaults,noatime,nofail 0 2"

root_device() {
    sys findmnt -n -o SOURCE / 2>/dev/null
}

check_log_disk() {
    local mounted
    if [[ $(root_device) == /dev/nvme* ]]; then
        if [[ -d "$SYSROOT$LOG_MOUNT" ]]; then
            say "$LOG_MOUNT is a folder on the root disk: this Jetson boots from the NVMe"
            return 0
        fi
        say "$LOG_MOUNT does not exist (this Jetson boots from the NVMe, so it is a plain folder)"
        return 1
    fi
    if ! grep -qxF "$FSTAB_LINE" "$SYSROOT/etc/fstab" 2>/dev/null; then
        say "no /etc/fstab line mounts the '$LOG_LABEL' filesystem on $LOG_MOUNT"
        return 1
    fi
    mounted=$(sys findmnt -n -o SOURCE "$LOG_MOUNT" 2>/dev/null)
    if [[ -z $mounted ]]; then
        say "$LOG_MOUNT is in /etc/fstab but nothing is mounted there"
        return 1
    fi
    say "$mounted mounted on $LOG_MOUNT"
}

apply_log_disk() {
    local device kind
    if [[ $(root_device) == /dev/nvme* ]]; then
        mkdir -p "$SYSROOT$LOG_MOUNT"
        sys chown "$(flight_user)" "$SYSROOT$LOG_MOUNT"
        return 0
    fi
    device=$(sys blkid -L "$LOG_LABEL" 2>/dev/null)
    if [[ -z $device ]]; then
        if [[ -z $LOG_DEVICE ]]; then
            say "no filesystem is labelled '$LOG_LABEL'. Partition the NVMe, then run again with --log-device <partition> (formats a blank partition only)"
            return 1
        fi
        if [[ $LOG_DEVICE == "$(root_device)" ]]; then
            say "$LOG_DEVICE is the disk the system boots from: not formatting it"
            return 1
        fi
        kind=$(sys blkid -o value -s TYPE "$LOG_DEVICE" 2>/dev/null)
        if [[ -n $kind ]]; then
            say "$LOG_DEVICE already holds a $kind filesystem: not formatting it. To use it as it is: e2label $LOG_DEVICE $LOG_LABEL"
            return 1
        fi
        sys mkfs.ext4 -L "$LOG_LABEL" "$LOG_DEVICE" >/dev/null || return 1
    fi
    grep -qxF "$FSTAB_LINE" "$SYSROOT/etc/fstab" || echo "$FSTAB_LINE" >>"$SYSROOT/etc/fstab"
    mkdir -p "$SYSROOT$LOG_MOUNT"
    sys mount "$LOG_MOUNT" || return 1
    sys chown "$(flight_user)" "$SYSROOT$LOG_MOUNT"
}

# --------------------------------------------------------------------- ros

ROS_KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
GEOID="$SYSROOT/usr/share/GeographicLib/geoids/egm96-5.pgm"

missing_packages() {
    local package
    for package in "${ROS_PACKAGES[@]}"; do
        [[ $(sys dpkg-query -W -f '${Status}' "$package" 2>/dev/null) == "install ok installed" ]] || echo "$package"
    done
}

check_ros() {
    local missing
    missing=$(missing_packages | tr '\n' ' ')
    if [[ -n $missing ]]; then
        say "not installed: ${missing% }"
        return 1
    fi
    if [[ ! -f "$SYSROOT/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]]; then
        say "/opt/ros/$ROS_DISTRO_NAME/setup.bash is missing"
        return 1
    fi
    if [[ ! -f $GEOID ]]; then
        say "the GeographicLib geoid model egm96-5 (MAVROS needs it) is missing"
        return 1
    fi
    say "ROS 2 $ROS_DISTRO_NAME, MAVROS and ${#ROS_PACKAGES[@]} packages, geoid egm96-5"
}

apply_ros() {
    local missing
    export DEBIAN_FRONTEND=noninteractive
    if [[ ! -f "$SYSROOT/etc/apt/sources.list.d/ros2.list" ]]; then
        sys apt-get update || return 1
        sys apt-get install -y curl gnupg || return 1
        mkdir -p "$SYSROOT/usr/share/keyrings" "$SYSROOT/etc/apt/sources.list.d"
        sys curl -fsSL "$ROS_KEY_URL" -o "$SYSROOT$ROS_KEYRING" || return 1
        echo "deb [arch=arm64 signed-by=$ROS_KEYRING] $ROS_APT_URL $UBUNTU_CODENAME main" >"$SYSROOT/etc/apt/sources.list.d/ros2.list"
    fi
    mapfile -t missing < <(missing_packages)
    if ((${#missing[@]})); then
        sys apt-get update || return 1
        sys apt-get install -y "${missing[@]}" || return 1
    fi
    if [[ ! -f $GEOID ]]; then
        sys geographiclib-get-geoids egm96-5 >/dev/null || return 1
    fi
}

# ------------------------------------------------------------- onnxruntime

ORT_QUERY='import onnxruntime as ort; print(ort.__version__, ",".join(ort.get_available_providers()))'

check_onnxruntime() {
    local found
    found=$(sys python3 -c "$ORT_QUERY" 2>/dev/null | tail -n 1)
    if [[ -z $found ]]; then
        say "python3 cannot import onnxruntime"
        return 1
    fi
    if [[ $found != *CUDAExecutionProvider* && $found != *TensorrtExecutionProvider* ]]; then
        say "onnxruntime $found has no GPU provider: this is the CPU build, not the Jetson one"
        return 1
    fi
    say "onnxruntime $found"
}

apply_onnxruntime() {
    sys python3 -m pip install --index-url "$ORT_INDEX_URL" "$ORT_PACKAGE" >/dev/null 2>&1
}

# -------------------------------------------------------------------- udev

UDEV_RULE=/etc/udev/rules.d/99-argus-pixhawk.rules

udev_rule_text() {
    local serial="" vendor
    [[ -n ${ARGUS_PIXHAWK_SERIAL:-} ]] && serial=", ATTRS{serial}==\"$ARGUS_PIXHAWK_SERIAL\""
    echo "# ARGUS: the flight controller's MAVLink port is always /dev/argus/pixhawk (platform/jetson/setup.sh)."
    echo "# Interface 00 is MAVLink; ArduPilot's second USB serial port (SLCAN) must not take the name."
    for vendor in "${PIXHAWK_USB_VENDORS[@]}"; do
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\"$serial, ENV{ID_USB_INTERFACE_NUM}==\"00\", SYMLINK+=\"argus/pixhawk\", GROUP=\"dialout\", MODE=\"0660\""
    done
}

check_udev() {
    local user plugged
    user=$(flight_user)
    if ! udev_rule_text | cmp -s - "$SYSROOT$UDEV_RULE"; then
        say "$UDEV_RULE is missing or differs from this script's rule"
        return 1
    fi
    if [[ " $(sys id -nG "$user" 2>/dev/null) " != *" dialout "* ]]; then
        say "user $user is not in the dialout group"
        return 1
    fi
    plugged="not plugged in"
    [[ -e "$SYSROOT/dev/argus/pixhawk" ]] && plugged="/dev/argus/pixhawk is present"
    say "rule installed, $user in dialout; flight controller: $plugged"
}

apply_udev() {
    mkdir -p "$SYSROOT/etc/udev/rules.d"
    udev_rule_text >"$SYSROOT$UDEV_RULE"
    sys udevadm control --reload-rules
    sys udevadm trigger --subsystem-match=tty
    sys usermod -aG dialout "$(flight_user)"
}

# -------------------------------------------------------------------- main

mode=""
only=""
while (($#)); do
    case $1 in
        --check | --apply | --list) mode=${1#--} ;;
        --only)
            only=${2:-}
            shift
            ;;
        --log-device)
            LOG_DEVICE=${2:-}
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "setup.sh: unknown argument '$1'" >&2
            usage
            exit 2
            ;;
    esac
    shift
done

if [[ -z $mode ]]; then
    echo "setup.sh: say --check, --apply or --list" >&2
    usage
    exit 2
fi
if [[ $mode == list ]]; then
    printf '%s\n' "${STEPS[@]}"
    exit 0
fi

select_steps setup.sh "$only" "${STEPS[@]}"
guard_run setup.sh "$mode"

run_steps "$mode" "${selected[@]}"
result=$?

# The record of a whole run, kept on the Jetson beside the disk image's notes.
if [[ $mode == apply && -z $only && $first_ok == 1 ]]; then
    mkdir -p "$SYSROOT/var/lib/argus"
    {
        echo "platform/jetson/setup.sh --apply, $(sys date -u +%Y-%m-%dT%H:%M:%SZ), repo commit $(sys git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
        printf '%s\n' "${lines[@]}" "$summary"
    } >"$SYSROOT/var/lib/argus/setup-checklist.txt"
fi

exit "$result"
