# Environment inside the argus-gps container (MAVROS and our GPS missions).
# Sourced by the scripts in this folder; never run on the host.
# shellcheck shell=bash

: "${ARGUS_REPO:?ARGUS_REPO must be set (sim/gps/start_mavros.sh sets it)}"
export ARGUS_GPS_STATE="$ARGUS_REPO/sim/.state/gps"
export ARGUS_GPS_WS="$ARGUS_GPS_STATE/ws"

# The container runs as the host user, who has no home in the image. ROS logs
# go here, inside the only writable mount.
export HOME="$ARGUS_GPS_STATE/home"
mkdir -p "$HOME"

# Isolation from ROS 2 Jazzy on the host: host networking shares localhost,
# so ROS_LOCALHOST_ONLY alone does not separate us. A non-default domain does.
export ROS_DOMAIN_ID=${ARGUS_ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=1

# The base image pip-installs MAVProxy and a newer setuptools into the user
# site. ROS and colcon must not pick those up.
export PYTHONNOUSERSITE=1

# ROS setup scripts read unset variables, so relax nounset while sourcing.
_argus_nounset=0
[[ $- == *u* ]] && _argus_nounset=1 && set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ARGUS_GPS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ARGUS_GPS_WS/install/setup.bash"
fi
((_argus_nounset)) && set -u
unset _argus_nounset

cd "$ARGUS_REPO" || return 1
