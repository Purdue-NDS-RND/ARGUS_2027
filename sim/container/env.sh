# Environment for everything inside the argus-sim container. Sourced by
# start.sh, by `sim/run_sim.sh --shell` and by `sim/run_sim.sh --exec`.
# shellcheck shell=bash

if [[ -z ${ARGUS_REPO:-} ]]; then
    ARGUS_REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
fi
export ARGUS_REPO
export ARGUS_SIM="$ARGUS_REPO/sim"
export ARGUS_STATE="$ARGUS_SIM/.state"
export ARGUS_WS="$ARGUS_SIM/.ws"

# The container runs as the host user, who has no home in the image. Keep
# ROS logs, the session logger's ~/cuasc_logs and Gazebo's cache here.
export HOME="$ARGUS_STATE/home"
mkdir -p "$HOME"
export XDG_RUNTIME_DIR="$ARGUS_STATE/runtime"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 0700 "$XDG_RUNTIME_DIR"

# Keep ROS and Gazebo traffic on this machine: with host networking, two club
# drives on the same Wi-Fi would otherwise see each other's topics.
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
export GZ_PARTITION=argus_sim

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH="$ARGUS_SIM/models:$ARGUS_SIM/worlds:/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds"

case "${SOFTWARE_RENDERING:-0}" in
    1|true|TRUE|yes|YES|on|ON) export LIBGL_ALWAYS_SOFTWARE=1 ;;
esac

# The base image pip-installs MAVProxy and a newer setuptools into
# /opt/ardupilot-python. ROS and colcon must not pick those up; only
# sim_vehicle.py (start.sh) turns the user site back on.
export PYTHONNOUSERSITE=1

# ROS setup scripts read unset variables, so relax nounset while sourcing.
_argus_nounset=0
[[ $- == *u* ]] && _argus_nounset=1 && set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ARGUS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ARGUS_WS/install/setup.bash"
fi
((_argus_nounset)) && set -u
unset _argus_nounset

cd "$ARGUS_REPO" || return 1
