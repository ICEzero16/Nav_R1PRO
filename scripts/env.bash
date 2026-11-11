#!/usr/bin/env bash
export R1PRO_NAV_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
source /opt/ros/humble/setup.bash || return 1
source /home/nvidia/galaxea215/install/setup.bash || return 1
if [ -f "$R1PRO_NAV_WS/install/local_setup.bash" ]; then
    source "$R1PRO_NAV_WS/install/local_setup.bash" || return 1
fi
export ROS_DOMAIN_ID=79
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$R1PRO_NAV_WS/reports/ros_logs"
