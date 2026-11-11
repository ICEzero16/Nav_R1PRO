#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/env.bash
mkdir -p reports
# Sensor startup must be explicit. Opening the dashboard should never implicitly start lidar.
drive_flag=""
with_sensors=false
case "${1:-}" in
    '') ;;
    --enable-drive) drive_flag=--enable-drive ;;
    --with-sensors) with_sensors=true ;;
    *) echo 'Usage: bash scripts/start_web.sh [--with-sensors]' >&2; exit 2 ;;
esac
if [ "$with_sensors" = true ] && ! tmux has-session -t r1pro_nav_sensors 2>/dev/null && ! pgrep -f '/r1pro_waypoint_nav/adapters' >/dev/null; then
    lidar=true
    if pgrep -f 'livox_ros_driver2_node' >/dev/null; then lidar=false; fi
    tmux new-session -d -s r1pro_nav_sensors -c "$R1PRO_NAV_WS" \
        "bash -c 'source scripts/env.bash; exec ros2 launch r1pro_waypoint_nav sensors.launch.py start_lidar:=$lidar > reports/web_sensors.log 2>&1'"
fi
if ! tmux has-session -t r1pro_nav_web 2>/dev/null; then
    tmux new-session -d -s r1pro_nav_web -c "$R1PRO_NAV_WS" \
        "bash -c 'source scripts/env.bash; exec python3 -u web/server.py --port 8088 $drive_flag > reports/web_server.log 2>&1'"
fi
echo 'Dashboard on robot: http://127.0.0.1:8088'
echo 'From your computer: ssh -N -L 8088:127.0.0.1:8088 R1PRO'
