#!/usr/bin/env bash
# 建图/导航可视化（临时件，只读）。
#
#   浏览器打开  http://<Pi IP>:8850
#   停止        Ctrl-C
#   删除        rm -rf ~/ROS-car/pi/ros2_lidar/upper
#
# 参数透传给 server.py，例如 --port 8860 / --demo。
set -e
source /opt/ros/jazzy/setup.bash
# nav2 overlay：只含打过补丁的 nav2_smac_planner + RPP。没 build 就退回系统版。
if [ -f "$HOME/nav2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/nav2_ws/install/setup.bash"
fi
exec python3 "$(dirname "$0")/server.py" "$@"
