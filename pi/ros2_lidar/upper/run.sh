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
exec python3 "$(dirname "$0")/server.py" "$@"
