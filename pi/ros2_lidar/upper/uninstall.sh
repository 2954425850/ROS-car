#!/usr/bin/env bash
# 卸载开机自启。代码目录另删：rm -rf ~/ROS-car/pi/ros2_lidar/upper
set -e
sudo systemctl disable --now upper 2>/dev/null || true
sudo rm -f /etc/systemd/system/upper.service
sudo systemctl daemon-reload
echo "upper.service 已移除（若它拉起的建图/导航还在，KillMode=control-group 会一并停掉）"
