#!/usr/bin/env bash
# Pi 侧部署：拉代码 → 重建 ROS 工作区 → 重启服务。
#
# 用法（在 Pi 上）：
#   bash ~/ROS-car/pi/deploy.sh                # 拉取+构建+重启
#   bash ~/ROS-car/pi/deploy.sh --build-only   # 只拉取+构建，不重启（安全，不动机械臂）
#
# 日常改 Python 代码其实不需要跑这个：工作区是 --symlink-install 构建的，
# 源码是软链进去的，`git pull` 之后直接重启服务就行。构建这一步是给
# 「新增文件 / 改 entry_points / 改 package.xml」兜底的。
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

echo "== 1/3 git pull =="
git pull --ff-only

echo
echo "== 2/3 colcon build =="
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
( cd pi/raspot_ws  && colcon build --symlink-install )
( cd pi/l150pro_ws && colcon build --symlink-install )

if [ "${1:-}" = "--build-only" ]; then
  echo
  echo "== 只构建，已跳过重启 =="
  exit 0
fi

echo
echo "== 3/3 重启服务 =="
echo "  ⚠️ 重启 ps2-teleop：2026-10-21 实测不挪臂（ps2_teleop 按实际位姿初始化目标），"
echo "     但抓取脚本收尾那次 restart 会走 INIT_HOME —— 臂附近别放东西"
echo "  ⚠️ l150pro-driver.service 已退役（2026-10-21 里程计/IMU/偏航闭环并进了 ps2-teleop 的驱动），别启"
sudo systemctl restart k230-resultd k230-framed k230-bridge rosbridge ps2-teleop
systemctl --user restart voice-chatbot
echo
echo "== 完成 =="
systemctl --no-pager --lines=0 status k230-resultd k230-framed k230-bridge rosbridge ps2-teleop || true
