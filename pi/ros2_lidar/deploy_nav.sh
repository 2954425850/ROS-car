#!/usr/bin/env bash
# Morning sync: repo -> ~/ros2_lidar (and optionally install nav/slam units).
#
#   bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh                 # sync only
#   bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh --install-units # sync + systemd
#
# This does NOT start anything.
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$REPO/pi/ros2_lidar"
DST="$HOME/ros2_lidar"

echo "== sync $SRC -> $DST"
mkdir -p "$DST"
# maps/ is kept (posegraph/data/last_pose.json are Pi-only); the tracked
# map files are overwritten by the repo copies.
if command -v rsync >/dev/null 2>&1; then
  rsync -a --exclude '__pycache__' "$SRC/" "$DST/"
else
  cp -a "$SRC/." "$DST/"
fi

echo "== syntax checks"
for f in "$DST"/tools/*.py; do python3 -m py_compile "$f"; done
python3 - "$DST/config/bt_nav_to_pose.xml" <<'PY'
import sys, xml.etree.ElementTree as ET
ET.parse(sys.argv[1])
print('bt xml ok')
PY

if [ "${1:-}" = "--install-units" ]; then
  echo "== install nav.service / slam.service"
  sudo cp "$REPO/pi/systemd/system/nav.service" \
          "$REPO/pi/systemd/system/slam.service" /etc/systemd/system/
  sudo systemctl daemon-reload
  echo "installed (not started)"
fi

echo
echo "next:"
echo "  ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py"
echo "  # or: sudo systemctl start nav"
echo "  python3 ~/ros2_lidar/tools/nav_accept.py --rounds 20"
