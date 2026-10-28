#!/bin/bash
# Save the current SLAM map as <name>.pgm + <name>.yaml + <name>.png
#
#   ./save_map.sh                    -> /home/cy/ros2_lidar/maps/map.*
#   ./save_map.sh /home/cy/ros2_lidar/maps/room
#
# The map is read straight from the latched /map topic. slam_toolbox's own
# /slam_toolbox/save_map service is NOT used: on this install it returns 255
# because nav2_map_server is missing (see tools/save_map_snapshot.py).
source /opt/ros/jazzy/setup.bash

NAME="${1:-/home/cy/ros2_lidar/maps/map}"
DIR="$(dirname "$NAME")"
mkdir -p "$DIR"

python3 "$(dirname "$0")/tools/save_map_snapshot.py" "$NAME"

echo
echo "--- files in $DIR ---"
ls -la "$DIR"
