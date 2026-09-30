#!/bin/bash
# Save the current SLAM map as <name>.pgm + <name>.yaml
#
#   ./save_map.sh                    -> /home/cy/maps/map.pgm + .yaml
#   ./save_map.sh /home/cy/maps/room -> /home/cy/maps/room.pgm + .yaml
#
# This uses slam_toolbox's own SaveMap service, so nav2_map_server is not
# needed.
source /opt/ros/jazzy/setup.bash

NAME="${1:-/home/cy/maps/map}"
DIR="$(dirname "$NAME")"
mkdir -p "$DIR"

echo "saving map to ${NAME}.pgm / ${NAME}.yaml ..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${NAME}'}}"

echo
echo "--- files in $DIR ---"
ls -la "$DIR"
