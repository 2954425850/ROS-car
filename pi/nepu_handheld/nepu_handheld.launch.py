"""Handheld mapping on the desktop (10.222.152.118, hostname 'cy').

    ros2 launch ~/nepu_handheld/nepu_handheld.launch.py

Runs cartographer + its occupancy-grid node HERE on the desktop, consuming
/scan over WiFi from the rplidar driver on the Pi (nepu). Nothing but the
lidar driver runs on the Pi, so no sudo is needed there and the Pi's CPU
stays free.

Requires on this machine: ros-jazzy-cartographer-ros, ros-jazzy-rviz2.
/scan and /map both cross the link inside the RTPS-layer fragmentation
enforced by fastdds_wifi.xml, so the profile must stay exported (below).
"""
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    return LaunchDescription([
        # Same profile as the Pi side. Without it the growing /map dies on
        # this WiFi link; see the header of fastdds_wifi.xml.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.expanduser('~/fastdds_wifi.xml')),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', os.path.join(HERE, 'config'),
                '-configuration_basename', 'nepu_handheld.lua',
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(HERE, 'handheld.rviz')],
        ),
    ])
