"""RPLIDAR driver only, for when SLAM runs on the desktop.

    ros2 launch /home/cy/ros2_lidar/launch/lidar_only.launch.py

Same node + parameters as mapping.launch.py (see that file's docstring for
the serial-port and auto_standby reasoning). The DDS profile is exported so
/scan survives the WiFi hop to 10.222.152.118.
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    ports = sorted(glob.glob('/dev/serial/by-id/*CP2102*'))
    if not ports:
        raise RuntimeError(
            'No CP2102 lidar adapter found under /dev/serial/by-id/ -- '
            'is the RPLIDAR plugged in?\n'
            'Present: %s' % (sorted(glob.glob('/dev/serial/by-id/*')) or 'nothing'))

    return LaunchDescription([
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(HERE, 'config', 'fastdds_wifi.xml')),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': ports[0],
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'auto_standby': False,
            }],
        ),
    ])
