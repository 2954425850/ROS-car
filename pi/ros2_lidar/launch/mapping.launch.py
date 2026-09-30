"""Bring-up for the nepu robot's 2D SLAM pipeline.

    ros2 launch /home/cy/ros2_lidar/launch/mapping.launch.py

This replaces the old start_mapping.sh. What it no longer hand-rolls:

  * slam_toolbox is started through the stock online_async_launch.py that
    ships with the slam_toolbox package. That file drives the node's
    lifecycle (configure -> activate) itself, so the old trap -- "the node
    starts unconfigured, silently subscribes to nothing, and you must call
    `ros2 lifecycle set` by hand" -- cannot happen here.
  * base_link -> laser is published by robot_state_publisher from urdf/, not
    by a hand-started static_transform_publisher.

What it still overrides, because the stock files are wrong for THIS hardware
(both confirmed on the bench):

  * rplidar_ros: the stock rplidar.launch.py hardcodes /dev/ttyUSB0 and never
    sets auto_standby. On this CP2102 adapter auto_standby=true leaves the
    motor stopped: the driver opens the port, reports health status 0,
    advertises /scan, and publishes zero frames forever while burning ~95% of
    a core busy-waiting. The stock file also declares no LaunchArguments, so
    it cannot be included-and-overridden -- the node is declared here instead,
    mirroring the stock file plus the two required parameters.
  * base_frame: base_link  (the stock slam_toolbox config uses base_footprint,
    which does not exist on this robot)
  * max_laser_range: 12.0  (the A1M8's spec; the stock config says 20.0)

Everything else in config/slam_params.yaml is the stock
mapper_params_online_async.yaml, unmodified.
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    # ttyUSB numbers drift whenever the adapter re-enumerates, so resolve the
    # port by id at launch time rather than hardcoding it.
    ports = sorted(glob.glob('/dev/serial/by-id/*CP2102*'))
    if not ports:
        raise RuntimeError(
            'No CP2102 lidar adapter found under /dev/serial/by-id/ -- '
            'is the RPLIDAR plugged in?\n'
            'Present: %s' % (sorted(glob.glob('/dev/serial/by-id/*')) or 'nothing'))

    dds_profile = os.path.join(HERE, 'config', 'fastdds_wifi.xml')
    slam_params = os.path.join(HERE, 'config', 'slam_params.yaml')
    urdf_file = os.path.join(HERE, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Must be set before any node creates a DDS participant. Without it a
        # ~20 KB /map goes out as one UDP datagram that IP chops into ~14
        # fragments, and this WiFi link drops most fragmented packets. See the
        # header of config/fastdds_wifi.xml.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile),

        # Mirrors the stock rplidar.launch.py plus auto_standby and a resolved
        # serial port. See the module docstring.
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': ports[0],
                'serial_baudrate': 115200,   # A1 / A2 (A3 wants 256000)
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'auto_standby': False,
            }],
        ),

        # Publishes base_link -> laser (fixed joint) from urdf/robot.urdf.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Stationary placeholder for odometry. Deliberately NOT in the URDF:
        # odom -> base_link is a dynamic transform that the wheel-odometry
        # driver must own. Delete this node once that driver exists.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            output='screen',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link',
                       '--x', '0', '--y', '0', '--z', '0',
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1'],
        ),

        # The stock slam_toolbox bringup. It activates its own lifecycle.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': slam_params,
                # The stock launch defaults use_sim_time to true. A real robot
                # has no /clock topic, and with use_sim_time=true every TF
                # stamp would be garbage. Must stay false here.
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
