"""Bring-up for the nepu robot's 2D SLAM pipeline (SLAM runs ON the Pi).

    ros2 launch /home/cy/ros2_lidar/launch/mapping.launch.py

TF tree this produces (each edge has exactly one owner):

    map -> odom            slam_toolbox (dynamic)
    odom -> base_footprint l150pro_driver, already running as a systemd
                           service (~50 Hz) -- NOT started or duplicated here
    base_footprint -> base_link -> laser
                           robot_state_publisher from urdf/robot.urdf

What this launch does NOT do, and why:

  * It does not start a static odom -> base_link. The old version did; that
    edge now belongs to l150pro_driver (which names the frame base_footprint),
    and a second publisher for the same pair fights with it.
  * It does not export FASTRTPS_DEFAULT_PROFILES_FILE. That profile exists to
    keep /map from being IP-fragmented when it crosses WiFi to a desktop RViz,
    and its initialPeersList contains stale addresses. SLAM and rosbridge both
    run on the Pi, so the map never crosses WiFi as DDS samples. If you do
    want remote RViz over DDS, fix the peer IPs in config/fastdds_wifi.xml
    first (currently 10.222.152.x) and re-add the export here.

Hardware quirks kept from the old file (both confirmed on the bench):

  * rplidar_ros: the stock rplidar.launch.py hardcodes /dev/ttyUSB0 and never
    sets auto_standby. On this CP2102 adapter auto_standby=true leaves the
    motor stopped: the driver opens the port, reports health status 0,
    advertises /scan, and publishes zero frames forever while burning ~95% of
    a core busy-waiting. The stock file also declares no LaunchArguments, so
    it cannot be included-and-overridden -- the node is declared here instead.
  * max_laser_range: 12.0 (A1M8 spec; stock config says 20.0).
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def generate_launch_description():
    # ttyUSB numbers drift whenever the adapter re-enumerates (ttyUSB2 before
    # the 2026-10-23 reboot, ttyUSB3 after), so resolve the port by id at
    # launch time rather than hardcoding it.
    ports = sorted(glob.glob('/dev/serial/by-id/*CP2102*'))
    if not ports:
        raise RuntimeError(
            'No CP2102 lidar adapter found under /dev/serial/by-id/ -- '
            'is the RPLIDAR plugged in?\n'
            'Present: %s' % (sorted(glob.glob('/dev/serial/by-id/*')) or 'nothing'))

    slam_params = os.path.join(HERE, 'config', 'slam_params.yaml')
    urdf_file = os.path.join(HERE, 'urdf', 'robot.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Mirrors the stock rplidar.launch.py plus auto_standby and a resolved
        # serial port. Measured on the bench: ~6.5 Hz, frame_id 'laser'.
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

        # Publishes base_footprint -> base_link -> laser (fixed joints).
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
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
