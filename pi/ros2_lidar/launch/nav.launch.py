"""建图之后的定位 + 导航：雷达 + TF + AMCL + nav2。

    ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py

链路（速度优先级由 cmd_mux 仲裁，导航最低）：
  controller_server -> cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed
    -> collision_monitor -> cmd_vel_nav -> cmd_mux -> cmd_vel_mux -> 驱动

启动顺序里有两个必要的前置：
  * lidar_reset.py 先把雷达 STOP+RESET —— 上个驱动退出后雷达常停在扫描状态，
    新驱动会卡在 getDeviceInfo（无 S/N、无 /scan）。
  * scan_self_filter.py -> /scan_filtered —— 机械臂在雷达 0deg 方向有固定回波
    （车尾雷达到车头臂 ~0.44 m），不滤掉会被当成障碍，collision_monitor 永久停车。

注意：config/nav2_params.yaml 里的 AMCL initial_pose 是 2026-11-17 这次跑车时
的位姿，换位置起车必须更新，否则定位会从错的位置开始。
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PARAMS = os.path.join(HERE, 'config', 'nav2_params.yaml')
MAP_YAML = os.path.join(HERE, 'maps', 'nav_20261117.yaml')
URDF = os.path.join(HERE, 'urdf', 'robot.urdf')


def generate_launch_description():
    ports = sorted(glob.glob('/dev/serial/by-id/*CP2102*'))
    if not ports:
        raise RuntimeError('No CP2102 lidar adapter found under /dev/serial/by-id/')

    with open(URDF, 'r') as f:
        robot_description = f.read()

    reset = ExecuteProcess(cmd=['python3', os.path.join(HERE, 'tools', 'lidar_reset.py'), ports[0]], output='screen')

    lidar = Node(
        package='rplidar_ros', executable='rplidar_composition',
        name='rplidar_composition', output='screen',
        parameters=[{
            'serial_port': ports[0], 'serial_baudrate': 115200,
            'frame_id': 'laser', 'inverted': False,
            'angle_compensate': True, 'auto_standby': False,
        }],
    )

    scan_filter = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'tools', 'scan_self_filter.py')],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'localization_launch.py')),
        launch_arguments={
            'map': MAP_YAML, 'use_sim_time': 'false',
            'params_file': PARAMS, 'autostart': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            HERE, 'launch', 'nav2_navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false', 'params_file': PARAMS,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        # nav2 进程反复重启后 Fast DDS 的 SHM 端口会残留冲突
        # (`RTPS_TRANSPORT_SHM ... open_and_lock_file failed`，服务调用要几十秒)。
        # 这个 launch 起的节点改走 UDP，避开 SHM；与走默认传输的基础服务仍能互通。
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),

        reset,
        TimerAction(period=3.0, actions=[lidar]),
        TimerAction(period=5.0, actions=[scan_filter]),
        rsp, localization, navigation,
    ])
