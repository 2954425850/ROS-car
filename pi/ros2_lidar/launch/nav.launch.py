"""建图之后的定位 + 导航：雷达 + TF + AMCL + nav2（2026-11-18 方案）。

    ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py
    ros2 launch ... use_composition:=False      # 回退到多进程（排障用）
    ros2 launch ... map:=/home/cy/ros2_lidar/maps/xxx.yaml   # 换一张地图


链路（速度优先级由 cmd_mux 仲裁，导航最低）：
  controller_server -> cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed
    -> no_pivot_guard -> cmd_vel_guard -> collision_monitor -> cmd_vel_nav
    -> cmd_mux -> cmd_vel_mux -> 驱动

启动顺序里有几个必要的前置：
  * lidar_reset.py 先把雷达 STOP+RESET —— 上个驱动退出后雷达常停在扫描状态，
    新驱动会卡在 getDeviceInfo（无 S/N、无 /scan）。
  * scan_self_filter.py -> /scan_filtered —— 机械臂在雷达 0deg 方向有固定回波，
    不滤掉会被当成障碍，collision_monitor 永久停车。
  * scan_motion_gate.py -> /scan_motion_guard 只给 collision_monitor 用，
    只保留当前运动方向会扫到的点；身后/侧边近点不再触发停车。它由
    scan-gate.service 常驻拉起（不再放 launch 里），nav.service Wants 它。

2026-11-18 方案新增：
  * no_pivot_guard：禁止原地转（|w| <= |v|/R_min）。
  * pose_persist：把 /amcl_pose 存盘，下次启动自动发 /initialpose。
  * use_composition 默认 true：nav2 节点放进一个容器，减少 DDS 参与者。
  * 不再启动 smoother/route/waypoint/docking。

注意：config/nav2_params.yaml 里的 AMCL initial_pose 只是兜底；正常情况下
pose_persist 会用上次保存的位姿覆盖它。换场地/搬动车子后删掉
maps/last_pose.json。
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PARAMS = os.path.join(HERE, 'config', 'nav2_params.yaml')
# 默认地图保持不变（systemd 的 nav.service 不带参数，行为与之前一致）。
# 临时入口/语音可以用 `map:=/abs/path/to/xxx.yaml` 覆盖。
MAP_DEFAULT = os.path.join(HERE, 'maps', 'nav_20261124.yaml')
URDF = os.path.join(HERE, 'urdf', 'robot.urdf')


def generate_launch_description():
    ports = sorted(glob.glob('/dev/serial/by-id/*CP2102*'))
    if not ports:
        raise RuntimeError('No CP2102 lidar adapter found under /dev/serial/by-id/')

    with open(URDF, 'r') as f:
        robot_description = f.read()

    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    map_yaml = LaunchConfiguration('map')

    reset = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'tools', 'lidar_reset.py'), ports[0]],
        output='screen')

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

    pivot_guard = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'tools', 'no_pivot_guard.py')],
        output='screen')

    # pose_persist only SAVES now (max_attempts:=0); auto_localize owns the
    # startup restore (it validates the saved pose and falls back to a global
    # scan-to-map search).
    pose_persist = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'tools', 'pose_persist.py'),
             '--ros-args', '-p', 'max_attempts:=0',
             # 与 auto_localize 同一张地图、同一个 *_last_pose.json
             '-p', ['map_yaml:=', map_yaml]],
        output='screen')

    auto_localize = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'tools', 'auto_localize.py'),
             '--ros-args', '-p', ['map_yaml:=', map_yaml],
             '-p', 'scan_topic:=scan_filtered'],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    container = ComposableNodeContainer(
        condition=IfCondition(use_composition),
        name=container_name, namespace='',
        package='rclcpp_components', executable='component_container_isolated',
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml, 'use_sim_time': 'false',
            'params_file': PARAMS, 'autostart': 'true',
            'use_composition': use_composition,
            'container_name': container_name,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            HERE, 'launch', 'nav2_navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false', 'params_file': PARAMS,
            'autostart': 'true',
            'use_composition': use_composition,
            'container_name': container_name,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_composition', default_value='True'),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument(
            'map', default_value=MAP_DEFAULT,
            description='静态地图 yaml（绝对路径）；默认 nav_20261124.yaml'),

        # nav2 进程反复重启后 Fast DDS 的 SHM 端口会残留冲突。
        # 这个 launch 起的节点改走 UDP，避开 SHM；与基础服务仍能互通。
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),

        reset,
        TimerAction(period=3.0, actions=[lidar]),
        TimerAction(period=5.0, actions=[scan_filter]),
        TimerAction(period=6.0, actions=[pivot_guard]),
        TimerAction(period=6.0, actions=[pose_persist]),
        TimerAction(period=6.0, actions=[auto_localize]),
        rsp, container, localization, navigation,
    ])
