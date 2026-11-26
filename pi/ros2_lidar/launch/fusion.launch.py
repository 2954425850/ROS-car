"""融合模式：slam_toolbox 提供 map->odom，nav2 只跑规划/控制。

    ros2 launch /home/cy/ros2_lidar/launch/fusion.launch.py

不启动 map_server / AMCL / pose_persist / auto_localize。
速度链路与 nav.launch.py 相同（no_pivot_guard + cmd_mux 不变）。
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
SLAM_PARAMS = os.path.join(HERE, "config", "slam_params_fusion.yaml")
NAV_PARAMS = os.path.join(HERE, "config", "nav2_params_fusion.yaml")
URDF = os.path.join(HERE, "urdf", "robot.urdf")


def generate_launch_description():
    ports = sorted(glob.glob("/dev/serial/by-id/*CP2102*"))
    if not ports:
        raise RuntimeError(
            "No CP2102 lidar adapter found under /dev/serial/by-id/ -- "
            "is the RPLIDAR plugged in?")

    with open(URDF, "r", encoding="utf-8") as f:
        robot_description = f.read()

    reset = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "lidar_reset.py"),
             ports[0]],
        output="screen")

    lidar = Node(
        package="rplidar_ros", executable="rplidar_composition",
        name="rplidar_composition", output="screen",
        parameters=[{
            "serial_port": ports[0], "serial_baudrate": 115200,
            "frame_id": "laser", "inverted": False,
            "angle_compensate": True, "auto_standby": False,
        }],
    )

    scan_filter = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "scan_self_filter.py")],
        output="screen")

    # 只把“当前运动方向扫过的车体”交给 collision_monitor；原地转时它会
    # 按转向预测未来的扫掠角，身后/侧边无关点不再触发停车。
    motion_gate = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "scan_motion_gate.py")],
        output="screen")

    # 融合/探索模式允许慢速原地转（死角和 U 型调头需要）；导航模式
    # nav.launch.py 仍用默认 allow_slow_pivot=false，5/5 验收规则不变。
    pivot_guard = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "no_pivot_guard.py"),
             "--ros-args",
             "-p", "allow_slow_pivot:=true",
             "-p", "slow_pivot_omega:=0.35"],
        output="screen")

    fusion_nav = ExecuteProcess(
        cmd=["python3", os.path.join(HERE, "tools", "fusion_nav.py")],
        output="screen")

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch", "online_async_launch.py")),
        launch_arguments={
            "slam_params_file": SLAM_PARAMS,
            "use_sim_time": "false",
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            HERE, "launch", "nav2_navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": NAV_PARAMS,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        reset,
        rsp,
        TimerAction(period=3.0, actions=[lidar]),
        TimerAction(period=5.0, actions=[scan_filter]),
        TimerAction(period=5.5, actions=[motion_gate]),
        TimerAction(period=6.0, actions=[pivot_guard]),
        TimerAction(period=6.0, actions=[slam]),
        navigation,
        TimerAction(period=12.0, actions=[fusion_nav]),
    ])
