"""启动 L150Pro 驱动 + EKF。

    ros2 launch l150pro_driver l150pro.launch.py
    ros2 launch l150pro_driver l150pro.launch.py port:=/dev/ttyUSB1
    ros2 launch l150pro_driver l150pro.launch.py use_ekf:=false   # 只跑驱动

前置：
    sudo apt install ros-$ROS_DISTRO-robot-localization
    sudo usermod -aG dialout $USER      # 加完要重新登录
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("l150pro_driver")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="下位机串口设备（接的是【串口1】）"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("use_ekf", default_value="true",
                              description="是否同时启动 robot_localization EKF"),
        DeclareLaunchArgument("publish_imu_orientation", default_value="false",
                              description="是否发布 IMU orientation（默认否，"
                                          "因为 yaw 会漂，见 ekf.yaml 说明）"),
        DeclareLaunchArgument("track_eff", default_value="1.000",
                              description="有效【全】轮距(m)，标定后修改。"
                                          "默认 0.460+0.540（2026-09 换车）。"
                                          "外推初值，滑移转向需原地转标定"),
        DeclareLaunchArgument("wheel_scale", default_value="1.0",
                              description="轮速比例修正，补偿固件轮径配置与实际不符。"
                                          "= 实际距离 / 指令距离"),

        Node(
            package="l150pro_driver",
            executable="l150pro_driver",
            name="l150pro_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "publish_imu_orientation":
                    LaunchConfiguration("publish_imu_orientation"),
                "track_eff": LaunchConfiguration("track_eff"),
                "wheel_scale": LaunchConfiguration("wheel_scale"),
            }],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_ekf")),
            parameters=[PathJoinSubstitution([pkg, "config", "ekf.yaml"])],
            remappings=[("odometry/filtered", "odom")],
        ),
    ])
