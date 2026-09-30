from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    js_device = LaunchConfiguration('js_device')
    serial_path = LaunchConfiguration('serial_path')

    return LaunchDescription([
        DeclareLaunchArgument('js_device', default_value='/dev/input/js0'),
        DeclareLaunchArgument('serial_path', default_value='/dev/l150pro'),

        Node(
            package='raspot_teleop',
            executable='ps2_teleop_node',
            name='ps2_teleop',
            output='screen',
            parameters=[{'js_device': js_device}],
        ),
        Node(
            package='raspot_teleop',
            executable='l150pro_driver_node',
            name='l150pro_driver',
            output='screen',
            parameters=[{
                'serial_path': serial_path,
                # 烧录版固件 0x56 从不置 ONLINE 位（与源码行为不符，待查）；
                # 先放开门控让臂可用。固件问题解决后改回 true。
                'require_online': False,
            }],
        ),
        Node(
            package='raspot_teleop',
            executable='arm_recorder_node',
            name='arm_recorder',
            output='screen',
            parameters=[{'log_path': '/home/cy/arm_recorder.log'}],
        ),
    ])
