from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xrce_ros2_control',
            executable='thr_controller_lidar_2d',  # matches entry point in setup.py
            name='att_controller_node',
            output='screen',
            emulate_tty=True,
            parameters=[],
            remappings=[],
        ),
        Node(
            package='xrce_ros2_control',
            executable='pos_sp_filter',  # matches entry point in setup.py
            name='pos_sp_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'drone_name': 'holybro'}],
            remappings=[],
        ),
        Node(
            package='xrce_ros2_control',
            executable='vins2fmu_relay',  # matches entry point in setup.py
            name='odom_relay_node',
            output='screen',
            emulate_tty=True,
            parameters=[],
            remappings=[],
        ),
    ])
