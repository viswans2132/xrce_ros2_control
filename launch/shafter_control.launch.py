from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    python_env = "/pip_env/bin/python3"

    # Declare a command-line argument called 'hw_test_name' with a default value
    drone_name_arg = DeclareLaunchArgument(
        'drone_name',
        default_value='Holybro',  # default value as string
        description='Choose the drone name'
    )
    hw_test_arg = DeclareLaunchArgument(
        'hw_test',
        default_value='false',  # default value as string
        description='Enable or disable hardware test'
    )
    ext_odom_source_arg = DeclareLaunchArgument(
        'ext_odom_source',
        default_value='LIDAR',  # default value as string
        description='Choose the external odom source'
    )
    ext_arming_arg = DeclareLaunchArgument(
        'ext_arming',
        default_value='false',  # default value as string
        description='Enable or disable arming by the code'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',  # default value as string
        description='Enable or disable arming by the code'
    )

    # Use LaunchConfiguration to access the value
    drone_name_value = LaunchConfiguration('drone_name')
    hw_test_value = LaunchConfiguration('hw_test')
    ext_odom_source_value = LaunchConfiguration('ext_odom_source')
    ext_arming_value = LaunchConfiguration('ext_arming')
    auto_start_value = LaunchConfiguration('auto_start')


    return LaunchDescription([
        drone_name_arg,
        hw_test_arg,
        ext_odom_source_arg,
        ext_arming_arg,
        Node(
            package='xrce_ros2_control',
            executable='att_controller_pcl',  # matches entry point in setup.py
            name='att_controller_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'hw_test': hw_test_value, 
                        'ext_odom_source': ext_odom_source_value, 
                        'ext_arming': ext_arming_value,
                        'auto_start': auto_start_value}],
            remappings=[],
            prefix=[python_env + " "],
        ),
        Node(
            package='xrce_ros2_control',
            executable='pos_sp_filter',  # matches entry point in setup.py
            name='pos_sp_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'drone_name': 'shafterx2',
                        'hw_test': hw_test_value, 
                        'ext_odom_source': ext_odom_source_value, 
                        'ext_arming': ext_arming_value}],
            remappings=[],
            prefix=[python_env + " "],
        ),
        Node(
            package='xrce_ros2_control',
            executable='dlio2fmu_relay',  # matches entry point in setup.py
            name='odom_relay_node',
            output='screen',
            emulate_tty=True,
            parameters=[],
            remappings=[],
            prefix=[python_env + " "],
            condition=IfCondition(hw_test_value)
        ),
    ])
