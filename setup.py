from setuptools import setup
import os
import glob  # <-- add this line
from setuptools import find_packages

package_name = 'xrce_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'cvxpy',
        'tf_transformations',
        'ros2-numpy',
    ],
    zip_safe=True,
    maintainer='Viswa Narayanan Sankaranarayanan',
    maintainer_email='vissan@ltu.se',
    description='Offboard UAV control node using PX4 and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: executable_name = package.module:function
            'att_controller_pcl = xrce_ros2_control.base_shafter_land:main',
            'att_controller_lidar_2d = xrce_ros2_control.att_control_lidar_2d:main',
            'att_controller_lidar_2d_vel = xrce_ros2_control.att_control_lidar_2d_vel:main',
            'thr_controller_lidar_2d = xrce_ros2_control.thr_control_lidar_2d:main',
            'px4_tf = xrce_ros2_control.px4_tf:main',
            'vins2fmu_relay = xrce_ros2_control.vins2fmu:main',
            'dlio2fmu_relay = xrce_ros2_control.dlio2fmu:main',
            'mocap2fmu_relay = xrce_ros2_control.mocap2fmu:main',
            'pos_controller = xrce_ros2_control.offboard_traj_control:main',
            'vel_controller = xrce_ros2_control.vel_controller:main',
            'keyboard_teleop = xrce_ros2_control.keyboard_teleop:main',
            'pos_sp_filter = xrce_ros2_control.hw_pos_sp:main'
        ],
    },
)
