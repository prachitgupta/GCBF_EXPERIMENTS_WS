import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'crazyflie_vicon_bringup'
    package_share = get_package_share_directory(package_name)
    crazyflie_share = get_package_share_directory('crazyflie')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(crazyflie_share, 'launch'), '/launch.py']
                ),
                launch_arguments={
                    'backend': 'cflib',
                    'gui': 'false',
                    'rviz': 'false',
                    'teleop': 'false',
                    'mocap': 'true',
                    'crazyflies_yaml_file': os.path.join(
                        package_share, 'config', 'crazyflies.yaml'
                    ),
                    'motion_capture_yaml_file': os.path.join(
                        package_share, 'config', 'motion_capture.yaml'
                    ),
                }.items(),
            ),
            Node(
                package='crazyflie',
                executable='vel_mux.py',
                name='vel_mux',
                output='screen',
                parameters=[
                    {'hover_height': 0.4},
                    {'incoming_twist_topic': '/cmd_vel'},
                    {'robot_prefix': '/cf8'},
                ],
            ),
        ]
    )
