import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                    'mocap': 'True',
                    'crazyflies_yaml_file': os.path.join(
                        package_share, 'config', 'crazyflies.yaml'
                    ),
                    'motion_capture_yaml_file': os.path.join(
                        package_share, 'config', 'motion_capture.yaml'
                    ),
                }.items(),
            ),
        ]
    )
