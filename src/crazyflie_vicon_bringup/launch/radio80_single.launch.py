import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('crazyflie_vicon_bringup')
    crazyflie_share = get_package_share_directory('crazyflie')

    return LaunchDescription(
        [
            DeclareLaunchArgument('backend', default_value='cflib'),
            DeclareLaunchArgument('mocap', default_value='True'),
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(crazyflie_share, 'launch'), '/launch.py']
                ),
                launch_arguments={
                    'backend': LaunchConfiguration('backend'),
                    'gui': LaunchConfiguration('gui'),
                    'rviz': LaunchConfiguration('rviz'),
                    'teleop': 'false',
                    'mocap': LaunchConfiguration('mocap'),
                    'crazyflies_yaml_file': os.path.join(
                        package_share,
                        'config',
                        'crazyflies_radio80_single.yaml',
                    ),
                    'motion_capture_yaml_file': os.path.join(
                        package_share, 'config', 'motion_capture.yaml'
                    ),
                }.items(),
            ),
        ]
    )
