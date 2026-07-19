import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    package_name = 'crazyflie_vicon_bringup'
    package_share = get_package_share_directory(package_name)
    crazyflie_share = get_package_share_directory('crazyflie')
    firmware_bindings = '/home/james/Research/GCBF/deps/crazyflie-firmware/build'

    return LaunchDescription(
        [
            DeclareLaunchArgument('backend', default_value='cflib'),
            DeclareLaunchArgument('mocap', default_value='True'),
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', '88'),
            SetEnvironmentVariable(
                'PYTHONPATH',
                [firmware_bindings, ':', EnvironmentVariable('PYTHONPATH', default_value='')],
            ),
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
                        package_share, 'config', 'crazyflies_8_debug_single.yaml'
                    ),
                    'motion_capture_yaml_file': os.path.join(
                        package_share, 'config', 'motion_capture.yaml'
                    ),
                    'rviz_config_file': os.path.join(
                        package_share, 'config', 'rviz_8_tf.yaml'
                    ),
                }.items(),
            ),
        ]
    )
