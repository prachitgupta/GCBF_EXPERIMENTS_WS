import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression


def generate_launch_description():
    package_name = 'crazyflie_vicon_bringup'
    package_share = get_package_share_directory(package_name)
    crazyflie_share = get_package_share_directory('crazyflie')
    hardware_yaml = os.path.join(package_share, 'config', 'crazyflies_8.yaml')
    simulation_yaml = os.path.join(package_share, 'config', 'crazyflies_8_sim.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument('backend', default_value='cflib'),
            DeclareLaunchArgument('mocap', default_value='True'),
            DeclareLaunchArgument('gui', default_value='false'),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'ros_domain_id',
                default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='88'),
            ),
            DeclareLaunchArgument(
                'firmware_bindings',
                default_value=EnvironmentVariable(
                    'CRAZYFLIE_FIRMWARE_BINDINGS', default_value=''
                ),
            ),
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
            SetEnvironmentVariable(
                'PYTHONPATH',
                [
                    LaunchConfiguration('firmware_bindings'),
                    ':',
                    EnvironmentVariable('PYTHONPATH', default_value=''),
                ],
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
                    'crazyflies_yaml_file': PythonExpression(
                        [
                            "'", simulation_yaml, "' if '",
                            LaunchConfiguration('backend'),
                            "' == 'sim' else '", hardware_yaml, "'",
                        ]
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
