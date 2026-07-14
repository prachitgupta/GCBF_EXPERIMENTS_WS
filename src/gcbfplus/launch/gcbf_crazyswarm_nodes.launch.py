import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("crazyflie_vicon_bringup")
    crazyflies_yaml = os.path.join(bringup_share, "config", "crazyflies_8.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rate_hz", default_value="30.0"),
            DeclareLaunchArgument("hover_height", default_value="0.5"),
            DeclareLaunchArgument("max_runtime", default_value="30.0"),
            DeclareLaunchArgument("save_error_plots", default_value="false"),
            DeclareLaunchArgument("save_animation", default_value="false"),
            Node(
                package="gcbfplus",
                executable="gcbf_state_bridge",
                namespace="gcbf",
                name="gcbf_state_bridge",
                output="screen",
                parameters=[
                    {
                        "crazyflies_yaml": crazyflies_yaml,
                        "rate_hz": LaunchConfiguration("rate_hz"),
                    }
                ],
            ),
            Node(
                package="gcbfplus",
                executable="gcbf_actor",
                namespace="gcbf",
                name="gcbf_actor",
                output="screen",
                parameters=[
                    {"crazyflies_yaml": crazyflies_yaml},
                    {
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "hover_height": LaunchConfiguration("hover_height"),
                        "max_runtime": LaunchConfiguration("max_runtime"),
                    }
                ],
            ),
            Node(
                package="gcbfplus",
                executable="gcbf_monitor",
                namespace="gcbf",
                name="gcbf_monitor",
                output="screen",
                parameters=[
                    {
                        "crazyflies_yaml": crazyflies_yaml,
                        "hover_height": LaunchConfiguration("hover_height"),
                        "save_error_plots": LaunchConfiguration("save_error_plots"),
                        "save_animation": LaunchConfiguration("save_animation"),
                    }
                ],
            ),
        ]
    )
