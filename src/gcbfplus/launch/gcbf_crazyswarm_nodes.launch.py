import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("crazyflie_vicon_bringup")
    hardware_yaml = os.path.join(bringup_share, "config", "crazyflies_8.yaml")
    simulation_yaml = os.path.join(bringup_share, "config", "crazyflies_8_sim.yaml")
    crazyflies_yaml = PythonExpression(
        [
            "'", simulation_yaml, "' if '", LaunchConfiguration("mode"),
            "' == 'sim' else '", hardware_yaml, "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="88"),
            ),
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument("lookahead_dt", default_value="0.05"),
            DeclareLaunchArgument("hover_height", default_value="0.5"),
            DeclareLaunchArgument("hover_epsilon", default_value="0.06"),
            DeclareLaunchArgument("takeoff_timeout", default_value="60.0"),
            DeclareLaunchArgument("goal_tolerance", default_value="0.12"),
            DeclareLaunchArgument("max_runtime", default_value="30.0"),
            DeclareLaunchArgument("area_size", default_value="4.0"),
            DeclareLaunchArgument("car_radius", default_value="0.05"),
            DeclareLaunchArgument("max_action_age", default_value="0.02"),
            DeclareLaunchArgument("print_latency", default_value="false"),
            DeclareLaunchArgument("save_error_plots", default_value="false"),
            DeclareLaunchArgument("save_animation", default_value="false"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
            Node(
                package="gcbfplus",
                executable="gcbf_actor",
                namespace="gcbf",
                name="gcbf_actor",
                output="screen",
                parameters=[
                    {"crazyflies_yaml": crazyflies_yaml},
                    {
                        "mode": LaunchConfiguration("mode"),
                        "use_sim_time": PythonExpression(
                            ["'", LaunchConfiguration("mode"), "' == 'sim'"]
                        ),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "lookahead_dt": LaunchConfiguration("lookahead_dt"),
                        "hover_height": LaunchConfiguration("hover_height"),
                        "hover_epsilon": LaunchConfiguration("hover_epsilon"),
                        "takeoff_timeout": LaunchConfiguration("takeoff_timeout"),
                        "goal_tolerance": LaunchConfiguration("goal_tolerance"),
                        "max_runtime": LaunchConfiguration("max_runtime"),
                        "area_size": LaunchConfiguration("area_size"),
                        "car_radius": LaunchConfiguration("car_radius"),
                        "max_action_age": LaunchConfiguration("max_action_age"),
                        "print_latency": LaunchConfiguration("print_latency"),
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
                        "use_sim_time": PythonExpression(
                            ["'", LaunchConfiguration("mode"), "' == 'sim'"]
                        ),
                        "crazyflies_yaml": crazyflies_yaml,
                        "hover_height": LaunchConfiguration("hover_height"),
                        "save_error_plots": LaunchConfiguration("save_error_plots"),
                        "save_animation": LaunchConfiguration("save_animation"),
                    }
                ],
            ),
        ]
    )
