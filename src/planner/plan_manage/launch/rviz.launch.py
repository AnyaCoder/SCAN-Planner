"""Start RViz2 with the SCAN-Planner display configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_rviz(context):
    share = get_package_share_directory("scan_planner")
    selection = LaunchConfiguration("rviz_config").perform(context)
    configs = {
        "default": os.path.join(share, "rviz", "default.rviz"),
        "sim": os.path.join(share, "rviz", "default.rviz"),
        "real": os.path.join(share, "rviz", "default_real.rviz"),
        "go2": os.path.join(share, "rviz", "default_real.rviz"),
    }
    config = configs.get(selection, selection)
    if not os.path.isfile(config):
        raise RuntimeError(f"RViz config not found: {config}")
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="default",
                description="default, sim, real, go2, or an absolute .rviz path",
            ),
            OpaqueFunction(function=_start_rviz),
        ]
    )
