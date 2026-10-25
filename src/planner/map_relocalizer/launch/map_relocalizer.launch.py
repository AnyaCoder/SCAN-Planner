import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("map_relocalizer")
    config = os.path.join(share, "config", "map_relocalizer.yaml")
    rviz_config = os.path.join(share, "rviz", "relocalization.rviz")

    prior_map_file = LaunchConfiguration("prior_map_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prior_map_file",
                description="World-aligned PCD map used as the relocalization target",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="map_relocalizer",
                executable="map_relocalizer_node",
                name="map_relocalizer",
                output="screen",
                parameters=[
                    config,
                    {
                        "prior_map_file": prior_map_file,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_relocalizer_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
