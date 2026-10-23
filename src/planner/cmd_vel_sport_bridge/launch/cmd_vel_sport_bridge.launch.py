from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cmd_vel_sport_bridge",
                executable="cmd_vel_sport_bridge",
                name="cmd_vel_sport_bridge",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": "/cmd_vel",
                        "sport_request_topic": "/api/sport/request",
                        "api_id": 1008,
                    }
                ],
            )
        ]
    )
