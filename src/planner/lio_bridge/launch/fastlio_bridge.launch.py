from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_frame_id", default_value="world"),
        DeclareLaunchArgument("output_child_frame_id", default_value="base"),
        DeclareLaunchArgument("velocity_window_sec", default_value="1.0"),
        DeclareLaunchArgument("velocity_deadband", default_value="0.12"),
        DeclareLaunchArgument("max_velocity", default_value="2.0"),
        DeclareLaunchArgument("max_timestamp_gap_sec", default_value="0.25"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="lio_bridge",
            executable="lio_bridge_node",
            name="lio_bridge",
            output="screen",
            parameters=[{
                "odom_topic": "/Odometry",
                "cloud_topic": "/cloud_registered_body",
                "output_vehicle_topic": "/LIO/odom_vehicle",
                "output_imu_topic": "/LIO/odom_imu",
                "output_cloud_topic": "/LIO/clouds_lidar",
                "output_frame_id": LaunchConfiguration("output_frame_id"),
                "output_child_frame_id": LaunchConfiguration("output_child_frame_id"),
                "velocity_window_sec": LaunchConfiguration("velocity_window_sec"),
                "velocity_deadband": LaunchConfiguration("velocity_deadband"),
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_timestamp_gap_sec": LaunchConfiguration("max_timestamp_gap_sec"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
