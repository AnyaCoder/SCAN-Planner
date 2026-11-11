"""Publish the calibrated Go2 front-camera TF and CameraInfo."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _load_calibration(path):
    with open(path, "r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    if calibration.get("schema_version") != 1:
        raise RuntimeError(f"unsupported calibration schema: {path}")
    return calibration


def _launch_setup(context, *args, **kwargs):
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    publish_tf = _as_bool(LaunchConfiguration("publish_tf").perform(context))
    publish_camera_info = _as_bool(
        LaunchConfiguration("publish_camera_info").perform(context)
    )
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)

    calibration = _load_calibration(calibration_file)
    extrinsic = calibration["extrinsic"]
    translation = extrinsic["translation_m"]
    quaternion = extrinsic["quaternion_xyzw"]

    actions = []
    if publish_tf:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="go2_front_camera_static_tf",
                output="screen",
                arguments=[
                    *[str(value) for value in translation],
                    *[str(value) for value in quaternion],
                    extrinsic["parent_frame"],
                    extrinsic["child_frame"],
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    if publish_camera_info:
        actions.append(
            Node(
                package="go2_camera_calibration",
                executable="camera_info_publisher.py",
                name="go2_front_camera_info",
                output="screen",
                arguments=[
                    "--calibration-file",
                    calibration_file,
                    "--topic",
                    camera_info_topic,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description():
    default_calibration = os.path.join(
        get_package_share_directory("go2_camera_calibration"),
        "config",
        "go2_front_camera.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value=default_calibration,
                description="Calibration YAML installed by go2_camera_calibration",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/go2/front_camera/camera_info",
            ),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_camera_info", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
