"""Main ROS 2 launch entry point for simulation and real-robot remapping."""

import atexit
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


_temporary_parameter_files = []


def _cleanup_temporary_parameter_files():
    for path in _temporary_parameter_files:
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


atexit.register(_cleanup_temporary_parameter_files)


def _write_node_parameter_file(node_name, parameters):
    """Write launch overrides at node scope.

    Foxy gives a node-specific YAML block precedence over launch_ros' ``/**``
    wildcard dictionary regardless of argument order. Keeping the generated
    override node-specific makes launch arguments authoritative.
    """
    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"{node_name}_", suffix=".yaml", delete=False
    )
    try:
        yaml.safe_dump({node_name: {"ros__parameters": parameters}}, handle)
    finally:
        handle.close()
    _temporary_parameter_files.append(handle.name)
    return handle.name


def _setup(context):
    scan_share = get_package_share_directory("scan_planner")
    go2_share = get_package_share_directory("go2_description")
    planner_yaml = os.path.join(scan_share, "config", "planner.yaml")
    controllers_yaml = os.path.join(scan_share, "config", "controllers.yaml")
    is_real = _as_bool(LaunchConfiguration("is_real_world").perform(context))
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    sensor_type = LaunchConfiguration("sensor_type").perform(context)
    controller_mode = LaunchConfiguration("controller_mode").perform(context)
    keypoints_file = LaunchConfiguration("keypoints_file").perform(context)
    publish_robot_state = _as_bool(
        LaunchConfiguration("publish_robot_state").perform(context)
    )
    lidar_need_extrinsic = _as_bool(
        LaunchConfiguration("lidar_need_extrinsic").perform(context)
    )
    navi_mode = int(LaunchConfiguration("navi_mode").perform(context))
    if sensor_type not in ("lidar", "depth"):
        raise RuntimeError("sensor_type must be 'lidar' or 'depth'")
    if controller_mode not in ("open_loop", "closed_loop"):
        raise RuntimeError("controller_mode must be 'open_loop' or 'closed_loop'")
    if navi_mode not in (1, 2, 3):
        raise RuntimeError("navi_mode must be 1, 2, or 3")
    if navi_mode == 2 and (not keypoints_file or not os.path.isfile(keypoints_file)):
        raise RuntimeError(
            "navi_mode=2 requires keypoints_file to reference a ROS 2 parameter YAML"
        )

    if is_real:
        body_pose = "/LIO/odom_vehicle"
        sensor_pose = "/LIO/odom_imu"
        cloud = "/LIO/clouds_lidar"
        depth = "/camera/aligned_depth_to_color/image_raw"
        cloud_is_world = False
        need_extrinsic = lidar_need_extrinsic
        intrinsics = {
            "grid_map.cx": 317.19183349609375,
            "grid_map.cy": 256.4806823730469,
            "grid_map.fx": 609.5884399414062,
            "grid_map.fy": 609.22021484375,
        }
    else:
        body_pose = "/quad_0/body_pose"
        sensor_pose = "/quad_0/camera_pose" if sensor_type == "depth" else "/quad_0/lidar_pose"
        cloud = "/quad_0/cloud"
        depth = "/quad_0/depth"
        cloud_is_world = True
        need_extrinsic = False
        intrinsics = {}

    common = {"use_sim_time": use_sim_time}
    planner_overrides = {
        **common,
        **intrinsics,
        "fsm.navi_mode": navi_mode,
        "grid_map.sensor_type": sensor_type,
        "grid_map.cloud_is_world": cloud_is_world,
        "grid_map.need_extrinsic": need_extrinsic,
    }
    planner_overrides_file = _write_node_parameter_file(
        "scan_planner_node", planner_overrides
    )
    actions = [
        Node(
            package="scan_planner",
            executable="scan_planner_node",
            name="scan_planner_node",
            output="screen",
            parameters=[planner_yaml]
            + ([keypoints_file] if keypoints_file else [])
            + [planner_overrides_file],
            remappings=[
                ("body_pose", body_pose),
                ("sensor_pose", sensor_pose),
                ("cloud", cloud),
                ("depth", depth),
                ("move_base_simple/goal", "/move_base_simple/goal"),
                ("initial_path", "/initial_path"),
            ],
        )
    ]
    if publish_robot_state:
        try:
            import xacro
        except ImportError as error:
            raise RuntimeError(
                "xacro Python module is required for robot_state_publisher; "
                "install ros-foxy-xacro or launch with publish_robot_state:=false"
            ) from error
        robot_description = xacro.process_file(
            os.path.join(go2_share, "xacro", "robot.xacro"),
            mappings={"use_gazebo": "false"},
        ).toxml()
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[common, {"robot_description": robot_description}],
            )
        )

    if controller_mode == "open_loop":
        actions.append(
            Node(
                package="scan_planner",
                executable="open_loop_controller",
                name="open_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("planning/bspline", "/planning/bspline"),
                    ("body_pose", body_pose),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="scan_planner",
                executable="closed_loop_controller",
                name="closed_loop_controller",
                output="screen",
                parameters=[controllers_yaml, common],
                remappings=[
                    ("body_pose", body_pose),
                    ("cmd_vel", "/cmd_vel" if is_real else "/quad_0/cmd_vel"),
                ],
            )
        )
        if not is_real:
            actions.append(
                Node(
                    package="scan_planner",
                    executable="go2_kinematic_sim",
                    name="go2_kinematic_sim",
                    output="screen",
                    parameters=[
                        controllers_yaml,
                        common,
                        {
                            "init_x": float(LaunchConfiguration("init_x").perform(context)),
                            "init_y": float(LaunchConfiguration("init_y").perform(context)),
                            "init_z": float(LaunchConfiguration("init_z").perform(context)),
                            "publish_tf": False,
                        },
                    ],
                    remappings=[
                        ("body_pose", "/quad_0/body_pose"),
                        ("cmd_vel", "/quad_0/cmd_vel"),
                    ],
                )
            )

    if not is_real:
        actions.extend(
            [
                Node(
                    package="scan_planner",
                    executable="go2_gait_publisher",
                    name="go2_gait_publisher",
                    output="screen",
                    parameters=[controllers_yaml, common],
                    remappings=[("body_pose", body_pose)],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(scan_share, "launch", "simulator.launch.py")
                    ),
                    launch_arguments={
                        name: LaunchConfiguration(name)
                        for name in (
                            "is_real_world",
                            "sensor_type",
                            "use_gpu",
                            "use_pcd_map",
                            "pcd_map_file",
                            "map_size_x",
                            "map_size_y",
                            "map_size_z",
                            "use_sim_time",
                        )
                    }.items(),
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("is_real_world", default_value="false"),
            DeclareLaunchArgument("navi_mode", default_value="1"),
            DeclareLaunchArgument("sensor_type", default_value="lidar"),
            DeclareLaunchArgument("controller_mode", default_value="closed_loop"),
            DeclareLaunchArgument("keypoints_file", default_value=""),
            DeclareLaunchArgument("use_gpu", default_value="false"),
            DeclareLaunchArgument("use_pcd_map", default_value="false"),
            DeclareLaunchArgument("pcd_map_file", default_value=""),
            DeclareLaunchArgument(
                "publish_robot_state",
                default_value="true",
                description="Publish the Go2 URDF; requires the xacro Python module",
            ),
            DeclareLaunchArgument(
                "lidar_need_extrinsic",
                default_value="false",
                description="Apply grid_map's built-in LiDAR extrinsic to real lidar poses",
            ),
            DeclareLaunchArgument("map_size_x", default_value="40.0"),
            DeclareLaunchArgument("map_size_y", default_value="40.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("init_x", default_value="-19.0"),
            DeclareLaunchArgument("init_y", default_value="1.0"),
            DeclareLaunchArgument("init_z", default_value="0.3"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
